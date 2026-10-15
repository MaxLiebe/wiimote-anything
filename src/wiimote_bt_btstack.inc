/* SPDX-License-Identifier: MIT */
/*
 * Bluetooth Classic HID peripheral built on BlueKitchen BTstack.
 *
 * The ESP32 presents itself as a Nintendo RVL-CNT-01:
 *   - local name "Nintendo RVL-CNT-01", class of device 0x002504
 *   - HID SDP record with the real remote's descriptor, PnP record 057e:0306
 *   - L2CAP PSM 0x11 (HID control) and 0x13 (HID interrupt)
 *   - legacy PIN pairing, PIN = the console's BD_ADDR reversed
 *   - reconnects to the last synced console by paging it, like a real remote
 *
 * Sharing BTstack with an HID host (Bluepad32)
 * --------------------------------------------
 * A Wii (host -> us) and a gamepad that reconnects to us (device -> host) both
 * open L2CAP channels on PSM 0x11 / 0x13. BTstack allows only one service per
 * PSM, so this file owns both PSMs and hands everything that is not the Wii to
 * the fallback handler set with wiimote_btstack_set_fallback_handler(). The
 * Wii link is told apart by its class of device (a console is not a
 * "peripheral") or by being the console we synced with before.
 *
 * Our HCI event handler sits at the head of BTstack's handler list. For the Wii
 * link it consumes the connection request and the PIN request (by rewriting the
 * event type to HCI_EVENT_NOP), so the host stack neither allocates a device
 * slot for the console nor answers the PIN with the wrong code. Every other
 * event passes through untouched.
 *
 * Nothing in here starts, stops, or otherwise manages scanning for gamepads.
 * That belongs to the application.
 */
#include "wiimote_bt.h"

#include "wiimote_internal.h"
#include "wiimote_target.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "btstack.h"
#include "btstack_tlv.h"
#include "classic/device_id_server.h"
#include "classic/sdp_server.h"
#include "classic/sdp_util.h"

#if __has_include("btstack_port_esp32.h")
#include "btstack_port_esp32.h"
#define WIIMOTE_HAVE_ESP32_PORT 1
#endif

#include "../include/wiimote_btstack.h"

static const char *TAG = "wiimote_bt";

#define WM_PSM_CONTROL 0x0011
#define WM_PSM_INTERRUPT 0x0013
#define WM_L2CAP_MTU 672
#define WM_REPORT_TICK_MS 5
#define WM_TLV_TAG_HOST 0x57694948u /* 'WiIH' */

#define WM_COD 0x002504u
#define WM_COD_MAJOR_MASK 0x001f00u
#define WM_COD_MAJOR_PERIPHERAL 0x000500u
#define WM_COD_WII 0x000448u
/* Page timeout for our own page to the console (5.12 s), and BTstack's
 * default, which the host stack uses for gamepads (15.36 s). */
#define WM_PAGE_TIMEOUT_CONSOLE 0x2000
#define WM_PAGE_TIMEOUT_DEFAULT 0x6000

static const char k_local_name[] = "Nintendo RVL-CNT-01";

/* Real RVL-CNT-01 HID report descriptor (Dolphin / hcidump of attribute 0x0206). */
static const uint8_t k_rvl_hid_descriptor[] = {
    0x05, 0x01, 0x09, 0x05, 0xa1, 0x01,
    0x85, 0x10, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x01, 0x06, 0x00, 0xff, 0x09, 0x01, 0x91, 0x00,
    0x85, 0x11, 0x95, 0x01, 0x09, 0x01, 0x91, 0x00,
    0x85, 0x12, 0x95, 0x02, 0x09, 0x01, 0x91, 0x00,
    0x85, 0x13, 0x95, 0x01, 0x09, 0x01, 0x91, 0x00,
    0x85, 0x14, 0x95, 0x01, 0x09, 0x01, 0x91, 0x00,
    0x85, 0x15, 0x95, 0x01, 0x09, 0x01, 0x91, 0x00,
    0x85, 0x16, 0x95, 0x15, 0x09, 0x01, 0x91, 0x00,
    0x85, 0x17, 0x95, 0x06, 0x09, 0x01, 0x91, 0x00,
    0x85, 0x18, 0x95, 0x15, 0x09, 0x01, 0x91, 0x00,
    0x85, 0x19, 0x95, 0x01, 0x09, 0x01, 0x91, 0x00,
    0x85, 0x1a, 0x95, 0x01, 0x09, 0x01, 0x91, 0x00,
    0x85, 0x20, 0x95, 0x06, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x21, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x22, 0x95, 0x04, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x30, 0x95, 0x02, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x31, 0x95, 0x05, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x32, 0x95, 0x0a, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x33, 0x95, 0x11, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x34, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x35, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x36, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x37, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x3d, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x3e, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00,
    0x85, 0x3f, 0x95, 0x15, 0x09, 0x01, 0x81, 0x00,
    0xc0,
};

/* ---------------------------------------------------------------- state -- */

static bool s_external_stack;
static bool s_setup_done;
static bool s_sdp_registered;
static btstack_packet_handler_t s_fallback;

/* ~370 bytes with BTstack's 3-byte sequence headers; the test checks the fit. */
static uint8_t s_hid_record[600];
static uint8_t s_pnp_record[128];
static uint32_t s_hid_record_handle;
static uint32_t s_pnp_record_handle;

static btstack_packet_callback_registration_t s_hci_registration;
static btstack_timer_source_t s_report_timer;
static bool s_report_timer_active;

/* The console we synced with last (persisted in TLV). */
static bd_addr_t s_host_addr;
static volatile bool s_have_host;

/* The console link that is up or being set up. */
static bool s_link_claimed;
static bd_addr_t s_link_addr;
static hci_con_handle_t s_link_handle = HCI_CON_HANDLE_INVALID;
static bool s_link_outgoing;
static uint16_t s_cid_control;
static uint16_t s_cid_interrupt;
static bool s_control_open;
static bool s_interrupt_open;
static bool s_link_up;
static bool s_link_master; /* our role on the console link, from ROLE_CHANGE */
static btstack_timer_source_t s_connect_watchdog;
#define WM_CONNECT_TIMEOUT_MS 15000

/* Incoming host-stack (gamepad) channels held back until the link reaches the
 * host stack's GAP security level, as its own PSM services would have done.
 * Our PSM services are level 0 because the console must never be challenged. */
#define WM_HELD_MAX 4
static struct {
    uint16_t cid;
    hci_con_handle_t handle;
    uint16_t size;
    uint8_t packet[24];
} s_held[WM_HELD_MAX];

static bool s_sync_active;

/* HID handshake owed on the control channel (0xff = none). */
static uint8_t s_pending_handshake = 0xff;

static uint8_t s_last_leds;
static bool s_have_leds;
static bool s_last_rumble;
static bool s_have_rumble;

/* Raw HCI commands that BTstack has no GAP setter for. */
#define WM_CMD_INQUIRY_SCAN_INTERLACED (1u << 0)
#define WM_CMD_WRITE_IAC_LIAC_GIAC (1u << 1)
#define WM_CMD_CANCEL_PAGE (1u << 2)
#define WM_CMD_POWER_OFF_DISCONNECT (1u << 3)
static uint32_t s_pending_cmds;
static bd_addr_t s_cancel_addr;
static hci_con_handle_t s_poweroff_handle;
/* Per-link policy writes (no sniff: sniff adds up to one sniff interval of
 * input latency on every report). */
static struct {
    hci_con_handle_t handle;
    uint16_t policy;
} s_policy_q[4];
static int s_policy_count;
/* Per-link QoS requests. As master, the controller decides how often it
 * polls each slave; a slave (the gamepad) can only send when polled, so a
 * long poll interval is pure input latency. QoS Setup asks for a short one. */
#ifndef WIIMOTE_QOS_LATENCY_US
#define WIIMOTE_QOS_LATENCY_US 1250 /* 2 slots */
#endif
#define WM_QOS_BEST_EFFORT 0x01
#define WM_QOS_GUARANTEED 0x02
static struct {
    hci_con_handle_t handle;
    uint8_t service_type;
} s_qos_q[4];
static int s_qos_count;
static hci_con_handle_t s_qos_inflight_handle = HCI_CON_HANDLE_INVALID;
static uint8_t s_qos_inflight_type;
static uint8_t s_last_disconnect_reason;
/* Role switch was turned off for our page to the console; turn it back on. */
static bool s_restore_role_switch;
/* The page timeout was shortened for our page to the console; restore it. */
static bool s_restore_page_timeout;

/* Requests posted from other tasks; executed on the BTstack thread. */
#define WM_REQ_SYNC (1u << 0)
#define WM_REQ_LISTEN (1u << 1)
#define WM_REQ_STOP (1u << 2)
#define WM_REQ_CONNECT (1u << 3)
#define WM_REQ_DISCONNECT (1u << 4)
#define WM_REQ_FORGET (1u << 5)
#define WM_REQ_SETUP (1u << 6)
#define WM_REQ_FLUSH (1u << 7)
static portMUX_TYPE s_req_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_requests;
static btstack_context_callback_registration_t s_req_registration;

static void wm_l2cap_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void wm_hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void flush_reports(void);

/* -------------------------------------------------------------- helpers -- */

static bool addr_eq(const bd_addr_t a, const bd_addr_t b)
{
    return memcmp(a, b, BD_ADDR_LEN) == 0;
}

static bool on_btstack_thread_or_not_running(void)
{
    /* Before the run loop starts (HCI off), app_main may call BTstack directly. */
    return hci_get_state() == HCI_STATE_OFF;
}

static void load_host(void)
{
    const btstack_tlv_t *tlv = NULL;
    void *ctx = NULL;
    uint8_t buf[BD_ADDR_LEN];
    btstack_tlv_get_instance(&tlv, &ctx);
    if (tlv == NULL) {
        return;
    }
    if (tlv->get_tag(ctx, WM_TLV_TAG_HOST, buf, sizeof(buf)) == (int)sizeof(buf)) {
        memcpy(s_host_addr, buf, BD_ADDR_LEN);
        s_have_host = true;
        ESP_LOGI(TAG, "Paired console: %s", bd_addr_to_str(s_host_addr));
    }
}

static void store_host(const bd_addr_t addr)
{
    const btstack_tlv_t *tlv = NULL;
    void *ctx = NULL;
    if (s_have_host && addr_eq(addr, s_host_addr)) {
        return;
    }
    memcpy(s_host_addr, addr, BD_ADDR_LEN);
    s_have_host = true;
    btstack_tlv_get_instance(&tlv, &ctx);
    if (tlv != NULL) {
        tlv->store_tag(ctx, WM_TLV_TAG_HOST, (const uint8_t *)s_host_addr, BD_ADDR_LEN);
    }
    ESP_LOGI(TAG, "Remembering console %s", bd_addr_to_str(s_host_addr));
}

static void forget_host(void)
{
    const btstack_tlv_t *tlv = NULL;
    void *ctx = NULL;
    if (s_have_host) {
        gap_drop_link_key_for_bd_addr(s_host_addr);
    }
    s_have_host = false;
    memset(s_host_addr, 0, sizeof(s_host_addr));
    btstack_tlv_get_instance(&tlv, &ctx);
    if (tlv != NULL) {
        tlv->delete_tag(ctx, WM_TLV_TAG_HOST);
    }
    ESP_LOGI(TAG, "Forgot paired console");
}

/* Is this remote (address + class of device) the console rather than a gamepad? */
static bool is_console(const bd_addr_t addr, uint32_t cod, bool cod_known)
{
    if (s_fallback == NULL) {
        return true; /* Standalone: every host that connects is a console. */
    }
    if (s_link_claimed && addr_eq(addr, s_link_addr)) {
        return true;
    }
    if (s_have_host && addr_eq(addr, s_host_addr)) {
        return true;
    }
    if (!cod_known) {
        return false;
    }
    /* The Wii is Audio/Video "gaming/toy" (0x000448). */
    if ((cod & 0xffffffu) == WM_COD_WII) {
        return true;
    }
    /* Other non-peripheral hosts (Dolphin on a PC, a Wii U) only while the
     * sync window is open: a console never pages a remote it has not just
     * found. Outside the window everything unknown belongs to the host stack,
     * whatever class of device a gamepad claims to be. */
    return s_sync_active && (cod & WM_COD_MAJOR_MASK) != WM_COD_MAJOR_PERIPHERAL;
}

static bool is_console_link_addr(const bd_addr_t addr)
{
    return s_link_claimed && addr_eq(addr, s_link_addr);
}

static void claim_link(const bd_addr_t addr, bool outgoing)
{
    s_link_claimed = true;
    memcpy(s_link_addr, addr, BD_ADDR_LEN);
    s_link_outgoing = outgoing;
    s_link_master = outgoing; /* the pager starts as master */
    s_link_handle = HCI_CON_HANDLE_INVALID;
    s_last_disconnect_reason = 0; /* 0 = closed without an HCI reason */
}

static void report_timer_cb(btstack_timer_source_t *ts)
{
    flush_reports();
    btstack_run_loop_set_timer(ts, WM_REPORT_TICK_MS);
    btstack_run_loop_add_timer(ts);
}

static void report_timer_start(void)
{
    if (s_report_timer_active) {
        return;
    }
    s_report_timer_active = true;
    btstack_run_loop_set_timer_handler(&s_report_timer, report_timer_cb);
    btstack_run_loop_set_timer(&s_report_timer, WM_REPORT_TICK_MS);
    btstack_run_loop_add_timer(&s_report_timer);
}

static void report_timer_stop(void)
{
    if (!s_report_timer_active) {
        return;
    }
    s_report_timer_active = false;
    btstack_run_loop_remove_timer(&s_report_timer);
}

/* Undo the radio settings changed for our page to the console, so the host
 * stack pages gamepads the way it always does. */
static void restore_page_settings(void)
{
    if (s_restore_role_switch) {
        s_restore_role_switch = false;
        gap_set_allow_role_switch(true);
    }
    if (s_restore_page_timeout) {
        s_restore_page_timeout = false;
        gap_set_page_timeout(WM_PAGE_TIMEOUT_DEFAULT);
    }
}

/* Open a channel to the console without authenticating first. A real remote
 * never starts authentication: the console does (the system menu and libogc
 * both request it as soon as the ACL is up). BTstack's outgoing channels use
 * the global GAP level (2 by default, and Bluepad32 sets 2), which would make
 * us authenticate and encrypt before HID control opens: slower, and it
 * collides with the console's own authentication in homebrew (libogc). */
static uint8_t create_console_channel(uint16_t psm, uint16_t *cid)
{
    gap_security_level_t saved = gap_get_security_level();
    uint8_t status;
    gap_set_security_level(LEVEL_0);
    status = l2cap_create_channel(wm_l2cap_handler, s_link_addr, psm, WM_L2CAP_MTU, cid);
    gap_set_security_level(saved);
    return status;
}

static void release_link(void)
{
    bool was_up = s_link_up;
    s_link_claimed = false;
    s_link_outgoing = false;
    s_link_up = false;
    s_link_handle = HCI_CON_HANDLE_INVALID;
    s_cid_control = 0;
    s_cid_interrupt = 0;
    s_control_open = false;
    s_interrupt_open = false;
    s_pending_handshake = 0xff;
    s_have_leds = false;
    s_have_rumble = false;
    report_timer_stop();
    btstack_run_loop_remove_timer(&s_connect_watchdog);
    restore_page_settings();
    if (was_up) {
        ESP_LOGI(TAG, "Console link down");
    }
    wiimote_set_connected(false);
}

/* --------------------------------------------------------- HCI commands -- */

static void pump_pending_cmds(void)
{
    if ((s_pending_cmds == 0 && s_policy_count == 0 && s_qos_count == 0) || hci_get_state() != HCI_STATE_WORKING ||
        !hci_can_send_command_packet_now()) {
        return;
    }
    if (s_pending_cmds & WM_CMD_POWER_OFF_DISCONNECT) {
        s_pending_cmds &= ~WM_CMD_POWER_OFF_DISCONNECT;
        /* 0x15: "remote device terminated connection due to power off", what
         * a remote that is switched off sends. */
        hci_send_cmd(&hci_disconnect, s_poweroff_handle, ERROR_CODE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF);
        return;
    }
    if (s_policy_count > 0) {
        s_policy_count--;
        hci_send_cmd(&hci_write_link_policy_settings, s_policy_q[s_policy_count].handle,
                     s_policy_q[s_policy_count].policy);
        return;
    }
    if (s_qos_count > 0) {
        s_qos_count--;
        s_qos_inflight_handle = s_qos_q[s_qos_count].handle;
        s_qos_inflight_type = s_qos_q[s_qos_count].service_type;
        /* flags, service type, token rate (B/s), peak bandwidth (0 = not
         * specified), latency (us), delay variation (don't care). */
        hci_send_cmd(&hci_qos_setup, s_qos_inflight_handle, 0, s_qos_inflight_type, 20000, 0,
                     (uint32_t)WIIMOTE_QOS_LATENCY_US, 0xffffffffu);
        return;
    }
    if (s_pending_cmds & WM_CMD_CANCEL_PAGE) {
        s_pending_cmds &= ~WM_CMD_CANCEL_PAGE;
        hci_send_cmd(&hci_create_connection_cancel, s_cancel_addr);
        return;
    }
    if (s_pending_cmds & WM_CMD_INQUIRY_SCAN_INTERLACED) {
        s_pending_cmds &= ~WM_CMD_INQUIRY_SCAN_INTERLACED;
        hci_send_cmd(&hci_write_inquiry_scan_type, 1);
        return;
    }
    if (s_pending_cmds & WM_CMD_WRITE_IAC_LIAC_GIAC) {
        s_pending_cmds &= ~WM_CMD_WRITE_IAC_LIAC_GIAC;
        hci_send_cmd(&hci_write_current_iac_lap_two_iacs, 2, GAP_IAC_LIMITED_INQUIRY, GAP_IAC_GENERAL_INQUIRY);
        return;
    }
}

static void queue_link_policy(hci_con_handle_t handle, uint16_t policy)
{
    if (s_policy_count < (int)(sizeof(s_policy_q) / sizeof(s_policy_q[0]))) {
        s_policy_q[s_policy_count].handle = handle;
        s_policy_q[s_policy_count].policy = policy;
        s_policy_count++;
    }
    pump_pending_cmds();
}

static void queue_qos(hci_con_handle_t handle, uint8_t service_type)
{
    if (s_qos_count < (int)(sizeof(s_qos_q) / sizeof(s_qos_q[0]))) {
        s_qos_q[s_qos_count].handle = handle;
        s_qos_q[s_qos_count].service_type = service_type;
        s_qos_count++;
    }
    pump_pending_cmds();
}

/* The controller refused the request: retry once as best effort, which
 * controllers that cannot guarantee anything usually still accept. */
static void qos_failed(hci_con_handle_t handle, uint8_t status)
{
    if (s_qos_inflight_type == WM_QOS_GUARANTEED) {
        ESP_LOGI(TAG, "QoS (guaranteed) refused on 0x%04x (status 0x%02x), trying best effort", handle, status);
        queue_qos(handle, WM_QOS_BEST_EFFORT);
    } else {
        ESP_LOGW(TAG, "QoS refused on 0x%04x (status 0x%02x); the controller keeps its poll interval", handle,
                 status);
    }
}

/* ------------------------------------------------------------------ SDP -- */

static void build_hid_record(uint8_t *service, uint32_t handle)
{
    uint8_t *attr;
    uint8_t *seq;
    uint8_t *inner;

    de_create_sequence(service);

    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_SERVICE_RECORD_HANDLE);
    de_add_number(service, DE_UINT, DE_SIZE_32, handle);

    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_SERVICE_CLASS_ID_LIST);
    attr = de_push_sequence(service);
    de_add_number(attr, DE_UUID, DE_SIZE_16, BLUETOOTH_SERVICE_CLASS_HUMAN_INTERFACE_DEVICE_SERVICE);
    de_pop_sequence(service, attr);

    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_PROTOCOL_DESCRIPTOR_LIST);
    attr = de_push_sequence(service);
    seq = de_push_sequence(attr);
    de_add_number(seq, DE_UUID, DE_SIZE_16, BLUETOOTH_PROTOCOL_L2CAP);
    de_add_number(seq, DE_UINT, DE_SIZE_16, WM_PSM_CONTROL);
    de_pop_sequence(attr, seq);
    seq = de_push_sequence(attr);
    de_add_number(seq, DE_UUID, DE_SIZE_16, BLUETOOTH_PROTOCOL_HIDP);
    de_pop_sequence(attr, seq);
    de_pop_sequence(service, attr);

    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_BROWSE_GROUP_LIST);
    attr = de_push_sequence(service);
    de_add_number(attr, DE_UUID, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_PUBLIC_BROWSE_ROOT);
    de_pop_sequence(service, attr);

    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_LANGUAGE_BASE_ATTRIBUTE_ID_LIST);
    attr = de_push_sequence(service);
    de_add_number(attr, DE_UINT, DE_SIZE_16, 0x656e); /* "en" */
    de_add_number(attr, DE_UINT, DE_SIZE_16, 0x006a); /* UTF-8 */
    de_add_number(attr, DE_UINT, DE_SIZE_16, 0x0100); /* base */
    de_pop_sequence(service, attr);

    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_BLUETOOTH_PROFILE_DESCRIPTOR_LIST);
    attr = de_push_sequence(service);
    seq = de_push_sequence(attr);
    de_add_number(seq, DE_UUID, DE_SIZE_16, BLUETOOTH_SERVICE_CLASS_HUMAN_INTERFACE_DEVICE_SERVICE);
    de_add_number(seq, DE_UINT, DE_SIZE_16, 0x0100);
    de_pop_sequence(attr, seq);
    de_pop_sequence(service, attr);

    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_ADDITIONAL_PROTOCOL_DESCRIPTOR_LISTS);
    attr = de_push_sequence(service);
    seq = de_push_sequence(attr);
    inner = de_push_sequence(seq);
    de_add_number(inner, DE_UUID, DE_SIZE_16, BLUETOOTH_PROTOCOL_L2CAP);
    de_add_number(inner, DE_UINT, DE_SIZE_16, WM_PSM_INTERRUPT);
    de_pop_sequence(seq, inner);
    inner = de_push_sequence(seq);
    de_add_number(inner, DE_UUID, DE_SIZE_16, BLUETOOTH_PROTOCOL_HIDP);
    de_pop_sequence(seq, inner);
    de_pop_sequence(attr, seq);
    de_pop_sequence(service, attr);

    de_add_number(service, DE_UINT, DE_SIZE_16, 0x0100); /* ServiceName */
    de_add_data(service, DE_STRING, (uint16_t)strlen(k_local_name), (uint8_t *)k_local_name);
    de_add_number(service, DE_UINT, DE_SIZE_16, 0x0101); /* ServiceDescription */
    de_add_data(service, DE_STRING, (uint16_t)strlen(k_local_name), (uint8_t *)k_local_name);
    de_add_number(service, DE_UINT, DE_SIZE_16, 0x0102); /* ProviderName */
    de_add_data(service, DE_STRING, 8, (uint8_t *)"Nintendo");

    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_DEVICE_RELEASE_NUMBER);
    de_add_number(service, DE_UINT, DE_SIZE_16, 0x0100);
    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_PARSER_VERSION);
    de_add_number(service, DE_UINT, DE_SIZE_16, 0x0111);
    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_DEVICE_SUBCLASS);
    de_add_number(service, DE_UINT, DE_SIZE_8, 0x04);
    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_COUNTRY_CODE);
    de_add_number(service, DE_UINT, DE_SIZE_8, 0x33);
    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_VIRTUAL_CABLE);
    de_add_number(service, DE_BOOL, DE_SIZE_8, 0);
    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_RECONNECT_INITIATE);
    de_add_number(service, DE_BOOL, DE_SIZE_8, 1);

    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_DESCRIPTOR_LIST);
    attr = de_push_sequence(service);
    seq = de_push_sequence(attr);
    de_add_number(seq, DE_UINT, DE_SIZE_8, 0x22); /* Report descriptor */
    de_add_data(seq, DE_STRING, (uint16_t)sizeof(k_rvl_hid_descriptor), (uint8_t *)k_rvl_hid_descriptor);
    de_pop_sequence(attr, seq);
    de_pop_sequence(service, attr);

    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HIDLANGID_BASE_LIST);
    attr = de_push_sequence(service);
    seq = de_push_sequence(attr);
    de_add_number(seq, DE_UINT, DE_SIZE_16, 0x0409);
    de_add_number(seq, DE_UINT, DE_SIZE_16, 0x0100);
    de_pop_sequence(attr, seq);
    de_pop_sequence(service, attr);

    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HIDSDP_DISABLE);
    de_add_number(service, DE_BOOL, DE_SIZE_8, 0);
    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_BATTERY_POWER);
    de_add_number(service, DE_BOOL, DE_SIZE_8, 1);
    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_REMOTE_WAKE);
    de_add_number(service, DE_BOOL, DE_SIZE_8, 1);
    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_PROFILE_VERSION);
    de_add_number(service, DE_UINT, DE_SIZE_16, 0x0100);
    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_SUPERVISION_TIMEOUT);
    de_add_number(service, DE_UINT, DE_SIZE_16, 0x0c80);
    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_NORMALLY_CONNECTABLE);
    de_add_number(service, DE_BOOL, DE_SIZE_8, 0);
    de_add_number(service, DE_UINT, DE_SIZE_16, BLUETOOTH_ATTRIBUTE_HID_BOOT_DEVICE);
    de_add_number(service, DE_BOOL, DE_SIZE_8, 0);
}

static void register_sdp_records(void)
{
    if (s_sdp_registered) {
        return;
    }
    /* Bluepad32 registers its own Device ID record at 0x10003. A remote has
     * exactly one PnP record, Nintendo's, so drop the host's. */
    if (sdp_get_record_for_handle(0x10003) != NULL) {
        sdp_unregister_service(0x10003);
    }

    s_hid_record_handle = sdp_create_service_record_handle();
    memset(s_hid_record, 0, sizeof(s_hid_record));
    build_hid_record(s_hid_record, s_hid_record_handle);
    if (de_get_len(s_hid_record) > sizeof(s_hid_record)) {
        ESP_LOGE(TAG, "HID SDP record overflow (%u bytes)", (unsigned)de_get_len(s_hid_record));
        return;
    }
    sdp_register_service(s_hid_record);

    s_pnp_record_handle = sdp_create_service_record_handle();
    memset(s_pnp_record, 0, sizeof(s_pnp_record));
    device_id_create_sdp_record(s_pnp_record, s_pnp_record_handle, DEVICE_ID_VENDOR_ID_SOURCE_USB, 0x057e, 0x0306,
                                0x0600);
    sdp_register_service(s_pnp_record);

    s_sdp_registered = true;
    ESP_LOGI(TAG, "SDP: HID record 0x%08lx (%u bytes), PnP record 0x%08lx", (unsigned long)s_hid_record_handle,
             (unsigned)de_get_len(s_hid_record), (unsigned long)s_pnp_record_handle);
}

/* ------------------------------------------------------- GAP / identity -- */

static void apply_identity(void)
{
    gap_set_local_name(k_local_name);
    gap_set_class_of_device(WM_COD);
    gap_set_allow_role_switch(true);
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE);
    if (s_fallback == NULL) {
        /* 5.12 s instead of 15.36 s: a page to a console that is still
         * booting a game fails quickly and can be retried. Shared with a host
         * stack, this is set only around our own page (start_outgoing). */
        gap_set_page_timeout(WM_PAGE_TIMEOUT_CONSOLE);
    }
    gap_connectable_control(1);
}

static void scan_activity_normal(void)
{
    gap_set_page_scan_activity(0x0800, 0x0012);
    gap_inquiry_set_scan_activity(0x0800, 0x0012);
}

static void scan_activity_fast(void)
{
    /* About 11% duty while syncing, so the Wii's inquiry and page land fast. */
    gap_set_page_scan_activity(0x0200, 0x0024);
    gap_inquiry_set_scan_activity(0x0200, 0x0024);
}

static void on_hci_working(void)
{
    apply_identity();
    register_sdp_records();
    s_pending_cmds |= WM_CMD_INQUIRY_SCAN_INTERLACED | WM_CMD_WRITE_IAC_LIAC_GIAC;
    pump_pending_cmds();
    ESP_LOGI(TAG, "Wiimote BTstack services up (%s)", s_fallback ? "shared with host stack" : "standalone");
}

/* Called before BTstack accepts an incoming ACL: choose the role we ask for. */
static int connection_filter(bd_addr_t addr, hci_link_type_t link_type)
{
    if (link_type == HCI_LINK_TYPE_ACL) {
        link_key_t key;
        link_key_type_t type;
        bool known_console = s_have_host && addr_eq(addr, s_host_addr);
        bool has_key = gap_get_link_key_for_bd_addr(addr, key, &type);
        /* A console pages a remote and stays master, exactly like with a real
         * Wiimote. Everything else gets "become master", as Bluepad32 asks
         * for, so the ESP32 is master of its own piconet. Unknown devices
         * during the sync window may be the console: no role switch then. */
        bool slave = (s_fallback == NULL) || known_console || (!has_key && s_sync_active);
        hci_set_master_slave_policy(slave ? HCI_ROLE_SLAVE : HCI_ROLE_MASTER);
    }
    return 1;
}

/* ---------------------------------------------------- report transport -- */

static void note_outputs(uint8_t leds, bool rumble)
{
    if (!s_have_leds || leds != s_last_leds) {
        s_last_leds = leds;
        s_have_leds = true;
        wiimote_note_leds(leds);
    }
    if (!s_have_rumble || rumble != s_last_rumble) {
        s_last_rumble = rumble;
        s_have_rumble = true;
        wiimote_note_rumble(rumble);
    }
}

static void flush_reports(void)
{
    uint8_t pkt[32];
    uint8_t leds = 0;
    bool rumble = false;
    bool have_outputs = false;
    uint32_t now;
    int budget = 6;

    if (!wiimote_ready()) {
        return;
    }

    if (s_pending_handshake != 0xff && s_cid_control != 0) {
        if (l2cap_can_send_packet_now(s_cid_control)) {
            uint8_t hs = s_pending_handshake;
            s_pending_handshake = 0xff;
            l2cap_send(s_cid_control, &hs, 1);
        } else {
            l2cap_request_can_send_now_event(s_cid_control);
        }
    }

    if (!s_link_up || s_cid_interrupt == 0) {
        return;
    }

    now = (uint32_t)(esp_timer_get_time() / 1000);
    while (budget-- > 0) {
        int n;
        bool more_queued;
        wiimote_proto_t *proto;

        if (!l2cap_can_send_packet_now(s_cid_interrupt)) {
            wiimote_lock();
            proto = wiimote_state();
            more_queued = proto->q_count > 0 || proto->read.active;
            wiimote_unlock();
            /* Only protocol replies (acks, status, memory reads) ask to be
             * woken up. Periodic input waits for the next timer tick, which
             * keeps the console link from monopolising the radio. */
            if (more_queued) {
                l2cap_request_can_send_now_event(s_cid_interrupt);
            }
            break;
        }

        wiimote_lock();
        proto = wiimote_state();
        n = wiimote_proto_poll(proto, pkt, sizeof(pkt), now);
        leds = proto->leds;
        rumble = proto->rumble;
        have_outputs = true;
        wiimote_unlock();

        if (n <= 0) {
            break;
        }
        l2cap_send(s_cid_interrupt, pkt, (uint16_t)n);
    }

    if (have_outputs) {
        note_outputs(leds, rumble);
    }
}

static void deliver_output(const uint8_t *report, size_t len)
{
    uint8_t level;
    if (len < 1 || report[0] < 0x10 || report[0] > 0x1a) {
        return;
    }
    /* One line per configuration request; the frequent ones stay quiet. */
    switch (report[0]) {
    case 0x12:
        if (len >= 3) {
            ESP_LOGI(TAG, "Wii: report mode 0x%02x%s", report[2], (report[1] & 0x04) ? " (continuous)" : "");
        }
        break;
    case 0x16:
        if (len >= 6) {
            ESP_LOGI(TAG, "Wii: write %s 0x%02x%02x%02x len %u first 0x%02x", (report[1] & 0x0c) ? "reg" : "eeprom",
                     report[2], report[3], report[4], report[5], len >= 7 ? report[6] : 0);
        }
        break;
    case 0x17:
        if (len >= 7) {
            ESP_LOGI(TAG, "Wii: read %s 0x%02x%02x%02x len %u", (report[1] & 0x0c) ? "reg" : "eeprom", report[2],
                     report[3], report[4], (unsigned)((report[5] << 8) | report[6]));
        }
        break;
    case 0x13:
    case 0x15:
    case 0x1a:
        ESP_LOGI(TAG, "Wii: report 0x%02x", report[0]);
        break;
    default:
        break;
    }
    level = wiimote_current_battery();
    wiimote_lock();
    wiimote_proto_set_battery(wiimote_state(), level);
    wiimote_proto_handle_output(wiimote_state(), report, len);
    wiimote_unlock();
}

static void send_handshake(uint8_t code)
{
    s_pending_handshake = code;
}

static void handle_hid(uint16_t cid, const uint8_t *data, size_t len)
{
    uint8_t type;
    uint8_t param;

    if (len < 1 || !wiimote_ready()) {
        return;
    }
    type = data[0] >> 4;
    param = data[0] & 0x0f;

    if (cid == s_cid_interrupt) {
        /* DATA | OUTPUT (0xA2) is how the Wii talks to a remote. */
        if (data[0] == 0xa2) {
            deliver_output(data + 1, len - 1);
        }
        flush_reports();
        return;
    }

    /* Control channel. */
    switch (type) {
    case 0x1: /* HID_CONTROL */
        if (param == 0x05) {
            ESP_LOGI(TAG, "Console sent VIRTUAL_CABLE_UNPLUG");
            if (s_link_handle != HCI_CON_HANDLE_INVALID) {
                gap_disconnect(s_link_handle);
            }
        }
        return;
    case 0x5: /* SET_REPORT */
        if ((param & 0x03) == 0x02) {
            deliver_output(data + 1, len - 1);
            send_handshake(0x00);
        } else {
            send_handshake(0x03);
        }
        break;
    case 0xa: /* DATA on control */
        if (param == 0x02) {
            deliver_output(data + 1, len - 1);
        }
        break;
    default: /* GET_REPORT, GET/SET_PROTOCOL, ... are not used by the Wii */
        send_handshake(0x03);
        break;
    }
    flush_reports();
}

/* ---------------------------------------------------------- link setup -- */

static void on_channels_up(void)
{
    if (s_link_up) {
        return;
    }
    s_link_up = true;
    if (s_sync_active) {
        s_sync_active = false;
        gap_discoverable_control(0);
        scan_activity_normal();
    }
    store_host(s_link_addr);
    ESP_LOGI(TAG, "Console %s connected (control cid 0x%04x, interrupt cid 0x%04x, %s)", bd_addr_to_str(s_link_addr),
             s_cid_control, s_cid_interrupt, s_link_outgoing ? "we paged it" : "it paged us");
    report_timer_start();
    btstack_run_loop_remove_timer(&s_connect_watchdog);
    /* No role switch request here: a Wii refuses it (0x0c) only after ~17 s,
     * and the link carries no data while the request is pending. The Wii is
     * master of its link, exactly as with a real remote. */
    wiimote_set_connected(true);
}

static void connect_watchdog_cb(btstack_timer_source_t *ts)
{
    (void)ts;
    if (!s_link_claimed || s_link_up || !s_link_outgoing) {
        return;
    }
    ESP_LOGW(TAG, "Connecting to the console timed out; giving up for now");
    if (s_link_handle != HCI_CON_HANDLE_INVALID) {
        gap_disconnect(s_link_handle);
        return; /* DISCONNECTION_COMPLETE releases the link. */
    }
    memcpy(s_cancel_addr, s_link_addr, BD_ADDR_LEN);
    s_pending_cmds |= WM_CMD_CANCEL_PAGE;
    pump_pending_cmds();
    release_link();
}

static void start_outgoing(void)
{
    uint8_t status;
    if (!s_have_host) {
        ESP_LOGW(TAG, "Connect requested, but no console has been synced yet");
        return;
    }
    if (s_link_claimed) {
        return; /* Already connecting or connected. */
    }
    claim_link(s_host_addr, true);
    if (s_fallback != NULL) {
        /* Sharing the radio with a gamepad: a Wii takes over as master while
         * the connection is set up, if allowed. As slave on the Wii link and
         * master on the gamepad link the ESP32 controller stalls both links
         * (gamepad supervision timeout 0x08). So page with role switch off:
         * the ESP32 stays master of both, one piconet. */
        gap_set_allow_role_switch(false);
        s_restore_role_switch = true;
        gap_set_page_timeout(WM_PAGE_TIMEOUT_CONSOLE);
        s_restore_page_timeout = true;
    }
    status = create_console_channel(WM_PSM_CONTROL, &s_cid_control);
    if (status != ERROR_CODE_SUCCESS) {
        ESP_LOGW(TAG, "Paging console failed to start (0x%02x)", status);
        s_cid_control = 0;
        s_link_claimed = false;
        restore_page_settings();
        return;
    }
    ESP_LOGI(TAG, "Paging console %s", bd_addr_to_str(s_host_addr));
    /* The controller can lose a page when another one (a gamepad) starts at
     * the same time, and then never reports it. Do not wait forever. */
    btstack_run_loop_set_timer_handler(&s_connect_watchdog, connect_watchdog_cb);
    btstack_run_loop_set_timer(&s_connect_watchdog, WM_CONNECT_TIMEOUT_MS);
    btstack_run_loop_add_timer(&s_connect_watchdog);
}

static void drop_link(void)
{
    if (!s_link_claimed) {
        return;
    }
    if (s_link_handle != HCI_CON_HANDLE_INVALID) {
        /* Hang up like a remote that is switched off. */
        s_poweroff_handle = s_link_handle;
        s_pending_cmds |= WM_CMD_POWER_OFF_DISCONNECT;
        pump_pending_cmds();
        return; /* DISCONNECTION_COMPLETE releases the link. */
    }
    if (s_cid_interrupt) {
        l2cap_disconnect(s_cid_interrupt);
    }
    if (s_cid_control) {
        l2cap_disconnect(s_cid_control);
    }
    release_link();
}

/* ------------------------------------------------------- L2CAP handler -- */

static void forward(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (s_fallback != NULL) {
        s_fallback(packet_type, channel, packet, size);
    }
}

/* Hold an incoming host-stack channel until its link is as secure as the host
 * stack requires (Bluepad32: level 2). Stock Bluepad32 registers PSM 0x11/0x13
 * at that level, so L2CAP authenticates and encrypts before the channel is
 * offered; pads in Switch mode (Switch Pro, 8BitDo) only connect that way.
 * Returns false when the link is already secure enough (forward now). */
static bool hold_until_secure(uint16_t channel, hci_con_handle_t handle, uint8_t *packet, uint16_t size)
{
    gap_security_level_t need = gap_get_security_level();
    int i;
    if (need == LEVEL_0 || gap_security_level(handle) >= need || size > sizeof(s_held[0].packet)) {
        return false;
    }
    for (i = 0; i < WM_HELD_MAX; i++) {
        if (s_held[i].cid == 0) {
            break;
        }
    }
    if (i == WM_HELD_MAX) {
        return false;
    }
    s_held[i].cid = channel;
    s_held[i].handle = handle;
    s_held[i].size = size;
    memcpy(s_held[i].packet, packet, size);
    /* May report GAP_EVENT_SECURITY_LEVEL right away (release_held). */
    gap_request_security_level(handle, need);
    return true;
}

/* GAP_EVENT_SECURITY_LEVEL, or level < 0 when the link went down. Offer the
 * held channels to the host stack, or refuse them as L2CAP would have. */
static void release_held(hci_con_handle_t handle, int level)
{
    gap_security_level_t need = gap_get_security_level();
    for (int i = 0; i < WM_HELD_MAX; i++) {
        if (s_held[i].cid == 0 || s_held[i].handle != handle) {
            continue;
        }
        uint16_t cid = s_held[i].cid;
        s_held[i].cid = 0;
        if (level < 0) {
            continue; /* ACL gone; L2CAP already dropped the channel. */
        }
        if (level >= (int)need) {
            forward(HCI_EVENT_PACKET, cid, s_held[i].packet, s_held[i].size);
        } else {
            ESP_LOGW(TAG, "Link 0x%04x stayed at security level %d (need %d); refusing its channel", handle, level,
                     (int)need);
            l2cap_decline_connection(cid);
        }
    }
}

static bool is_our_cid(uint16_t cid)
{
    return cid != 0 && (cid == s_cid_control || cid == s_cid_interrupt);
}

static void wm_l2cap_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (packet_type == L2CAP_DATA_PACKET) {
        if (is_our_cid(channel)) {
            handle_hid(channel, packet, size);
        } else {
            forward(packet_type, channel, packet, size);
        }
        return;
    }
    if (packet_type != HCI_EVENT_PACKET) {
        forward(packet_type, channel, packet, size);
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
    case L2CAP_EVENT_INCOMING_CONNECTION: {
        bd_addr_t addr;
        uint16_t psm = l2cap_event_incoming_connection_get_psm(packet);
        uint16_t cid = l2cap_event_incoming_connection_get_local_cid(packet);
        hci_con_handle_t handle = l2cap_event_incoming_connection_get_handle(packet);
        l2cap_event_incoming_connection_get_address(packet, addr);
        if (!is_console_link_addr(addr) && !is_console(addr, 0, false)) {
            if (s_fallback == NULL) {
                l2cap_decline_connection(cid);
            } else if (!hold_until_secure(channel, handle, packet, size)) {
                forward(packet_type, channel, packet, size);
            }
            return;
        }
        if (!s_link_claimed) {
            claim_link(addr, false);
        }
        s_link_handle = handle;
        if (psm == WM_PSM_CONTROL && s_cid_control == 0) {
            s_cid_control = cid;
            l2cap_accept_connection(cid);
        } else if (psm == WM_PSM_INTERRUPT && s_cid_interrupt == 0) {
            s_cid_interrupt = cid;
            l2cap_accept_connection(cid);
        } else {
            ESP_LOGW(TAG, "Declining duplicate/unknown PSM 0x%04x from console", psm);
            l2cap_decline_connection(cid);
        }
        return;
    }
    case L2CAP_EVENT_CHANNEL_OPENED: {
        uint16_t cid = l2cap_event_channel_opened_get_local_cid(packet);
        uint8_t status = l2cap_event_channel_opened_get_status(packet);
        uint16_t psm = l2cap_event_channel_opened_get_psm(packet);
        if (!is_our_cid(cid)) {
            forward(packet_type, channel, packet, size);
            return;
        }
        if (status != ERROR_CODE_SUCCESS) {
            ESP_LOGW(TAG, "Console channel PSM 0x%04x failed (status 0x%02x)", psm, status);
            if (cid == s_cid_control) {
                s_cid_control = 0;
            } else {
                s_cid_interrupt = 0;
            }
            /* Outgoing: the page or the channel failed. Incoming: the
             * console gives up and drops the ACL itself. */
            if (s_link_outgoing) {
                drop_link();
            }
            return;
        }
        s_link_handle = l2cap_event_channel_opened_get_handle(packet);
        if (cid == s_cid_control) {
            s_control_open = true;
        } else {
            s_interrupt_open = true;
        }
        ESP_LOGI(TAG, "Console channel PSM 0x%04x open (cid 0x%04x)", psm, cid);
        if (s_link_outgoing && cid == s_cid_control && s_cid_interrupt == 0) {
            /* A real remote opens control first, then interrupt. */
            uint8_t st = create_console_channel(WM_PSM_INTERRUPT, &s_cid_interrupt);
            if (st != ERROR_CODE_SUCCESS) {
                ESP_LOGW(TAG, "Opening interrupt channel failed (0x%02x)", st);
                s_cid_interrupt = 0;
                drop_link();
            }
        }
        if (s_control_open && s_interrupt_open) {
            on_channels_up();
        }
        return;
    }
    case L2CAP_EVENT_CHANNEL_CLOSED: {
        uint16_t cid = l2cap_event_channel_closed_get_local_cid(packet);
        if (!is_our_cid(cid)) {
            forward(packet_type, channel, packet, size);
            return;
        }
        ESP_LOGI(TAG, "Console channel cid 0x%04x closed", cid);
        if (cid == s_cid_control) {
            s_cid_control = 0;
            s_control_open = false;
        } else {
            s_cid_interrupt = 0;
            s_interrupt_open = false;
        }
        if (s_cid_control == 0 && s_cid_interrupt == 0) {
            if (s_link_handle != HCI_CON_HANDLE_INVALID) {
                /* Mirror a real remote: no HID channels, no baseband link. */
                gap_disconnect(s_link_handle);
            } else {
                release_link();
            }
        } else if (s_link_up) {
            /* Half a link is useless; tell the app now. */
            s_link_up = false;
            report_timer_stop();
            wiimote_set_connected(false);
        }
        return;
    }
    case L2CAP_EVENT_CAN_SEND_NOW: {
        uint16_t cid = l2cap_event_can_send_now_get_local_cid(packet);
        if (is_our_cid(cid)) {
            flush_reports();
        } else {
            forward(packet_type, channel, packet, size);
        }
        return;
    }
    default:
        forward(packet_type, channel, packet, size);
        return;
    }
}

/* --------------------------------------------------------- HCI handler -- */

static void wm_hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            on_hci_working();
        }
        break;

    case HCI_EVENT_COMMAND_COMPLETE: {
        uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
        if (opcode == HCI_OPCODE_HCI_WRITE_CURRENT_IAC_LAP_TWO_IACS) {
            const uint8_t *ret = hci_event_command_complete_get_return_parameters(packet);
            ESP_LOGI(TAG, "IAC LAPs LIAC+GIAC written (status 0x%02x)", ret ? ret[0] : 0xff);
        }
        break;
    }

    case HCI_EVENT_COMMAND_STATUS: {
        /* QoS Setup answers with Command Status first; an error ends it. */
        if (hci_event_command_status_get_command_opcode(packet) == HCI_OPCODE_HCI_QOS_SETUP &&
            hci_event_command_status_get_status(packet) != ERROR_CODE_SUCCESS &&
            s_qos_inflight_handle != HCI_CON_HANDLE_INVALID) {
            hci_con_handle_t h = s_qos_inflight_handle;
            s_qos_inflight_handle = HCI_CON_HANDLE_INVALID;
            qos_failed(h, hci_event_command_status_get_status(packet));
        }
        break;
    }

    case HCI_EVENT_QOS_SETUP_COMPLETE: {
        if (size < 23) {
            break;
        }
        uint8_t status = packet[2];
        hci_con_handle_t h = little_endian_read_16(packet, 3) & 0x0fff;
        s_qos_inflight_handle = HCI_CON_HANDLE_INVALID;
        if (status != ERROR_CODE_SUCCESS) {
            qos_failed(h, status);
        } else {
            /* The granted latency is the controller's poll interval. */
            ESP_LOGI(TAG, "QoS on 0x%04x (%s): granted latency %lu us (asked %u)", h,
                     (s_link_claimed && h == s_link_handle) ? "console" : "gamepad",
                     (unsigned long)little_endian_read_32(packet, 15), (unsigned)WIIMOTE_QOS_LATENCY_US);
        }
        break;
    }

    case HCI_EVENT_CONNECTION_REQUEST: {
        bd_addr_t addr;
        uint32_t cod = hci_event_connection_request_get_class_of_device(packet);
        hci_link_type_t link_type = (hci_link_type_t)hci_event_connection_request_get_link_type(packet);
        hci_event_connection_request_get_bd_addr(packet, addr);
        if (link_type != HCI_LINK_TYPE_ACL) {
            break;
        }
        if (is_console(addr, cod, true)) {
            if (s_link_claimed && !addr_eq(addr, s_link_addr)) {
                ESP_LOGW(TAG, "Second console %s while %s is active; it will not get HID channels",
                         bd_addr_to_str(addr), bd_addr_to_str(s_link_addr));
            } else {
                ESP_LOGI(TAG, "Incoming ACL from console %s (cod 0x%06lx)", bd_addr_to_str(addr), (unsigned long)cod);
                claim_link(addr, false);
            }
            /* Hide the console from the host stack (no device slot, no timeout). */
            if (s_fallback != NULL) {
                packet[0] = HCI_EVENT_NOP;
            }
        } else {
            ESP_LOGI(TAG, "Incoming ACL from %s (cod 0x%06lx): host stack", bd_addr_to_str(addr), (unsigned long)cod);
        }
        break;
    }

    case HCI_EVENT_CONNECTION_COMPLETE: {
        bd_addr_t addr;
        uint8_t status = hci_event_connection_complete_get_status(packet);
        hci_con_handle_t handle = hci_event_connection_complete_get_connection_handle(packet);
        hci_event_connection_complete_get_bd_addr(packet, addr);
        if (is_console_link_addr(addr)) {
            restore_page_settings();
            if (status == ERROR_CODE_SUCCESS) {
                s_link_handle = handle;
                ESP_LOGI(TAG, "ACL to console up (handle 0x%04x, ESP32 is %s)", handle,
                         s_link_master ? "master" : "slave");
                if (s_fallback != NULL && s_link_outgoing && s_link_master) {
                    /* Keep it that way after setup (no role switch), no sniff. */
                    queue_link_policy(handle, 0);
                    /* No QoS here: as master we send to the Wii whenever we
                     * like; the poll interval only delays its LED/rumble
                     * reports, and the radio time is better left to the
                     * gamepad link. */
                }
            } else {
                ESP_LOGW(TAG, "ACL to console failed (status 0x%02x)", status);
                /* Outgoing channels get L2CAP_EVENT_CHANNEL_OPENED with an
                 * error and are released there; an incoming one just ends. */
                if (!s_link_outgoing) {
                    release_link();
                }
            }
        }
        /* Gamepad links are left alone while the host stack sets them up
         * (name request, SDP, pairing): extra HCI traffic there makes some
         * pads give up. The application tunes them once they are ready
         * (wiimote_btstack_tune_link). */
        break;
    }

    case HCI_EVENT_PIN_CODE_REQUEST: {
        /* gap_pin_code_response_binary keeps the pointer: must be static. */
        static uint8_t s_pin[6];
        bd_addr_t addr;
        hci_event_pin_code_request_get_bd_addr(packet, addr);
        if (!is_console_link_addr(addr) && !(s_have_host && addr_eq(addr, s_host_addr))) {
            break;
        }
        for (int i = 0; i < 6; i++) {
            s_pin[i] = addr[5 - i];
        }
        ESP_LOGI(TAG, "PIN request from console %s: answering with its address reversed", bd_addr_to_str(addr));
        gap_pin_code_response_binary(addr, s_pin, sizeof(s_pin));
        packet[0] = HCI_EVENT_NOP;
        break;
    }

    case HCI_EVENT_LINK_KEY_NOTIFICATION: {
        bd_addr_t addr;
        reverse_bd_addr(&packet[2], addr);
        if (is_console_link_addr(addr)) {
            ESP_LOGI(TAG, "Bonded with console %s", bd_addr_to_str(addr));
        }
        break;
    }

    case HCI_EVENT_ROLE_CHANGE: {
        bd_addr_t addr;
        hci_event_role_change_get_bd_addr(packet, addr);
        if (is_console_link_addr(addr)) {
            uint8_t status = hci_event_role_change_get_status(packet);
            uint8_t role = hci_event_role_change_get_role(packet);
            if (status == ERROR_CODE_SUCCESS) {
                s_link_master = (role == HCI_ROLE_MASTER);
                ESP_LOGI(TAG, "Console link: ESP32 is now %s", role == HCI_ROLE_MASTER ? "master" : "slave");
            } else {
                ESP_LOGW(TAG, "Console refused role switch (0x%02x); ESP32 stays %s", status,
                         role == HCI_ROLE_MASTER ? "master" : "slave");
            }
        }
        break;
    }

    case HCI_EVENT_AUTHENTICATION_COMPLETE_EVENT: {
        hci_con_handle_t handle = hci_event_authentication_complete_get_connection_handle(packet);
        uint8_t status = hci_event_authentication_complete_get_status(packet);
        if (s_link_claimed && handle == s_link_handle && status != ERROR_CODE_SUCCESS) {
            ESP_LOGW(TAG, "Console authentication failed (0x%02x). Sync again with the red button.", status);
        }
        break;
    }

    case GAP_EVENT_SECURITY_LEVEL:
        release_held(gap_event_security_level_get_handle(packet), gap_event_security_level_get_security_level(packet));
        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE: {
        hci_con_handle_t handle = hci_event_disconnection_complete_get_connection_handle(packet);
        release_held(handle, -1);
        if (!(s_link_claimed && handle == s_link_handle)) {
            /* Not the console: say why, it tells radio trouble (0x08 timeout)
             * from the gamepad hanging up (0x13) or us closing it (0x16). */
            ESP_LOGI(TAG, "Link 0x%04x (gamepad) disconnected, reason 0x%02x", handle,
                     hci_event_disconnection_complete_get_reason(packet));
        }
        if (s_link_claimed && handle == s_link_handle) {
            s_last_disconnect_reason = hci_event_disconnection_complete_get_reason(packet);
            ESP_LOGI(TAG, "Console disconnected (reason 0x%02x)", s_last_disconnect_reason);
            release_link();
        }
        break;
    }

    default:
        break;
    }
    pump_pending_cmds();
}

/* ------------------------------------------------------------ requests -- */

static void sync_start(void)
{
    s_sync_active = true;
    gap_connectable_control(1);
    gap_discoverable_control(1);
    scan_activity_fast();
    s_pending_cmds |= WM_CMD_INQUIRY_SCAN_INTERLACED | WM_CMD_WRITE_IAC_LIAC_GIAC;
    pump_pending_cmds();
    ESP_LOGI(TAG, "Sync window open: discoverable as \"%s\". Press the red SYNC button on the console.", k_local_name);
}

static void sync_end(void)
{
    if (s_sync_active) {
        s_sync_active = false;
        gap_discoverable_control(0);
        scan_activity_normal();
        ESP_LOGI(TAG, "Sync window closed; still connectable");
    }
    wiimote_note_sync_ended();
}

static void stop_all(void)
{
    s_sync_active = false;
    gap_discoverable_control(0);
    scan_activity_normal();
    drop_link();
    if (s_fallback == NULL) {
        gap_connectable_control(0);
    }
    wiimote_note_sync_ended();
}

static void setup_services(void);

static void run_requests(void *context)
{
    uint32_t req;
    (void)context;
    taskENTER_CRITICAL(&s_req_mux);
    req = s_requests;
    s_requests = 0;
    taskEXIT_CRITICAL(&s_req_mux);

    if (req & WM_REQ_SETUP) {
        setup_services();
    }
    if (req & WM_REQ_STOP) {
        stop_all();
    }
    if (req & WM_REQ_FORGET) {
        drop_link();
        forget_host();
    }
    if (req & WM_REQ_DISCONNECT) {
        drop_link();
    }
    if (req & WM_REQ_SYNC) {
        sync_start();
    }
    if (req & WM_REQ_LISTEN) {
        sync_end();
    }
    if (req & WM_REQ_CONNECT) {
        start_outgoing();
    }
    if (req & WM_REQ_FLUSH) {
        flush_reports();
    }
}

static void post_request(uint32_t req)
{
    taskENTER_CRITICAL(&s_req_mux);
    s_requests |= req;
    taskEXIT_CRITICAL(&s_req_mux);
    s_req_registration.callback = run_requests;
    s_req_registration.context = NULL;
    btstack_run_loop_execute_on_main_thread(&s_req_registration);
}

/* --------------------------------------------------------------- setup -- */

static void setup_services(void)
{
    if (s_setup_done) {
        return;
    }
    s_setup_done = true;

    load_host();

    /* If a host stack registered the HID PSMs already, take them over; we
     * forward its traffic to the fallback handler. */
    l2cap_unregister_service(WM_PSM_CONTROL);
    l2cap_unregister_service(WM_PSM_INTERRUPT);
    /* Level 0: the console authenticates on its own terms (legacy PIN). */
    l2cap_register_service(wm_l2cap_handler, WM_PSM_CONTROL, WM_L2CAP_MTU, LEVEL_0);
    l2cap_register_service(wm_l2cap_handler, WM_PSM_INTERRUPT, WM_L2CAP_MTU, LEVEL_0);

    /* Head of the list: we must see connection and PIN requests first. */
    s_hci_registration.callback = wm_hci_handler;
    hci_add_event_handler_for_security_manager(&s_hci_registration);

    gap_register_classic_connection_filter(connection_filter);
    apply_identity();

    if (hci_get_state() == HCI_STATE_WORKING) {
        /* Attached late to a running stack: BTSTACK_EVENT_STATE has passed. */
        on_hci_working();
    }
}

#if defined(WIIMOTE_HAVE_ESP32_PORT)
static void standalone_run_loop_task(void *arg)
{
    (void)arg;
    btstack_run_loop_execute();
    vTaskDelete(NULL);
}
#endif

/* ------------------------------------------------------------ public -- */

void wiimote_btstack_set_fallback_handler(btstack_packet_handler_t handler)
{
    s_fallback = handler;
}

void wiimote_btstack_tune_link(hci_con_handle_t handle, const bd_addr_t addr)
{
    if (handle == HCI_CON_HANDLE_INVALID || gap_get_connection_type(handle) != GAP_CONNECTION_ACL ||
        (s_link_claimed && handle == s_link_handle)) {
        return;
    }
    /* Master of every link: as slave on one link and master on another the
     * ESP32 controller starves both. */
    if (gap_get_role(handle) == HCI_ROLE_SLAVE) {
        gap_request_role(addr, HCI_ROLE_MASTER);
    }
    /* Role switch allowed, sniff not (sniff adds latency to every report),
     * and a short poll interval. */
    queue_link_policy(handle, LM_LINK_POLICY_ENABLE_ROLE_SWITCH);
    queue_qos(handle, WM_QOS_GUARANTEED);
}

void wiimote_bt_set_external_stack(bool external)
{
    s_external_stack = external;
}

esp_err_t wiimote_bt_start(void)
{
    if (!s_external_stack) {
#if defined(WIIMOTE_HAVE_ESP32_PORT)
        if (hci_get_state() != HCI_STATE_OFF) {
            ESP_LOGE(TAG, "BTstack is already running; set external_stack = true");
            return ESP_ERR_INVALID_STATE;
        }
        /* btstack_init() sets up NVS-backed TLV for link keys. */
        if (btstack_init() != ERROR_CODE_SUCCESS) {
            return ESP_FAIL;
        }
        l2cap_init();
        sdp_init();
        setup_services();
        if (hci_power_control(HCI_POWER_ON) != 0) {
            return ESP_FAIL;
        }
        if (xTaskCreatePinnedToCore(standalone_run_loop_task, "btstack", 8192, NULL, 5, NULL, 0) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
        return ESP_OK;
#else
        ESP_LOGE(TAG, "Standalone mode needs BTstack's ESP32 port (btstack_port_esp32.h)");
        return ESP_ERR_NOT_SUPPORTED;
#endif
    }

    if (on_btstack_thread_or_not_running()) {
        /* Typical: called after btstack_init() and before the host stack
         * (uni_init) powers HCI on. l2cap_init / sdp_init are idempotent
         * enough to be called again by the host stack. */
        l2cap_init();
        sdp_init();
        setup_services();
    } else {
        post_request(WM_REQ_SETUP);
    }
    return ESP_OK;
}

esp_err_t wiimote_btstack_register_services(void)
{
    if (on_btstack_thread_or_not_running()) {
        setup_services();
    } else {
        post_request(WM_REQ_SETUP);
    }
    return ESP_OK;
}

esp_err_t wiimote_bt_sync(void)
{
    post_request(WM_REQ_SYNC);
    return ESP_OK;
}

void wiimote_bt_listen_only(void)
{
    post_request(WM_REQ_LISTEN);
}

esp_err_t wiimote_bt_stop(void)
{
    post_request(WM_REQ_STOP);
    return ESP_OK;
}

esp_err_t wiimote_bt_connect(void)
{
    if (!s_have_host) {
        return ESP_ERR_NOT_FOUND;
    }
    post_request(WM_REQ_CONNECT);
    return ESP_OK;
}

esp_err_t wiimote_bt_disconnect(void)
{
    post_request(WM_REQ_DISCONNECT);
    return ESP_OK;
}

esp_err_t wiimote_bt_forget_host(void)
{
    post_request(WM_REQ_FORGET);
    return ESP_OK;
}

bool wiimote_bt_has_host(void)
{
    return s_have_host;
}

void wiimote_bt_send_now(void)
{
    if (s_link_up) {
        post_request(WM_REQ_FLUSH);
    }
}

bool wiimote_bt_is_connecting(void)
{
    return (s_link_claimed && !s_link_up) || (s_requests & WM_REQ_CONNECT) != 0;
}

bool wiimote_bt_link_is_master(void)
{
    return s_link_claimed && s_link_master;
}

uint8_t wiimote_bt_last_disconnect_reason(void)
{
    return s_last_disconnect_reason;
}
