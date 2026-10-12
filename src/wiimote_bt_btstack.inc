/* SPDX-License-Identifier: MIT */
/*
 * Bluetooth Classic HID peripheral built on BlueKitchen BTstack.
 *
 * Implements Nintendo RVL-CNT-01 (Wiimote) over BTstack L2CAP:
 *   - L2CAP PSM 0x0011 (HID Control)
 *   - L2CAP PSM 0x0013 (HID Interrupt)
 *   - Service Discovery Protocol (SDP) record with the real RVL-CNT-01 HID descriptor
 *   - Device ID (PnP) SDP record (Nintendo 0x057E / 0x0306)
 *   - Device Class: 0x002504 (Peripheral, Joystick, Limited Discoverable)
 *   - Device Name: "Nintendo RVL-CNT-01"
 *   - Legacy PIN pairing with console BD_ADDR reversed (6 bytes, SSP disabled)
 *   - 100 Hz periodic core reporting via BTstack run loop timer
 *
 * Can run standalone on ESP32 or attach into an existing BTstack environment (e.g. Bluepad32).
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
#include "nvs_flash.h"

#include "btstack.h"
#include "classic/sdp_server.h"
#include "classic/sdp_util.h"
#include "classic/core.h"
#include "classic/device_id_server.h"

#if __has_include("btstack_port_esp32.h")
#include "btstack_port_esp32.h"
#endif

#if __has_include("esp_bt.h")
#include "esp_bt.h"
#endif

static const char *TAG = "wiimote_btstack";

/* Dolphin hcidump of a real RVL-CNT-01 HID descriptor list (attr 0x0206).
 * Output items are 0x91 0x00 and input items are 0x81 0x00. */
static const uint8_t s_rvl_hid_report[] = {
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

static uint8_t s_hid_sdp_record[512];
static uint32_t s_hid_handle = 0;
static uint8_t s_device_id_sdp_record[128];
static uint32_t s_device_id_handle = 0;

static uint16_t s_cid_control = 0;
static uint16_t s_cid_interrupt = 0;
static uint16_t s_con_handle = HCI_CON_HANDLE_INVALID;
static bd_addr_t s_bda;
static bool s_have_host_bda = false;
static bool s_sync_window = false;
static bool s_hid_ready = false;
static bool s_btstack_started = false;

static bool s_gamepad_connecting = false;
static bool s_gamepad_connected = false;
static bd_addr_t s_gamepad_addr;
static uint16_t s_gamepad_handle = HCI_CON_HANDLE_INVALID;
static int64_t s_outbound_busy_until_us = 0;
static bool s_name_req_pending = false;
static int64_t s_name_req_cancel_at_us = 0;

/* Optional Bluepad32 hooks when sharing BTstack with Bluepad32 */
extern void uni_bt_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
    __attribute__((weak));
extern void uni_bt_stop_scanning_unsafe(void) __attribute__((weak));
extern void uni_bt_start_scanning_and_autoconnect_unsafe(void) __attribute__((weak));

static void pause_gamepad_scanning(void)
{
    if (uni_bt_stop_scanning_unsafe != NULL) {
        uni_bt_stop_scanning_unsafe();
    }
    gap_inquiry_stop();
    gap_stop_scan();
}

static void resume_gamepad_scanning(void)
{
    if (!s_sync_window && !s_gamepad_connected && uni_bt_start_scanning_and_autoconnect_unsafe != NULL) {
        uni_bt_start_scanning_and_autoconnect_unsafe();
    }
}

static bool is_wii_host_addr(const bd_addr_t addr)
{
    if (uni_bt_packet_handler == NULL) {
        return true;
    }
    return s_have_host_bda && (bd_addr_cmp(addr, s_bda) == 0);
}

static btstack_packet_callback_registration_t s_hci_event_callback_registration;
static btstack_timer_source_t s_report_timer;
static btstack_context_callback_registration_t s_init_registration;
static btstack_context_callback_registration_t s_sync_registration;
static btstack_context_callback_registration_t s_listen_registration;
static btstack_context_callback_registration_t s_stop_registration;

static uint8_t s_last_leds = 0;
static bool s_have_leds = false;
static uint8_t s_last_rumble = 0;
static bool s_have_rumble = false;

static void flush_reports(void);

typedef struct {
    uint8_t *p;
    uint8_t *end;
} wm_w;

static void ww_u8(wm_w *w, uint8_t v)
{
    if (w->p < w->end) {
        *w->p++ = v;
    }
}

static void ww_u16(wm_w *w, uint16_t v)
{
    ww_u8(w, (uint8_t)(v >> 8));
    ww_u8(w, (uint8_t)v);
}

static void ww_bytes(wm_w *w, const uint8_t *b, int n)
{
    for (int i = 0; i < n; i++) {
        ww_u8(w, b[i]);
    }
}

static uint8_t *ww_seq_open(wm_w *w, int wide)
{
    uint8_t *lenp;
    ww_u8(w, (uint8_t)(wide ? 0x36 : 0x35));
    lenp = w->p;
    ww_u8(w, 0);
    if (wide) {
        ww_u8(w, 0);
    }
    return lenp;
}

static void ww_seq_close(uint8_t *lenp, uint8_t *end, int wide)
{
    int n = (int)(end - (lenp + (wide ? 2 : 1)));
    if (n < 0) {
        n = 0;
    }
    if (wide) {
        lenp[0] = (uint8_t)(n >> 8);
        lenp[1] = (uint8_t)n;
    } else if (lenp < end) {
        lenp[0] = (uint8_t)n;
    }
}

static void emit_id(wm_w *w, uint16_t id)
{
    ww_u8(w, 0x09);
    ww_u16(w, id);
}

static void emit_u8_attr(wm_w *w, uint16_t id, uint8_t v)
{
    emit_id(w, id);
    ww_u8(w, 0x08);
    ww_u8(w, v);
}

static void emit_u16_attr(wm_w *w, uint16_t id, uint16_t v)
{
    emit_id(w, id);
    ww_u8(w, 0x09);
    ww_u16(w, v);
}

static void emit_bool_attr(wm_w *w, uint16_t id, uint8_t v)
{
    emit_id(w, id);
    ww_u8(w, 0x28);
    ww_u8(w, v);
}

static void emit_text_attr(wm_w *w, uint16_t id, const char *s)
{
    int n = (int)strlen(s);
    emit_id(w, id);
    ww_u8(w, 0x25);
    ww_u8(w, (uint8_t)n);
    ww_bytes(w, (const uint8_t *)s, n);
}

static void emit_proto_pair(wm_w *w, uint16_t psm)
{
    ww_u8(w, 0x35);
    ww_u8(w, 0x06);
    ww_u8(w, 0x19);
    ww_u16(w, 0x0100);
    ww_u8(w, 0x09);
    ww_u16(w, psm);
    ww_u8(w, 0x35);
    ww_u8(w, 0x03);
    ww_u8(w, 0x19);
    ww_u16(w, 0x0011);
}

static int build_hid_sdp_record(uint8_t *dst, int cap, uint32_t handle)
{
    wm_w w;
    uint8_t *record_seq;
    uint8_t *seq;
    uint8_t *outer;
    uint8_t *inner;

    w.p = dst;
    w.end = dst + cap;

    record_seq = ww_seq_open(&w, 1);

    /* 0x0000: ServiceRecordHandle */
    emit_id(&w, 0x0000);
    ww_u8(&w, 0x0a);
    ww_u8(&w, (uint8_t)(handle >> 24));
    ww_u8(&w, (uint8_t)(handle >> 16));
    ww_u8(&w, (uint8_t)(handle >> 8));
    ww_u8(&w, (uint8_t)handle);

    /* 0x0001: ServiceClassIDList (UUID 0x1124 - HID) */
    emit_id(&w, 0x0001);
    ww_u8(&w, 0x35);
    ww_u8(&w, 0x03);
    ww_u8(&w, 0x19);
    ww_u16(&w, 0x1124);

    /* 0x0004: ProtocolDescriptorList (L2CAP PSM 0x0011, HIDP 0x0011) */
    emit_id(&w, 0x0004);
    seq = ww_seq_open(&w, 0);
    emit_proto_pair(&w, 0x0011);
    ww_seq_close(seq, w.p, 0);

    /* 0x0005: BrowseGroupList (PublicBrowseGroup 0x1002) */
    emit_id(&w, 0x0005);
    ww_u8(&w, 0x35);
    ww_u8(&w, 0x03);
    ww_u8(&w, 0x19);
    ww_u16(&w, 0x1002);

    /* 0x0006: LanguageBaseAttributeIDList ("en", UTF-8 0x006a, base 0x0100) */
    emit_id(&w, 0x0006);
    ww_u8(&w, 0x35);
    ww_u8(&w, 0x09);
    ww_u8(&w, 0x09);
    ww_u16(&w, 0x656e);
    ww_u8(&w, 0x09);
    ww_u16(&w, 0x006a);
    ww_u8(&w, 0x09);
    ww_u16(&w, 0x0100);

    /* 0x0009: BluetoothProfileDescriptorList (HID 1.0) */
    emit_id(&w, 0x0009);
    ww_u8(&w, 0x35);
    ww_u8(&w, 0x08);
    ww_u8(&w, 0x35);
    ww_u8(&w, 0x06);
    ww_u8(&w, 0x19);
    ww_u16(&w, 0x1124);
    ww_u8(&w, 0x09);
    ww_u16(&w, 0x0100);

    /* 0x000d: AdditionalProtocolDescriptorList (L2CAP PSM 0x0013, HIDP 0x0011) */
    emit_id(&w, 0x000d);
    seq = ww_seq_open(&w, 0);
    ww_u8(&w, 0x35);
    ww_u8(&w, 0x0d);
    emit_proto_pair(&w, 0x0013);
    ww_seq_close(seq, w.p, 0);

    /* String attributes */
    emit_text_attr(&w, 0x0100, "Nintendo RVL-CNT-01");
    emit_text_attr(&w, 0x0101, "Nintendo RVL-CNT-01");
    emit_text_attr(&w, 0x0102, "Nintendo");

    /* HID attributes */
    emit_u16_attr(&w, 0x0200, 0x0100); /* Release number */
    emit_u16_attr(&w, 0x0201, 0x0111); /* Parser version */
    emit_u8_attr(&w, 0x0202, 0x04);    /* Subclass: Joystick */
    emit_u8_attr(&w, 0x0203, 0x33);    /* Country code */
    emit_bool_attr(&w, 0x0204, 0);     /* Virtual cable */
    emit_bool_attr(&w, 0x0205, 1);     /* Reconnect initiate */

    /* 0x0206: HIDDescriptorList */
    emit_id(&w, 0x0206);
    outer = ww_seq_open(&w, 0);
    inner = ww_seq_open(&w, 0);
    ww_u8(&w, 0x08);
    ww_u8(&w, 0x22); /* Report descriptor tag */
    ww_u8(&w, 0x25); /* Text/byte string tag */
    ww_u8(&w, (uint8_t)sizeof(s_rvl_hid_report));
    ww_bytes(&w, s_rvl_hid_report, (int)sizeof(s_rvl_hid_report));
    ww_seq_close(inner, w.p, 0);
    ww_seq_close(outer, w.p, 0);

    /* 0x0207: HIDLANGIDBaseList */
    emit_id(&w, 0x0207);
    ww_u8(&w, 0x35);
    ww_u8(&w, 0x08);
    ww_u8(&w, 0x35);
    ww_u8(&w, 0x06);
    ww_u8(&w, 0x09);
    ww_u16(&w, 0x0409);
    ww_u8(&w, 0x09);
    ww_u16(&w, 0x0100);

    emit_bool_attr(&w, 0x0208, 0);     /* SDP disable */
    emit_bool_attr(&w, 0x0209, 1);     /* Battery power */
    emit_bool_attr(&w, 0x020a, 1);     /* Remote wake */
    emit_u16_attr(&w, 0x020b, 0x0100); /* Profile version */
    emit_u16_attr(&w, 0x020c, 0x0c80); /* Supervision timeout */
    emit_bool_attr(&w, 0x020d, 0);     /* Normally connectable */
    emit_bool_attr(&w, 0x020e, 0);     /* Boot device */

    ww_seq_close(record_seq, w.p, 1);

    if (w.p >= w.end) {
        return -1;
    }
    return (int)(w.p - dst);
}

static void note_outputs(uint8_t leds, bool rumble)
{
    if (!s_have_leds || leds != s_last_leds) {
        s_last_leds = leds;
        s_have_leds = true;
        wiimote_note_leds(leds);
    }
    if (!s_have_rumble || (rumble ? 1 : 0) != s_last_rumble) {
        s_last_rumble = rumble ? 1 : 0;
        s_have_rumble = true;
        wiimote_note_rumble(rumble);
    }
}

static uint32_t s_last_input_tx_ms = 0;

static void flush_reports(void)
{
    uint8_t pkt[32];
    int n;
    uint8_t leds;
    bool rumble;
    uint32_t now;
    wiimote_proto_t *proto;

    if (!wiimote_ready() || s_cid_interrupt == 0) {
        return;
    }

    now = (uint32_t)(esp_timer_get_time() / 1000);
    wiimote_lock();
    proto = wiimote_state();

    /* 1. Drain any queued protocol responses (status reports, memory read/write ACKs).
     *    These are required by the Wii handshake and must be sent promptly. */
    while (s_cid_interrupt != 0 && (proto->q_count > 0 || proto->read.active)) {
        if (!l2cap_can_send_packet_now(s_cid_interrupt)) {
            l2cap_request_can_send_now_event(s_cid_interrupt);
            leds = proto->leds;
            rumble = proto->rumble;
            wiimote_unlock();
            note_outputs(leds, rumble);
            return;
        }
        n = wiimote_proto_poll(proto, pkt, sizeof(pkt), now);
        if (n <= 0) {
            break;
        }
        leds = proto->leds;
        rumble = proto->rumble;
        wiimote_unlock();

        l2cap_send(s_cid_interrupt, pkt, (uint16_t)n);
        note_outputs(leds, rumble);

        wiimote_lock();
        proto = wiimote_state();
    }

    /* 2. Send at most ONE periodic input report per call, and NEVER request
     *    L2CAP_EVENT_CAN_SEND_NOW for continuous input reports. Requesting CAN_SEND_NOW
     *    for continuous input reports creates a back-to-back Slave TX chain on the Wii
     *    link that starves the ESP32 Bluetooth Master piconet (gamepad link). */
    if (s_cid_interrupt != 0 && l2cap_can_send_packet_now(s_cid_interrupt)) {
        uint32_t min_interval_ms = 10;
        if (s_gamepad_connecting) {
            min_interval_ms = 50;
        } else if (s_outbound_busy_until_us > 0) {
            min_interval_ms = 40;
        } else if (s_gamepad_handle != HCI_CON_HANDLE_INVALID) {
            /* Scatternet coexistence: send changed inputs at up to 40 Hz (25 ms)
             * and unchanged keepalive frames at 25 Hz (40 ms) so the ESP32 radio
             * spends >80% of slots servicing the Master gamepad link without latency. */
            min_interval_ms = proto->have_last_payload ? 40 : 25;
        }
        if ((uint32_t)(now - s_last_input_tx_ms) >= min_interval_ms) {
            n = wiimote_proto_poll(proto, pkt, sizeof(pkt), now);
            if (n > 0) {
                s_last_input_tx_ms = now;
                leds = proto->leds;
                rumble = proto->rumble;
                wiimote_unlock();

                l2cap_send(s_cid_interrupt, pkt, (uint16_t)n);
                note_outputs(leds, rumble);
                return;
            }
        }
    }

    leds = proto->leds;
    rumble = proto->rumble;
    wiimote_unlock();
    note_outputs(leds, rumble);
}

static void handle_hid(const uint8_t *data, size_t len)
{
    const uint8_t *report = data;
    size_t report_len = len;

    if (len < 1 || !wiimote_ready()) {
        return;
    }

    if (data[0] == 0xa2 || data[0] == 0x52) {
        report = data + 1;
        report_len = len - 1;
    } else if ((data[0] & 0xf0) == 0x40 && len >= 2) {
        uint8_t built[32];
        int n;
        wiimote_lock();
        n = wiimote_proto_build(wiimote_state(), data[1], built, sizeof(built));
        wiimote_unlock();
        if (n > 0) {
            uint16_t target_cid = s_cid_interrupt ? s_cid_interrupt : s_cid_control;
            if (target_cid && l2cap_can_send_packet_now(target_cid)) {
                l2cap_send(target_cid, built, (uint16_t)n);
            }
        }
        return;
    } else if (data[0] < 0x10 || data[0] > 0x1a) {
        return;
    }

    {
        uint8_t level = wiimote_current_battery();
        wiimote_lock();
        wiimote_proto_set_battery(wiimote_state(), level);
        wiimote_proto_handle_output(wiimote_state(), report, report_len);
        wiimote_unlock();
    }
    flush_reports();
}

#define WM_CMD_STOP_INQUIRY        (1u << 0)
#define WM_CMD_DELETE_LINK_KEYS    (1u << 1)
#define WM_CMD_INQ_SCAN_INTERLACED (1u << 2)
#define WM_CMD_WRITE_LIAC          (1u << 3)

static uint32_t s_pending_cmds = 0;

static void pump_pending_cmds(void)
{
    if (s_pending_cmds == 0) {
        return;
    }
    if (hci_get_state() != HCI_STATE_WORKING || !hci_can_send_command_packet_now()) {
        return;
    }

    if (s_pending_cmds & WM_CMD_STOP_INQUIRY) {
        s_pending_cmds &= ~WM_CMD_STOP_INQUIRY;
        gap_inquiry_stop();
        if (!hci_can_send_command_packet_now()) {
            return;
        }
    }
    if (s_pending_cmds & WM_CMD_DELETE_LINK_KEYS) {
        bd_addr_t zero_addr = {0};
        s_pending_cmds &= ~WM_CMD_DELETE_LINK_KEYS;
        hci_send_cmd(&hci_delete_stored_link_key, zero_addr, 1);
        return;
    }
    if (s_pending_cmds & WM_CMD_INQ_SCAN_INTERLACED) {
        s_pending_cmds &= ~WM_CMD_INQ_SCAN_INTERLACED;
        hci_send_cmd(&hci_write_inquiry_scan_type, 1);
        return;
    }
    if (s_pending_cmds & WM_CMD_WRITE_LIAC) {
        s_pending_cmds &= ~WM_CMD_WRITE_LIAC;
        hci_send_cmd(&hci_write_current_iac_lap_two_iacs, 2, GAP_IAC_LIMITED_INQUIRY, GAP_IAC_GENERAL_INQUIRY);
        return;
    }
}

static void report_timer_cb(btstack_timer_source_t *ts)
{
    int64_t now = esp_timer_get_time();

    if (s_name_req_pending && now >= s_name_req_cancel_at_us) {
        if (hci_get_state() == HCI_STATE_WORKING && hci_can_send_command_packet_now()) {
            s_name_req_pending = false;
            s_name_req_cancel_at_us = 0;
            ESP_LOGW(TAG, "Cancelling slow HCI_Remote_Name_Request before L2CAP connect");
            hci_send_cmd(&hci_remote_name_request_cancel, s_gamepad_addr);
            btstack_run_loop_set_timer(ts, 10);
            btstack_run_loop_add_timer(ts);
            return;
        }
    }

    if (s_outbound_busy_until_us > 0 && now >= s_outbound_busy_until_us) {
        s_outbound_busy_until_us = 0;
        if (s_gamepad_connecting) {
            if (s_gamepad_handle != HCI_CON_HANDLE_INVALID) {
                s_gamepad_connecting = false;
                s_gamepad_connected = true;
            } else {
                if (hci_get_state() == HCI_STATE_WORKING && hci_can_send_command_packet_now()) {
                    s_gamepad_connecting = false;
                    ESP_LOGI(TAG, "Gamepad connection attempt timed out; cancelling page & resuming scan");
                    hci_send_cmd(&hci_create_connection_cancel, s_gamepad_addr);
                    resume_gamepad_scanning();
                    btstack_run_loop_set_timer(ts, 10);
                    btstack_run_loop_add_timer(ts);
                    return;
                }
                /* Wait until next tick if command buffer is currently busy */
                s_outbound_busy_until_us = now + 20000LL;
            }
        }
    }

    pump_pending_cmds();
    flush_reports();
    btstack_run_loop_set_timer(ts, 10);
    btstack_run_loop_add_timer(ts);
}

static void wiimote_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (packet_type == HCI_EVENT_PACKET) {
        uint8_t event = hci_event_packet_get_type(packet);
        switch (event) {
        case HCI_EVENT_COMMAND_COMPLETE: {
            uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
            const uint8_t *ret = hci_event_command_complete_get_return_parameters(packet);
            uint8_t status = ret ? ret[0] : 0xff;
            if (opcode == HCI_OPCODE_HCI_PERIODIC_INQUIRY_MODE &&
                (s_sync_window || s_gamepad_connecting || s_gamepad_connected)) {
                ESP_LOGI(TAG, "Stopping periodic inquiry (sync=%d connecting=%d connected=%d)",
                         s_sync_window, s_gamepad_connecting, s_gamepad_connected);
                gap_inquiry_stop();
            } else if (opcode == HCI_OPCODE_HCI_WRITE_CURRENT_IAC_LAP_TWO_IACS) {
                ESP_LOGI(TAG, "HCI_Write_Current_IAC_LAP status=0x%02x (LIAC 0x9E8B00 + GIAC 0x9E8B33)", status);
            }
            break;
        }
        case GAP_EVENT_INQUIRY_RESULT: {
            bd_addr_t addr;
            uint32_t cod = gap_event_inquiry_result_get_class_of_device(packet);
            gap_event_inquiry_result_get_bd_addr(packet, addr);
            if (!s_sync_window && ((cod & 0x1f00u) == 0x0500u)) {
                if (s_gamepad_connected || (s_gamepad_connecting && bd_addr_cmp(addr, s_gamepad_addr) != 0)) {
                    /* Ignore additional controllers while one is already connecting or connected
                     * so concurrent HCI_Remote_Name_Request / HCI_Create_Connection calls do not collide. */
                    packet[0] = HCI_EVENT_NOP;
                    break;
                }
                if (!s_gamepad_connecting) {
                    s_gamepad_connecting = true;
                    memcpy(s_gamepad_addr, addr, 6);
                    s_outbound_busy_until_us = esp_timer_get_time() + 14000000LL;
                    if (!gap_event_inquiry_result_get_name_available(packet)) {
                        s_name_req_pending = true;
                        s_name_req_cancel_at_us = esp_timer_get_time() + 3800000LL;
                    }
                    pause_gamepad_scanning();
                    ESP_LOGI(TAG, "Gamepad %02x:%02x:%02x:%02x:%02x:%02x discovered; paused inquiry & BLE scan for connection",
                             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
                }
            }
            break;
        }
        case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE: {
            s_name_req_pending = false;
            s_name_req_cancel_at_us = 0;
            break;
        }
        case HCI_EVENT_CONNECTION_REQUEST: {
            bd_addr_t addr;
            uint32_t cod = hci_event_connection_request_get_class_of_device(packet);
            uint8_t link_type = hci_event_connection_request_get_link_type(packet);
            hci_event_connection_request_get_bd_addr(packet, addr);
            ESP_LOGI(TAG, "HCI connection request from %02x:%02x:%02x:%02x:%02x:%02x cod=0x%06lx link_type=%u",
                     addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
                     (unsigned long)cod, (unsigned)link_type);
            if ((cod & 0x1f00u) != 0x0500u) {
                /* Connecting peer is a host/console (e.g. Nintendo Wii, cod=0x000448).
                 * Accept inbound ACL as slave so the Wii console remains master, and consume
                 * the event so Bluepad32 does not allocate a gamepad slot for the Wii. */
                hci_set_master_slave_policy(HCI_ROLE_SLAVE);
                memcpy(s_bda, addr, 6);
                s_have_host_bda = true;
                packet[0] = HCI_EVENT_NOP;
            } else {
                /* Connecting peer is a peripheral gamepad: become master as Bluepad32 expects. */
                hci_set_master_slave_policy(HCI_ROLE_MASTER);
                s_gamepad_connecting = true;
                memcpy(s_gamepad_addr, addr, 6);
                s_outbound_busy_until_us = esp_timer_get_time() + 10000000LL;
                pause_gamepad_scanning();
            }
            break;
        }
        case HCI_EVENT_CONNECTION_COMPLETE: {
            bd_addr_t addr;
            uint8_t status = hci_event_connection_complete_get_status(packet);
            uint16_t handle = hci_event_connection_complete_get_connection_handle(packet);
            hci_event_connection_complete_get_bd_addr(packet, addr);
            ESP_LOGI(TAG, "HCI connection complete from %02x:%02x:%02x:%02x:%02x:%02x handle=0x%04x status=0x%02x",
                     addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
                     handle, status);
            if (is_wii_host_addr(addr)) {
                if (status == ERROR_CODE_SUCCESS) {
                    s_con_handle = handle;
                }
                if (uni_bt_packet_handler != NULL) {
                    packet[0] = HCI_EVENT_NOP;
                }
            } else {
                s_name_req_pending = false;
                s_name_req_cancel_at_us = 0;
                if (status == ERROR_CODE_SUCCESS) {
                    s_gamepad_handle = handle;
                    s_gamepad_connected = true;
                    s_gamepad_connecting = false;
                    s_outbound_busy_until_us = esp_timer_get_time() + 4000000LL;
                    pause_gamepad_scanning();
                } else {
                    s_gamepad_connecting = false;
                    s_gamepad_connected = false;
                    s_gamepad_handle = HCI_CON_HANDLE_INVALID;
                    s_outbound_busy_until_us = 0;
                    ESP_LOGI(TAG, "Gamepad ACL connection failed (status=0x%02x); resuming scan", status);
                    resume_gamepad_scanning();
                }
            }
            break;
        }
        case L2CAP_EVENT_INCOMING_CONNECTION: {
            uint16_t psm = l2cap_event_incoming_connection_get_psm(packet);
            uint16_t cid = l2cap_event_incoming_connection_get_local_cid(packet);
            bd_addr_t addr;
            l2cap_event_incoming_connection_get_address(packet, addr);
            if (!is_wii_host_addr(addr)) {
                if (uni_bt_packet_handler != NULL) {
                    uni_bt_packet_handler(packet_type, channel, packet, size);
                } else {
                    l2cap_decline_connection(cid);
                }
                break;
            }
            ESP_LOGI(TAG, "L2CAP incoming connection PSM 0x%04x cid %u from %02x:%02x:%02x:%02x:%02x:%02x",
                     psm, cid, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
            if (psm == 0x0011 || psm == 0x0013) {
                memcpy(s_bda, addr, 6);
                s_have_host_bda = true;
                l2cap_accept_connection(cid);
            } else {
                l2cap_decline_connection(cid);
            }
            break;
        }
        case L2CAP_EVENT_CHANNEL_OPENED: {
            uint8_t status = l2cap_event_channel_opened_get_status(packet);
            uint16_t cid = l2cap_event_channel_opened_get_local_cid(packet);
            uint16_t psm = l2cap_event_channel_opened_get_psm(packet);
            bd_addr_t addr;
            l2cap_event_channel_opened_get_address(packet, addr);
            if (!is_wii_host_addr(addr)) {
                if (uni_bt_packet_handler != NULL) {
                    uni_bt_packet_handler(packet_type, channel, packet, size);
                }
                break;
            }
            if (status != ERROR_CODE_SUCCESS) {
                ESP_LOGE(TAG, "L2CAP channel open failed: status 0x%02x cid %u", status, cid);
                break;
            }
            s_con_handle = l2cap_event_channel_opened_get_handle(packet);
            ESP_LOGI(TAG, "L2CAP channel open: PSM 0x%04x cid %u", psm, cid);
            if (psm == 0x0011) {
                s_cid_control = cid;
            } else if (psm == 0x0013) {
                s_cid_interrupt = cid;
            }
            if (s_cid_control != 0 && s_cid_interrupt != 0) {
                /* Close sync window BEFORE invoking wiimote_set_connected(true) so that
                 * if the connected callback starts Bluepad32 periodic inquiry, s_sync_window
                 * is already false and does not cancel the newly started inquiry! */
                s_sync_window = false;
                s_pending_cmds &= ~WM_CMD_STOP_INQUIRY;
                gap_discoverable_control(0);
                gap_connectable_control(0);
                gap_set_page_scan_activity(0x0800, 0x0012);
                gap_inquiry_set_scan_activity(0x0800, 0x0012);
                /* Set page timeout to 6.4s (0x2800 slots) instead of BTstack's 15.36s default
                 * so a missed page fails and cleans up well before Bluepad32's 20s timeout. */
                gap_set_page_timeout(0x2800);
                if (uni_bt_packet_handler != NULL) {
                    /* Reduce BLE scan duty cycle from 100% (48/48) to 12.5% (48/384)
                     * so BLE scanning does not starve BR/EDR inquiry & paging while connected to Wii. */
                    gap_set_scan_parameters(0, 0x0180, 0x0030);
                }
                ESP_LOGI(TAG, "Wiimote connected to Wii console (Control & Interrupt channels open)");
                wiimote_set_connected(true);
            }
            break;
        }
        case L2CAP_EVENT_CHANNEL_CLOSED: {
            uint16_t cid = l2cap_event_channel_closed_get_local_cid(packet);
            if (cid != s_cid_control && cid != s_cid_interrupt) {
                if (uni_bt_packet_handler != NULL) {
                    uni_bt_packet_handler(packet_type, channel, packet, size);
                }
                break;
            }
            ESP_LOGI(TAG, "L2CAP channel closed: cid %u", cid);
            if (cid == s_cid_control) s_cid_control = 0;
            if (cid == s_cid_interrupt) s_cid_interrupt = 0;
            if (s_cid_interrupt == 0) {
                gap_connectable_control(1);
                wiimote_set_connected(false);
            }
            break;
        }
        case L2CAP_EVENT_CAN_SEND_NOW: {
            uint16_t cid = l2cap_event_can_send_now_get_local_cid(packet);
            if (cid == s_cid_interrupt || cid == s_cid_control) {
                flush_reports();
            } else if (uni_bt_packet_handler != NULL) {
                uni_bt_packet_handler(packet_type, channel, packet, size);
            }
            break;
        }
        case HCI_EVENT_PIN_CODE_REQUEST: {
            /* gap_pin_code_response_binary stores the pointer without copying, so s_pin must be static */
            static uint8_t s_pin[6];
            bd_addr_t addr;
            hci_event_pin_code_request_get_bd_addr(packet, addr);
            if (is_wii_host_addr(addr)) {
                ESP_LOGI(TAG, "PIN request from %02x:%02x:%02x:%02x:%02x:%02x, reply reversed peer BD_ADDR (6 bytes)",
                         addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
                for (int i = 0; i < 6; i++) {
                    s_pin[i] = addr[5 - i];
                }
                gap_pin_code_response_binary(addr, s_pin, 6);
                /* Consume event so Bluepad32's uni_bt_bredr_on_hci_pin_code_request does not overwrite the PIN */
                packet[0] = HCI_EVENT_NOP;
            }
            break;
        }
        case HCI_EVENT_LINK_KEY_NOTIFICATION: {
            bd_addr_t addr;
            reverse_bd_addr(&packet[2], addr);
            ESP_LOGI(TAG, "Link key stored for %02x:%02x:%02x:%02x:%02x:%02x",
                     addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
            break;
        }
        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            uint16_t handle = hci_event_disconnection_complete_get_connection_handle(packet);
            uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            if (s_con_handle != HCI_CON_HANDLE_INVALID && handle == s_con_handle) {
                ESP_LOGI(TAG, "Wii HCI disconnection complete: handle 0x%04x reason 0x%02x", handle, reason);
                s_con_handle = HCI_CON_HANDLE_INVALID;
                s_cid_control = 0;
                s_cid_interrupt = 0;
                gap_connectable_control(1);
                wiimote_set_connected(false);
            } else if (s_gamepad_handle != HCI_CON_HANDLE_INVALID && handle == s_gamepad_handle) {
                s_gamepad_handle = HCI_CON_HANDLE_INVALID;
                s_outbound_busy_until_us = 0;
                if (s_gamepad_connected || s_gamepad_connecting) {
                    s_gamepad_connected = false;
                    s_gamepad_connecting = false;
                    ESP_LOGI(TAG, "Gamepad disconnected (handle 0x%04x reason 0x%02x); resuming scan", handle, reason);
                    resume_gamepad_scanning();
                }
            }
            break;
        }
        default:
            break;
        }
        pump_pending_cmds();
    } else if (packet_type == L2CAP_DATA_PACKET) {
        if (channel == s_cid_control || channel == s_cid_interrupt) {
            handle_hid(packet, size);
        } else if (uni_bt_packet_handler != NULL) {
            uni_bt_packet_handler(packet_type, channel, packet, size);
        }
    }
}

static void register_hid_services_main_thread(void *context)
{
    (void)context;
    if (s_hid_ready) {
        return;
    }

    s_hid_handle = sdp_create_service_record_handle();
    int rlen = build_hid_sdp_record(s_hid_sdp_record, sizeof(s_hid_sdp_record), s_hid_handle);
    if (rlen > 0) {
        sdp_register_service(s_hid_sdp_record);
        ESP_LOGI(TAG, "Registered RVL-CNT-01 HID SDP record (len %d)", rlen);
    } else {
        ESP_LOGE(TAG, "Failed to build RVL-CNT-01 HID SDP record");
    }

    /* Remove Bluepad32's default Device ID record (0x10003) if present so only Nintendo's is advertised */
    sdp_unregister_service(0x10003);

    s_device_id_handle = sdp_create_service_record_handle();
    device_id_create_sdp_record(
        s_device_id_sdp_record,
        s_device_id_handle,
        DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH,
        0x057E, /* Nintendo */
        0x0306, /* RVL-CNT-01 */
        0x0100  /* Version 1.0.0 */
    );
    sdp_register_service(s_device_id_sdp_record);

    gap_set_local_name("Nintendo RVL-CNT-01");
    gap_set_class_of_device(0x002504);
    gap_set_security_mode(GAP_SECURITY_MODE_2);
    gap_ssp_set_auto_accept(true);
    gap_set_allow_role_switch(true);
    hci_set_master_slave_policy(HCI_ROLE_SLAVE);
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_HOLD_MODE |
                                         LM_LINK_POLICY_ENABLE_SNIFF_MODE);

    /* Ensure PSM 0x0011 and 0x0013 are free before registering (e.g. if Bluepad32 registered them) */
    l2cap_unregister_service(0x0011);
    l2cap_unregister_service(0x0013);
    l2cap_register_service(wiimote_packet_handler, 0x0011, 672, LEVEL_0);
    l2cap_register_service(wiimote_packet_handler, 0x0013, 672, LEVEL_0);

    /* Insert handler at the head of HCI event handlers so we intercept Wii PIN requests before Bluepad32 */
    s_hci_event_callback_registration.callback = wiimote_packet_handler;
    hci_add_event_handler_for_security_manager(&s_hci_event_callback_registration);

    btstack_run_loop_set_timer_handler(&s_report_timer, report_timer_cb);
    btstack_run_loop_set_timer(&s_report_timer, 10);
    btstack_run_loop_add_timer(&s_report_timer);

    gap_connectable_control(1);
    gap_discoverable_control(0);

    s_hid_ready = true;
    ESP_LOGI(TAG, "Wiimote BTstack services active (PSM 0x11 & 0x13)");
}

static bool s_external_stack = false;

void wiimote_bt_set_external_stack(bool external)
{
    s_external_stack = external;
}

#if __has_include("btstack_port_esp32.h") || defined(ESP_PLATFORM)
static void standalone_btstack_run_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "BTstack run loop task starting");
    btstack_run_loop_execute();
    vTaskDelete(NULL);
}
#endif

esp_err_t wiimote_bt_start(void)
{
    /* If BTstack is not managed externally and not yet powered on, bring up controller and run loop */
    if (!s_external_stack && !s_btstack_started && hci_get_state() == HCI_STATE_OFF) {
#if __has_include("btstack_port_esp32.h")
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            nvs_flash_erase();
            err = nvs_flash_init();
        }
        if (err != ESP_OK) return err;

        if (btstack_init() != ERROR_CODE_SUCCESS) {
            return ESP_FAIL;
        }
        l2cap_init();
        sdp_init();
        s_btstack_started = true;

        /* Register Wiimote services during startup */
        register_hid_services_main_thread(NULL);

        hci_power_control(HCI_POWER_ON);

        /* Start BTstack run loop in FreeRTOS task if running standalone */
        xTaskCreatePinnedToCore(standalone_btstack_run_task, "btstack", 6144, NULL, 5, NULL, 0);
        return ESP_OK;
#else
        return ESP_ERR_NOT_SUPPORTED;
#endif
    }

    /* BTstack is already managed externally (e.g. by Bluepad32 or app) */
    s_init_registration.callback = register_hid_services_main_thread;
    btstack_run_loop_execute_on_main_thread(&s_init_registration);
    return ESP_OK;
}

esp_err_t wiimote_btstack_register_services(void)
{
    s_init_registration.callback = register_hid_services_main_thread;
    btstack_run_loop_execute_on_main_thread(&s_init_registration);
    return ESP_OK;
}

static void sync_main_thread(void *context)
{
    (void)context;
    s_sync_window = true;
    pause_gamepad_scanning();
    gap_set_local_name("Nintendo RVL-CNT-01");
    gap_set_class_of_device(0x002504);
    gap_set_security_mode(GAP_SECURITY_MODE_2);
    gap_set_allow_role_switch(true);
    hci_set_master_slave_policy(HCI_ROLE_SLAVE);
    gap_set_page_scan_type(PAGE_SCAN_MODE_INTERLACED);
    gap_set_page_scan_activity(0x0200, 0x0024);
    gap_inquiry_set_scan_activity(0x0200, 0x0024);
    if (s_have_host_bda) {
        gap_drop_link_key_for_bd_addr(s_bda);
    }
    gap_connectable_control(1);
    gap_discoverable_control(1);

    /* Queue controller-level LIAC+GIAC registration and interlaced inquiry scan */
    s_pending_cmds |= WM_CMD_STOP_INQUIRY | WM_CMD_INQ_SCAN_INTERLACED | WM_CMD_WRITE_LIAC;
    pump_pending_cmds();
    ESP_LOGI(TAG, "Wiimote sync window active (LIAC 0x9E8B00 + GIAC 0x9E8B33 discoverable + connectable)");
}

static void listen_main_thread(void *context)
{
    (void)context;
    s_sync_window = false;
    gap_discoverable_control(0);
    gap_connectable_control(1);
    ESP_LOGI(TAG, "Wiimote sync window closed; connectable, non-discoverable");
}

static void stop_main_thread(void *context)
{
    (void)context;
    s_sync_window = false;
    gap_discoverable_control(0);
    gap_connectable_control(0);
    if (s_cid_interrupt) {
        l2cap_disconnect(s_cid_interrupt);
        s_cid_interrupt = 0;
    }
    if (s_cid_control) {
        l2cap_disconnect(s_cid_control);
        s_cid_control = 0;
    }
    wiimote_set_connected(false);
}

esp_err_t wiimote_bt_sync(void)
{
    s_sync_registration.callback = sync_main_thread;
    btstack_run_loop_execute_on_main_thread(&s_sync_registration);
    return ESP_OK;
}

void wiimote_bt_listen_only(void)
{
    s_listen_registration.callback = listen_main_thread;
    btstack_run_loop_execute_on_main_thread(&s_listen_registration);
}

esp_err_t wiimote_bt_stop(void)
{
    s_stop_registration.callback = stop_main_thread;
    btstack_run_loop_execute_on_main_thread(&s_stop_registration);
    return ESP_OK;
}
