/* SPDX-License-Identifier: MIT */
/*
 * Host test for the BTstack backend (src/wiimote_bt_btstack.inc).
 *
 * BTstack is replaced by small fakes that record what the backend does; the
 * test feeds the backend the HCI / L2CAP events a Wii, a reconnecting gamepad
 * and a page to the Wii produce, and checks routing, PIN replies, event
 * hiding, link state and the HID replies that go out. The protocol layer
 * (wiimote_proto.c / wiimote_ext.c) is the real one.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/wiimote_bt_btstack.inc"

int g_verbose;
static int s_failures;
#define CHECK(c)                                                              \
    do {                                                                      \
        if (!(c)) {                                                           \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);               \
            s_failures++;                                                     \
        }                                                                     \
    } while (0)

/* ------------------------------------------------------- fake "core" -- */

static wiimote_proto_t g_proto;
static bool g_connected;
static int g_connected_events, g_disconnected_events, g_sync_ended;
static uint8_t g_leds_seen = 0xff;

void wiimote_lock(void) {}
void wiimote_unlock(void) {}
wiimote_proto_t *wiimote_state(void) { return &g_proto; }
bool wiimote_ready(void) { return true; }
bool wiimote_connected_flag(void) { return g_connected; }
uint8_t wiimote_current_battery(void) { return 0xc0; }
void wiimote_note_leds(uint8_t mask) { g_leds_seen = mask; }
void wiimote_note_rumble(bool on) { (void)on; }
void wiimote_note_sync_ended(void) { g_sync_ended++; }
void wiimote_set_connected(bool up)
{
    bool was = g_connected;
    g_connected = up;
    g_proto.link_up = up;
    if (!up) {
        g_proto.q_count = 0;
        g_proto.read.active = false;
        g_proto.reporting = false;
    }
    if (was != up) {
        if (up) {
            g_connected_events++;
        } else {
            g_disconnected_events++;
        }
    }
}

/* ---------------------------------------------------- fake platform -- */

static int64_t g_now_us = 1000000;
int64_t esp_timer_get_time(void) { return g_now_us; }
int xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, uint32_t stack, void *arg, int prio,
                            TaskHandle_t *h, int core)
{
    (void)fn; (void)name; (void)stack; (void)arg; (void)prio; (void)h; (void)core;
    return pdPASS;
}
void vTaskDelete(TaskHandle_t h) { (void)h; }

/* ------------------------------------------------------ fake BTstack -- */

static HCI_STATE g_hci_state = HCI_STATE_OFF;
HCI_STATE hci_get_state(void) { return g_hci_state; }
bool hci_can_send_command_packet_now(void) { return true; }
static int g_cmds_sent;
uint8_t hci_send_cmd(const hci_cmd_t *cmd, ...) { (void)cmd; g_cmds_sent++; return 0; }
const hci_cmd_t hci_write_inquiry_scan_type;
const hci_cmd_t hci_write_current_iac_lap_two_iacs;
const hci_cmd_t hci_create_connection_cancel;
const hci_cmd_t hci_write_link_policy_settings;
const hci_cmd_t hci_disconnect;
const hci_cmd_t hci_qos_setup;
uint8_t btstack_init(void) { return 0; }
void l2cap_init(void) {}
void sdp_init(void) {}
int hci_power_control(HCI_POWER_MODE mode) { (void)mode; return 0; }
void btstack_run_loop_execute(void) {}

static btstack_packet_callback_registration_t *g_hci_handler;
void hci_add_event_handler_for_security_manager(btstack_packet_callback_registration_t *r) { g_hci_handler = r; }
static int (*g_filter)(bd_addr_t, hci_link_type_t);
void gap_register_classic_connection_filter(int (*cb)(bd_addr_t, hci_link_type_t)) { g_filter = cb; }
static uint8_t g_policy = 0xff;
void hci_set_master_slave_policy(uint8_t p) { g_policy = p; }

/* GAP */
static int g_discoverable = -1, g_connectable = -1;
void gap_set_local_name(const char *n) { (void)n; }
void gap_set_class_of_device(uint32_t c) { (void)c; }
static bool g_allow_role_switch = true;
void gap_set_allow_role_switch(bool a) { g_allow_role_switch = a; }
void gap_set_default_link_policy_settings(uint16_t s) { (void)s; }
static hci_role_t g_role = HCI_ROLE_SLAVE;
hci_role_t gap_get_role(hci_con_handle_t h) { (void)h; return g_role; }
void gap_connectable_control(uint8_t e) { g_connectable = e; }
void gap_discoverable_control(uint8_t e) { g_discoverable = e; }
void gap_set_page_scan_activity(uint16_t i, uint16_t w) { (void)i; (void)w; }
void gap_inquiry_set_scan_activity(uint16_t i, uint16_t w) { (void)i; (void)w; }
static bd_addr_t g_key_addr;
static bool g_have_key;
bool gap_get_link_key_for_bd_addr(bd_addr_t addr, link_key_t key, link_key_type_t *type)
{
    (void)key; (void)type;
    return g_have_key && memcmp(addr, g_key_addr, 6) == 0;
}
void gap_drop_link_key_for_bd_addr(bd_addr_t addr) { (void)addr; }
static uint8_t g_pin[16];
static int g_pin_len;
static bd_addr_t g_pin_addr;
int gap_pin_code_response_binary(const bd_addr_t addr, const uint8_t *pin, uint8_t len)
{
    memcpy(g_pin_addr, addr, 6);
    memcpy(g_pin, pin, len);
    g_pin_len = len;
    return 0;
}
static int g_role_requests, g_security_requests, g_disconnects;
static hci_con_handle_t g_security_handle;
static gap_security_level_t g_link_level = LEVEL_0;
uint8_t gap_request_role(const bd_addr_t addr, hci_role_t role) { (void)addr; (void)role; g_role_requests++; return 0; }
void gap_request_security_level(hci_con_handle_t h, gap_security_level_t l) { (void)l; g_security_handle = h; g_security_requests++; }
static gap_security_level_t g_gap_level = LEVEL_2, g_level_at_create = LEVEL_2;
gap_security_level_t gap_get_security_level(void) { return g_gap_level; }
void gap_set_security_level(gap_security_level_t l) { g_gap_level = l; }
gap_security_level_t gap_security_level(hci_con_handle_t h) { (void)h; return g_link_level; }
gap_connection_type_t gap_get_connection_type(hci_con_handle_t h) { (void)h; return GAP_CONNECTION_ACL; }
static uint16_t g_page_timeout;
void gap_set_page_timeout(uint16_t t) { g_page_timeout = t; }
uint8_t gap_disconnect(hci_con_handle_t h) { (void)h; g_disconnects++; return 0; }

/* L2CAP */
static btstack_packet_handler_t g_services[2];
uint8_t l2cap_register_service(btstack_packet_handler_t h, uint16_t psm, uint16_t mtu, gap_security_level_t l)
{
    (void)mtu; (void)l;
    g_services[psm == WM_PSM_CONTROL ? 0 : 1] = h;
    return 0;
}
uint8_t l2cap_unregister_service(uint16_t psm) { (void)psm; return 0; }
static int g_accepts, g_declines;
void l2cap_accept_connection(uint16_t cid) { (void)cid; g_accepts++; }
void l2cap_decline_connection(uint16_t cid) { (void)cid; g_declines++; }
static uint16_t g_next_cid = 0x60;
static uint16_t g_created_psm[4];
static int g_created;
uint8_t l2cap_create_channel(btstack_packet_handler_t h, bd_addr_t a, uint16_t psm, uint16_t mtu, uint16_t *cid)
{
    (void)h; (void)a; (void)mtu;
    g_level_at_create = g_gap_level;
    *cid = g_next_cid++;
    g_created_psm[g_created++ & 3] = psm;
    return 0;
}
uint8_t l2cap_disconnect(uint16_t cid) { (void)cid; return 0; }
static bool g_can_send = true;
bool l2cap_can_send_packet_now(uint16_t cid) { (void)cid; return g_can_send; }
static int g_can_send_requests;
uint8_t l2cap_request_can_send_now_event(uint16_t cid) { (void)cid; g_can_send_requests++; return 0; }
static uint8_t g_sent[64][32];
static uint16_t g_sent_cid[64];
static int g_sent_len[64];
static int g_sent_count;
uint8_t l2cap_send(uint16_t cid, const uint8_t *data, uint16_t len)
{
    if (g_sent_count < 64) {
        memcpy(g_sent[g_sent_count], data, len > 32 ? 32 : len);
        g_sent_len[g_sent_count] = len;
        g_sent_cid[g_sent_count] = cid;
        g_sent_count++;
    }
    return 0;
}

/* SDP */
static uint8_t *g_records[4];
static int g_record_count;
static uint32_t g_next_handle = 0x10001;
uint32_t sdp_create_service_record_handle(void) { return g_next_handle++; }
uint8_t sdp_register_service(const uint8_t *record) { g_records[g_record_count++ & 3] = (uint8_t *)record; return 0; }
void sdp_unregister_service(uint32_t h) { (void)h; }
uint8_t *sdp_get_record_for_handle(uint32_t h) { (void)h; return NULL; }
void device_id_create_sdp_record(uint8_t *service, uint32_t handle, uint16_t src, uint16_t vid, uint16_t pid,
                                 uint16_t ver)
{
    (void)handle; (void)src; (void)vid; (void)pid; (void)ver;
    de_create_sequence(service);
}

/* Run loop */
static btstack_context_callback_registration_t *g_main_thread[8];
static int g_main_count;
void btstack_run_loop_execute_on_main_thread(btstack_context_callback_registration_t *r)
{
    for (int i = 0; i < g_main_count; i++) {
        if (g_main_thread[i] == r) {
            return;
        }
    }
    g_main_thread[g_main_count++] = r;
}
static void run_main_thread(void)
{
    while (g_main_count > 0) {
        btstack_context_callback_registration_t *r = g_main_thread[0];
        memmove(g_main_thread, g_main_thread + 1, sizeof(g_main_thread[0]) * (size_t)(--g_main_count));
        r->callback(r->context);
    }
}
/* Only the report timer is observed through g_timer_armed. */
static bool g_timer_armed;
static bool g_watchdog_armed;
void btstack_run_loop_set_timer_handler(btstack_timer_source_t *ts, void (*fn)(btstack_timer_source_t *)) { ts->process = fn; }
void btstack_run_loop_set_timer(btstack_timer_source_t *ts, uint32_t ms) { (void)ts; (void)ms; }
static void set_armed(btstack_timer_source_t *ts, bool on)
{
    if (ts == &s_report_timer) g_timer_armed = on;
    if (ts == &s_connect_watchdog) g_watchdog_armed = on;
}
void btstack_run_loop_add_timer(btstack_timer_source_t *ts) { set_armed(ts, true); }
int btstack_run_loop_remove_timer(btstack_timer_source_t *ts) { set_armed(ts, false); return 1; }

/* TLV: one slot is enough here. */
static uint8_t g_tlv_data[6];
static bool g_tlv_have;
static int tlv_get(void *ctx, uint32_t tag, uint8_t *buf, uint32_t len)
{
    (void)ctx; (void)tag;
    if (!g_tlv_have || len < 6) return 0;
    memcpy(buf, g_tlv_data, 6);
    return 6;
}
static int tlv_store(void *ctx, uint32_t tag, const uint8_t *data, uint32_t len)
{
    (void)ctx; (void)tag; (void)len;
    memcpy(g_tlv_data, data, 6);
    g_tlv_have = true;
    return 0;
}
static void tlv_delete(void *ctx, uint32_t tag) { (void)ctx; (void)tag; g_tlv_have = false; }
static const btstack_tlv_t g_tlv = {tlv_get, tlv_store, tlv_delete};
void btstack_tlv_get_instance(const btstack_tlv_t **tlv, void **ctx) { *tlv = &g_tlv; *ctx = NULL; }

/* --------------------------------------------------- fake Bluepad32 -- */

static int g_fallback_calls;
static uint8_t g_fallback_last_event;
static int g_bp32_hci_events[256];
static void fake_bp32(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel; (void)size;
    g_fallback_calls++;
    if (packet_type == HCI_EVENT_PACKET) {
        g_fallback_last_event = packet[0];
    }
}

/* Deliver an HCI event the way hci_emit_event does: our handler first, then
 * (with the same, possibly rewritten, buffer) the host stack's. */
static void emit_hci(uint8_t *ev, uint16_t len)
{
    g_hci_handler->callback(HCI_EVENT_PACKET, 0, ev, len);
    g_bp32_hci_events[ev[0]]++;
}

/* ------------------------------------------------------ event builders -- */

static const bd_addr_t WII = {0x00, 0x1f, 0x32, 0xaa, 0xbb, 0xcc};
static const bd_addr_t PAD = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};

static void put_addr(uint8_t *p, const bd_addr_t a) { reverse_bytes(a, p, 6); }

static void ev_state(HCI_STATE st)
{
    uint8_t ev[3] = {BTSTACK_EVENT_STATE, 1, (uint8_t)st};
    g_hci_state = st;
    emit_hci(ev, sizeof(ev));
}

static uint8_t ev_conn_request(const bd_addr_t a, uint32_t cod)
{
    uint8_t ev[12] = {HCI_EVENT_CONNECTION_REQUEST, 10};
    put_addr(ev + 2, a);
    little_endian_store_24(ev, 8, cod);
    ev[11] = HCI_LINK_TYPE_ACL;
    /* hci.c calls the connection filter before emitting the event. */
    g_filter((uint8_t *)a, HCI_LINK_TYPE_ACL);
    emit_hci(ev, sizeof(ev));
    return ev[0];
}

static uint8_t ev_conn_complete(const bd_addr_t a, uint16_t handle, uint8_t status)
{
    uint8_t ev[13] = {HCI_EVENT_CONNECTION_COMPLETE, 11, status};
    little_endian_store_16(ev, 3, handle);
    put_addr(ev + 5, a);
    ev[11] = HCI_LINK_TYPE_ACL;
    emit_hci(ev, sizeof(ev));
    return ev[0];
}

static uint8_t ev_pin_request(const bd_addr_t a)
{
    uint8_t ev[8] = {HCI_EVENT_PIN_CODE_REQUEST, 6};
    put_addr(ev + 2, a);
    emit_hci(ev, sizeof(ev));
    return ev[0];
}

static void ev_disconnect(uint16_t handle)
{
    uint8_t ev[6] = {HCI_EVENT_DISCONNECTION_COMPLETE, 4, 0};
    little_endian_store_16(ev, 3, handle);
    ev[5] = 0x13;
    emit_hci(ev, sizeof(ev));
}

static void l2cap_incoming(const bd_addr_t a, uint16_t handle, uint16_t psm, uint16_t cid)
{
    uint8_t ev[16] = {L2CAP_EVENT_INCOMING_CONNECTION, 14};
    put_addr(ev + 2, a);
    little_endian_store_16(ev, 8, handle);
    little_endian_store_16(ev, 10, psm);
    little_endian_store_16(ev, 12, cid);
    little_endian_store_16(ev, 14, 0x40);
    g_services[psm == WM_PSM_CONTROL ? 0 : 1](HCI_EVENT_PACKET, cid, ev, sizeof(ev));
}

static void l2cap_opened(const bd_addr_t a, uint16_t handle, uint16_t psm, uint16_t cid, uint8_t status)
{
    uint8_t ev[26] = {L2CAP_EVENT_CHANNEL_OPENED, 24, status};
    put_addr(ev + 3, a);
    little_endian_store_16(ev, 9, handle);
    little_endian_store_16(ev, 11, psm);
    little_endian_store_16(ev, 13, cid);
    wm_l2cap_handler(HCI_EVENT_PACKET, cid, ev, sizeof(ev));
}

static void ev_security_level(uint16_t handle, uint8_t level)
{
    uint8_t ev[6] = {GAP_EVENT_SECURITY_LEVEL, 4};
    little_endian_store_16(ev, 2, handle);
    ev[4] = level;
    ev[5] = 0;
    emit_hci(ev, sizeof(ev));
}

static void l2cap_closed(uint16_t cid)
{
    uint8_t ev[4] = {L2CAP_EVENT_CHANNEL_CLOSED, 2};
    little_endian_store_16(ev, 2, cid);
    wm_l2cap_handler(HCI_EVENT_PACKET, cid, ev, sizeof(ev));
}

static void l2cap_data(uint16_t cid, const uint8_t *data, uint16_t len)
{
    wm_l2cap_handler(L2CAP_DATA_PACKET, cid, (uint8_t *)data, len);
}

static int find_sent(uint8_t report_id)
{
    for (int i = 0; i < g_sent_count; i++) {
        if (g_sent_len[i] >= 2 && g_sent[i][0] == 0xa1 && g_sent[i][1] == report_id) {
            return i;
        }
    }
    return -1;
}

/* --------------------------------------------------------------- tests -- */

static void boot_shared(void)
{
    wiimote_proto_init(&g_proto, WIIMOTE_EXT_CLASSIC, 0xc0);
    wiimote_btstack_set_fallback_handler(fake_bp32);
    wiimote_bt_set_external_stack(true);
    CHECK(wiimote_bt_start() == ESP_OK);
    CHECK(g_hci_handler != NULL && g_filter != NULL);
    CHECK(g_services[0] == wm_l2cap_handler && g_services[1] == wm_l2cap_handler);
    ev_state(HCI_STATE_WORKING);
    CHECK(g_record_count == 2);
    CHECK(de_get_len(s_hid_record) < sizeof(s_hid_record));
    CHECK(g_connectable == 1);
}

static void test_sdp_record(void)
{
    /* The HID record must carry the attributes a remote has, in order. */
    static const uint16_t want[] = {0x0000, 0x0001, 0x0004, 0x0005, 0x0006, 0x0009, 0x000d, 0x0100, 0x0101,
                                    0x0102, 0x0200, 0x0201, 0x0202, 0x0203, 0x0204, 0x0205, 0x0206, 0x0207,
                                    0x0208, 0x0209, 0x020a, 0x020b, 0x020c, 0x020d, 0x020e};
    des_iterator_t it;
    size_t n = 0;
    uint16_t prev = 0;
    bool first = true;
    for (des_iterator_init(&it, s_hid_record); des_iterator_has_more(&it); des_iterator_next(&it)) {
        uint16_t id;
        CHECK(de_element_get_uint16(des_iterator_get_element(&it), &id));
        CHECK(n < sizeof(want) / sizeof(want[0]) && id == want[n]);
        CHECK(first || id > prev);
        prev = id;
        first = false;
        n++;
        des_iterator_next(&it); /* value */
        if (!des_iterator_has_more(&it)) {
            break;
        }
    }
    CHECK(n == sizeof(want) / sizeof(want[0]));
    {
        uint8_t *desc = sdp_get_attribute_value_for_attribute_id(s_hid_record, 0x0206);
        CHECK(desc != NULL);
        CHECK(de_get_len(desc) > sizeof(k_rvl_hid_descriptor));
    }
}

static void test_wii_sync_and_hid(void)
{
    uint8_t ev;
    int bp32_conn_req_before = g_bp32_hci_events[HCI_EVENT_CONNECTION_REQUEST];

    post_request(WM_REQ_SYNC);
    run_main_thread();
    CHECK(g_discoverable == 1);

    ev = ev_conn_request(WII, 0x000448);
    CHECK(g_policy == HCI_ROLE_SLAVE);              /* stay slave, like a remote */
    CHECK(ev == HCI_EVENT_NOP);                      /* hidden from Bluepad32 */
    CHECK(g_bp32_hci_events[HCI_EVENT_CONNECTION_REQUEST] == bp32_conn_req_before);
    ev_conn_complete(WII, 0x0081, 0);
    CHECK(s_link_handle == 0x0081);
    CHECK(g_security_requests == 0);                 /* console is never challenged */

    ev = ev_pin_request(WII);
    CHECK(ev == HCI_EVENT_NOP);
    CHECK(g_pin_len == 6);
    for (int i = 0; i < 6; i++) {
        CHECK(g_pin[i] == WII[5 - i]);              /* host address backwards */
    }

    g_fallback_calls = 0;
    l2cap_incoming(WII, 0x0081, WM_PSM_CONTROL, 0x41);
    l2cap_opened(WII, 0x0081, WM_PSM_CONTROL, 0x41, 0);
    CHECK(!g_connected);
    l2cap_incoming(WII, 0x0081, WM_PSM_INTERRUPT, 0x42);
    l2cap_opened(WII, 0x0081, WM_PSM_INTERRUPT, 0x42, 0);
    CHECK(g_accepts == 2);
    CHECK(g_fallback_calls == 0);
    CHECK(g_connected && g_connected_events == 1);
    CHECK(g_discoverable == 0);                     /* sync window closed */
    CHECK(g_tlv_have && memcmp(g_tlv_data, WII, 6) == 0);
    CHECK(s_have_host);
    CHECK(g_timer_armed);
    CHECK(g_role_requests == 0);                    /* never ask the Wii to switch */

    /* Status request -> 0x20 with the extension bit. */
    g_sent_count = 0;
    {
        const uint8_t req[] = {0xa2, 0x15, 0x00};
        l2cap_data(0x42, req, sizeof(req));
    }
    {
        int i = find_sent(0x20);
        CHECK(i >= 0);
        if (i >= 0) {
            CHECK(g_sent_cid[i] == 0x42);
            CHECK(g_sent[i][4] & 0x02);
        }
    }

    /* LEDs report reaches the core. */
    {
        const uint8_t leds[] = {0xa2, 0x11, 0x10};
        l2cap_data(0x42, leds, sizeof(leds));
        CHECK(g_leds_seen == 0x01);
    }

    /* Extension init + identify, with the link busy for one round. */
    g_sent_count = 0;
    g_can_send = false;
    {
        const uint8_t w1[] = {0xa2, 0x16, 0x04, 0xa4, 0x00, 0xf0, 0x01, 0x55, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        l2cap_data(0x42, w1, sizeof(w1));
    }
    CHECK(g_sent_count == 0);
    CHECK(g_can_send_requests > 0);                 /* a queued ack asks to be woken */
    g_can_send = true;
    {
        uint8_t can[4] = {L2CAP_EVENT_CAN_SEND_NOW, 2};
        little_endian_store_16(can, 2, 0x42);
        wm_l2cap_handler(HCI_EVENT_PACKET, 0x42, can, sizeof(can));
    }
    CHECK(find_sent(0x22) >= 0);
    g_sent_count = 0;
    {
        const uint8_t rd[] = {0xa2, 0x17, 0x04, 0xa4, 0x00, 0xfa, 0x00, 0x06};
        l2cap_data(0x42, rd, sizeof(rd));
        int i = find_sent(0x21);
        CHECK(i >= 0);
        if (i >= 0) {
            static const uint8_t id[6] = {0x00, 0x00, 0xa4, 0x20, 0x01, 0x01};
            CHECK(memcmp(&g_sent[i][7], id, 6) == 0);
        }
    }

    /* Report mode 0x32 -> core + classic report on the next tick. */
    g_sent_count = 0;
    {
        const uint8_t mode[] = {0xa2, 0x12, 0x00, 0x32};
        l2cap_data(0x42, mode, sizeof(mode));
        CHECK(find_sent(0x32) >= 0);
    }

    /* SET_REPORT on the control channel gets a handshake. */
    g_sent_count = 0;
    {
        const uint8_t sr[] = {0x52, 0x11, 0x20};
        l2cap_data(0x41, sr, sizeof(sr));
        CHECK(g_sent_count >= 1 && g_sent_cid[0] == 0x41 && g_sent[0][0] == 0x00);
        CHECK(g_leds_seen == 0x02);
    }
}

static void test_gamepad_passthrough(void)
{
    uint8_t ev;
    int sec_before = g_security_requests;
    g_fallback_calls = 0;

    /* Bonded gamepad pages us while the Wii is connected. */
    memcpy(g_key_addr, PAD, 6);
    g_have_key = true;
    ev = ev_conn_request(PAD, 0x002508);
    CHECK(ev == HCI_EVENT_CONNECTION_REQUEST);       /* visible to Bluepad32 */
    CHECK(g_policy == HCI_ROLE_MASTER);
    ev = ev_conn_complete(PAD, 0x0082, 0);
    CHECK(ev == HCI_EVENT_CONNECTION_COMPLETE);
    /* Left alone while Bluepad32 sets it up: no auth, role, policy or QoS. */
    CHECK(g_security_requests == sec_before);
    CHECK(g_role_requests == 0);
    CHECK(s_qos_inflight_handle == HCI_CON_HANDLE_INVALID && s_policy_count == 0);

    ev = ev_pin_request(PAD);
    CHECK(ev == HCI_EVENT_PIN_CODE_REQUEST);         /* Bluepad32 answers this one */

    /* The pad opens HID control before the link is encrypted: held back
     * until it reaches Bluepad32's level 2, like Bluepad32's own service. */
    g_link_level = LEVEL_0;
    l2cap_incoming(PAD, 0x0082, WM_PSM_CONTROL, 0x50);
    CHECK(g_fallback_calls == 0);
    CHECK(g_security_requests == sec_before + 1 && g_security_handle == 0x0082);
    ev_security_level(0x0082, LEVEL_2);
    CHECK(g_fallback_calls == 1 && g_fallback_last_event == L2CAP_EVENT_INCOMING_CONNECTION);
    /* Already secure: interrupt goes straight through. */
    g_link_level = LEVEL_2;
    l2cap_incoming(PAD, 0x0082, WM_PSM_INTERRUPT, 0x51);
    CHECK(g_fallback_calls == 2 && g_security_requests == sec_before + 1);
    g_fallback_calls = 1;
    g_link_level = LEVEL_0;

    /* Ready: the application tunes the link. */
    g_role = HCI_ROLE_MASTER;
    wiimote_btstack_tune_link(0x0082, PAD);
    CHECK(g_role_requests == 0);
    /* Short poll interval asked for the gamepad link. */
    CHECK(s_qos_inflight_handle == 0x0082 && s_qos_inflight_type == WM_QOS_GUARANTEED);
    {
        /* Refused: one retry as best effort, then success clears it. */
        uint8_t qe[23] = {HCI_EVENT_QOS_SETUP_COMPLETE, 21, 0x11, 0x82, 0x00};
        int c2 = g_cmds_sent;
        wm_hci_handler(HCI_EVENT_PACKET, 0, qe, sizeof(qe));
        CHECK(g_cmds_sent == c2 + 1 && s_qos_inflight_type == WM_QOS_BEST_EFFORT);
        qe[2] = 0;
        wm_hci_handler(HCI_EVENT_PACKET, 0, qe, sizeof(qe));
        CHECK(g_cmds_sent == c2 + 1 && s_qos_inflight_handle == HCI_CON_HANDLE_INVALID);
    }

    l2cap_opened(PAD, 0x0082, WM_PSM_CONTROL, 0x50, 0);
    CHECK(g_fallback_calls == 2);
    {
        const uint8_t in[] = {0xa1, 0x01, 0x80, 0x80};
        l2cap_data(0x50, in, sizeof(in));
        CHECK(g_fallback_calls == 3);
    }
    l2cap_closed(0x50);
    CHECK(g_fallback_calls == 4);
    CHECK(g_connected);                              /* the Wii link is untouched */
    ev_disconnect(0x0082);
    CHECK(g_connected);

    /* A brand-new gamepad (no key) outside the sync window: become master,
     * as Bluepad32 would. */
    g_have_key = false;
    {
        static const bd_addr_t NEWPAD = {0x77, 0x66, 0x55, 0x44, 0x33, 0x22};
        ev = ev_conn_request(NEWPAD, 0x000508);
        CHECK(g_policy == HCI_ROLE_MASTER);
        CHECK(ev == HCI_EVENT_CONNECTION_REQUEST);
        ev_conn_complete(NEWPAD, 0x0083, 0);
        CHECK(g_role_requests == 0);
        /* A channel held for security is dropped with the link. */
        int calls = g_fallback_calls;
        l2cap_incoming(NEWPAD, 0x0083, WM_PSM_CONTROL, 0x52);
        ev_disconnect(0x0083);
        ev_security_level(0x0083, LEVEL_2);
        CHECK(g_fallback_calls == calls);
        /* Tuning a slave link asks for master. */
        g_role = HCI_ROLE_SLAVE;
        wiimote_btstack_tune_link(0x0083, NEWPAD);
        CHECK(g_role_requests == 1);
    }
    g_role_requests = 0;

    /* A gamepad with an odd (non-peripheral) class of device outside the
     * sync window still belongs to Bluepad32. */
    {
        static const bd_addr_t ODDPAD = {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc};
        ev = ev_conn_request(ODDPAD, 0x000104);
        CHECK(ev == HCI_EVENT_CONNECTION_REQUEST);
        CHECK(!(s_link_claimed && memcmp(s_link_addr, ODDPAD, 6) == 0));
    }
}

static void test_wii_disconnect_and_reconnect(void)
{
    int disc_before = g_disconnected_events;
    ev_disconnect(0x0081);
    CHECK(!g_connected && g_disconnected_events == disc_before + 1);
    CHECK(!s_link_claimed && !g_timer_armed);
    CHECK(s_have_host);

    /* Page the Wii: control first, then interrupt. */
    g_created = 0;
    CHECK(wiimote_bt_connect() == ESP_OK);
    run_main_thread();
    CHECK(g_created == 1 && g_created_psm[0] == WM_PSM_CONTROL);
    CHECK(g_level_at_create == LEVEL_0);             /* the console authenticates, not us */
    CHECK(g_gap_level == LEVEL_2);                   /* host stack level restored */
    CHECK(wiimote_bt_is_connecting());
    CHECK(!g_allow_role_switch);                     /* page the Wii as master only */
    {
        uint16_t ctrl = s_cid_control;
        int cmds = g_cmds_sent;
        uint8_t ev = ev_conn_complete(WII, 0x0090, 0);
        CHECK(g_allow_role_switch);                  /* restored for everyone else */
        CHECK(g_cmds_sent == cmds + 1);              /* link policy: no switch later */
        CHECK(ev == HCI_EVENT_CONNECTION_COMPLETE);  /* L2CAP needs to see it */
        l2cap_opened(WII, 0x0090, WM_PSM_CONTROL, ctrl, 0);
        CHECK(g_created == 2 && g_created_psm[1] == WM_PSM_INTERRUPT);
        CHECK(!g_connected);
        l2cap_opened(WII, 0x0090, WM_PSM_INTERRUPT, s_cid_interrupt, 0);
        CHECK(g_connected);
    }

    /* Console closes both channels: we drop the baseband link like a remote. */
    {
        int d = g_disconnects;
        l2cap_closed(s_cid_interrupt);
        CHECK(!g_connected);
        l2cap_closed(s_cid_control);
        CHECK(g_disconnects == d + 1);
        ev_disconnect(0x0090);
        CHECK(!s_link_claimed);
    }

    /* Console is off: page times out. */
    CHECK(wiimote_bt_connect() == ESP_OK);
    run_main_thread();
    {
        uint16_t ctrl = s_cid_control;
        uint8_t ev = ev_conn_complete(WII, 0, 0x04);
        (void)ev;
        CHECK(s_link_claimed);
        l2cap_opened(WII, HCI_CON_HANDLE_INVALID, WM_PSM_CONTROL, ctrl, 0x04);
        CHECK(!s_link_claimed && !g_connected);
    }

    /* After a failed page the Wii can still page us (sync again). */
    {
        uint8_t ev = ev_conn_request(WII, 0x000448);
        CHECK(ev == HCI_EVENT_NOP);
        CHECK(g_policy == HCI_ROLE_SLAVE);
        ev_conn_complete(WII, 0x0091, 0);
        l2cap_incoming(WII, 0x0091, WM_PSM_CONTROL, 0x45);
        l2cap_incoming(WII, 0x0091, WM_PSM_INTERRUPT, 0x46);
        l2cap_opened(WII, 0x0091, WM_PSM_INTERRUPT, 0x46, 0); /* interrupt finishes first */
        CHECK(!g_connected);
        l2cap_opened(WII, 0x0091, WM_PSM_CONTROL, 0x45, 0);
        CHECK(g_connected);
    }
}

static void test_connect_watchdog(void)
{
    /* Disconnect the current link, then page a console whose page the
     * controller "forgets": no completion event ever arrives. */
    ev_disconnect(0x0091);
    CHECK(!s_link_claimed);
    CHECK(wiimote_bt_connect() == ESP_OK);
    run_main_thread();
    CHECK(s_link_claimed && g_watchdog_armed);
    {
        int cmds = g_cmds_sent;
        s_connect_watchdog.process(&s_connect_watchdog);
        CHECK(!s_link_claimed);
        CHECK(g_cmds_sent == cmds + 1);              /* create_connection_cancel */
    }
    g_role_requests = 0;
    /* A later attempt starts a fresh page instead of being ignored. */
    g_created = 0;
    CHECK(wiimote_bt_connect() == ESP_OK);
    run_main_thread();
    CHECK(g_created == 1 && s_link_claimed);
    CHECK(g_page_timeout == WM_PAGE_TIMEOUT_CONSOLE);  /* short page for the Wii */
    /* It works this time; the watchdog is disarmed once the link is up. */
    ev_conn_complete(WII, 0x0092, 0);
    CHECK(g_page_timeout == WM_PAGE_TIMEOUT_DEFAULT);  /* gamepads page as usual */
    l2cap_opened(WII, 0x0092, WM_PSM_CONTROL, s_cid_control, 0);
    l2cap_opened(WII, 0x0092, WM_PSM_INTERRUPT, s_cid_interrupt, 0);
    CHECK(g_connected && !g_watchdog_armed);
    CHECK(g_role_requests == 0);                     /* we paged: already master */
}

static void test_forget(void)
{
    CHECK(wiimote_bt_forget_host() == ESP_OK);
    run_main_thread();
    CHECK(!s_have_host && !g_tlv_have);
    CHECK(wiimote_bt_connect() == ESP_ERR_NOT_FOUND);
    /* The disconnect is pending until the controller reports it. */
    ev_disconnect(0x0092);
    CHECK(!g_connected);
}

int main(int argc, char **argv)
{
    g_verbose = argc > 1 && strcmp(argv[1], "-v") == 0;
    boot_shared();
    test_sdp_record();
    test_wii_sync_and_hid();
    test_gamepad_passthrough();
    test_wii_disconnect_and_reconnect();
    test_connect_watchdog();
    test_forget();
    if (s_failures) {
        printf("%d backend check(s) failed\n", s_failures);
        return 1;
    }
    printf("btstack backend tests passed\n");
    return 0;
}
