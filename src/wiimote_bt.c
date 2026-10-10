/* SPDX-License-Identifier: MIT */
/*
 * Bluetooth Classic HID peripheral built on public Bluedroid L2CAP and SDP
 * entry points. Arduino-ESP32 does not link the HID device profile, so the
 * control (PSM 0x11) and interrupt (PSM 0x13) channels are registered here.
 * L2CAP calls run on the BTU task.
 */
#include "wiimote_bt.h"

#include "wiimote_internal.h"
#include "wiimote_target.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#if defined(ARDUINO)
/* Arduino 3.x frees Classic and BLE controller pools in initArduino() unless
 * a constructor marks that pool in use. A BTDM prebuilt (3.3.12) needs both. */
#if __has_include("esp32-hal-alloc-bt-classic-mem.h")
#include "esp32-hal-alloc-bt-classic-mem.h"
#endif
#if __has_include("esp32-hal-alloc-ble-mem.h")
#include "esp32-hal-alloc-ble-mem.h"
#endif
#endif

#define L2CAP_MIN_OFFSET 13
#define BTU_HACK_EVENT 0x1700
#define SIG_BTU_HCI_MSG 1

typedef struct {
    uint16_t event;
    uint16_t len;
    uint16_t offset;
    uint16_t layer_specific;
    uint8_t data[];
} wm_bt_hdr_t;

typedef struct {
    void (*connect_ind)(uint8_t *bda, uint16_t cid, uint16_t psm, uint8_t id);
    void (*connect_cfm)(uint16_t cid, uint16_t result);
    void (*connect_pnd)(uint16_t cid);
    void (*config_ind)(uint16_t cid, void *cfg);
    void (*config_cfm)(uint16_t cid, void *cfg);
    void (*disconnect_ind)(uint16_t cid, uint8_t ack_needed);
    void (*disconnect_cfm)(uint16_t cid, uint16_t result);
    void (*qos_violation)(uint8_t *bda);
    void (*data_ind)(uint16_t cid, wm_bt_hdr_t *msg);
    void (*congestion)(uint16_t cid, uint8_t congested);
    void (*tx_complete)(uint16_t cid, uint16_t count);
} wm_l2cap_appl_t;

typedef struct {
    uint16_t protocol_uuid;
    uint16_t num_params;
    uint16_t params[2];
} wm_sdp_proto_t;

typedef struct {
    uint16_t num_elems;
    wm_sdp_proto_t list_elem[3];
} wm_sdp_proto_list_t;

extern uint16_t L2CA_Register(uint16_t psm, wm_l2cap_appl_t *cb);
extern void L2CA_Deregister(uint16_t psm);
extern uint8_t L2CA_ConnectRsp(uint8_t *bda, uint8_t id, uint16_t cid, uint16_t result, uint16_t status);
extern uint8_t L2CA_ConfigReq(uint16_t cid, void *cfg);
extern uint8_t L2CA_ConfigRsp(uint16_t cid, void *cfg);
extern uint8_t L2CA_DisconnectReq(uint16_t cid);
extern uint8_t L2CA_DisconnectRsp(uint16_t cid);
extern uint8_t L2CA_DataWrite(uint16_t cid, wm_bt_hdr_t *msg);
extern uint8_t L2CA_SetIdleTimeout(uint16_t cid, uint16_t timeout, uint8_t is_global);
extern uint32_t SDP_CreateRecord(void);
extern uint8_t SDP_AddAttribute(uint32_t handle, uint16_t attr_id, uint8_t attr_type, uint32_t attr_len, uint8_t *value);
extern uint8_t SDP_AddProtocolList(uint32_t handle, uint16_t num_elem, wm_sdp_proto_t *list);
extern uint8_t SDP_AddAdditionProtoLists(uint32_t handle, uint16_t num_elem, wm_sdp_proto_list_t *list);
extern uint8_t SDP_AddProfileDescriptorList(uint32_t handle, uint16_t profile_uuid, uint16_t version);
extern uint8_t SDP_AddLanguageBaseAttrIDList(uint32_t handle, uint16_t lang, uint16_t char_enc, uint16_t base_id);
extern uint8_t SDP_AddServiceClassIdList(uint32_t handle, uint16_t num_services, uint16_t *uuids);
extern uint8_t SDP_AddUuidSequence(uint32_t handle, uint16_t attr_id, uint16_t num_uuids, uint16_t *uuids);
extern uint8_t BTM_SetSecurityLevel(uint8_t is_originator, const char *name, uint8_t service_id,
                                    uint16_t sec_level, uint16_t psm, uint32_t proto_id, uint32_t chan_id);
extern bool btu_task_post(uint32_t sig, void *param, uint32_t timeout);
/* Legacy security mode. Bluedroid boots in SSP (mode 4); the Wii drops that. */
extern void btm_sec_init(uint8_t sec_mode);
extern uint8_t BTM_SecBond(uint8_t *bd_addr, uint8_t pin_len, uint8_t *pin, uint32_t *trusted_mask);

static const char *TAG = "wiimote";
static wm_l2cap_appl_t s_appl;
static uint16_t s_cid_control;
static uint16_t s_cid_interrupt;
static uint8_t s_bda[6];
static uint8_t s_cfg_ind_control;
static uint8_t s_cfg_cfm_control;
static uint8_t s_cfg_ind_interrupt;
static uint8_t s_cfg_cfm_interrupt;
static volatile int s_pump_queued;
static esp_timer_handle_t s_timer;
static uint8_t s_syncing;
static uint8_t s_paging;
static uint8_t s_inq_limited;
static uint8_t s_name_pending;
static uint8_t s_name_bda[6];
static uint8_t s_page_bda[6];
static uint8_t s_last_leds;
static uint8_t s_have_leds;
static uint8_t s_last_rumble;
static uint8_t s_have_rumble;

static bool post_btu(void (*fn)(wm_bt_hdr_t *));

static void zero_cfg(uint8_t *buf, size_t n)
{
    memset(buf, 0, n);
}

static void send_packet(uint16_t cid, const uint8_t *data, int len)
{
    wm_bt_hdr_t *msg;
    if (cid == 0 || len <= 0) {
        return;
    }
    msg = (wm_bt_hdr_t *)calloc(1, sizeof(wm_bt_hdr_t) + L2CAP_MIN_OFFSET + (size_t)len);
    if (msg == NULL) {
        return;
    }
    msg->offset = L2CAP_MIN_OFFSET;
    msg->len = (uint16_t)len;
    memcpy(msg->data + L2CAP_MIN_OFFSET, data, (size_t)len);
    /* The stack frees the buffer on both success and failure. */
    L2CA_DataWrite(cid, msg);
}

static void note_outputs(uint8_t leds, bool rumble)
{
    if (!s_have_leds || leds != s_last_leds) {
        s_last_leds = leds;
        s_have_leds = 1;
        wiimote_note_leds(leds);
    }
    if (!s_have_rumble || (uint8_t)rumble != s_last_rumble) {
        s_last_rumble = rumble ? 1 : 0;
        s_have_rumble = 1;
        wiimote_note_rumble(rumble);
    }
}

static void flush_reports(void)
{
    uint8_t pkt[32];
    int n;
    uint8_t leds;
    bool rumble;
    uint32_t now;
    if (!wiimote_ready() || s_cid_interrupt == 0 || !s_cfg_cfm_interrupt || !s_cfg_ind_interrupt) {
        return;
    }
    now = (uint32_t)(esp_timer_get_time() / 1000);
    wiimote_lock();
    for (;;) {
        n = wiimote_proto_poll(wiimote_state(), pkt, sizeof(pkt), now);
        if (n <= 0) {
            break;
        }
        leds = wiimote_state()->leds;
        rumble = wiimote_state()->rumble;
        wiimote_unlock();
        send_packet(s_cid_interrupt, pkt, n);
        note_outputs(leds, rumble);
        wiimote_lock();
    }
    leds = wiimote_state()->leds;
    rumble = wiimote_state()->rumble;
    wiimote_unlock();
    note_outputs(leds, rumble);
}

static void mark_open(uint16_t cid, int is_cfm)
{
    if (cid == s_cid_control) {
        if (is_cfm) s_cfg_cfm_control = 1;
        else s_cfg_ind_control = 1;
    } else if (cid == s_cid_interrupt) {
        if (is_cfm) s_cfg_cfm_interrupt = 1;
        else s_cfg_ind_interrupt = 1;
        if (s_cfg_cfm_interrupt && s_cfg_ind_interrupt) {
            L2CA_SetIdleTimeout(cid, 0xffff, 0);
            wiimote_set_connected(true);
            ESP_LOGI(TAG, "interrupt channel open");
        }
    }
}

static void on_connect_ind(uint8_t *bda, uint16_t cid, uint16_t psm, uint8_t id)
{
    uint8_t cfg[512];
    memcpy(s_bda, bda, 6);
    ESP_LOGE(TAG, "L2CAP PSM 0x%04x cid %u from %02x:%02x:%02x:%02x:%02x:%02x",
             psm, cid, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    if (psm == 0x0011) {
        s_cid_control = cid;
        s_cfg_ind_control = 0;
        s_cfg_cfm_control = 0;
    } else if (psm == 0x0013) {
        s_cid_interrupt = cid;
        s_cfg_ind_interrupt = 0;
        s_cfg_cfm_interrupt = 0;
    }
    L2CA_ConnectRsp(bda, id, cid, 0, 0);
    zero_cfg(cfg, sizeof(cfg));
    L2CA_ConfigReq(cid, cfg);
}

static void on_connect_cfm(uint16_t cid, uint16_t result)
{
    (void)cid;
    (void)result;
}

static void on_connect_pnd(uint16_t cid)
{
    (void)cid;
}

static void on_config_ind(uint16_t cid, void *cfg)
{
    uint8_t rsp[512];
    (void)cfg;
    zero_cfg(rsp, sizeof(rsp));
    L2CA_ConfigRsp(cid, rsp);
    mark_open(cid, 0);
}

static void on_config_cfm(uint16_t cid, void *cfg)
{
    (void)cfg;
    mark_open(cid, 1);
}

static void drop_cid(uint16_t cid)
{
    int was = s_cfg_cfm_interrupt && s_cfg_ind_interrupt;
    if (cid == s_cid_control) {
        s_cid_control = 0;
        s_cfg_ind_control = 0;
        s_cfg_cfm_control = 0;
    }
    if (cid == s_cid_interrupt) {
        s_cid_interrupt = 0;
        s_cfg_ind_interrupt = 0;
        s_cfg_cfm_interrupt = 0;
    }
    if (was && s_cid_interrupt == 0) {
        wiimote_set_connected(false);
        ESP_LOGI(TAG, "interrupt channel closed");
    }
}

static void on_disconnect_ind(uint16_t cid, uint8_t ack_needed)
{
    ESP_LOGE(TAG, "L2CAP disconnect indication cid %u", cid);
    if (ack_needed) {
        L2CA_DisconnectRsp(cid);
    }
    drop_cid(cid);
}

static void on_disconnect_cfm(uint16_t cid, uint16_t result)
{
    ESP_LOGE(TAG, "L2CAP disconnect confirm cid %u result %u", cid, result);
    drop_cid(cid);
}

static void on_qos(uint8_t *bda)
{
    (void)bda;
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
            send_packet(s_cid_interrupt ? s_cid_interrupt : s_cid_control, built, n);
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

static void on_data(uint16_t cid, wm_bt_hdr_t *msg)
{
    uint8_t copy[64];
    size_t n;
    (void)cid;
    if (msg == NULL) {
        return;
    }
    n = msg->len;
    if (n > sizeof(copy)) {
        n = sizeof(copy);
    }
    if (msg->offset + n <= msg->offset) {
        free(msg);
        return;
    }
    memcpy(copy, msg->data + msg->offset, n);
    free(msg);
    handle_hid(copy, n);
}

static void on_cong(uint16_t cid, uint8_t congested)
{
    (void)cid;
    (void)congested;
}

static void on_tx(uint16_t cid, uint16_t count)
{
    (void)cid;
    (void)count;
}

static int hid_descriptor(uint8_t *dst, int cap)
{
    static const uint8_t out_id[] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a};
    static const uint8_t out_n[] = {1, 1, 2, 1, 1, 1, 21, 6, 21, 1, 1};
    static const uint8_t in_id[] = {0x20, 0x21, 0x22, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x3d, 0x3e, 0x3f};
    static const uint8_t in_n[] = {6, 21, 4, 2, 5, 10, 17, 21, 21, 21, 21, 21, 21, 21};
    int n = 0;
    size_t i;
    if (cap < 220) {
        return 0;
    }
    dst[n++] = 0x05; dst[n++] = 0x01;
    dst[n++] = 0x09; dst[n++] = 0x05;
    dst[n++] = 0xa1; dst[n++] = 0x01;
    for (i = 0; i < sizeof(out_id); i++) {
        dst[n++] = 0x85; dst[n++] = out_id[i];
        dst[n++] = 0x75; dst[n++] = 0x08;
        dst[n++] = 0x95; dst[n++] = out_n[i];
        dst[n++] = 0x91; dst[n++] = 0x02;
    }
    for (i = 0; i < sizeof(in_id); i++) {
        dst[n++] = 0x85; dst[n++] = in_id[i];
        dst[n++] = 0x75; dst[n++] = 0x08;
        dst[n++] = 0x95; dst[n++] = in_n[i];
        dst[n++] = 0x81; dst[n++] = 0x02;
    }
    dst[n++] = 0xc0;
    return n;
}

static void add_u16(uint32_t handle, uint16_t attr, uint16_t value)
{
    uint8_t be[2] = {(uint8_t)(value >> 8), (uint8_t)value};
    SDP_AddAttribute(handle, attr, 1, 2, be);
}

static void add_u8(uint32_t handle, uint16_t attr, uint8_t value)
{
    SDP_AddAttribute(handle, attr, 1, 1, &value);
}

static void add_bool(uint32_t handle, uint16_t attr, uint8_t value)
{
    SDP_AddAttribute(handle, attr, 5, 1, &value);
}

static void add_hid_record(void)
{
    uint32_t handle = SDP_CreateRecord();
    uint16_t hid = 0x1124;
    uint8_t desc[220];
    int desc_len;
    uint8_t *list;
    uint8_t *p;
    uint8_t seq_len;
    uint8_t name[] = "Nintendo RVL-CNT-01";
    uint16_t browse = 0x1002;
    wm_sdp_proto_t proto[2];
    wm_sdp_proto_list_t extra;
    uint8_t lang[8];
    if (handle == 0) {
        ESP_LOGE(TAG, "SDP_CreateRecord failed");
        return;
    }
    SDP_AddServiceClassIdList(handle, 1, &hid);
    memset(proto, 0, sizeof(proto));
    proto[0].protocol_uuid = 0x0100;
    proto[0].num_params = 1;
    proto[0].params[0] = 0x0011;
    proto[1].protocol_uuid = 0x0011;
    proto[1].num_params = 0;
    SDP_AddProtocolList(handle, 2, proto);
    SDP_AddLanguageBaseAttrIDList(handle, 0x656e, 0x006a, 0x0100);
    memset(&extra, 0, sizeof(extra));
    extra.num_elems = 2;
    extra.list_elem[0].protocol_uuid = 0x0100;
    extra.list_elem[0].num_params = 1;
    extra.list_elem[0].params[0] = 0x0013;
    extra.list_elem[1].protocol_uuid = 0x0011;
    SDP_AddAdditionProtoLists(handle, 1, &extra);
    SDP_AddAttribute(handle, 0x0100, 4, sizeof(name), name);
    SDP_AddProfileDescriptorList(handle, hid, 0x0100);
    add_u16(handle, 0x0200, 0x0100);
    add_u16(handle, 0x0201, 0x0111);
    add_u8(handle, 0x0202, 0x08);
    add_u8(handle, 0x0203, 0x21);
    add_bool(handle, 0x0204, 1);
    add_bool(handle, 0x0205, 1);
    desc_len = hid_descriptor(desc, (int)sizeof(desc));
    seq_len = (uint8_t)(4 + desc_len);
    list = (uint8_t *)malloc((size_t)seq_len + 2);
    if (list != NULL) {
        p = list;
        *p++ = (uint8_t)((6 << 3) | 5);
        *p++ = seq_len;
        *p++ = (uint8_t)((1 << 3) | 0);
        *p++ = 0x22;
        *p++ = (uint8_t)((4 << 3) | 5);
        *p++ = (uint8_t)desc_len;
        memcpy(p, desc, (size_t)desc_len);
        SDP_AddAttribute(handle, 0x0206, 6, (uint32_t)(seq_len + 2), list);
        free(list);
    }
    p = lang;
    *p++ = (uint8_t)((6 << 3) | 5);
    *p++ = 6;
    *p++ = (uint8_t)((1 << 3) | 1);
    *p++ = 0x04; *p++ = 0x09;
    *p++ = (uint8_t)((1 << 3) | 1);
    *p++ = 0x01; *p++ = 0x00;
    SDP_AddAttribute(handle, 0x0207, 6, (uint32_t)(p - lang), lang);
    add_bool(handle, 0x0209, 1);
    add_bool(handle, 0x020a, 1);
    add_u16(handle, 0x020b, 0x0100);
    add_u16(handle, 0x020c, 0x1f40);
    add_bool(handle, 0x020d, 1);
    add_bool(handle, 0x020e, 0);
    SDP_AddUuidSequence(handle, 0x0005, 1, &browse);
}

static void add_device_id_record(void)
{
    uint32_t handle = SDP_CreateRecord();
    uint16_t pnp = 0x1200;
    if (handle == 0) {
        return;
    }
    SDP_AddServiceClassIdList(handle, 1, &pnp);
    add_u16(handle, 0x0200, 0x0103);
    add_u16(handle, 0x0201, 0x057e);
    add_u16(handle, 0x0202, 0x0306);
    add_u16(handle, 0x0203, 0x0100);
    add_bool(handle, 0x0204, 1);
    add_u16(handle, 0x0205, 0x0002);
}

static void allow_hid_security(void)
{
    /* No extra link security on the HID PSMs; the Wii bonds with the reversed BD_ADDR PIN. */
    BTM_SetSecurityLevel(0, "HID", 51, 0, 0x0011, 6, 0);
    BTM_SetSecurityLevel(1, "HID", 51, 0, 0x0011, 6, 0);
    BTM_SetSecurityLevel(0, "HID", 52, 0, 0x0011, 6, 1);
    BTM_SetSecurityLevel(1, "HID", 52, 0, 0x0011, 6, 1);
    BTM_SetSecurityLevel(0, "HID", 53, 0, 0x0013, 6, 0);
    BTM_SetSecurityLevel(1, "HID", 53, 0, 0x0013, 6, 0);
}

static void register_cb(wm_bt_hdr_t *msg)
{
    free(msg);
    memset(&s_appl, 0, sizeof(s_appl));
    s_appl.connect_ind = on_connect_ind;
    s_appl.connect_cfm = on_connect_cfm;
    s_appl.connect_pnd = on_connect_pnd;
    s_appl.config_ind = on_config_ind;
    s_appl.config_cfm = on_config_cfm;
    s_appl.disconnect_ind = on_disconnect_ind;
    s_appl.disconnect_cfm = on_disconnect_cfm;
    s_appl.qos_violation = on_qos;
    s_appl.data_ind = on_data;
    s_appl.congestion = on_cong;
    s_appl.tx_complete = on_tx;
    allow_hid_security();
    add_hid_record();
    add_device_id_record();
    L2CA_Register(0x0011, &s_appl);
    L2CA_Register(0x0013, &s_appl);
    ESP_LOGI(TAG, "L2CAP PSM 0x11 and 0x13 registered");
}

static void pump_cb(wm_bt_hdr_t *msg)
{
    free(msg);
    s_pump_queued = 0;
    flush_reports();
}

static void stop_cb(wm_bt_hdr_t *msg)
{
    free(msg);
    if (s_cid_interrupt) {
        L2CA_DisconnectReq(s_cid_interrupt);
    }
    if (s_cid_control) {
        L2CA_DisconnectReq(s_cid_control);
    }
}

static bool post_btu(void (*fn)(wm_bt_hdr_t *))
{
    wm_bt_hdr_t *msg = (wm_bt_hdr_t *)calloc(1, sizeof(wm_bt_hdr_t) + sizeof(fn));
    if (msg == NULL) {
        return false;
    }
    msg->event = BTU_HACK_EVENT;
    msg->len = (uint16_t)sizeof(fn);
    memcpy(msg->data, &fn, sizeof(fn));
    if (!btu_task_post(SIG_BTU_HCI_MSG, msg, 0xffffffffu)) {
        free(msg);
        return false;
    }
    return true;
}

static void timer_cb(void *arg)
{
    (void)arg;
    if (s_pump_queued) {
        return;
    }
    s_pump_queued = 1;
    if (!post_btu(pump_cb)) {
        s_pump_queued = 0;
    }
}

static void format_bda(const uint8_t *bda, char *out, size_t n)
{
    snprintf(out, n, "%02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

static int name_is_remote(const char *name)
{
    return strncmp(name, "Nintendo RVL-CNT-01", 19) == 0;
}

static int name_is_console(const char *name)
{
    if (name[0] == 0 || name_is_remote(name)) {
        return 0;
    }
    /* Wiibrew names the remote RVL-CNT-01. A console is the other RVL
     * product, or the short name captured from a console in sync. */
    return strncmp(name, "Nintendo RVL", 12) == 0 || strcmp(name, "Wii") == 0;
}

static void start_inquiry(void)
{
    esp_bt_inq_mode_t mode;
    esp_err_t err;
    if (!s_syncing || s_paging) {
        return;
    }
    mode = s_inq_limited ? ESP_BT_INQ_MODE_LIMITED_INQUIRY : ESP_BT_INQ_MODE_GENERAL_INQUIRY;
    err = esp_bt_gap_start_discovery(mode, 8, 0);
    ESP_LOGE(TAG, "inquiry %s: %s", s_inq_limited ? "LIAC" : "GIAC", esp_err_to_name(err));
}

static void page_on_btu(wm_bt_hdr_t *msg)
{
    uint8_t status;
    free(msg);
    status = BTM_SecBond(s_page_bda, 0, NULL, NULL);
    if (status != 0 && status != 1) {
        ESP_LOGE(TAG, "page failed, BTM_SecBond status %u", status);
        s_paging = 0;
        start_inquiry();
    }
}

static void page_console(const uint8_t *bda, const char *why)
{
    char addr[18];
    if (s_paging || bda == NULL) {
        return;
    }
    s_paging = 1;
    memcpy(s_page_bda, bda, 6);
    format_bda(s_page_bda, addr, sizeof(addr));
    ESP_LOGE(TAG, "paging Wii %s (%s)", addr, why);
    esp_bt_gap_cancel_discovery();
    /* BTM_SecBond runs on the BTU task. It creates the ACL and starts
     * legacy bonding; the PIN comes back as ESP_BT_GAP_PIN_REQ_EVT. */
    if (!post_btu(page_on_btu)) {
        ESP_LOGE(TAG, "page failed, BTU queue full");
        s_paging = 0;
        start_inquiry();
    }
}

static void copy_name(char *dst, size_t dst_n, const uint8_t *src, int len)
{
    size_t n;
    if (dst_n == 0) {
        return;
    }
    if (src == NULL || len <= 0) {
        dst[0] = 0;
        return;
    }
    n = (size_t)len;
    if (n >= dst_n) {
        n = dst_n - 1;
    }
    memcpy(dst, src, n);
    dst[n] = 0;
}

static void consider_peer(const uint8_t *bda, int have_cod, uint32_t cod, const char *name)
{
    char addr[18];
    uint32_t cls = cod & 0x00ffffffu;
    format_bda(bda, addr, sizeof(addr));
    ESP_LOGE(TAG, "inquiry %s cod %s0x%06x name \"%s\"",
             addr, have_cod ? "" : "?", (unsigned)cls, name ? name : "");
    if (!s_syncing || s_paging) {
        return;
    }
    /* 0x002504 is RVL-CNT-01, 0x000508 is RVL-CNT-01-TR. Both are remotes. */
    if (have_cod && (cls == 0x002504u || cls == 0x000508u)) {
        return;
    }
    if (have_cod && cls == 0x000408u) {
        page_console(bda, "class 0x000408");
        return;
    }
    if (name != NULL && name[0] != 0) {
        if (name_is_console(name)) {
            page_console(bda, name);
        }
        return;
    }
    if (!s_name_pending || memcmp(s_name_bda, bda, 6) != 0) {
        memcpy(s_name_bda, bda, 6);
        s_name_pending = 1;
        esp_bt_gap_read_remote_name((uint8_t *)bda);
    }
}

static void on_discovery_result(esp_bt_gap_cb_param_t *param)
{
    int have_cod = 0;
    uint32_t cod = 0;
    char name[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
    int i;
    name[0] = 0;
    for (i = 0; i < param->disc_res.num_prop; i++) {
        esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
        if (prop->type == ESP_BT_GAP_DEV_PROP_COD && prop->val != NULL && prop->len >= (int)sizeof(uint32_t)) {
            memcpy(&cod, prop->val, sizeof(cod));
            have_cod = 1;
        } else if (prop->type == ESP_BT_GAP_DEV_PROP_BDNAME && prop->val != NULL) {
            copy_name(name, sizeof(name), prop->val, prop->len);
        } else if (prop->type == ESP_BT_GAP_DEV_PROP_EIR && prop->val != NULL && name[0] == 0) {
            uint8_t elen = 0;
            uint8_t *eir = esp_bt_gap_resolve_eir_data(prop->val, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &elen);
            if (eir == NULL) {
                eir = esp_bt_gap_resolve_eir_data(prop->val, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &elen);
            }
            copy_name(name, sizeof(name), eir, elen);
        }
    }
    consider_peer(param->disc_res.bda, have_cod, cod, name[0] ? name : NULL);
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char addr[18];
    if (event == ESP_BT_GAP_PIN_REQ_EVT) {
        esp_bt_pin_code_t pin;
        int i;
        format_bda(param->pin_req.bda, addr, sizeof(addr));
        /* Host BD_ADDR, reversed, 6 bytes. Not an ASCII PIN and not SSP. */
        for (i = 0; i < 6; i++) {
            pin[i] = param->pin_req.bda[5 - i];
        }
        ESP_LOGE(TAG, "PIN request from %s, reply reversed BD_ADDR (6 bytes)", addr);
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 6, pin);
    } else if (event == ESP_BT_GAP_CFM_REQ_EVT) {
        format_bda(param->cfm_req.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "SSP confirm from %s rejected", addr);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, false);
    } else if (event == ESP_BT_GAP_KEY_REQ_EVT) {
        format_bda(param->key_req.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "SSP passkey request from %s rejected", addr);
        esp_bt_gap_ssp_passkey_reply(param->key_req.bda, false, 0);
    } else if (event == ESP_BT_GAP_KEY_NOTIF_EVT) {
        format_bda(param->key_notif.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "SSP passkey notification from %s ignored", addr);
    } else if (event == ESP_BT_GAP_AUTH_CMPL_EVT) {
        format_bda(param->auth_cmpl.bda, addr, sizeof(addr));
        if (param->auth_cmpl.stat != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "auth fail %s status %d", addr, (int)param->auth_cmpl.stat);
            if (s_paging) {
                s_paging = 0;
                start_inquiry();
            }
        } else {
            ESP_LOGE(TAG, "auth ok %s", addr);
        }
    } else if (event == ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT) {
        format_bda(param->acl_conn_cmpl_stat.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "ACL up %s handle %u status %d", addr,
                 param->acl_conn_cmpl_stat.handle, (int)param->acl_conn_cmpl_stat.stat);
        if (param->acl_conn_cmpl_stat.stat != ESP_BT_STATUS_SUCCESS && s_paging) {
            s_paging = 0;
            start_inquiry();
        }
    } else if (event == ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT) {
        format_bda(param->acl_disconn_cmpl_stat.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "ACL down %s handle %u reason %d", addr,
                 param->acl_disconn_cmpl_stat.handle, (int)param->acl_disconn_cmpl_stat.reason);
        if (s_paging && s_syncing) {
            s_paging = 0;
            start_inquiry();
        }
    } else if (event == ESP_BT_GAP_DISC_RES_EVT) {
        on_discovery_result(param);
    } else if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT) {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED && s_syncing && !s_paging) {
            s_inq_limited = !s_inq_limited;
            start_inquiry();
        }
    } else if (event == ESP_BT_GAP_READ_REMOTE_NAME_EVT) {
        s_name_pending = 0;
        if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS) {
            char name[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
            copy_name(name, sizeof(name), param->read_rmt_name.rmt_name,
                      (int)strlen((const char *)param->read_rmt_name.rmt_name));
            consider_peer(param->read_rmt_name.bda, 0, 0, name);
        } else {
            format_bda(param->read_rmt_name.bda, addr, sizeof(addr));
            ESP_LOGE(TAG, "remote name %s failed status %d", addr, (int)param->read_rmt_name.stat);
        }
    }
}

static esp_err_t bring_up_controller(void)
{
    esp_err_t err;
    /* Magic, mode, and stack come from the headers this translation unit
     * actually included. Arduino-ESP32 3.3.12's libbt.a is BTDM, so that
     * default mode is ESP_BT_MODE_BTDM. Overriding it to Classic, or
     * releasing a pool the chosen mode still needs, makes
     * esp_bt_controller_init return ESP_ERR_INVALID_ARG. Enable must use
     * the same mode init was given. */
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        if (cfg.mode == ESP_BT_MODE_CLASSIC_BT) {
            esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
        }
        err = esp_bt_controller_init(&cfg);
        if (err != ESP_OK) {
            return err;
        }
    }
    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
        err = esp_bt_controller_enable((esp_bt_mode_t)cfg.mode);
        if (err != ESP_OK) {
            return err;
        }
    }
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
#ifdef BT_BLUEDROID_INIT_CONFIG_DEFAULT
        esp_bluedroid_config_t bt_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
        err = esp_bluedroid_init_with_cfg(&bt_cfg);
#else
        err = esp_bluedroid_init();
#endif
        if (err != ESP_OK) {
            return err;
        }
    }
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        err = esp_bluedroid_enable();
        if (err != ESP_OK) {
            return err;
        }
    }
    /* Mode 2 is legacy PIN. Mode 4 (the Bluedroid default) negotiates SSP. */
    btm_sec_init(2);
    return ESP_OK;
}

esp_err_t wiimote_bt_start(void)
{
    esp_err_t err = nvs_flash_init();
    esp_bt_cod_t cod = {0};
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin = {0};
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    const esp_timer_create_args_t timer_args = {
        .callback = timer_cb,
        .name = "wiimote",
    };
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }
    err = bring_up_controller();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "controller init failed: %s", esp_err_to_name(err));
        return err;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 2)
    esp_bt_gap_set_device_name("Nintendo RVL-CNT-01");
#else
    esp_bt_dev_set_device_name("Nintendo RVL-CNT-01");
#endif
    cod.major = 5;
    cod.minor = 1;
    cod.service = 0x1;
    esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);
    esp_bt_gap_set_pin(pin_type, 0, pin);
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_bt_gap_register_callback(gap_cb);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    if (!post_btu(register_cb)) {
        return ESP_FAIL;
    }
    err = esp_timer_create(&timer_args, &s_timer);
    if (err != ESP_OK) {
        return err;
    }
    return esp_timer_start_periodic(s_timer, 5000);
}

esp_err_t wiimote_bt_sync(void)
{
    esp_err_t err;
    /* Inbound 1+2: stay connectable and general-discoverable so a scanning
     * Wii (or a PC) can open the ACL. Red-button sync is the opposite
     * direction: the console is discoverable, so inquire LIAC then GIAC
     * and page the first console. */
    s_syncing = 1;
    s_paging = 0;
    s_name_pending = 0;
    s_inq_limited = 1;
    err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (err != ESP_OK) {
        s_syncing = 0;
        return err;
    }
    start_inquiry();
    return ESP_OK;
}

void wiimote_bt_listen_only(void)
{
    s_syncing = 0;
    s_paging = 0;
    esp_bt_gap_cancel_discovery();
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
}

esp_err_t wiimote_bt_stop(void)
{
    s_syncing = 0;
    s_paging = 0;
    esp_bt_gap_cancel_discovery();
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    if (!post_btu(stop_cb)) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
