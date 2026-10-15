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
extern uint8_t SDP_DeleteRecord(uint32_t handle);
extern uint8_t BTM_SetSecurityLevel(uint8_t is_originator, const char *name, uint8_t service_id,
                                    uint16_t sec_level, uint16_t psm, uint32_t proto_id, uint32_t chan_id);
extern void BTM_SetDefaultLinkPolicy(uint16_t settings);
extern uint8_t BTM_DeleteStoredLinkKey(uint8_t *bd_addr, void *cb);
extern uint8_t L2CA_SetDesireRole(uint8_t new_role);
extern bool btu_task_post(uint32_t sig, void *param, uint32_t timeout);
/* Legacy security mode. Bluedroid boots in SSP (mode 4); the Wii drops that. */
extern void btm_sec_init(uint8_t sec_mode);

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
static uint8_t s_last_leds;
static uint8_t s_have_leds;
static uint8_t s_last_rumble;
static uint8_t s_have_rumble;
static int s_hid_ready;
static volatile int s_sync_window;
/* Set when Bluedroid's 8-attribute record cannot hold the HID descriptor. */
static int s_own_sdp;
static uint16_t s_sdp_cid;

static bool post_btu(void (*fn)(wm_bt_hdr_t *));
static void sdp_on_data(const uint8_t *data, int len);

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
    if (psm == 0x0001) {
        s_sdp_cid = cid;
        L2CA_ConnectRsp(bda, id, cid, 0, 0);
        zero_cfg(cfg, sizeof(cfg));
        L2CA_ConfigReq(cid, cfg);
        return;
    }
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
    if (cid == s_sdp_cid) {
        s_sdp_cid = 0;
    }
    if (ack_needed) {
        L2CA_DisconnectRsp(cid);
    }
    drop_cid(cid);
}

static void on_disconnect_cfm(uint16_t cid, uint16_t result)
{
    ESP_LOGE(TAG, "L2CAP disconnect confirm cid %u result %u", cid, result);
    if (cid == s_sdp_cid) {
        s_sdp_cid = 0;
    }
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
    if (msg == NULL) {
        return;
    }
    if (s_own_sdp && cid == s_sdp_cid) {
        if (msg->len > 0 && msg->offset + msg->len > msg->offset) {
            sdp_on_data(msg->data + msg->offset, msg->len);
        }
        free(msg);
        return;
    }
    (void)cid;
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

/* Dolphin hcidump of a real RVL-CNT-01 HID descriptor list (attr 0x0206).
 * Output items are 0x91 0x00 and input items are 0x81 0x00, not the generic
 * 0x02 data/variable/absolute flags. */
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

static void note_attr(uint16_t attr, uint8_t ok);

static void add_u16(uint32_t handle, uint16_t attr, uint16_t value)
{
    uint8_t be[2] = {(uint8_t)(value >> 8), (uint8_t)value};
    note_attr(attr, SDP_AddAttribute(handle, attr, 1, 2, be));
}

static void add_bool(uint32_t handle, uint16_t attr, uint8_t value)
{
    note_attr(attr, SDP_AddAttribute(handle, attr, 5, 1, &value));
}

static void note_attr(uint16_t attr, uint8_t ok)
{
    if (!ok) {
        ESP_LOGE(TAG, "SDP attr 0x%04x was not stored", attr);
    }
}

/* Arduino-ESP32 3.3.12 and 2.0.17 libbt sdp_db.c.obj:
 * SDP_AddAttribute returns false when num_attributes is already 8,
 * the value pad is 300 bytes (0x12c), and SDP_MAX_ATTR_LEN is 300.
 * SDP_CreateRecord mallocs 0x198 bytes and allows 6 records. */
#define WM_SDP_MAX_REC_ATTR 8
#define WM_SDP_MAX_ATTR_LEN 300
#define WM_SDP_MAX_PAD_LEN 300
#define WM_SDP_MAX_RECORDS 6
#define WM_SDP_RECORD_BYTES 0x198
#define WM_SDP_HANDLE 0x00010000u

static int s_slots;
static int s_pad;
static uint16_t s_ids[WM_SDP_MAX_REC_ATTR];
static int s_id_n;
static int s_got_0202;
static int s_got_0206;
static uint8_t s_sdp_body[640];
static uint8_t s_sdp_wrap[700];

static void remember_id(uint16_t id)
{
    if (s_id_n < WM_SDP_MAX_REC_ATTR) {
        s_ids[s_id_n++] = id;
    }
}

static const char *reject_reason(int value_len)
{
    if (s_slots >= WM_SDP_MAX_REC_ATTR) {
        return "too many attributes";
    }
    if (value_len > WM_SDP_MAX_ATTR_LEN) {
        return "longer than SDP_MAX_ATTR_LEN";
    }
    /* 3.3.12 SDP_AddAttribute returns false when free_pad is already past
     * 299, or when free_pad + len is past 300 for a non-text value. */
    if (s_pad > WM_SDP_MAX_PAD_LEN - 1 || s_pad + value_len > WM_SDP_MAX_PAD_LEN) {
        return "SDP pad full";
    }
    return "bad type or record missing";
}

static int store_attr(uint32_t handle, uint16_t id, int value_len, uint8_t ok)
{
    (void)handle;
    if (!ok) {
        ESP_LOGE(TAG, "SDP attr 0x%04x failed: %s (attrs %d, pad %d, value %d)",
                 id, reject_reason(value_len), s_slots, s_pad, value_len);
        return 0;
    }
    s_slots++;
    s_pad += value_len;
    remember_id(id);
    if (id == 0x0202) {
        s_got_0202 = 1;
    }
    if (id == 0x0206) {
        s_got_0206 = 1;
    }
    return 1;
}

static void log_stored_ids(void)
{
    char line[96];
    int n = 0;
    int i;
    n = snprintf(line, sizeof(line), "SDP stored");
    for (i = 0; i < s_id_n && n > 0 && n < (int)sizeof(line) - 8; i++) {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " 0x%04x", s_ids[i]);
    }
    ESP_LOGE(TAG, "%s", line);
}

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
    int i;
    for (i = 0; i < n; i++) {
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

static int de_hdr(const uint8_t *p, int n, int *data_len)
{
    int size;
    int hdr = 1;
    int d = 0;
    if (n < 1) {
        return -1;
    }
    size = p[0] & 7;
    if (size <= 4) {
        static const int map[5] = {1, 2, 4, 8, 16};
        d = map[size];
    } else if (size == 5) {
        if (n < 2) {
            return -1;
        }
        hdr = 2;
        d = p[1];
    } else if (size == 6) {
        if (n < 3) {
            return -1;
        }
        hdr = 3;
        d = (p[1] << 8) | p[2];
    } else {
        if (n < 5) {
            return -1;
        }
        hdr = 5;
        d = (p[1] << 24) | (p[2] << 16) | (p[3] << 8) | p[4];
    }
    if (d < 0 || hdr + d > n) {
        return -1;
    }
    *data_len = d;
    return hdr;
}

static int pattern_has_hid(const uint8_t *p, int n)
{
    int i = 0;
    while (i < n) {
        int dlen = 0;
        int hdr = de_hdr(p + i, n - i, &dlen);
        if (hdr < 0) {
            return 0;
        }
        if (p[i] == 0x19 && dlen == 2 && ((p[i + hdr] << 8) | p[i + hdr + 1]) == 0x1124) {
            return 1;
        }
        if (p[i] == 0x1c && dlen == 16 && p[i + hdr + 2] == 0x11 && p[i + hdr + 3] == 0x24) {
            return 1;
        }
        if ((p[i] >> 3) == 6 && pattern_has_hid(p + i + hdr, dlen)) {
            return 1;
        }
        i += hdr + dlen;
    }
    return 0;
}

static int attr_wanted(const uint8_t *list, int n, uint16_t id)
{
    int i = 0;
    if (list == NULL) {
        return 1;
    }
    while (i < n) {
        if (list[i] == 0x09 && i + 3 <= n) {
            uint16_t one = (uint16_t)((list[i + 1] << 8) | list[i + 2]);
            if (one == id) {
                return 1;
            }
            i += 3;
        } else if (list[i] == 0x0a && i + 5 <= n) {
            uint16_t a = (uint16_t)((list[i + 1] << 8) | list[i + 2]);
            uint16_t b = (uint16_t)((list[i + 3] << 8) | list[i + 4]);
            if (id >= a && id <= b) {
                return 1;
            }
            i += 5;
        } else {
            return 1;
        }
    }
    return 0;
}

static void emit_id(wm_w *w, uint16_t id)
{
    ww_u8(w, 0x09);
    ww_u16(w, id);
}

static void emit_u8_attr(wm_w *w, const uint8_t *filter, int flen, uint16_t id, uint8_t v)
{
    if (!attr_wanted(filter, flen, id)) {
        return;
    }
    emit_id(w, id);
    ww_u8(w, 0x08);
    ww_u8(w, v);
}

static void emit_u16_attr(wm_w *w, const uint8_t *filter, int flen, uint16_t id, uint16_t v)
{
    if (!attr_wanted(filter, flen, id)) {
        return;
    }
    emit_id(w, id);
    ww_u8(w, 0x09);
    ww_u16(w, v);
}

static void emit_bool_attr(wm_w *w, const uint8_t *filter, int flen, uint16_t id, uint8_t v)
{
    if (!attr_wanted(filter, flen, id)) {
        return;
    }
    emit_id(w, id);
    ww_u8(w, 0x28);
    ww_u8(w, v);
}

static void emit_text_attr(wm_w *w, const uint8_t *filter, int flen, uint16_t id, const char *s)
{
    int n = (int)strlen(s);
    if (!attr_wanted(filter, flen, id)) {
        return;
    }
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

static int build_attr_body(uint8_t *dst, int cap, const uint8_t *filter, int flen)
{
    wm_w w;
    uint8_t *seq;
    uint8_t *start;
    w.p = dst;
    w.end = dst + cap;
    if (attr_wanted(filter, flen, 0x0000)) {
        emit_id(&w, 0x0000);
        ww_u8(&w, 0x0a);
        ww_u8(&w, 0x00);
        ww_u8(&w, 0x01);
        ww_u8(&w, 0x00);
        ww_u8(&w, 0x00);
    }
    if (attr_wanted(filter, flen, 0x0001)) {
        emit_id(&w, 0x0001);
        ww_u8(&w, 0x35);
        ww_u8(&w, 0x03);
        ww_u8(&w, 0x19);
        ww_u16(&w, 0x1124);
    }
    if (attr_wanted(filter, flen, 0x0004)) {
        emit_id(&w, 0x0004);
        seq = ww_seq_open(&w, 0);
        start = w.p;
        emit_proto_pair(&w, 0x0011);
        ww_seq_close(seq, w.p, 0);
        (void)start;
    }
    if (attr_wanted(filter, flen, 0x0005)) {
        emit_id(&w, 0x0005);
        ww_u8(&w, 0x35);
        ww_u8(&w, 0x03);
        ww_u8(&w, 0x19);
        ww_u16(&w, 0x1002);
    }
    if (attr_wanted(filter, flen, 0x0006)) {
        emit_id(&w, 0x0006);
        ww_u8(&w, 0x35);
        ww_u8(&w, 0x09);
        ww_u8(&w, 0x09);
        ww_u16(&w, 0x656e);
        ww_u8(&w, 0x09);
        ww_u16(&w, 0x006a);
        ww_u8(&w, 0x09);
        ww_u16(&w, 0x0100);
    }
    if (attr_wanted(filter, flen, 0x0009)) {
        emit_id(&w, 0x0009);
        ww_u8(&w, 0x35);
        ww_u8(&w, 0x08);
        ww_u8(&w, 0x35);
        ww_u8(&w, 0x06);
        ww_u8(&w, 0x19);
        ww_u16(&w, 0x1124);
        ww_u8(&w, 0x09);
        ww_u16(&w, 0x0100);
    }
    if (attr_wanted(filter, flen, 0x000d)) {
        emit_id(&w, 0x000d);
        seq = ww_seq_open(&w, 0);
        ww_u8(&w, 0x35);
        ww_u8(&w, 0x0d);
        emit_proto_pair(&w, 0x0013);
        ww_seq_close(seq, w.p, 0);
    }
    emit_text_attr(&w, filter, flen, 0x0100, "Nintendo RVL-CNT-01");
    emit_text_attr(&w, filter, flen, 0x0101, "Nintendo RVL-CNT-01");
    emit_text_attr(&w, filter, flen, 0x0102, "Nintendo");
    emit_u16_attr(&w, filter, flen, 0x0200, 0x0100);
    emit_u16_attr(&w, filter, flen, 0x0201, 0x0111);
    emit_u8_attr(&w, filter, flen, 0x0202, 0x04);
    emit_u8_attr(&w, filter, flen, 0x0203, 0x33);
    emit_bool_attr(&w, filter, flen, 0x0204, 0);
    emit_bool_attr(&w, filter, flen, 0x0205, 1);
    if (attr_wanted(filter, flen, 0x0206)) {
        uint8_t *outer;
        uint8_t *inner;
        emit_id(&w, 0x0206);
        outer = ww_seq_open(&w, 0);
        inner = ww_seq_open(&w, 0);
        ww_u8(&w, 0x08);
        ww_u8(&w, 0x22);
        ww_u8(&w, 0x25);
        ww_u8(&w, (uint8_t)sizeof(s_rvl_hid_report));
        ww_bytes(&w, s_rvl_hid_report, (int)sizeof(s_rvl_hid_report));
        ww_seq_close(inner, w.p, 0);
        ww_seq_close(outer, w.p, 0);
    }
    if (attr_wanted(filter, flen, 0x0207)) {
        emit_id(&w, 0x0207);
        ww_u8(&w, 0x35);
        ww_u8(&w, 0x08);
        ww_u8(&w, 0x35);
        ww_u8(&w, 0x06);
        ww_u8(&w, 0x09);
        ww_u16(&w, 0x0409);
        ww_u8(&w, 0x09);
        ww_u16(&w, 0x0100);
    }
    emit_bool_attr(&w, filter, flen, 0x0208, 0);
    emit_bool_attr(&w, filter, flen, 0x0209, 1);
    emit_bool_attr(&w, filter, flen, 0x020a, 1);
    emit_u16_attr(&w, filter, flen, 0x020b, 0x0100);
    emit_u16_attr(&w, filter, flen, 0x020c, 0x0c80);
    emit_bool_attr(&w, filter, flen, 0x020d, 0);
    emit_bool_attr(&w, filter, flen, 0x020e, 0);
    if (w.p == w.end) {
        return -1;
    }
    return (int)(w.p - dst);
}

static int wrap_attrs(uint8_t *dst, int cap, const uint8_t *body, int blen, int records)
{
    wm_w w;
    uint8_t *outer = NULL;
    uint8_t *inner;
    int wide = blen > 255;
    w.p = dst;
    w.end = dst + cap;
    if (records) {
        int inner_total = (wide ? 3 : 2) + blen;
        outer = ww_seq_open(&w, inner_total > 255);
    }
    inner = ww_seq_open(&w, wide);
    ww_bytes(&w, body, blen);
    ww_seq_close(inner, w.p, wide);
    if (records && outer != NULL) {
        ww_seq_close(outer, w.p, ((wide ? 3 : 2) + blen) > 255);
    }
    if (w.p == w.end) {
        return -1;
    }
    return (int)(w.p - dst);
}

static void sdp_error(uint16_t tid, uint16_t code)
{
    uint8_t p[7];
    p[0] = 0x01;
    p[1] = (uint8_t)(tid >> 8);
    p[2] = (uint8_t)tid;
    p[3] = 0x00;
    p[4] = 0x02;
    p[5] = (uint8_t)(code >> 8);
    p[6] = (uint8_t)code;
    send_packet(s_sdp_cid, p, 7);
}

static void sdp_send_list(uint8_t pdu, uint16_t tid, const uint8_t *blob, int blob_len,
                          uint16_t max_count, uint16_t offset)
{
    uint8_t pkt[640];
    int chunk;
    int cont;
    int n;
    if (offset > (uint16_t)blob_len) {
        offset = (uint16_t)blob_len;
    }
    chunk = blob_len - offset;
    if (max_count == 0) {
        max_count = 1;
    }
    if (chunk > (int)max_count) {
        chunk = (int)max_count;
    }
    cont = (offset + chunk < blob_len) ? 2 : 0;
    if (7 + chunk + 1 + cont > (int)sizeof(pkt)) {
        chunk = (int)sizeof(pkt) - 8 - cont;
        if (chunk < 0) {
            chunk = 0;
        }
        cont = (offset + chunk < blob_len) ? 2 : 0;
    }
    pkt[0] = pdu;
    pkt[1] = (uint8_t)(tid >> 8);
    pkt[2] = (uint8_t)tid;
    n = 2 + chunk + 1 + cont;
    pkt[3] = (uint8_t)(n >> 8);
    pkt[4] = (uint8_t)n;
    pkt[5] = (uint8_t)(chunk >> 8);
    pkt[6] = (uint8_t)chunk;
    if (chunk > 0) {
        memcpy(pkt + 7, blob + offset, (size_t)chunk);
    }
    pkt[7 + chunk] = (uint8_t)cont;
    if (cont) {
        uint16_t next = (uint16_t)(offset + chunk);
        pkt[8 + chunk] = (uint8_t)(next >> 8);
        pkt[9 + chunk] = (uint8_t)next;
    }
    send_packet(s_sdp_cid, pkt, 7 + chunk + 1 + cont);
}

static int sdp_continuation(const uint8_t *p, int n, int used, uint16_t *offset)
{
    int clen;
    if (used >= n) {
        return -1;
    }
    clen = p[used];
    if (clen == 0) {
        *offset = 0;
        return 0;
    }
    if (clen == 2 && used + 3 <= n) {
        *offset = (uint16_t)((p[used + 1] << 8) | p[used + 2]);
        return 0;
    }
    return -1;
}

static void sdp_on_data(const uint8_t *p, int len)
{
    uint8_t pdu;
    uint16_t tid;
    uint16_t plen;
    uint16_t maxc;
    uint16_t offset = 0;
    const uint8_t *par;
    const uint8_t *filter = NULL;
    int flen = 0;
    int hdr;
    int dlen = 0;
    int used;
    int blen;
    int wlen;
    int match;
    if (len < 5) {
        return;
    }
    pdu = p[0];
    tid = (uint16_t)((p[1] << 8) | p[2]);
    plen = (uint16_t)((p[3] << 8) | p[4]);
    if ((int)plen > len - 5) {
        plen = (uint16_t)(len - 5);
    }
    par = p + 5;
    ESP_LOGE(TAG, "SDP pdu 0x%02x tid %u plen %u", pdu, tid, plen);
    if (pdu == 0x02) {
        uint8_t rsp[14];
        hdr = de_hdr(par, plen, &dlen);
        match = (hdr > 0) && pattern_has_hid(par + hdr, dlen);
        ESP_LOGE(TAG, "SDP service search hid %d", match);
        rsp[0] = 0x03;
        rsp[1] = (uint8_t)(tid >> 8);
        rsp[2] = (uint8_t)tid;
        if (!match) {
            rsp[3] = 0;
            rsp[4] = 5;
            memset(rsp + 5, 0, 5);
            send_packet(s_sdp_cid, rsp, 10);
            return;
        }
        rsp[3] = 0;
        rsp[4] = 9;
        rsp[5] = 0;
        rsp[6] = 1;
        rsp[7] = 0;
        rsp[8] = 1;
        rsp[9] = 0x00;
        rsp[10] = 0x01;
        rsp[11] = 0x00;
        rsp[12] = 0x00;
        rsp[13] = 0;
        send_packet(s_sdp_cid, rsp, 14);
        return;
    }
    if (pdu != 0x04 && pdu != 0x06) {
        sdp_error(tid, 0x0003);
        return;
    }
    used = 0;
    if (pdu == 0x04) {
        uint32_t handle;
        if (plen < 6) {
            sdp_error(tid, 0x0003);
            return;
        }
        handle = ((uint32_t)par[0] << 24) | ((uint32_t)par[1] << 16) | ((uint32_t)par[2] << 8) | par[3];
        if (handle != WM_SDP_HANDLE) {
            sdp_error(tid, 0x0002);
            return;
        }
        maxc = (uint16_t)((par[4] << 8) | par[5]);
        used = 6;
        match = 1;
    } else {
        hdr = de_hdr(par, plen, &dlen);
        if (hdr < 0) {
            sdp_error(tid, 0x0003);
            return;
        }
        match = pattern_has_hid(par + hdr, dlen);
        used = hdr + dlen;
        if (used + 2 > plen) {
            sdp_error(tid, 0x0003);
            return;
        }
        maxc = (uint16_t)((par[used] << 8) | par[used + 1]);
        used += 2;
        ESP_LOGE(TAG, "SDP search attribute hid %d max %u", match, maxc);
    }
    hdr = de_hdr(par + used, plen - used, &dlen);
    if (hdr < 0) {
        sdp_error(tid, 0x0003);
        return;
    }
    filter = par + used + hdr;
    flen = dlen;
    used += hdr + dlen;
    if (sdp_continuation(par, plen, used, &offset) != 0) {
        sdp_error(tid, 0x0005);
        return;
    }
    if (!match) {
        uint8_t empty[2] = {0x35, 0x00};
        sdp_send_list(pdu == 0x04 ? 0x05 : 0x07, tid, empty, 2, maxc, 0);
        return;
    }
    blen = build_attr_body(s_sdp_body, (int)sizeof(s_sdp_body), filter, flen);
    if (blen < 0) {
        sdp_error(tid, 0x0006);
        return;
    }
    wlen = wrap_attrs(s_sdp_wrap, (int)sizeof(s_sdp_wrap), s_sdp_body, blen, pdu == 0x06);
    if (wlen < 0) {
        sdp_error(tid, 0x0006);
        return;
    }
    sdp_send_list(pdu == 0x04 ? 0x05 : 0x07, tid, s_sdp_wrap, wlen, maxc, offset);
}

static int stored_id(uint16_t id)
{
    int i;
    for (i = 0; i < s_id_n; i++) {
        if (s_ids[i] == id) {
            return 1;
        }
    }
    return 0;
}

static void serve_full_sdp(void)
{
    uint16_t got;
    s_own_sdp = 1;
    got = L2CA_Register(0x0001, &s_appl);
    if (got == 0) {
        s_own_sdp = 0;
        ESP_LOGE(TAG, "could not take SDP PSM 0x0001");
        return;
    }
    ESP_LOGE(TAG, "required HID attribute missing from the SDP DB; serving the full RVL-CNT-01 record on PSM 0x0001");
}

static void add_hid_record(void)
{
    uint32_t handle;
    uint16_t hid = 0x1124;
    wm_sdp_proto_t proto[2];
    wm_sdp_proto_list_t extra;
    uint8_t list[sizeof(s_rvl_hid_report) + 8];
    uint8_t *p;
    uint8_t inner;
    uint8_t subclass = 0x04;
    uint8_t reconnect = 1;
    uint8_t virtual_cable = 0;
    int desc_len;
    /* Arduino-ESP32 3.3.12 libbt (HID device and SDP common compiled out)
     * allows 8 attributes per record, counting the handle CreateRecord
     * inserts. The 217-byte descriptor is not what rejected 0x0202; that
     * attribute was the 9th. Keep only the attributes a console reads,
     * in the order they must survive a full table. */
    ESP_LOGE(TAG,
             "SDP limits (3.3.12 libbt) MAX_REC_ATTR %d (includes handle), MAX_ATTR_LEN %d, MAX_PAD_LEN %d, MAX_RECORDS %d, record %d",
             WM_SDP_MAX_REC_ATTR, WM_SDP_MAX_ATTR_LEN, WM_SDP_MAX_PAD_LEN,
             WM_SDP_MAX_RECORDS, WM_SDP_RECORD_BYTES);
    s_slots = 0;
    s_pad = 0;
    s_id_n = 0;
    s_got_0202 = 0;
    s_got_0206 = 0;
    s_own_sdp = 0;
    if (sizeof(s_rvl_hid_report) != 217 || sizeof(s_rvl_hid_report) > 255) {
        ESP_LOGE(TAG, "HID descriptor length %u is not the RVL-CNT-01 dump",
                 (unsigned)sizeof(s_rvl_hid_report));
        serve_full_sdp();
        return;
    }
    handle = SDP_CreateRecord();
    if (handle == 0) {
        ESP_LOGE(TAG, "SDP_CreateRecord failed");
        serve_full_sdp();
        return;
    }
    s_slots = 1;
    s_pad = 4;
    remember_id(0x0000);

    store_attr(handle, 0x0001, 3, SDP_AddServiceClassIdList(handle, 1, &hid));

    memset(proto, 0, sizeof(proto));
    proto[0].protocol_uuid = 0x0100;
    proto[0].num_params = 1;
    proto[0].params[0] = 0x0011;
    proto[1].protocol_uuid = 0x0011;
    proto[1].num_params = 0;
    store_attr(handle, 0x0004, 13, SDP_AddProtocolList(handle, 2, proto));

    memset(&extra, 0, sizeof(extra));
    extra.num_elems = 2;
    extra.list_elem[0].protocol_uuid = 0x0100;
    extra.list_elem[0].num_params = 1;
    extra.list_elem[0].params[0] = 0x0013;
    extra.list_elem[1].protocol_uuid = 0x0011;
    store_attr(handle, 0x000d, 15, SDP_AddAdditionProtoLists(handle, 1, &extra));

    store_attr(handle, 0x0009, 8, SDP_AddProfileDescriptorList(handle, hid, 0x0100));
    store_attr(handle, 0x0202, 1, SDP_AddAttribute(handle, 0x0202, 1, 1, &subclass));

    inner = (uint8_t)(4 + sizeof(s_rvl_hid_report));
    p = list;
    *p++ = (uint8_t)((6 << 3) | 5);
    *p++ = inner;
    *p++ = (uint8_t)((1 << 3) | 0);
    *p++ = 0x22;
    *p++ = (uint8_t)((4 << 3) | 5);
    *p++ = (uint8_t)sizeof(s_rvl_hid_report);
    memcpy(p, s_rvl_hid_report, sizeof(s_rvl_hid_report));
    p += sizeof(s_rvl_hid_report);
    desc_len = (int)(p - list);
    store_attr(handle, 0x0206, desc_len,
               SDP_AddAttribute(handle, 0x0206, 6, (uint32_t)desc_len, list));

    store_attr(handle, 0x0205, 1, SDP_AddAttribute(handle, 0x0205, 5, 1, &reconnect));
    store_attr(handle, 0x0204, 1, SDP_AddAttribute(handle, 0x0204, 5, 1, &virtual_cable));

    log_stored_ids();
    /* Subclass and the descriptor are the ones the last log claimed without
     * storing. Both PSMs have to be in the same record or the console still
     * hangs up. 0x0204 is attempted and may be the attribute that does not fit. */
    if (!stored_id(0x0001) || !stored_id(0x0004) || !stored_id(0x000d) ||
        !stored_id(0x0009) || !s_got_0202 || !s_got_0206) {
        /* Handle 0 would delete every record. The short record must not
         * stay for Bluedroid to answer with. */
        SDP_DeleteRecord(handle);
        serve_full_sdp();
        return;
    }
    ESP_LOGE(TAG, "RVL-CNT-01 HID SDP record registered (subclass 0x04, PSM 0x11/0x13)");
}

static void add_device_id_record(void)
{
    uint32_t handle = SDP_CreateRecord();
    uint16_t pnp = 0x1200;
    if (handle == 0) {
        ESP_LOGE(TAG, "device-id SDP record was not created");
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
    /* Level 0 on the no-security control and interrupt services. The first
     * record for a PSM is what L2CAP checks, so the authenticated control
     * service (id 51) is not registered. SDP PSM 0x0001 stays open as well.
     * The Wii authenticates the ACL itself after the channels exist. */
    BTM_SetSecurityLevel(0, "HID", 52, 0, 0x0011, 6, 4);
    BTM_SetSecurityLevel(1, "HID", 52, 0, 0x0011, 6, 4);
    BTM_SetSecurityLevel(0, "HID", 53, 0, 0x0013, 6, 0);
    BTM_SetSecurityLevel(1, "HID", 53, 0, 0x0013, 6, 0);
    BTM_SetSecurityLevel(0, "SDP", 0, 0, 0x0001, 0, 0);
    BTM_SetSecurityLevel(1, "SDP", 0, 0, 0x0001, 0, 0);
}

static void allow_role_switch(void)
{
    /* Link policy bit 0x0001 lets the peer request a role switch. Desire
     * role 1 accepts the inbound ACL as slave, so the Wii stays master.
     * 0x80 clears the "disallow switch" flag on outgoing creates. */
    BTM_SetDefaultLinkPolicy(0x0001 | 0x0002 | 0x0004 | 0x0008);
    L2CA_SetDesireRole(0x80);
    L2CA_SetDesireRole(0x01);
}

static void register_hid_once(void)
{
    if (s_hid_ready) {
        return;
    }
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
    allow_role_switch();
    allow_hid_security();
    add_hid_record();
    if (!s_own_sdp) {
        add_device_id_record();
    }
    L2CA_Register(0x0011, &s_appl);
    L2CA_Register(0x0013, &s_appl);
    s_hid_ready = 1;
    ESP_LOGE(TAG, "L2CAP PSM 0x11 and 0x13 registered");
}

static void register_cb(wm_bt_hdr_t *msg)
{
    free(msg);
    register_hid_once();
}

static void open_window_cb(wm_bt_hdr_t *msg)
{
    uint8_t deleted;
    free(msg);
    register_hid_once();
    /* NULL deletes every link key stored in the controller. */
    deleted = BTM_DeleteStoredLinkKey(NULL, NULL);
    ESP_LOGE(TAG, "controller link-key delete status %u", deleted);
    if (!s_sync_window) {
        return;
    }
    if (esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_LIMITED_DISCOVERABLE) != ESP_OK) {
        ESP_LOGE(TAG, "limited-discoverable scan mode failed");
        return;
    }
    ESP_LOGE(TAG, "connectable, limited-discoverable (LIAC)");
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

static const char *gap_event_name(esp_bt_gap_cb_event_t event)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: return "DISC_RES";
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: return "DISC_STATE_CHANGED";
    case ESP_BT_GAP_RMT_SRVCS_EVT: return "RMT_SRVCS";
    case ESP_BT_GAP_RMT_SRVC_REC_EVT: return "RMT_SRVC_REC";
    case ESP_BT_GAP_AUTH_CMPL_EVT: return "AUTH_CMPL";
    case ESP_BT_GAP_PIN_REQ_EVT: return "PIN_REQ";
    case ESP_BT_GAP_CFM_REQ_EVT: return "CFM_REQ";
    case ESP_BT_GAP_KEY_NOTIF_EVT: return "KEY_NOTIF";
    case ESP_BT_GAP_KEY_REQ_EVT: return "KEY_REQ";
    case ESP_BT_GAP_READ_RSSI_DELTA_EVT: return "READ_RSSI_DELTA";
    case ESP_BT_GAP_CONFIG_EIR_DATA_EVT: return "CONFIG_EIR";
    case ESP_BT_GAP_SET_AFH_CHANNELS_EVT: return "SET_AFH";
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT: return "READ_REMOTE_NAME";
    case ESP_BT_GAP_MODE_CHG_EVT: return "MODE_CHG";
    case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT: return "REMOVE_BOND";
    case ESP_BT_GAP_QOS_CMPL_EVT: return "QOS";
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT: return "ACL_CONN";
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT: return "ACL_DISCONN";
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    case ESP_BT_GAP_SET_PAGE_TO_EVT: return "SET_PAGE_TO";
    case ESP_BT_GAP_GET_PAGE_TO_EVT: return "GET_PAGE_TO";
    case ESP_BT_GAP_ACL_PKT_TYPE_CHANGED_EVT: return "ACL_PKT_TYPE";
    case ESP_BT_GAP_ENC_CHG_EVT: return "ENC_CHG";
    case ESP_BT_GAP_SET_MIN_ENC_KEY_SIZE_EVT: return "SET_MIN_ENC_KEY_SIZE";
    case ESP_BT_GAP_GET_DEV_NAME_CMPL_EVT: return "GET_DEV_NAME";
#endif
    default: return "UNKNOWN";
    }
}

static const char *pm_mode_name(esp_bt_pm_mode_t mode)
{
    switch (mode) {
    case ESP_BT_PM_MD_ACTIVE: return "active";
    case ESP_BT_PM_MD_HOLD: return "hold";
    case ESP_BT_PM_MD_SNIFF: return "sniff";
    case ESP_BT_PM_MD_PARK: return "park";
    default: return "other";
    }
}

static void log_gap_window(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char addr[18];
    const char *name = gap_event_name(event);
    if (param == NULL) {
        ESP_LOGE(TAG, "GAP %s (%d)", name, (int)event);
        return;
    }
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
        format_bda(param->disc_res.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP DISC_RES %s props %d", addr, param->disc_res.num_prop);
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGE(TAG, "GAP DISC_STATE_CHANGED state %d", (int)param->disc_st_chg.state);
        break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
        format_bda(param->rmt_srvcs.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP RMT_SRVCS %s stat %d uuids %d", addr,
                 (int)param->rmt_srvcs.stat, param->rmt_srvcs.num_uuids);
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        format_bda(param->rmt_srvc_rec.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP RMT_SRVC_REC %s stat %d", addr, (int)param->rmt_srvc_rec.stat);
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        format_bda(param->auth_cmpl.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP AUTH_CMPL %s stat %d", addr, (int)param->auth_cmpl.stat);
        break;
    case ESP_BT_GAP_PIN_REQ_EVT:
        format_bda(param->pin_req.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP PIN_REQ %s min16 %d", addr, param->pin_req.min_16_digit ? 1 : 0);
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:
        format_bda(param->cfm_req.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP CFM_REQ %s", addr);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        format_bda(param->key_notif.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP KEY_NOTIF %s", addr);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        format_bda(param->key_req.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP KEY_REQ %s", addr);
        break;
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        format_bda(param->read_rmt_name.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP READ_REMOTE_NAME %s stat %d name %s", addr,
                 (int)param->read_rmt_name.stat, (char *)param->read_rmt_name.rmt_name);
        break;
    case ESP_BT_GAP_MODE_CHG_EVT:
        format_bda(param->mode_chg.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP MODE_CHG %s mode %s (%u)", addr,
                 pm_mode_name(param->mode_chg.mode), (unsigned)param->mode_chg.mode);
        break;
    case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT:
        format_bda(param->remove_bond_dev_cmpl.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP REMOVE_BOND %s stat %d", addr, (int)param->remove_bond_dev_cmpl.status);
        break;
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        format_bda(param->acl_conn_cmpl_stat.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP ACL_CONN %s handle %u status %d", addr,
                 param->acl_conn_cmpl_stat.handle, (int)param->acl_conn_cmpl_stat.stat);
        break;
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        format_bda(param->acl_disconn_cmpl_stat.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP ACL_DISCONN %s handle %u reason %d", addr,
                 param->acl_disconn_cmpl_stat.handle, (int)param->acl_disconn_cmpl_stat.reason);
        break;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    case ESP_BT_GAP_ENC_CHG_EVT:
        format_bda(param->enc_chg.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP ENC_CHG %s mode %u", addr, (unsigned)param->enc_chg.enc_mode);
        break;
    case ESP_BT_GAP_ACL_PKT_TYPE_CHANGED_EVT:
        format_bda(param->set_acl_pkt_types.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "GAP ACL_PKT_TYPE %s stat %d types 0x%04x", addr,
                 (int)param->set_acl_pkt_types.status, param->set_acl_pkt_types.pkt_types);
        break;
#endif
    default:
        ESP_LOGE(TAG, "GAP %s (%d)", name, (int)event);
        break;
    }
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char addr[18];
    if (s_sync_window) {
        log_gap_window(event, param);
    }
    if (param == NULL) {
        return;
    }
    if (event == ESP_BT_GAP_PIN_REQ_EVT) {
        esp_bt_pin_code_t pin;
        int i;
        format_bda(param->pin_req.bda, addr, sizeof(addr));
        /* Peer BD_ADDR, reversed, 6 bytes. Only when the Wii asks. Not SSP. */
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
    } else if (!s_sync_window && event == ESP_BT_GAP_AUTH_CMPL_EVT) {
        format_bda(param->auth_cmpl.bda, addr, sizeof(addr));
        if (param->auth_cmpl.stat != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "auth fail %s status %d", addr, (int)param->auth_cmpl.stat);
        } else {
            ESP_LOGE(TAG, "auth ok %s", addr);
        }
    } else if (!s_sync_window && event == ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT) {
        format_bda(param->acl_conn_cmpl_stat.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "ACL up %s handle %u status %d", addr,
                 param->acl_conn_cmpl_stat.handle, (int)param->acl_conn_cmpl_stat.stat);
    } else if (!s_sync_window && event == ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT) {
        format_bda(param->acl_disconn_cmpl_stat.bda, addr, sizeof(addr));
        ESP_LOGE(TAG, "ACL down %s handle %u reason %d", addr,
                 param->acl_disconn_cmpl_stat.handle, (int)param->acl_disconn_cmpl_stat.reason);
    }
}

/* 0x002504: peripheral, joystick, limited-discoverable. Wiibrew RVL-CNT-01.
 * Minor class 1 is the joystick value. Arduino-ESP32 2.0.17's gap header
 * has no ESP_BT_COD_MINOR_PERIPHERAL_JOYSTICK name for it. */
static esp_err_t publish_wiimote_cod(void)
{
    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    cod.minor = 1;
    cod.service = ESP_BT_COD_SRVC_LMTD_DISCOVER;
    return esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);
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
    err = publish_wiimote_cod();
    if (err != ESP_OK) {
        return err;
    }
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

static void forget_link_keys(void)
{
    esp_bd_addr_t list[16];
    int n = esp_bt_gap_get_bond_device_num();
    int i;
    if (n < 0) {
        ESP_LOGE(TAG, "bond list unavailable (%d)", n);
        return;
    }
    if (n == 0) {
        ESP_LOGE(TAG, "no stored link key");
        return;
    }
    if (n > (int)(sizeof(list) / sizeof(list[0]))) {
        n = (int)(sizeof(list) / sizeof(list[0]));
    }
    if (esp_bt_gap_get_bond_device_list(&n, list) != ESP_OK) {
        ESP_LOGE(TAG, "bond list read failed");
        return;
    }
    for (i = 0; i < n; i++) {
        esp_err_t err = esp_bt_gap_remove_bond_device(list[i]);
        ESP_LOGE(TAG, "remove link key %02x:%02x:%02x:%02x:%02x:%02x -> %s",
                 list[i][0], list[i][1], list[i][2], list[i][3], list[i][4], list[i][5],
                 esp_err_to_name(err));
    }
}

esp_err_t wiimote_bt_sync(void)
{
    esp_err_t err;
    /* The Wii's red SYNC button inquires on LIAC. A real remote does not
     * inquire; it is limited-discoverable and connectable. Class of device
     * is published first so 0x002504 is on the air before inquiry scan.
     * The HID SDP record is registered on the BTU task before scan mode
     * turns inquiry scan on. Limited mode writes IAC LAPs LIAC 0x9E8B00
     * and GIAC 0x9E8B33. Connectable keeps page scan on. Stored link keys
     * are deleted here; bonding starts only if the Wii sends a PIN request. */
    err = publish_wiimote_cod();
    if (err != ESP_OK) {
        return err;
    }
    forget_link_keys();
    s_sync_window = 1;
    if (!post_btu(open_window_cb)) {
        s_sync_window = 0;
        return ESP_FAIL;
    }
    return ESP_OK;
}

void wiimote_bt_listen_only(void)
{
    s_sync_window = 0;
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    wiimote_note_sync_ended();
}

esp_err_t wiimote_bt_stop(void)
{
    s_sync_window = 0;
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    wiimote_note_sync_ended();
    if (!post_btu(stop_cb)) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Reconnect-by-paging is implemented in the BTstack backend only. */
esp_err_t wiimote_bt_connect(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t wiimote_bt_disconnect(void)
{
    return wiimote_bt_stop();
}

esp_err_t wiimote_bt_forget_host(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

bool wiimote_bt_has_host(void)
{
    return false;
}

void wiimote_bt_send_now(void)
{
}

bool wiimote_bt_is_connecting(void)
{
    return false;
}

bool wiimote_bt_link_is_master(void)
{
    return false;
}

uint8_t wiimote_bt_last_disconnect_reason(void)
{
    return 0;
}

void wiimote_bt_set_external_stack(bool external)
{
    (void)external;
}

esp_err_t wiimote_btstack_register_services(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}
