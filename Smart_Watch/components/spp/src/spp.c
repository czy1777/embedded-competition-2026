/**
 * @file spp.c
 * @brief BLE SPP 底层驱动实现
 *
 * 基于NimBLE协议栈，实现GATT Server角色。
 * 定义一个自定义健康数据服务(0xABF0)，包含一个Notify特征(0xABF1)。
 */

#include "spp.h"

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"

/* NimBLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_SPP";

/*=============================================================================
 * 内部状态
 *============================================================================*/

static ble_conn_callback_t s_conn_callback = NULL;
static ble_conn_state_t    s_conn_state    = BLE_STATE_IDLE;
static uint16_t            s_conn_handle   = 0;
static uint16_t            s_health_chr_val_handle = 0;
static bool                s_notify_enabled = false;
static uint8_t             s_own_addr_type  = 0;

/*=============================================================================
 * GATT 服务定义
 *============================================================================*/

static int health_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        /* 健康数据服务 */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_HEALTH_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* 健康数据特征: 支持读取、通知和写入（JDY-18主机透传需要WRITE属性） */
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_HEALTH_DATA_UUID16),
                .access_cb = health_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY
                       | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &s_health_chr_val_handle,
            },
            { 0 }, /* 结束标记 */
        },
    },
    { 0 }, /* 结束标记 */
};

/*=============================================================================
 * GATT 特征访问回调
 *============================================================================*/

static int health_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* 读取请求时返回空数据，实际数据通过Notification推送 */
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        ESP_LOGD(TAG, "健康数据特征被读取");
        return 0;
    }
    /* 写入请求（JDY-18透传模式会向此特征写入数据） */
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        ESP_LOGD(TAG, "健康数据特征收到写入");
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/*=============================================================================
 * GAP 事件处理
 *============================================================================*/

static void start_advertising(void);

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            /* 连接成功 */
            s_conn_handle = event->connect.conn_handle;
            s_conn_state = BLE_STATE_CONNECTED;
            s_notify_enabled = false;
            ESP_LOGI(TAG, "BLE 已连接, handle=%d", s_conn_handle);

            if (s_conn_callback) {
                s_conn_callback(BLE_STATE_CONNECTED);
            }
        } else {
            /* 连接失败，重新广播 */
            ESP_LOGW(TAG, "BLE 连接失败, status=%d", event->connect.status);
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE 已断开, reason=%d", event->disconnect.reason);
        s_conn_state = BLE_STATE_ADVERTISING;
        s_notify_enabled = false;

        if (s_conn_callback) {
            s_conn_callback(BLE_STATE_ADVERTISING);
        }

        /* 重新广播 */
        start_advertising();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_health_chr_val_handle) {
            s_notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "Notify %s", s_notify_enabled ? "已启用" : "已禁用");
        }
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGD(TAG, "广播完成");
        start_advertising();
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU更新: %d", event->mtu.value);
        break;

    default:
        break;
    }

    return 0;
}

/*=============================================================================
 * 广播配置
 *============================================================================*/

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = { 0 };
    struct ble_gap_adv_params adv_params = { 0 };
    int rc;

    /* 广播标志: 通用可发现 + 不支持经典蓝牙 */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* 设备名称 */
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    /* 包含TX Power */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "设置广播数据失败: %d", rc);
        return;
    }

    /* 扫描响应数据: 包含服务UUID */
    struct ble_hs_adv_fields rsp_fields = { 0 };
    ble_uuid16_t svc_uuid = BLE_UUID16_INIT(BLE_SVC_HEALTH_UUID16);
    rsp_fields.uuids16 = &svc_uuid;
    rsp_fields.num_uuids16 = 1;
    rsp_fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "设置扫描响应数据失败: %d", rc);
        return;
    }

    /* 广播参数: 可连接 + 通用可发现 */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "启动广播失败: %d", rc);
        return;
    }

    s_conn_state = BLE_STATE_ADVERTISING;
    ESP_LOGI(TAG, "BLE 广播已启动, 设备名: %s", name);
}

/*=============================================================================
 * NimBLE Host 回调
 *============================================================================*/

static void on_stack_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE 协议栈重置, reason=%d", reason);
}

static void on_stack_sync(void)
{
    /* 协议栈同步完成，获取地址类型并开始广播 */
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "获取设备地址失败");
        return;
    }

    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "推断地址类型失败: %d", rc);
        return;
    }

    start_advertising();
}

static void gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    /* GATT注册回调，用于调试 */
    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "注册服务: uuid=%04x handle=%d",
                 ble_uuid_u16(ctxt->svc.svc_def->uuid), ctxt->svc.handle);
        break;
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG, "注册特征: uuid=%04x def_handle=%d val_handle=%d",
                 ble_uuid_u16(ctxt->chr.chr_def->uuid),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;
    default:
        break;
    }
}

/*=============================================================================
 * NimBLE Host 任务
 *============================================================================*/

static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE Host 任务启动");
    nimble_port_run();
    /* nimble_port_run() 正常不会返回 */
    nimble_port_freertos_deinit();
}

/*=============================================================================
 * 公共接口
 *============================================================================*/

esp_err_t ble_spp_init(ble_conn_callback_t conn_cb)
{
    s_conn_callback = conn_cb;

    /* 初始化NimBLE */
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE 初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 配置NimBLE Host */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* 初始化GAP和GATT标准服务 */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* 注册自定义GATT服务 */
    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT 服务计数失败: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT 服务注册失败: %d", rc);
        return ESP_FAIL;
    }

    /* 设置设备名称 */
    rc = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGW(TAG, "设置设备名称失败: %d", rc);
    }

    /* 启动NimBLE Host任务 */
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "BLE SPP 初始化完成");
    return ESP_OK;
}

void ble_spp_deinit(void)
{
    int rc = nimble_port_stop();
    if (rc == 0) {
        nimble_port_deinit();
    }
    s_conn_state = BLE_STATE_IDLE;
    s_notify_enabled = false;
    ESP_LOGI(TAG, "BLE SPP 已关闭");
}

esp_err_t ble_spp_send(const uint8_t *data, uint16_t len)
{
    if (s_conn_state != BLE_STATE_CONNECTED) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_notify_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL) {
        ESP_LOGE(TAG, "分配mbuf失败");
        return ESP_ERR_NO_MEM;
    }

    int rc = ble_gatts_notify_custom(s_conn_handle,
                                     s_health_chr_val_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "发送通知失败: %d", rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

ble_conn_state_t ble_spp_get_state(void)
{
    return s_conn_state;
}

bool ble_spp_is_connected(void)
{
    return s_conn_state == BLE_STATE_CONNECTED;
}
