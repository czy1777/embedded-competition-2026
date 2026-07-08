/**
 * @file ssd1306_oled.c
 * @brief SSD1306 OLED 驱动实现 (基于软件 I2C)
 */

#include "ssd1306_oled.h"
#include "ssd1306_font.h"
#include "soft_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SSD1306";

/*=============================================================================
 * 内部变量
 *============================================================================*/
static soft_i2c_handle_t s_i2c_handle;
static bool s_initialized = false;
static uint8_t s_device_addr = SSD1306_I2C_ADDR;  // 实际使用的设备地址

/*=============================================================================
 * SSD1306 初始化命令序列 (128x32)
 *============================================================================*/
static const uint8_t s_init_cmds[] = {
    0xAE,           // 关闭显示
    0xD5, 0x80,     // 设置时钟分频因子
    0xA8, 0x1F,     // 设置驱动路数 (32行: 0x1F)
    0xD3, 0x00,     // 设置显示偏移
    0x40,           // 设置显示开始行
    0x8D, 0x14,     // 启用电荷泵
    0xA1,           // 段重映射 (左右翻转)
    0xC8,           // 扫描方向设置 (上下翻转)
    0xDA, 0x02,     // COM 硬件配置 - 128x32 顺序扫描模式
    0x81, 0x80,     // 对比度设置
    0xD9, 0x1F,     // 预充电周期
    0xDB, 0x40,     // VCOM 电压
    0xA4,           // 显示跟随 RAM 内容
    0xAF,           // 打开显示
};

/*=============================================================================
 * 底层写入函数 - 带错误检查
 *============================================================================*/
static esp_err_t oled_write_cmd(uint8_t cmd)
{
    return soft_i2c_mem_write(&s_i2c_handle, s_device_addr, 0x00, &cmd, 1);
}

static esp_err_t oled_write_data(uint8_t data)
{
    return soft_i2c_mem_write(&s_i2c_handle, s_device_addr, 0x40, &data, 1);
}

/**
 * @brief 检测 SSD1306 设备地址
 * @return true 找到设备，false 未找到
 */
static bool ssd1306_detect_address(void)
{
    uint8_t dummy = 0xAE;  // 显示关闭命令，安全的测试命令
    esp_err_t ret;

    // 尝试主地址 0x78 (0x3C << 1)
    ESP_LOGI(TAG, "尝试 I2C 地址 0x78 (0x3C)...");
    ret = soft_i2c_mem_write(&s_i2c_handle, SSD1306_I2C_ADDR, 0x00, &dummy, 1);
    if (ret == ESP_OK) {
        s_device_addr = SSD1306_I2C_ADDR;
        ESP_LOGI(TAG, "在地址 0x78 找到 SSD1306");
        return true;
    }
    ESP_LOGW(TAG, "地址 0x78 无响应 (ret=%d)", ret);

    // 尝试备用地址 0x7A (0x3D << 1)
    ESP_LOGI(TAG, "尝试 I2C 地址 0x7A (0x3D)...");
    ret = soft_i2c_mem_write(&s_i2c_handle, SSD1306_I2C_ADDR_ALT, 0x00, &dummy, 1);
    if (ret == ESP_OK) {
        s_device_addr = SSD1306_I2C_ADDR_ALT;
        ESP_LOGI(TAG, "在地址 0x7A 找到 SSD1306");
        return true;
    }
    ESP_LOGW(TAG, "地址 0x7A 无响应 (ret=%d)", ret);

    // 如果两个地址都失败，尝试强制使用默认地址（忽略 ACK）
    ESP_LOGW(TAG, "未检测到 ACK，尝试强制模式 (地址 0x78)...");
    s_device_addr = SSD1306_I2C_ADDR;

    // 返回 true 让初始化继续，看看 OLED 是否真的能工作
    // 如果硬件连接正确但只是缺少上拉电阻导致 ACK 读不到，OLED 可能仍然能工作
    return true;
}

/*=============================================================================
 * 辅助函数
 *============================================================================*/
static uint32_t oled_pow(uint8_t a, uint8_t n)
{
    uint32_t result = 1;
    while (n--) {
        result *= a;
    }
    return result;
}

/*=============================================================================
 * API 实现
 *============================================================================*/

esp_err_t ssd1306_init(const ssd1306_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 初始化软件 I2C
    soft_i2c_config_t i2c_config = {
        .sda_pin = config->sda_pin,
        .scl_pin = config->scl_pin,
        .freq_khz = config->freq_khz,
    };

    esp_err_t ret = soft_i2c_init(&s_i2c_handle, &i2c_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "软件 I2C 初始化失败");
        return ret;
    }

    // 等待上电稳定
    vTaskDelay(pdMS_TO_TICKS(100));

    // 检测 SSD1306 设备地址
    if (!ssd1306_detect_address()) {
        return ESP_ERR_NOT_FOUND;
    }

    // 发送初始化命令序列 - 作为一个I2C事务批量发送
    // 注意：双字节命令（如0x8D,0x14）必须在同一个事务中发送
    ESP_LOGI(TAG, "发送 SSD1306 初始化命令...");
    esp_err_t ret_init = soft_i2c_mem_write(&s_i2c_handle, s_device_addr, 0x00,
                                             s_init_cmds, sizeof(s_init_cmds));
    if (ret_init != ESP_OK) {
        ESP_LOGW(TAG, "初始化命令发送失败 (ret=%d)，但继续尝试...", ret_init);
    } else {
        ESP_LOGI(TAG, "初始化命令发送成功");
    }

    // 短暂延时等待 OLED 处理命令
    vTaskDelay(pdMS_TO_TICKS(10));

    // 清屏
    ssd1306_clear();
    ssd1306_set_position(0, 0);

    s_initialized = true;
    ESP_LOGI(TAG, "SSD1306 初始化完成 (SDA=GPIO%d, SCL=GPIO%d, ADDR=0x%02X)",
             config->sda_pin, config->scl_pin, s_device_addr);

    return ESP_OK;
}

void ssd1306_deinit(void)
{
    if (!s_initialized) {
        return;
    }
    ssd1306_display_off();
    soft_i2c_deinit(&s_i2c_handle);
    s_initialized = false;
}

void ssd1306_clear(void)
{
    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        oled_write_cmd(0xB0 + page);
        oled_write_cmd(0x00);
        oled_write_cmd(0x10);
        for (uint8_t col = 0; col < SSD1306_WIDTH; col++) {
            oled_write_data(0x00);
        }
    }
}

void ssd1306_fill(void)
{
    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        oled_write_cmd(0xB0 + page);
        oled_write_cmd(0x00);
        oled_write_cmd(0x10);
        for (uint8_t col = 0; col < SSD1306_WIDTH; col++) {
            oled_write_data(0xFF);
        }
    }
}

void ssd1306_display_on(void)
{
    oled_write_cmd(0x8D);
    oled_write_cmd(0x14);
    oled_write_cmd(0xAF);
}

void ssd1306_display_off(void)
{
    oled_write_cmd(0x8D);
    oled_write_cmd(0x10);
    oled_write_cmd(0xAE);
}

void ssd1306_set_position(uint8_t x, uint8_t y)
{
    oled_write_cmd(0xB0 + y);
    oled_write_cmd(((x & 0xF0) >> 4) | 0x10);
    oled_write_cmd((x & 0x0F) | 0x00);
}

void ssd1306_show_char(uint8_t x, uint8_t y, char ch, uint8_t fontsize)
{
    uint8_t c = ch - ' ';

    if (x > SSD1306_WIDTH - 1) {
        x = 0;
        y++;
    }

    if (fontsize == 16) {
        // 8x16 字体
        ssd1306_set_position(x, y);
        for (uint8_t i = 0; i < 8; i++) {
            oled_write_data(F8X16[c * 16 + i]);
        }
        ssd1306_set_position(x, y + 1);
        for (uint8_t i = 0; i < 8; i++) {
            oled_write_data(F8X16[c * 16 + i + 8]);
        }
    } else {
        // 6x8 字体
        ssd1306_set_position(x, y);
        for (uint8_t i = 0; i < 6; i++) {
            oled_write_data(F6X8[c][i]);
        }
    }
}

void ssd1306_show_string(uint8_t x, uint8_t y, const char *str, uint8_t fontsize)
{
    while (*str != '\0') {
        ssd1306_show_char(x, y, *str, fontsize);
        x += (fontsize == 16) ? 8 : 6;
        if (x > SSD1306_WIDTH - 8) {
            x = 0;
            y += (fontsize == 16) ? 2 : 1;
        }
        str++;
    }
}

void ssd1306_show_num(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t fontsize)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    uint8_t char_width = (fontsize == 16) ? 8 : 6;  // 6x8字体宽度6，8x16字体宽度8

    for (t = 0; t < len; t++) {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                ssd1306_show_char(x + char_width * t, y, ' ', fontsize);
                continue;
            } else {
                enshow = 1;
            }
        }
        ssd1306_show_char(x + char_width * t, y, temp + '0', fontsize);
    }
}

void ssd1306_show_float(uint8_t x, uint8_t y, float num, uint8_t decimals, uint8_t fontsize)
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t t = 0;
    uint8_t temp = 0;
    uint16_t numel = 0;
    uint32_t integer = 0;
    float frac = 0;
    uint8_t char_width = (fontsize == 16) ? 8 : 6;  // 6x8字体宽度6，8x16字体宽度8

    // 处理负数
    if (num < 0) {
        ssd1306_show_char(x, y, '-', fontsize);
        num = -num;
        i++;
    }

    integer = (uint32_t)num;
    frac = num - integer;

    // 整数部分
    if (integer) {
        numel = integer;
        while (numel) {
            numel /= 10;
            j++;
        }
        i += (j - 1);
        for (temp = 0; temp < j; temp++) {
            ssd1306_show_char(x + char_width * (i - temp), y, integer % 10 + '0', fontsize);
            integer /= 10;
        }
    } else {
        ssd1306_show_char(x + char_width * i, y, '0', fontsize);
    }
    i++;

    // 小数部分
    if (decimals) {
        ssd1306_show_char(x + char_width * i, y, '.', fontsize);
        i++;
        for (t = 0; t < decimals; t++) {
            frac *= 10;
            temp = (uint8_t)frac;
            ssd1306_show_char(x + char_width * (i + t), y, temp + '0', fontsize);
            frac -= temp;
        }
    }
}
