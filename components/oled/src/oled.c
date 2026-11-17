/*
 * oled.c - Driver OLED SSD1306 128x64 (I2C, alamat 0x3C).
 *
 * Asal: components/oled.c di project PPM-esp. Perubahan:
 *   - driver I2C lama (driver/i2c.h) -> driver I2C baru (driver/i2c_master.h)
 *   - oled_print menerima const char *
 *   - tambah oled_printf
 *   - logo bitmap tidak ikut (khusus project lama)
 */
#include "oled.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "oled";

#define I2C_PORT  I2C_NUM_0
#define SDA_PIN   21
#define SCL_PIN   22

#define OLED_ADDR 0x3C

#define OLED_WIDTH   128
#define OLED_PAGES   8
#define LINE_CHARS   21          /* 128 px / 6 px per karakter */

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

static uint8_t s_cursor_x;
static uint8_t s_cursor_y;

/* ================= I2C ================= */

/* ctrl: 0x00 = perintah, 0x40 = data */
static void oled_write(uint8_t ctrl, const uint8_t *data, size_t len)
{
    uint8_t buf[1 + OLED_WIDTH];

    if (!s_dev || len > OLED_WIDTH) return;

    buf[0] = ctrl;
    memcpy(&buf[1], data, len);
    i2c_master_transmit(s_dev, buf, len + 1, 20);
}

static void oled_cmd(uint8_t cmd)
{
    oled_write(0x00, &cmd, 1);
}

/* ================= OLED ================= */

void oled_set_cursor(uint8_t x, uint8_t y)
{
    s_cursor_x = x;
    s_cursor_y = y;

    oled_cmd(0xB0 + y);                  /* pilih halaman */
    oled_cmd(0x00 + (x & 0x0F));         /* kolom bawah   */
    oled_cmd(0x10 + ((x >> 4) & 0x0F));  /* kolom atas    */
}

void oled_clear(void)
{
    static const uint8_t zero[OLED_WIDTH] = { 0 };

    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        oled_set_cursor(0, page);
        oled_write(0x40, zero, OLED_WIDTH);
    }
    oled_set_cursor(0, 0);
}

void oled_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_PORT,
        .sda_io_num        = SDA_PIN,
        .scl_io_num        = SCL_PIN,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = OLED_ADDR,
        .scl_speed_hz    = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));

    /* Tanpa panel, setiap transaksi akan menunggu timeout dan itu memperlambat
     * loop pemanggil. Lebih baik ketahuan di awal. */
    if (i2c_master_probe(s_bus, OLED_ADDR, 50) != ESP_OK) {
        ESP_LOGW(TAG, "OLED tidak menjawab di 0x%02X, tampilan dilewati", OLED_ADDR);
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return;
    }

    oled_cmd(0xAE);
    oled_cmd(0x20);
    oled_cmd(0x00);
    oled_cmd(0xB0);
    oled_cmd(0xC8);
    oled_cmd(0x00);
    oled_cmd(0x10);
    oled_cmd(0x40);
    oled_cmd(0x81);
    oled_cmd(0x7F);
    oled_cmd(0xA1);
    oled_cmd(0xA6);
    oled_cmd(0xA8);
    oled_cmd(0x3F);
    oled_cmd(0xA4);
    oled_cmd(0xD3);
    oled_cmd(0x00);
    oled_cmd(0xD5);
    oled_cmd(0xF0);
    oled_cmd(0xD9);
    oled_cmd(0x22);
    oled_cmd(0xDA);
    oled_cmd(0x12);
    oled_cmd(0xDB);
    oled_cmd(0x20);
    oled_cmd(0x8D);
    oled_cmd(0x14);
    oled_cmd(0xAF);

    ESP_LOGI(TAG, "SSD1306 siap (SDA=%d SCL=%d)", SDA_PIN, SCL_PIN);
}

/* ================= FONT 5x7 ================= */

static const uint8_t font[][5] = {
/* angka */
{0x3E,0x51,0x49,0x45,0x3E}, // 0
{0x00,0x42,0x7F,0x40,0x00}, // 1
{0x62,0x51,0x49,0x49,0x46}, // 2
{0x22,0x49,0x49,0x49,0x36}, // 3
{0x18,0x14,0x12,0x7F,0x10}, // 4
{0x2F,0x49,0x49,0x49,0x31}, // 5
{0x3E,0x49,0x49,0x49,0x32}, // 6
{0x01,0x71,0x09,0x05,0x03}, // 7
{0x36,0x49,0x49,0x49,0x36}, // 8
{0x26,0x49,0x49,0x49,0x3E}, // 9
/* huruf besar */
{0x7E,0x11,0x11,0x11,0x7E}, // A
{0x7F,0x49,0x49,0x49,0x36}, // B
{0x3E,0x41,0x41,0x41,0x22}, // C
{0x7F,0x41,0x41,0x22,0x1C}, // D
{0x7F,0x49,0x49,0x49,0x41}, // E
{0x7F,0x09,0x09,0x09,0x01}, // F
{0x3E,0x41,0x49,0x49,0x7A}, // G
{0x7F,0x08,0x08,0x08,0x7F}, // H
{0x00,0x41,0x7F,0x41,0x00}, // I
{0x20,0x40,0x41,0x3F,0x01}, // J
{0x7F,0x08,0x14,0x22,0x41}, // K
{0x7F,0x40,0x40,0x40,0x40}, // L
{0x7F,0x02,0x04,0x02,0x7F}, // M
{0x7F,0x04,0x08,0x10,0x7F}, // N
{0x3E,0x41,0x41,0x41,0x3E}, // O
{0x7F,0x09,0x09,0x09,0x06}, // P
{0x3E,0x41,0x51,0x21,0x5E}, // Q
{0x7F,0x09,0x19,0x29,0x46}, // R
{0x46,0x49,0x49,0x49,0x31}, // S
{0x01,0x01,0x7F,0x01,0x01}, // T
{0x3F,0x40,0x40,0x40,0x3F}, // U
{0x1F,0x20,0x40,0x20,0x1F}, // V
{0x3F,0x40,0x38,0x40,0x3F}, // W
{0x63,0x14,0x08,0x14,0x63}, // X
{0x07,0x08,0x70,0x08,0x07}, // Y
{0x61,0x51,0x49,0x45,0x43}, // Z
/* huruf kecil */
{0x20,0x54,0x54,0x54,0x78}, // a
{0x7F,0x48,0x44,0x44,0x38}, // b
{0x38,0x44,0x44,0x44,0x20}, // c
{0x38,0x44,0x44,0x48,0x7F}, // d
{0x38,0x54,0x54,0x54,0x18}, // e
{0x08,0x7E,0x09,0x01,0x02}, // f
{0x08,0x14,0x54,0x54,0x3C}, // g
{0x7F,0x08,0x04,0x04,0x78}, // h
{0x00,0x44,0x7D,0x40,0x00}, // i
{0x20,0x40,0x44,0x3D,0x00}, // j
{0x7F,0x10,0x28,0x44,0x00}, // k
{0x00,0x41,0x7F,0x40,0x00}, // l
{0x7C,0x04,0x18,0x04,0x78}, // m
{0x7C,0x08,0x04,0x04,0x78}, // n
{0x38,0x44,0x44,0x44,0x38}, // o
{0x7C,0x14,0x14,0x14,0x08}, // p
{0x08,0x14,0x14,0x18,0x7C}, // q
{0x7C,0x08,0x04,0x04,0x08}, // r
{0x48,0x54,0x54,0x54,0x20}, // s
{0x04,0x3F,0x44,0x40,0x20}, // t
{0x3C,0x40,0x40,0x20,0x7C}, // u
{0x1C,0x20,0x40,0x20,0x1C}, // v
{0x3C,0x40,0x30,0x40,0x3C}, // w
{0x44,0x28,0x10,0x28,0x44}, // x
{0x0C,0x50,0x50,0x50,0x3C}, // y
{0x44,0x64,0x54,0x4C,0x44}, // z
/* tambahan */
{0x00,0x00,0x00,0x00,0x00}, // spasi
{0x00,0x60,0x60,0x00,0x00}, // .
{0x00,0x36,0x36,0x00,0x00}, // :
{0x08,0x08,0x08,0x08,0x08}, // -
{0x20,0x10,0x08,0x04,0x02}, // /
{0x62,0x64,0x08,0x13,0x23}, // %
};

static bool font_glyph(char c, uint8_t out[6])
{
    uint8_t index;

    if (c >= '0' && c <= '9')      index = (uint8_t)(c - '0');
    else if (c >= 'A' && c <= 'Z') index = (uint8_t)((c - 'A') + 10);
    else if (c >= 'a' && c <= 'z') index = (uint8_t)((c - 'a') + 36);
    else if (c == ' ')             index = 62;
    else if (c == '.')             index = 63;
    else if (c == ':')             index = 64;
    else if (c == '-')             index = 65;
    else if (c == '/')             index = 66;
    else if (c == '%')             index = 67;
    else return false;              /* karakter tak ada di font: lewati */

    for (int i = 0; i < 5; i++) out[i] = font[index][i];
    out[5] = 0x00;
    return true;
}

/*
 * Cetak satu baris dalam SATU transaksi I2C. Kalau per karakter, satu baris
 * butuh ~130 transaksi (~50 ms) dan itu cukup untuk merusak irama poll pedal
 * 10 ms.
 */
void oled_print(const char *str)
{
    uint8_t buf[OLED_WIDTH];
    size_t  n = 0;

    while (*str) {
        uint8_t glyph[6];
        str++;
        if (!font_glyph(str[-1], glyph)) continue;

        if (n + sizeof(glyph) > sizeof(buf)) {
            oled_write(0x40, buf, n);
            n = 0;
        }
        memcpy(&buf[n], glyph, sizeof(glyph));
        n += sizeof(glyph);
        s_cursor_x += 6;
    }
    if (n) oled_write(0x40, buf, n);
}

void oled_printf(uint8_t x, uint8_t y, const char *fmt, ...)
{
    char buf[LINE_CHARS + 1];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    /* isi sisa baris dengan spasi supaya teks lama tidak tertinggal */
    size_t n = strlen(buf);
    while (n < LINE_CHARS && (x + n) < LINE_CHARS) buf[n++] = ' ';
    buf[n] = '\0';

    oled_set_cursor(x, y);
    oled_print(buf);
}

void oled_draw_bitmap(const uint8_t *bitmap, uint8_t w, uint8_t h)
{
    if (!bitmap || w == 0 || w > OLED_WIDTH) return;

    for (uint8_t page = 0; page < (uint8_t)(h / 8); page++) {
        oled_set_cursor(0, page);
        oled_write(0x40, &bitmap[page * w], w);
    }
}
