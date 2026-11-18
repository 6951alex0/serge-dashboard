#include "screen.h"
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <TouchDrvCSTXXX.hpp>
#include "config.h"

namespace {

Arduino_DataBus *g_bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_CO5300 *g_gfx = new Arduino_CO5300(
    g_bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 0, 0, 0, 0);

TouchDrvCST92xx g_touch;

lv_disp_draw_buf_t g_drawBuf;
lv_color_t *g_buf1 = nullptr;
lv_color_t *g_buf2 = nullptr;

// CO5300 : fenetre de mise a jour obligatoirement alignee sur des
// coordonnees paires (rondage des aires LVGL).
void lvgl_rounder_cb(struct _lv_disp_drv_t *, lv_area_t *area)
{
    if (area->x1 % 2 != 0) area->x1--;
    if (area->y1 % 2 != 0) area->y1--;
    if (area->x2 % 2 == 0) area->x2++;
    if (area->y2 % 2 == 0) area->y2++;
}

void lvgl_flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
    g_gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    lv_disp_flush_ready(disp);
}

volatile bool g_touchPending = false;

void IRAM_ATTR touchIsr()
{
    g_touchPending = true;
}

bool takeTouchPending()
{
    noInterrupts();
    bool p = g_touchPending;
    g_touchPending = false;
    interrupts();
    return p;
}

void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static lv_coord_t lastX = 0;
    static lv_coord_t lastY = 0;
    static bool pressed = false;

    if (!takeTouchPending()) {
        data->state = pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
        data->point.x = lastX;
        data->point.y = lastY;
        return;
    }

    int16_t x[2], y[2];
    uint8_t n = g_touch.getPoint(x, y, g_touch.getSupportTouchPoint());
    if (n > 0) {
        pressed = true;
        lastX = x[0];
        lastY = y[0];
        data->point.x = lastX;
        data->point.y = lastY;
        data->state = LV_INDEV_STATE_PR;
    } else {
        pressed = false;
        data->point.x = lastX;
        data->point.y = lastY;
        data->state = LV_INDEV_STATE_REL;
    }
    (void)drv;
}

} // namespace

bool screen_begin()
{
    pinMode(TP_RST, OUTPUT);
    digitalWrite(TP_RST, HIGH);

    if (!g_touch.begin(Wire, CST92XX_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
        Serial.println("[screen] tactile CST9220 introuvable");
    } else {
        g_touch.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
        // Mapping du panneau rond (motif officiel Waveshare, rotation 0)
        g_touch.setSwapXY(true);
        g_touch.setMirrorXY(true, false);
        pinMode(TP_INT, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(TP_INT), touchIsr, FALLING);
    }

    if (!g_gfx->begin()) {
        Serial.println("[screen] echec init CO5300");
        return false;
    }
    g_bus->writeC8D8(0x36, 0xA0);   // orientation officielle du panneau rond
    g_gfx->fillScreen(RGB565_BLACK);
    g_gfx->setBrightness(LCD_BRIGHTNESS);

    return true;
}

void screen_lvgl_register()
{
    size_t bufPx = LCD_WIDTH * LCD_HEIGHT / 8;   // 2 x 57.6 Ko en RAM interne
    g_buf1 = (lv_color_t *)heap_caps_malloc(bufPx * sizeof(lv_color_t),
                                            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    g_buf2 = (lv_color_t *)heap_caps_malloc(bufPx * sizeof(lv_color_t),
                                            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    lv_disp_draw_buf_init(&g_drawBuf, g_buf1, g_buf2, bufPx);

    static lv_disp_drv_t dispDrv;
    lv_disp_drv_init(&dispDrv);
    dispDrv.hor_res = LCD_WIDTH;
    dispDrv.ver_res = LCD_HEIGHT;
    dispDrv.flush_cb = lvgl_flush_cb;
    dispDrv.rounder_cb = lvgl_rounder_cb;
    dispDrv.draw_buf = &g_drawBuf;
    lv_disp_drv_register(&dispDrv);

    static lv_indev_drv_t indevDrv;
    lv_indev_drv_init(&indevDrv);
    indevDrv.type = LV_INDEV_TYPE_POINTER;
    indevDrv.read_cb = lvgl_touch_cb;
    lv_indev_drv_register(&indevDrv);
}
