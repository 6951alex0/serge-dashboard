// ---------------------------------------------------------------------------
// Interface LVGL — calquee sur le design utilisateur (sketch de reference).
// 4 pages pleine page en tileview, fond noir pur :
//   Piste    : bargraph EGT en equerre arrondie (degrade continu) + chronos
//   G-Force  : cible + point dynamique, Lat/Lon, RPM, km/h
//   Banc     : puissance / couple + pics max + RAZ
//   Parametrage : Poids, S.Cx, tare IMU, ligne chrono GPS, record SD
// ---------------------------------------------------------------------------
#include "ui.h"
#include <Arduino.h>
#include <lvgl.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <esp_heap_caps.h>
#include "config.h"
#include "data_model.h"
#include "drivers/screen.h"
#include "compute/acq.h"
#include "ble/ble_server.h"
#include "storage/sd_logger.h"

namespace {

// ------------------------------ Palette -------------------------------------
const lv_color_t CL_GREEN  = lv_color_hex(0x00E676);
const lv_color_t CL_ORANGE = lv_color_hex(0xF58220);
const lv_color_t CL_RED    = lv_color_hex(0xFF1744);
const lv_color_t CL_YELLOW = lv_color_hex(0xF5C542);
const lv_color_t CL_BADGED = lv_color_hex(0xE53935);
const lv_color_t CL_CYAN   = lv_color_hex(0x00E5FF);
const lv_color_t CL_WHITE  = lv_color_hex(0xFFFFFF);
const lv_color_t CL_BLACK  = lv_color_hex(0x000000);

// ------------------- Equerre EGT (canvas degrade continu) -------------------
// Canvas 240x310 place en (8,8). Coude quart de cercle centre (60,60) :
//   branche verticale x 0..26  (bas y=309 -> coude y=60)
//   coude : anneau r34..r60 autour de (60,60), quadrant haut-gauche
//   branche horizontale y 0..26 (coude x=60 -> x=227)
// Graduent continu le long du chemin, plage 300..700 C.
const int EQ_W = 240, EQ_H = 310;
const int EQ_CX = 60, EQ_CY = 60;
const int EQ_R_IN = 34, EQ_R_OUT = 60;
const float EQ_MID_R = (EQ_R_IN + EQ_R_OUT) / 2.0f;          // 47
const float EQ_VERT_LEN = (EQ_H - 1) - EQ_CY;                 // 249
const float EQ_ARC_LEN = 1.5707963f * EQ_MID_R;               // ~73.8
const float EQ_HOR_LEN = (EQ_W - 13) - EQ_CX;                 // 167
const float EQ_LEN_TOT = EQ_VERT_LEN + EQ_ARC_LEN + EQ_HOR_LEN; // ~490

// temp <-> position sur le chemin (300 C en bas, 700 C au bout)
// norm = clamp((T-300)/400)

// Couleur continue : vert fonce (300) -> orange (550) -> rouge vif (700)
lv_color_t colorAtTemp(float temp)
{
    if (temp <= 550.0f) {
        float r = (temp - 300.0f) / 250.0f;
        if (r < 0) r = 0;
        uint8_t rr = (uint8_t)(0x1F + (0xF5 - 0x1F) * r);
        uint8_t gg = (uint8_t)(0xA0 + (0x82 - 0xA0) * r);
        uint8_t bb = (uint8_t)(0x55 + (0x20 - 0x55) * r);
        return lv_color_make(rr, gg, bb);
    }
    float r = (temp - 550.0f) / 150.0f;
    if (r > 1) r = 1;
    uint8_t rr = (uint8_t)(0xF5 + (0xFF - 0xF5) * r);
    uint8_t gg = (uint8_t)(0x82 + (0x17 - 0x82) * r);
    uint8_t bb = (uint8_t)(0x20 + (0x44 - 0x20) * r);
    return lv_color_make(rr, gg, bb);
}

// Dessin du canvas : 1 pixel = 1 position le long du chemin -> degrade continu
void drawEquerre(lv_obj_t *canvas)
{
    for (int py = 0; py < EQ_H; py++) {
        for (int px = 0; px < EQ_W; px++) {
            float distAlong = -1.0f;
            bool inBand = false;
            if (px <= EQ_R_OUT - EQ_R_IN && py > EQ_CY) {           // branche verticale
                inBand = (py <= EQ_H - 13) ||
                         ((px - 13) * (px - 13) + (py - (EQ_H - 13)) * (py - (EQ_H - 13)) <= 169);
                if (inBand) distAlong = (EQ_H - 1) - py;
            } else if (py <= EQ_R_OUT - EQ_R_IN && px > EQ_CX) {    // branche horizontale
                inBand = (px <= EQ_W - 14) ||
                         ((px - (EQ_W - 14)) * (px - (EQ_W - 14)) + (py - 13) * (py - 13) <= 169);
                if (inBand) distAlong = EQ_VERT_LEN + EQ_ARC_LEN + (px - EQ_CX);
            } else if (px <= EQ_CX && py <= EQ_CY) {                // coude (quart de cercle)
                float dx = px - EQ_CX, dy = py - EQ_CY;
                float d = sqrtf(dx * dx + dy * dy);
                if (d <= EQ_R_OUT + 0.5f && d >= EQ_R_IN - 0.5f) {
                    float frac = atan2f(EQ_CY - py, EQ_CX - px) / 1.5707963f;
                    distAlong = EQ_VERT_LEN + frac * EQ_ARC_LEN;
                    inBand = true;
                }
            }
            if (!inBand) continue;
            float t = distAlong / EQ_LEN_TOT;
            if (t < 0) t = 0;
            if (t > 1) t = 1;
            lv_canvas_set_px(canvas, px, py, colorAtTemp(300.0f + t * 400.0f));
        }
    }
}

// ------------------------ Widgets (pointeurs globaux) ------------------------
static lv_obj_t *g_tileview = nullptr;
static lv_obj_t *g_tilePiste = nullptr;
static lv_obj_t *g_tileGForce = nullptr;
static lv_obj_t *g_tileDyno = nullptr;
static lv_obj_t *g_tileSetup = nullptr;

// Piste
static lv_obj_t *wEgtVal, *wCurLap, *wBestLap, *wLastLap;
static lv_obj_t *wGpsBadge, *wGpsSats, *wGpsAcc, *wSdBadge, *wBleBadge;
static lv_obj_t *wEgtCursorMax;

// G-Force
static lv_obj_t *wGDot, *wGLat, *wGLon, *wRpmG, *wSpdG;

// Banc
static lv_obj_t *wDynoHp, *wDynoNm, *wDynoMaxHp, *wDynoMaxNm, *wDynoMaxEgt;

// Parametrage
static lv_obj_t *wMassVal, *wScxVal, *wSetupStatus, *wRecBtn, *wRecBtnLabel;

// Maxima pics banc (RAZ possibles)
static float s_dynoPMax = 0.0f;
static float s_dynoCMax = 0.0f;

// ------------------------------- Helpers ------------------------------------
lv_obj_t *makeLabel(lv_obj_t *parent, const char *txt, const lv_font_t *font,
                    lv_color_t color, int32_t x, int32_t y)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, color, 0);
    if (font) lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_pos(l, x, y);
    return l;
}

lv_obj_t *makeLabelAlign(lv_obj_t *parent, const char *txt, const lv_font_t *font,
                         lv_color_t color, lv_align_t align, int32_t x, int32_t y)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, color, 0);
    if (font) lv_obj_set_style_text_font(l, font, 0);
    lv_obj_align(l, align, x, y);
    return l;
}

lv_obj_t *makeBadgeBox(lv_obj_t *parent, int32_t w, int32_t h, int32_t x, int32_t y,
                       lv_color_t bg)
{
    lv_obj_t *b = lv_obj_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_border_width(b, 0, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    return b;
}

lv_obj_t *makeColorBtn(lv_obj_t *parent, int32_t w, int32_t h, lv_align_t align,
                       int32_t x, int32_t y, lv_color_t bg, const char *txt,
                       const lv_font_t *font, lv_event_cb_t cb)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_align(b, align, x, y);
    lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_border_width(b, 0, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_center(l);
    return b;
}

// Chrono au format de l'utilisateur : 00"26,98
void fmtLap(uint32_t ms, char *buf, size_t len)
{
    if (ms == 0) { snprintf(buf, len, "--\"--,--"); return; }
    uint32_t m = ms / 60000;
    uint32_t s = (ms % 60000) / 1000;
    uint32_t cs = (ms % 1000) / 10;
    snprintf(buf, len, "%02lu\"%02lu,%02lu", (unsigned long)m,
             (unsigned long)s, (unsigned long)cs);
}

// -------------------------- Page 1 : PISTE ----------------------------------
void buildTrackPage(lv_obj_t *parent)
{
    // A. Barre en equerre arrondie, degrade continu (canvas, dessin 1 fois)
    static lv_color_t *eqBuf = nullptr;
    if (!eqBuf) {
        size_t bytes = LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(EQ_W, EQ_H);
        eqBuf = (lv_color_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
        if (!eqBuf) eqBuf = (lv_color_t *)heap_caps_malloc(bytes, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    }
    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(canvas, eqBuf, EQ_W, EQ_H, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_canvas_fill_bg(canvas, CL_BLACK, LV_OPA_TRANSP);
    lv_obj_set_pos(canvas, 8, 8);
    drawEquerre(canvas);

    // Label EGT (dans le creux du coude)
    makeLabel(parent, "EGT", &lv_font_montserrat_24, CL_WHITE, 40, 40);

    // Curseur noir : T deg max ENREGISTREE durant la session (300..700 C)
    wEgtCursorMax = lv_obj_create(parent);
    lv_obj_remove_style_all(wEgtCursorMax);
    lv_obj_set_size(wEgtCursorMax, 30, 5);
    lv_obj_set_style_bg_color(wEgtCursorMax, CL_BLACK, 0);
    lv_obj_set_style_bg_opa(wEgtCursorMax, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(wEgtCursorMax, 2, 0);
    lv_obj_set_pos(wEgtCursorMax, 6, 316);

    // B. Badges etat haut droite
    wGpsBadge = makeBadgeBox(parent, 110, 42, 265, 10, CL_YELLOW);
    wGpsSats = lv_label_create(wGpsBadge);
    lv_label_set_text(wGpsSats, "0Gps");
    lv_obj_set_style_text_color(wGpsSats, CL_BLACK, 0);
    lv_obj_set_style_text_font(wGpsSats, &lv_font_montserrat_28, 0);
    lv_obj_set_style_pad_all(wGpsBadge, 0, 0);
    lv_obj_center(wGpsSats);

    wGpsAcc = makeLabel(parent, "~--m", &lv_font_montserrat_24, CL_WHITE, 272, 58);

    wSdBadge = makeBadgeBox(parent, 85, 42, 385, 10, CL_BADGED);
    lv_obj_t *lblSd = lv_label_create(wSdBadge);
    lv_label_set_text(lblSd, "SD");
    lv_obj_set_style_text_color(lblSd, CL_BLACK, 0);
    lv_obj_set_style_text_font(lblSd, &lv_font_montserrat_28, 0);
    lv_obj_center(lblSd);

    wBleBadge = makeBadgeBox(parent, 85, 36, 385, 58, CL_BADGED);
    lv_obj_t *lblBle = lv_label_create(wBleBadge);
    lv_label_set_text(lblBle, "BLE");
    lv_obj_set_style_text_color(lblBle, CL_BLACK, 0);
    lv_obj_set_style_text_font(lblBle, &lv_font_montserrat_22, 0);
    lv_obj_center(lblBle);

    // C. Valeur EGT geante (alarme = couleur du chiffre uniquement)
    wEgtVal = lv_label_create(parent);
    lv_label_set_text(wEgtVal, "---\xC2\xB0" "c");
    lv_obj_set_style_text_color(wEgtVal, CL_GREEN, 0);
    lv_obj_set_style_text_font(wEgtVal, &lv_font_montserrat_48, 0);
    lv_obj_set_style_transform_zoom(wEgtVal, 480, 0);   // 48 px -> ~90 px
    lv_obj_set_style_transform_pivot_x(wEgtVal, 75, 0);
    lv_obj_set_style_transform_pivot_y(wEgtVal, 28, 0);
    lv_obj_align(wEgtVal, LV_ALIGN_CENTER, 15, -35);

    // D. Chronos empiles centre
    wCurLap = makeLabelAlign(parent, "00\"00,00", &lv_font_montserrat_48, CL_WHITE,
                             LV_ALIGN_CENTER, 0, 75);
    wBestLap = makeLabelAlign(parent, "BestLap: --\"--,--", &lv_font_montserrat_32, CL_WHITE,
                              LV_ALIGN_CENTER, 0, 140);
    wLastLap = makeLabelAlign(parent, "LastLap: --\"--,--", &lv_font_montserrat_32, CL_WHITE,
                              LV_ALIGN_CENTER, 0, 185);
}

void updateEgtCursor(float egtMax)
{
    float ratio = (egtMax - 300.0f) / 400.0f;   // plage 300..700 C
    if (ratio < 0) ratio = 0;
    if (ratio > 1) ratio = 1;
    float dist = ratio * EQ_LEN_TOT;

    if (dist <= EQ_VERT_LEN) {
        // Branche verticale : curseur horizontal 30x5
        int yCanvas = (int)((EQ_H - 1) - dist);
        lv_obj_set_size(wEgtCursorMax, 30, 5);
        lv_obj_set_pos(wEgtCursorMax, 6, 8 + yCanvas - 2);
    } else if (dist <= EQ_VERT_LEN + EQ_ARC_LEN) {
        // Coude arrondi : petit point noir suivant l'arc
        float frac = (dist - EQ_VERT_LEN) / EQ_ARC_LEN;
        float a = frac * 1.5707963f;
        int cx = 8 + EQ_CX + (int)(-EQ_MID_R * cosf(a));
        int cy = 8 + EQ_CY + (int)(-EQ_MID_R * sinf(a));
        lv_obj_set_size(wEgtCursorMax, 12, 12);
        lv_obj_set_style_radius(wEgtCursorMax, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_pos(wEgtCursorMax, cx - 6, cy - 6);
    } else {
        // Branche horizontale : curseur vertical 5x30
        int x = 8 + EQ_CX + (int)(dist - EQ_VERT_LEN - EQ_ARC_LEN);
        lv_obj_set_size(wEgtCursorMax, 5, 30);
        lv_obj_set_pos(wEgtCursorMax, x - 2, 8 + 13 - 15);
    }
    lv_obj_invalidate(wEgtCursorMax);
}

void updateTrackPage(const TelemetryFrame &f)
{
    // EGT + coloration par seuils (alarme sur le chiffre uniquement)
    if (f.egt_c > 0) {
        char b[16];
        snprintf(b, sizeof(b), "%.0f\xC2\xB0" "c", f.egt_c);
        lv_label_set_text(wEgtVal, b);
    } else {
        lv_label_set_text(wEgtVal, "---\xC2\xB0" "c");
    }
    lv_color_t egtCol = CL_GREEN;
    if (f.egt_c >= EGT_RED_C) egtCol = CL_RED;
    else if (f.egt_c >= EGT_ORANGE_C) egtCol = CL_ORANGE;
    lv_obj_set_style_text_color(wEgtVal, egtCol, 0);

    if (f.egt_max_c > 0) updateEgtCursor(f.egt_max_c);

    // GPS compact : badge sats + precision (pad < 0.5u => ~%.1fm, val by user)
    GpsFix g = gps_get();
    lv_label_set_text_fmt(wGpsSats, "%uGps", (unsigned)g.sats);
    lv_label_set_text_fmt(wGpsAcc, "~%.1fm", g.hdop * 0.5f);
    lv_color_t gpsBg;
    if (g.sats >= 8 && g.hdop < 1.5f) gpsBg = CL_GREEN;
    else if (g.sats >= 5) gpsBg = CL_YELLOW;
    else gpsBg = CL_BADGED;
    lv_obj_set_style_bg_color(wGpsBadge, gpsBg, 0);

    // SD / BLE : vert ok, rouge pas ok
    lv_obj_set_style_bg_color(wSdBadge, sd_ready() ? CL_GREEN : CL_BADGED, 0);
    lv_obj_set_style_bg_color(wBleBadge, ble_connected() ? CL_GREEN : CL_BADGED, 0);

    // Chronos
    ChronoState c = chrono_get();
    char b1[24], b2[24], b3[24];
    fmtLap(c.current_lap_ms, b1, sizeof(b1));
    lv_label_set_text(wCurLap, b1);
    fmtLap(c.best_lap_ms, b2, sizeof(b2));
    lv_label_set_text_fmt(wBestLap, "BestLap: %s", b2);
    fmtLap(c.last_lap_ms, b3, sizeof(b3));
    lv_label_set_text_fmt(wLastLap, "LastLap: %s", b3);
}

// -------------------------- Page 2 : G-FORCE --------------------------------
void buildGForcePage(lv_obj_t *parent)
{
    makeLabelAlign(parent, "G-FORCE & DYNAMIQUE", &lv_font_montserrat_24, CL_ORANGE,
                   LV_ALIGN_TOP_MID, 0, 15);

    lv_obj_t *outer = lv_obj_create(parent);
    lv_obj_set_size(outer, 260, 260);
    lv_obj_align(outer, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_radius(outer, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(outer, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(outer, lv_color_hex(0x555555), 0);
    lv_obj_set_style_border_width(outer, 2, 0);
    lv_obj_clear_flag(outer, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *inner = lv_obj_create(outer);
    lv_obj_set_size(inner, 130, 130);
    lv_obj_center(inner);
    lv_obj_set_style_radius(inner, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(inner, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(inner, lv_color_hex(0x444444), 0);
    lv_obj_set_style_border_width(inner, 1, 0);
    lv_obj_clear_flag(inner, LV_OBJ_FLAG_SCROLLABLE);

    wGDot = lv_obj_create(outer);
    lv_obj_set_size(wGDot, 24, 24);
    lv_obj_set_style_radius(wGDot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(wGDot, CL_GREEN, 0);
    lv_obj_set_style_bg_opa(wGDot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(wGDot, 0, 0);
    lv_obj_set_style_shadow_width(wGDot, 0, 0);
    lv_obj_clear_flag(wGDot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_center(wGDot);

    wGLat = makeLabel(parent, "Lat: 0.00 G", &lv_font_montserrat_26, CL_WHITE, 30, 360);
    wGLon = makeLabel(parent, "Lon: 0.00 G", &lv_font_montserrat_26, CL_WHITE, 260, 360);
    wRpmG = makeLabel(parent, "0 RPM", &lv_font_montserrat_28, CL_ORANGE, 30, 415);
    wSpdG = makeLabel(parent, "0 km/h", &lv_font_montserrat_28, CL_CYAN, 280, 415);
}

void updateGForcePage(const TelemetryFrame &f)
{
    int dotX = (int)(f.g_lat * 80.0f);
    int dotY = (int)(-f.g_long * 80.0f);
    if (dotX < -110) dotX = -110;
    if (dotX > 110) dotX = 110;
    if (dotY < -110) dotY = -110;
    if (dotY > 110) dotY = 110;
    lv_obj_align(wGDot, LV_ALIGN_CENTER, dotX, dotY);
    lv_label_set_text_fmt(wGLat, "Lat: %.2f G", f.g_lat);
    lv_label_set_text_fmt(wGLon, "Lon: %.2f G", f.g_long);
    lv_label_set_text_fmt(wRpmG, "%u RPM", f.rpm);
    lv_label_set_text_fmt(wSpdG, "%.0f km/h", f.speed_kmh);
}

// -------------------------- Page 3 : BANC D'ESSAI ---------------------------
void updateDynoLabels()
{
    lv_label_set_text_fmt(wDynoMaxHp, "P. MAX  : %.1f ch", s_dynoPMax);
    lv_label_set_text_fmt(wDynoMaxNm, "C. MAX  : %.1f Nm", s_dynoCMax);
}

void razPicsCb(lv_event_t *)
{
    s_dynoPMax = 0.0f;
    s_dynoCMax = 0.0f;
    acq_resetMaxima();
    updateDynoLabels();
    lv_label_set_text_fmt(wDynoMaxEgt, "EGT MAX : --- \xC2\xB0" "c");
}

void buildDynoPage(lv_obj_t *parent)
{
    makeLabelAlign(parent, "MODE BANC D'ESSAI", &lv_font_montserrat_26, CL_GREEN,
                   LV_ALIGN_TOP_MID, 0, 15);

    wDynoHp = makeLabel(parent, "0.0 ch", &lv_font_montserrat_48, CL_WHITE, 35, 75);
    wDynoNm = makeLabel(parent, "0.0 Nm", &lv_font_montserrat_48, CL_CYAN, 250, 75);

    lv_obj_t *boxMax = lv_obj_create(parent);
    lv_obj_set_size(boxMax, 420, 200);
    lv_obj_align(boxMax, LV_ALIGN_CENTER, 0, 30);
    lv_obj_set_style_bg_color(boxMax, lv_color_hex(0x141414), 0);
    lv_obj_set_style_border_color(boxMax, CL_ORANGE, 0);
    lv_obj_set_style_border_width(boxMax, 2, 0);
    lv_obj_clear_flag(boxMax, LV_OBJ_FLAG_SCROLLABLE);

    wDynoMaxHp = makeLabel(boxMax, "P. MAX  : 0.0 ch", &lv_font_montserrat_30, CL_GREEN, 15, 15);
    wDynoMaxNm = makeLabel(boxMax, "C. MAX  : 0.0 Nm", &lv_font_montserrat_30, CL_CYAN, 15, 70);
    wDynoMaxEgt = makeLabel(boxMax, "EGT MAX : --- \xC2\xB0" "c", &lv_font_montserrat_30,
                            CL_ORANGE, 15, 125);

    makeColorBtn(parent, 260, 55, LV_ALIGN_BOTTOM_MID, 0, -25, lv_color_hex(0xD32F2F),
                 "RAZ PICS BANC", &lv_font_montserrat_24, razPicsCb);
}

void updateDynoPage(const TelemetryFrame &f)
{
    float hp = f.power_kw * 1.35962f;   // ch
    lv_label_set_text_fmt(wDynoHp, "%.1f ch", hp);
    lv_label_set_text_fmt(wDynoNm, "%.1f Nm", f.torque_nm);
    if (f.egt_max_c > 0) {
        lv_label_set_text_fmt(wDynoMaxEgt, "EGT MAX : %.0f \xC2\xB0" "c", f.egt_max_c);
    }
    updateDynoLabels();
}

// -------------------------- Page 4 : PARAMETRAGE ----------------------------
void recBtnCb(lv_event_t *)
{
    TelemetryFrame f = telemetry_get();
    cmds_requestRecord(!f.recording);
}

void tareImuCb(lv_event_t *)
{
    acq_calibrateFlat();
    lv_label_set_text(wSetupStatus, "IMU Tare OK !");
}

void setLineCb(lv_event_t *)
{
    GpsFix g = gps_get();
    if (g.valid) {
        settings_lapLineHere(g);
        lv_label_set_text(wSetupStatus, "Ligne GPS Enregistree !");
    } else {
        lv_label_set_text(wSetupStatus, "Attente Fix GPS...");
    }
}

void massMinusCb(lv_event_t *)
{
    VehicleSettings s = settings_get();
    s.mass_kg -= 1.0f;
    if (s.mass_kg < 40) s.mass_kg = 40;
    settings_update(s);
    lv_label_set_text_fmt(wMassVal, "Poids Total : %.0f kg", s.mass_kg);
}

void massPlusCb(lv_event_t *)
{
    VehicleSettings s = settings_get();
    s.mass_kg += 1.0f;
    if (s.mass_kg > 500) s.mass_kg = 500;
    settings_update(s);
    lv_label_set_text_fmt(wMassVal, "Poids Total : %.0f kg", s.mass_kg);
}

void scxMinusCb(lv_event_t *)
{
    VehicleSettings s = settings_get();
    s.cxsa -= 0.01f;
    if (s.cxsa < 0.10f) s.cxsa = 0.10f;
    settings_update(s);
    lv_label_set_text_fmt(wScxVal, "S.Cx Aero : %.2f", s.cxsa);
}

void scxPlusCb(lv_event_t *)
{
    VehicleSettings s = settings_get();
    s.cxsa += 0.01f;
    settings_update(s);
    lv_label_set_text_fmt(wScxVal, "S.Cx Aero : %.2f", s.cxsa);
}

void buildSetupPage(lv_obj_t *parent)
{
    makeLabelAlign(parent, "PARAMETRAGE & IMU", &lv_font_montserrat_26, CL_WHITE,
                   LV_ALIGN_TOP_MID, 0, 15);

    // Enregistrement SD autonome (bouton conserve, absent du sketch de base)
    wRecBtn = makeColorBtn(parent, 220, 40, LV_ALIGN_TOP_MID, 0, 52, CL_GREEN,
                           "ENREGISTRER", &lv_font_montserrat_20, recBtnCb);
    wRecBtnLabel = (lv_obj_t *)lv_obj_get_child(wRecBtn, 0);

    VehicleSettings st = settings_get();
    wMassVal = lv_label_create(parent);
    lv_label_set_text_fmt(wMassVal, "Poids Total : %.0f kg", st.mass_kg);
    lv_obj_set_style_text_color(wMassVal, CL_WHITE, 0);
    lv_obj_set_style_text_font(wMassVal, &lv_font_montserrat_24, 0);
    lv_obj_set_pos(wMassVal, 30, 110);

    makeColorBtn(parent, 70, 50, LV_ALIGN_TOP_LEFT, 300, 100,
                 lv_color_hex(0x333333), "-1", &lv_font_montserrat_24, massMinusCb);
    makeColorBtn(parent, 70, 50, LV_ALIGN_TOP_LEFT, 385, 100,
                 lv_color_hex(0x333333), "+1", &lv_font_montserrat_24, massPlusCb);

    wScxVal = lv_label_create(parent);
    lv_label_set_text_fmt(wScxVal, "S.Cx Aero : %.2f", st.cxsa);
    lv_obj_set_style_text_color(wScxVal, CL_WHITE, 0);
    lv_obj_set_style_text_font(wScxVal, &lv_font_montserrat_24, 0);
    lv_obj_set_pos(wScxVal, 30, 165);

    makeColorBtn(parent, 70, 50, LV_ALIGN_TOP_LEFT, 300, 155,
                 lv_color_hex(0x333333), "-", &lv_font_montserrat_24, scxMinusCb);
    makeColorBtn(parent, 70, 50, LV_ALIGN_TOP_LEFT, 385, 155,
                 lv_color_hex(0x333333), "+", &lv_font_montserrat_24, scxPlusCb);

    makeColorBtn(parent, 380, 60, LV_ALIGN_CENTER, 0, 55, lv_color_hex(0x0288D1),
                 "RAZ ACCELEROMETRES (TARE)", &lv_font_montserrat_20, tareImuCb);
    makeColorBtn(parent, 380, 55, LV_ALIGN_CENTER, 0, 130, lv_color_hex(0x2E7D32),
                 "DEFINIR LIGNE CHRONO GPS", &lv_font_montserrat_20, setLineCb);

    wSetupStatus = makeLabelAlign(parent, "Pret", &lv_font_montserrat_24, CL_GREEN,
                                  LV_ALIGN_BOTTOM_MID, 0, -15);
}

void updateSetupPage(const TelemetryFrame &f)
{
    lv_obj_set_style_bg_color(wRecBtn, f.recording ? CL_RED : CL_GREEN, 0);
    lv_label_set_text(wRecBtnLabel, f.recording ? "ARRETER" : "ENREGISTRER");
    VehicleSettings s = settings_get();
    lv_label_set_text_fmt(wMassVal, "Poids Total : %.0f kg", s.mass_kg);
    lv_label_set_text_fmt(wScxVal, "S.Cx Aero : %.2f", s.cxsa);
}

// ------------------------------ Timer UI ------------------------------------
void uiTimer(lv_timer_t *)
{
    TelemetryFrame f = telemetry_get();

    // Maxima pics banc : suivis en permanence (toutes pages)
    float hp = f.power_kw * 1.35962f;
    if (hp > s_dynoPMax) s_dynoPMax = hp;
    if (f.torque_nm > s_dynoCMax) s_dynoCMax = f.torque_nm;

    if (!g_tileview) return;
    lv_obj_t *act = lv_tileview_get_tile_act(g_tileview);
    if (act == g_tilePiste) updateTrackPage(f);
    else if (act == g_tileGForce) updateGForcePage(f);
    else if (act == g_tileDyno) updateDynoPage(f);
    else if (act == g_tileSetup) updateSetupPage(f);
}

// --------------------------------- Tache ------------------------------------
void uiTask(void *param)
{
    (void)param;
    for (;;) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

} // namespace

void ui_start()
{
    lv_init();
    screen_lvgl_register();

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, CL_BLACK, 0);

    g_tileview = lv_tileview_create(scr);
    lv_obj_set_style_bg_color(g_tileview, CL_BLACK, 0);

    g_tilePiste  = lv_tileview_add_tile(g_tileview, 0, 0, LV_DIR_RIGHT);
    g_tileGForce = lv_tileview_add_tile(g_tileview, 1, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    g_tileDyno   = lv_tileview_add_tile(g_tileview, 2, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
    g_tileSetup  = lv_tileview_add_tile(g_tileview, 3, 0, LV_DIR_LEFT);

    buildTrackPage(g_tilePiste);
    buildGForcePage(g_tileGForce);
    buildDynoPage(g_tileDyno);
    buildSetupPage(g_tileSetup);

    lv_timer_create(uiTimer, 50, nullptr);

    xTaskCreatePinnedToCore(uiTask, "ui", 16384, nullptr, 2, nullptr, 1);
}
