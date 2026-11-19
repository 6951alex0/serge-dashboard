#include "ui.h"
#include <Arduino.h>
#include <lvgl.h>
#include <cstdio>
#include <cstring>
#include <string>
#include "config.h"
#include "data_model.h"
#include "drivers/screen.h"
#include "compute/acq.h"
#include "ble/ble_server.h"
#include "storage/sd_logger.h"
#include "sensors/gps.h"

namespace {

lv_color_t C_GREEN = lv_color_hex(0x22C55E);
lv_color_t C_ORANGE = lv_color_hex(0xF59E0B);
lv_color_t C_RED = lv_color_hex(0xEF4444);
lv_color_t C_GRAY = lv_color_hex(0x6B7280);
lv_color_t C_WHITE = lv_color_hex(0xFFFFFF);
lv_color_t C_BG = lv_color_hex(0x000000);
lv_color_t C_ACCENT = lv_color_hex(0x38BDF8);
lv_color_t C_DIM = lv_color_hex(0x9CA3AF);

enum PageId : uint8_t { PAGE_RACE = 0, PAGE_TELEM, PAGE_ENGINE, PAGE_DYNO, PAGE_SETTINGS, PAGE_SESSIONS };
static const uint8_t NB_PAGES = 6;

lv_obj_t *g_pages[5];
uint8_t g_pageIdx = PAGE_RACE;

// Course
lv_obj_t *wEgtFill, *wEgtLabel, *wEgtMark, *wLapCur, *wLapBest, *wLapLast;
lv_obj_t *wGpsBadge, *wSdBadge, *wBleBadge, *wAccLabel;

// Telemetrie
lv_obj_t *wGZone, *wGDot, *wBankBar, *wBankLabel, *wGLongLabel, *wGLatLabel;

// Moteur
lv_obj_t *wRpmBar, *wRpmLabel, *wEgtMaxLabel, *wPowerLabel, *wTorqueLabel;

// Banc d'essai
lv_obj_t *wDynoEgt, *wDynoEgtMax, *wDynoRpm, *wDynoP, *wDynoC, *wDynoPMax, *wDynoCMax;
float s_dynoPMax = 0, s_dynoCMax = 0;

// Parametres
lv_obj_t *wMassVal, *wCxsaVal, *wEtaVal, *wRadiusVal, *wLapLineLabel;

// Sessions
lv_obj_t *wRecBtn, *wRecBtnLabel, *wSdState, *wSessionsList;

// ------------------------------ Helpers ------------------------------------

lv_obj_t *makeLabel(lv_obj_t *parent, const char *txt, const lv_font_t *font,
                    lv_color_t color, lv_align_t align, lv_coord_t xo, lv_coord_t yo)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, color, 0);
    lv_obj_align(l, align, xo, yo);
    return l;
}

lv_obj_t *makeBadge(lv_obj_t *parent, const char *txt, lv_coord_t xo, lv_coord_t yo, lv_color_t bg)
{
    lv_obj_t *b = lv_label_create(parent);
    lv_label_set_text(b, txt);
    lv_obj_set_style_text_font(b, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(b, C_BG, 0);
    lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(b, 4, 0);
    lv_obj_set_style_pad_hor(b, 8, 0);
    lv_obj_set_style_pad_ver(b, 4, 0);
    lv_obj_align(b, LV_ALIGN_TOP_RIGHT, xo, yo);
    return b;
}

const char *fmtLap(uint32_t ms, char *buf, size_t len)
{
    if (ms == 0) {
        snprintf(buf, len, "--'--,--");
    } else {
        uint32_t m = ms / 60000;
        uint32_t s = (ms % 60000) / 1000;
        uint32_t h = (ms % 1000) / 10;
        snprintf(buf, len, "%02lu'%02lu,%02lu", (unsigned long)m, (unsigned long)s, (unsigned long)h);
    }
    return buf;
}

// ----------------------------- Navigation ----------------------------------

void showPage(uint8_t idx);
void dynoRazCb(lv_event_t *e)
{
    (void)e;
    s_dynoPMax = 0;
    s_dynoCMax = 0;
}

void gestureCb(lv_event_t *e)
{
    (void)e;
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
    if (dir == LV_DIR_LEFT) showPage((g_pageIdx + 1) % NB_PAGES);
    else if (dir == LV_DIR_RIGHT) showPage((g_pageIdx + NB_PAGES - 1) % NB_PAGES);
}

void makeNavDots(lv_obj_t *page, uint8_t active)
{
    for (int i = 0; i < NB_PAGES; i++) {
        lv_obj_t *dot = lv_obj_create(page);
        lv_obj_remove_style_all(dot);
        lv_obj_set_size(dot, 8, 8);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, i == active ? C_ACCENT : C_GRAY, 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        lv_obj_align(dot, LV_ALIGN_TOP_MID, -(int)((NB_PAGES - 1) * 12) + i * 24, 14);
    }
}

lv_obj_t *newPage(const char *title, uint8_t id)
{
    lv_obj_t *page = lv_obj_create(lv_scr_act());
    lv_obj_set_size(page, LCD_WIDTH, LCD_HEIGHT);
    lv_obj_set_style_bg_color(page, C_BG, 0);
    lv_obj_set_style_bg_opa(page, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(page, 0, 0);
    lv_obj_set_style_radius(page, 0, 0);
    lv_obj_set_style_pad_all(page, 0, 0);
    lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(page, gestureCb, LV_EVENT_GESTURE, nullptr);
    lv_obj_add_flag(page, LV_OBJ_FLAG_HIDDEN);
    makeLabel(page, title, &lv_font_montserrat_14, C_GRAY, LV_ALIGN_TOP_MID, 0, 26);
    makeNavDots(page, id);
    g_pages[id] = page;
    return page;
}

lv_obj_t *makeButton(lv_obj_t *parent, const char *txt, lv_align_t align,
                     lv_coord_t xo, lv_coord_t yo, lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 150, 60);
    lv_obj_align(btn, align, xo, yo);
    lv_obj_t *l = lv_label_create(btn);
    lv_label_set_text(l, txt);
    lv_obj_center(l);
    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
    return btn;
}

// --------------------------- Page PARAMETRES -------------------------------

struct Stepper {
    float *field;
    float step;
};

VehicleSettings g_edit;       // copie de travail de la page Parametres

void refreshSteppers()
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%.0f kg", g_edit.mass_kg);
    lv_label_set_text(wMassVal, buf);
    snprintf(buf, sizeof(buf), "%.2f m2", g_edit.cxsa);
    lv_label_set_text(wCxsaVal, buf);
    snprintf(buf, sizeof(buf), "%.2f", g_edit.eta);
    lv_label_set_text(wEtaVal, buf);
    snprintf(buf, sizeof(buf), "%.0f m", g_edit.lap_radius_m);
    lv_label_set_text(wRadiusVal, buf);
    if (g_edit.lap_lat != 0.0 || g_edit.lap_lon != 0.0) {
        lv_label_set_text(wLapLineLabel, "Ligne : definie");
    } else {
        lv_label_set_text(wLapLineLabel, "Ligne : non definie");
    }
}

void stepperApply()
{
    settings_update(g_edit);
}

void stepperMinus(lv_event_t *e)
{
    Stepper *st = (Stepper *)lv_event_get_user_data(e);
    *st->field -= st->step;
    if (*st->field < 0) *st->field = 0;
    refreshSteppers();
    stepperApply();
}

void stepperPlus(lv_event_t *e)
{
    Stepper *st = (Stepper *)lv_event_get_user_data(e);
    *st->field += st->step;
    refreshSteppers();
    stepperApply();
}

lv_obj_t *makeStepperRow(lv_obj_t *parent, const char *title, float *field,
                         float step, lv_coord_t y)
{
    makeLabel(parent, title, &lv_font_montserrat_20, C_WHITE, LV_ALIGN_TOP_LEFT, 24, y);

    lv_obj_t *minus = lv_btn_create(parent);
    lv_obj_set_size(minus, 54, 44);
    lv_obj_align(minus, LV_ALIGN_TOP_RIGHT, -172, y - 4);
    lv_obj_t *ml = lv_label_create(minus);
    lv_label_set_text(ml, "-");
    lv_obj_center(ml);

    lv_obj_t *val = lv_label_create(parent);
    lv_obj_set_style_text_font(val, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(val, C_ACCENT, 0);
    lv_obj_align(val, LV_ALIGN_TOP_RIGHT, -104, y + 6);

    lv_obj_t *plus = lv_btn_create(parent);
    lv_obj_set_size(plus, 54, 44);
    lv_obj_align(plus, LV_ALIGN_TOP_RIGHT, -24, y - 4);
    lv_obj_t *pl = lv_label_create(plus);
    lv_label_set_text(pl, "+");
    lv_obj_center(pl);

    Stepper *st = new Stepper{field, step};
    lv_obj_add_event_cb(minus, stepperMinus, LV_EVENT_CLICKED, st);
    lv_obj_add_event_cb(plus, stepperPlus, LV_EVENT_CLICKED, st);
    return val;
}

void lapHereCb(lv_event_t *e)
{
    (void)e;
    GpsFix g = gps_get();
    if (!g.valid) {
        lv_label_set_text(wLapLineLabel, "Ligne : pas de GPS !");
        return;
    }
    settings_lapLineHere(g);
    g_edit = settings_get();
    refreshSteppers();
}

void calibCb(lv_event_t *e)
{
    (void)e;
    acq_calibrateFlat();
}

void buildSettingsPage()
{
    lv_obj_t *p = newPage("PARAMETRES", PAGE_SETTINGS);
    g_edit = settings_get();

    wMassVal = makeStepperRow(p, "MASSE (pilote + velo)", &g_edit.mass_kg, 5.0f, 64);
    wCxsaVal = makeStepperRow(p, "Cx*S (m2)", &g_edit.cxsa, 0.02f, 128);
    wEtaVal = makeStepperRow(p, "Renlement transmission", &g_edit.eta, 0.02f, 192);
    wRadiusVal = makeStepperRow(p, "Rayon ligne (m)", &g_edit.lap_radius_m, 5.0f, 256);

    makeButton(p, "LIGNE ICI", LV_ALIGN_BOTTOM_LEFT, 20, -180, lapHereCb);
    makeButton(p, "CALIBRER", LV_ALIGN_BOTTOM_LEFT, 20, -60, calibCb);

    wLapLineLabel = makeLabel(p, "Ligne : non definie", &lv_font_montserrat_14, C_DIM,
                              LV_ALIGN_BOTTOM_LEFT, 22, -120);
    refreshSteppers();
}

// ------------------------------ Page COURSE --------------------------------

void egtZoneColor(float egt, lv_color_t &out)
{
    if (egt >= 600) out = C_RED;
    else if (egt >= 500) out = C_ORANGE;
    else out = C_GREEN;
}

void buildRacePage()
{
    lv_obj_t *p = newPage("COURSE", PAGE_RACE);

    // Bargraph EGT vertical le long du bord gauche (echelle 0-700 C)
    const int EGT_BAR_X = 34;
    const int EGT_BAR_Y = 64;
    const int EGT_BAR_W = 34;
    const int EGT_BAR_H = 300;

    lv_obj_t *track = lv_obj_create(p);
    lv_obj_remove_style_all(track);
    lv_obj_set_pos(track, EGT_BAR_X, EGT_BAR_Y);
    lv_obj_set_size(track, EGT_BAR_W, EGT_BAR_H);
    lv_obj_set_style_bg_color(track, lv_color_hex(0x1F2937), 0);
    lv_obj_set_style_bg_opa(track, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(track, 4, 0);
    lv_obj_clear_flag(track, LV_OBJ_FLAG_SCROLLABLE);

    // Fond : rappel discret des 3 zones EGT (0-500 vert, 500-600 orange, 600-700 rouge)
    const int RED_H = (int)(100.0f / 700 * EGT_BAR_H);
    const int ORA_H = (int)(100.0f / 700 * EGT_BAR_H);
    const int GRN_H = EGT_BAR_H - ORA_H - RED_H;
    lv_obj_t *seg = lv_obj_create(track);
    lv_obj_remove_style_all(seg);
    lv_obj_set_size(seg, EGT_BAR_W, GRN_H);
    lv_obj_align(seg, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(seg, lv_color_hex(0x14532D), 0);
    lv_obj_set_style_bg_opa(seg, LV_OPA_COVER, 0);

    seg = lv_obj_create(track);
    lv_obj_remove_style_all(seg);
    lv_obj_set_size(seg, EGT_BAR_W, ORA_H);
    lv_obj_align(seg, LV_ALIGN_BOTTOM_MID, 0, -GRN_H);
    lv_obj_set_style_bg_color(seg, lv_color_hex(0x78350F), 0);
    lv_obj_set_style_bg_opa(seg, LV_OPA_COVER, 0);

    seg = lv_obj_create(track);
    lv_obj_remove_style_all(seg);
    lv_obj_set_size(seg, EGT_BAR_W, RED_H);
    lv_obj_align(seg, LV_ALIGN_BOTTOM_MID, 0, -(GRN_H + ORA_H));
    lv_obj_set_style_bg_color(seg, lv_color_hex(0x7F1D1D), 0);
    lv_obj_set_style_bg_opa(seg, LV_OPA_COVER, 0);

    wEgtFill = lv_obj_create(track);
    lv_obj_remove_style_all(wEgtFill);
    lv_obj_set_size(wEgtFill, EGT_BAR_W, 0);
    lv_obj_align(wEgtFill, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(wEgtFill, C_GREEN, 0);
    lv_obj_set_style_bg_opa(wEgtFill, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(wEgtFill, 4, 0);

    // Curseur : maximum EGT enregistre durant la session
    wEgtMark = lv_obj_create(track);
    lv_obj_remove_style_all(wEgtMark);
    lv_obj_set_size(wEgtMark, EGT_BAR_W + 12, 4);
    lv_obj_align(wEgtMark, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(wEgtMark, C_WHITE, 0);
    lv_obj_set_style_bg_opa(wEgtMark, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(wEgtMark, 2, 0);

    // Valeur EGT centree en haut
    wEgtLabel = makeLabel(p, "---", &lv_font_montserrat_48, C_GREEN,
                          LV_ALIGN_TOP_MID, 0, 90);

    // Temps au tour courant, sous l'EGT
    wLapCur = makeLabel(p, "--'--,--", &lv_font_montserrat_30, C_WHITE,
                        LV_ALIGN_TOP_MID, 0, 160);

    // Best / Last sur la meme ligne, en bas
    wLapBest = makeLabel(p, "BEST   --'--,--", &lv_font_montserrat_22, C_WHITE,
                         LV_ALIGN_BOTTOM_LEFT, 28, -34);
    wLapLast = makeLabel(p, "LAST   --'--,--", &lv_font_montserrat_22, C_WHITE,
                         LV_ALIGN_BOTTOM_RIGHT, -28, -34);

    // Badges en haut a droite : precision, GPS, SD, BLE
    wGpsBadge = makeBadge(p, "GPS --", -128, 10, C_RED);
    wSdBadge = makeBadge(p, "SD --", -64, 10, C_RED);
    wBleBadge = makeBadge(p, "BLE", -16, 10, C_RED);
    wAccLabel = makeLabel(p, "-- m", &lv_font_montserrat_14, C_DIM,
                          LV_ALIGN_TOP_RIGHT, -215, 16);
}

// ---------------------------- Page TELEMETRIE ------------------------------

void buildTelemPage()
{
    lv_obj_t *p = newPage("TELEMETRIE", PAGE_TELEM);

    // Cercle G-Force (lat en abscisse, long en ordonnee : haut = acceleration)
    wGZone = lv_obj_create(p);
    lv_obj_set_size(wGZone, 300, 300);
    lv_obj_align(wGZone, LV_ALIGN_CENTER, 0, -46);
    lv_obj_set_style_radius(wGZone, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_color(wGZone, lv_color_hex(0x374151), 0);
    lv_obj_set_style_border_width(wGZone, 3, 0);
    lv_obj_set_style_bg_color(wGZone, lv_color_hex(0x0B0F19), 0);
    lv_obj_set_style_bg_opa(wGZone, LV_OPA_COVER, 0);
    lv_obj_clear_flag(wGZone, LV_OBJ_FLAG_SCROLLABLE);

    makeLabel(wGZone, "LAT", &lv_font_montserrat_14, C_GRAY, LV_ALIGN_TOP_MID, 0, 2);
    makeLabel(wGZone, "LONG", &lv_font_montserrat_14, C_GRAY, LV_ALIGN_RIGHT_MID, -10, 0);

    wGDot = lv_obj_create(wGZone);
    lv_obj_remove_style_all(wGDot);
    lv_obj_set_size(wGDot, 22, 22);
    lv_obj_set_style_radius(wGDot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(wGDot, C_ACCENT, 0);
    lv_obj_set_style_bg_opa(wGDot, LV_OPA_COVER, 0);

    wGLatLabel = makeLabel(p, "LAT  0.00 g", &lv_font_montserrat_20, C_WHITE,
                           LV_ALIGN_BOTTOM_LEFT, 20, -64);
    wGLongLabel = makeLabel(p, "LONG 0.00 g", &lv_font_montserrat_20, C_WHITE,
                            LV_ALIGN_BOTTOM_LEFT, 20, -32);

    // Inclinaison : barre horizontale -60..+60 deg
    wBankBar = lv_bar_create(p);
    lv_obj_set_size(wBankBar, 300, 16);
    lv_obj_align(wBankBar, LV_ALIGN_BOTTOM_MID, 0, -96);
    lv_bar_set_range(wBankBar, -60, 60);
    lv_bar_set_value(wBankBar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(wBankBar, lv_color_hex(0x1F2937), LV_PART_MAIN);
    lv_obj_set_style_bg_color(wBankBar, C_ORANGE, LV_PART_INDICATOR);

    wBankLabel = makeLabel(p, "INCLINAISON  0 deg", &lv_font_montserrat_20, C_WHITE,
                           LV_ALIGN_BOTTOM_MID, 0, -124);
}

// ------------------------------ Page MOTEUR --------------------------------

void buildEnginePage()
{
    lv_obj_t *p = newPage("MOTEUR", PAGE_ENGINE);

    makeLabel(p, "REGIME (tr/min)", &lv_font_montserrat_14, C_DIM,
              LV_ALIGN_TOP_LEFT, 26, 56);
    wRpmLabel = makeLabel(p, "0", &lv_font_montserrat_48, C_WHITE,
                          LV_ALIGN_TOP_LEFT, 26, 78);

    wRpmBar = lv_bar_create(p);
    lv_obj_set_size(wRpmBar, 400, 18);
    lv_obj_align(wRpmBar, LV_ALIGN_TOP_MID, 0, 142);
    lv_bar_set_range(wRpmBar, 0, RPM_MAX_SPEED);
    lv_bar_set_value(wRpmBar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(wRpmBar, lv_color_hex(0x1F2937), LV_PART_MAIN);
    lv_obj_set_style_bg_color(wRpmBar, C_ACCENT, LV_PART_INDICATOR);

    makeLabel(p, "EGT MAX", &lv_font_montserrat_14, C_DIM,
              LV_ALIGN_BOTTOM_LEFT, 26, -216);
    wEgtMaxLabel = makeLabel(p, "--- C", &lv_font_montserrat_26, C_ORANGE,
                             LV_ALIGN_BOTTOM_LEFT, 26, -186);

    makeLabel(p, "PUISSANCE", &lv_font_montserrat_14, C_DIM,
              LV_ALIGN_BOTTOM_LEFT, 26, -136);
    wPowerLabel = makeLabel(p, "0.0 kW", &lv_font_montserrat_26, C_GREEN,
                            LV_ALIGN_BOTTOM_LEFT, 26, -106);

    makeLabel(p, "COUPLE", &lv_font_montserrat_14, C_DIM,
              LV_ALIGN_BOTTOM_LEFT, 250, -136);
    wTorqueLabel = makeLabel(p, "0.0 Nm", &lv_font_montserrat_26, C_GREEN,
                             LV_ALIGN_BOTTOM_LEFT, 250, -106);
}

// ----------------------------- Page BANC D'ESSAI ----------------------------

void buildDynoPage()
{
    lv_obj_t *p = newPage("BANC", PAGE_DYNO);

    makeLabel(p, "EGT", &lv_font_montserrat_16, C_DIM, LV_ALIGN_TOP_LEFT, 40, 66);
    wDynoEgt = makeLabel(p, "---", &lv_font_montserrat_48, C_GREEN,
                         LV_ALIGN_TOP_LEFT, 40, 90);
    wDynoEgtMax = makeLabel(p, "EGT MAX   ---", &lv_font_montserrat_20, C_ORANGE,
                            LV_ALIGN_TOP_LEFT, 40, 152);

    makeLabel(p, "REGIME", &lv_font_montserrat_16, C_DIM, LV_ALIGN_TOP_LEFT, 260, 66);
    wDynoRpm = makeLabel(p, "0", &lv_font_montserrat_48, C_WHITE,
                         LV_ALIGN_TOP_LEFT, 260, 90);
    makeLabel(p, "tr/min", &lv_font_montserrat_14, C_DIM, LV_ALIGN_TOP_LEFT, 264, 144);

    makeLabel(p, "PUISSANCE", &lv_font_montserrat_16, C_DIM, LV_ALIGN_TOP_LEFT, 40, 216);
    wDynoP = makeLabel(p, "-- kW", &lv_font_montserrat_36, C_WHITE,
                       LV_ALIGN_TOP_LEFT, 40, 240);
    wDynoPMax = makeLabel(p, "P MAX   -- kW", &lv_font_montserrat_20, C_ACCENT,
                          LV_ALIGN_TOP_LEFT, 40, 288);

    makeLabel(p, "COUPLE", &lv_font_montserrat_16, C_DIM, LV_ALIGN_TOP_LEFT, 40, 344);
    wDynoC = makeLabel(p, "-- Nm", &lv_font_montserrat_36, C_WHITE,
                       LV_ALIGN_TOP_LEFT, 40, 368);
    wDynoCMax = makeLabel(p, "C MAX   -- Nm", &lv_font_montserrat_20, C_ACCENT,
                          LV_ALIGN_TOP_LEFT, 40, 416);

    makeButton(p, "RAZ MAX", LV_ALIGN_BOTTOM_RIGHT, -20, -20, dynoRazCb);
}

// ----------------------------- Page SESSIONS -------------------------------

void recBtnCb(lv_event_t *e)
{
    (void)e;
    TelemetryFrame f = telemetry_get();
    cmds_requestRecord(!f.recording);
}

void refreshSessionList()
{
    if (!wSessionsList) return;
    lv_obj_clean(wSessionsList);
    auto files = sd_listSessions();
    if (files.empty()) {
        lv_list_add_text(wSessionsList, "Aucune session");
    }
    for (auto &f : files) {
        lv_list_add_text(wSessionsList, f.c_str());
    }
}

void refreshBtnCb(lv_event_t *e)
{
    (void)e;
    refreshSessionList();
}

void buildSessionsPage()
{
    lv_obj_t *p = newPage("SESSIONS", PAGE_SESSIONS);

    wRecBtn = makeButton(p, "ENREGISTRER", LV_ALIGN_TOP_MID, 0, 60, recBtnCb);
    wRecBtnLabel = lv_obj_get_child(wRecBtn, 0);

    wSdState = makeLabel(p, "SD : ...", &lv_font_montserrat_14, C_DIM,
                         LV_ALIGN_TOP_MID, 0, 140);
    makeLabel(p, "Export des sessions : via Pixel 5", &lv_font_montserrat_14, C_DIM,
              LV_ALIGN_TOP_MID, 0, 166);

    wSessionsList = lv_list_create(p);
    lv_obj_set_size(wSessionsList, 420, 170);
    lv_obj_align(wSessionsList, LV_ALIGN_CENTER, 0, 40);
    lv_obj_set_style_bg_color(wSessionsList, lv_color_hex(0x0B0F19), 0);

    makeButton(p, "RAFRAICHIR", LV_ALIGN_BOTTOM_MID, 0, -20, refreshBtnCb);
    refreshSessionList();
}

// ------------------------------ Alarme EGT ---------------------------------

void alarmTick(const TelemetryFrame &f)
{
    static bool alarmed = false;
    static bool phase = false;
    static uint32_t lastToggle = 0;

    if (!alarmed && f.egt_c >= EGT_ALARM_C) {
        alarmed = true;
        phase = true;
    } else if (alarmed && f.egt_c < EGT_ALARM_C - EGT_ALARM_HYST_C) {
        alarmed = false;
    }

    lv_color_t bg = C_BG;
    if (alarmed) {
        // "orange flash" : fond alternance orange / noir toutes les 250 ms
        uint32_t now = millis();
        if (now - lastToggle >= EGT_ALARM_PERIOD_MS) {
            lastToggle = now;
            phase = !phase;
        }
        if (phase) bg = lv_color_hex(0xFF8C00);
    }
    for (int i = 0; i < 5; i++) {
        lv_obj_set_style_bg_color(g_pages[i], bg, 0);
    }
}

// ------------------------------ Page switching -----------------------------

void showPage(uint8_t idx)
{
    if (idx > PAGE_SESSIONS) idx = 0;
    for (int i = 0; i < NB_PAGES; i++) {
        if (i == idx) lv_obj_clear_flag(g_pages[i], LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(g_pages[i], LV_OBJ_FLAG_HIDDEN);
    }
    g_pageIdx = idx;
    if (idx == PAGE_SESSIONS) refreshSessionList();
}

// ------------------------------ Mise a jour --------------------------------

void updateRacePage(const TelemetryFrame &f, const ChronoState &c)
{
    bool hasEgt = f.egt_c >= 0;
    lv_color_t zc;
    egtZoneColor(hasEgt ? f.egt_c : 0, zc);
    static const int EGT_BAR_H = 300;
    float egt = hasEgt ? f.egt_c : 0.0f;
    if (egt < 0) egt = 0;
    if (egt > 700) egt = 700;
    int h = (int)(egt / 700.0f * EGT_BAR_H);
    lv_obj_set_height(wEgtFill, h);
    lv_obj_set_style_bg_color(wEgtFill, zc, 0);
    if (hasEgt) lv_label_set_text_fmt(wEgtLabel, "%.0f °c", egt);
    else lv_label_set_text(wEgtLabel, "---");
    lv_obj_set_style_text_color(wEgtLabel, zc, 0);

    char cur[16], best[16], last[16];
    fmtLap(c.current_lap_ms, cur, sizeof(cur));
    lv_label_set_text(wLapCur, cur);
    fmtLap(c.best_lap_ms, best, sizeof(best));
    lv_label_set_text_fmt(wLapBest, "BEST   %s", best);
    fmtLap(c.last_lap_ms, last, sizeof(last));
    lv_label_set_text_fmt(wLapLast, "LAST   %s", last);

    int markH = (int)(f.egt_max_c / 700.0f * EGT_BAR_H);
    if (markH < 0) markH = 0;
    if (markH > EGT_BAR_H) markH = EGT_BAR_H;
    lv_obj_align(wEgtMark, LV_ALIGN_BOTTOM_MID, 0, -markH);

    GpsFix g = gps_get();
    if (g.valid) {
        lv_label_set_text_fmt(wGpsBadge, "%d sats", g.sats);
        lv_color_t q = g.sats >= 8 ? C_GREEN : (g.sats >= 4 ? C_ORANGE : C_RED);
        lv_obj_set_style_bg_color(wGpsBadge, q, 0);
        lv_label_set_text_fmt(wAccLabel, "%.1fm", g.hdop * 2.5f);
    } else {
        lv_label_set_text(wGpsBadge, "GPS --");
        lv_obj_set_style_bg_color(wGpsBadge, C_RED, 0);
        lv_label_set_text(wAccLabel, "-- m");
    }
    lv_obj_align(wGpsBadge, LV_ALIGN_TOP_RIGHT, -128, 10);
    lv_label_set_text_fmt(wSdBadge, "SD %s", sd_ready() ? "ok" : "--");
    lv_obj_set_style_bg_color(wSdBadge, sd_ready() ? C_GREEN : C_RED, 0);
    lv_obj_align(wSdBadge, LV_ALIGN_TOP_RIGHT, -64, 10);
    lv_obj_set_style_bg_color(wBleBadge, ble_connected() ? C_GREEN : C_RED, 0);
    lv_obj_align(wBleBadge, LV_ALIGN_TOP_RIGHT, -16, 10);
    lv_obj_align(wAccLabel, LV_ALIGN_TOP_RIGHT, -215, 16);
}

void updateTelemPage(const TelemetryFrame &f, const ChronoState &)
{
    // Cercle 300 px : rayon 138 px utile, pleine echelle a 2.0 g
    const float RADIUS_PX = 138;
    const float G_SCALE = 2.0f;
    float dx = f.g_lat / G_SCALE * RADIUS_PX;
    float dy = f.g_long / G_SCALE * RADIUS_PX;
    if (dx > RADIUS_PX) dx = RADIUS_PX;
    if (dx < -RADIUS_PX) dx = -RADIUS_PX;
    if (dy > RADIUS_PX) dy = RADIUS_PX;
    if (dy < -RADIUS_PX) dy = -RADIUS_PX;
    lv_obj_align(wGDot, LV_ALIGN_CENTER, (int)dx, -(int)dy);

    lv_label_set_text_fmt(wGLatLabel, "LAT  %+.2f g", f.g_lat);
    lv_label_set_text_fmt(wGLongLabel, "LONG %+.2f g", f.g_long);
    lv_bar_set_value(wBankBar, (int)f.bank_deg, LV_ANIM_OFF);
    lv_label_set_text_fmt(wBankLabel, "INCLINAISON  %+.0f deg", f.bank_deg);
}

void updateEnginePage(const TelemetryFrame &f)
{
    lv_label_set_text_fmt(wRpmLabel, "%u", f.rpm);
    lv_bar_set_value(wRpmBar, f.rpm, LV_ANIM_OFF);
    if (f.egt_max_c > 0) lv_label_set_text_fmt(wEgtMaxLabel, "%.0f C", f.egt_max_c);
    lv_label_set_text_fmt(wPowerLabel, "%.1f kW", f.power_kw);
    lv_label_set_text_fmt(wTorqueLabel, "%.1f Nm", f.torque_nm);
}

void updateDynoPage(const TelemetryFrame &f)
{
    bool hasEgt = f.egt_c >= 0;
    lv_color_t zc;
    egtZoneColor(hasEgt ? f.egt_c : 0, zc);

    if (hasEgt) lv_label_set_text_fmt(wDynoEgt, "%.0f °c", f.egt_c);
    else lv_label_set_text(wDynoEgt, "---");
    lv_obj_set_style_text_color(wDynoEgt, zc, 0);
    if (f.egt_max_c > 0) lv_label_set_text_fmt(wDynoEgtMax, "EGT MAX   %.0f °c", f.egt_max_c);

    lv_label_set_text_fmt(wDynoRpm, "%u", f.rpm);
    lv_label_set_text_fmt(wDynoP, "%.1f kW", f.power_kw);
    lv_label_set_text_fmt(wDynoC, "%.1f Nm", f.torque_nm);
    if (s_dynoPMax > 0) lv_label_set_text_fmt(wDynoPMax, "P MAX   %.1f kW", s_dynoPMax);
    if (s_dynoCMax > 0) lv_label_set_text_fmt(wDynoCMax, "C MAX   %.1f Nm", s_dynoCMax);
}

void updateSessionsPage(const TelemetryFrame &f)
{
    lv_obj_set_style_bg_color(wRecBtn, f.recording ? C_RED : C_GREEN, 0);
    lv_label_set_text(wRecBtnLabel, f.recording ? "ARRETER" : "ENREGISTRER");
    lv_label_set_text(wSdState, sd_ready() ? "SD : PRETE" : "SD : ABSENTE");
}

void uiTimer(lv_timer_t *)
{
    TelemetryFrame f = telemetry_get();
    ChronoState c = chrono_get();
    alarmTick(f);
    if (f.power_kw > s_dynoPMax) s_dynoPMax = f.power_kw;
    if (f.torque_nm > s_dynoCMax) s_dynoCMax = f.torque_nm;
    switch (g_pageIdx) {
    case PAGE_RACE: updateRacePage(f, c); break;
    case PAGE_TELEM: updateTelemPage(f, c); break;
    case PAGE_ENGINE: updateEnginePage(f); break;
    case PAGE_DYNO: updateDynoPage(f); break;
    case PAGE_SETTINGS: break;
    case PAGE_SESSIONS: updateSessionsPage(f); break;
    }
}

// -------------------------------- Tache ------------------------------------

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

    buildRacePage();
    buildTelemPage();
    buildEnginePage();
    buildDynoPage();
    buildSettingsPage();
    buildSessionsPage();
    showPage(PAGE_RACE);

    lv_timer_create(uiTimer, 100, nullptr);

    xTaskCreatePinnedToCore(uiTask, "ui", 16384, nullptr, 2, nullptr, 1);
}
