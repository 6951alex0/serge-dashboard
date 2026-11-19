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

enum PageId : uint8_t { PAGE_RACE = 0, PAGE_TELEM, PAGE_ENGINE, PAGE_SETTINGS, PAGE_SESSIONS };

lv_obj_t *g_pages[5];
uint8_t g_pageIdx = PAGE_RACE;

// Course
lv_obj_t *wSpeedLabel, *wEgtArc, *wEgtLabel, *wLapCur, *wLapBest, *wLapLast;
lv_obj_t *wRecDot, *wStatusLine;

// Telemetrie
lv_obj_t *wGZone, *wGDot, *wBankBar, *wBankLabel, *wGLongLabel, *wGLatLabel;

// Moteur
lv_obj_t *wRpmBar, *wRpmLabel, *wEgtMaxLabel, *wPowerLabel, *wTorqueLabel;

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

const char *fmtLap(uint32_t ms, char *buf, size_t len)
{
    if (ms == 0) {
        snprintf(buf, len, "--:--.-");
    } else {
        uint32_t m = ms / 60000;
        uint32_t s = (ms % 60000) / 1000;
        uint32_t d = (ms % 1000) / 100;
        snprintf(buf, len, "%lum %lu,%lu s", (unsigned long)m, (unsigned long)s, (unsigned long)d);
    }
    return buf;
}

// ----------------------------- Navigation ----------------------------------

void showPage(uint8_t idx);

void gestureCb(lv_event_t *e)
{
    (void)e;
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
    if (dir == LV_DIR_LEFT) showPage((g_pageIdx + 1) % 5);
    else if (dir == LV_DIR_RIGHT) showPage((g_pageIdx + 4) % 5);
}

void makeNavDots(lv_obj_t *page, uint8_t active)
{
    for (int i = 0; i < 5; i++) {
        lv_obj_t *dot = lv_obj_create(page);
        lv_obj_remove_style_all(dot);
        lv_obj_set_size(dot, 8, 8);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, i == active ? C_ACCENT : C_GRAY, 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        lv_obj_align(dot, LV_ALIGN_TOP_MID, -48 + i * 24, 14);
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

    // Jauge EGT (arc 270 deg, zones verte/orange/rouge 0-700 C)
    wEgtArc = lv_arc_create(p);
    lv_obj_set_size(wEgtArc, 330, 330);
    lv_obj_align(wEgtArc, LV_ALIGN_CENTER, 0, -66);
    lv_arc_set_rotation(wEgtArc, 135);
    lv_arc_set_bg_angles(wEgtArc, 0, 270);
    lv_arc_set_range(wEgtArc, 0, 700);
    lv_arc_set_value(wEgtArc, 0);
    lv_obj_remove_style(wEgtArc, nullptr, LV_PART_KNOB);
    lv_obj_clear_flag(wEgtArc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(wEgtArc, 16, LV_PART_MAIN);
    lv_obj_set_style_arc_width(wEgtArc, 16, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(wEgtArc, lv_color_hex(0x1F2937), LV_PART_MAIN);
    lv_obj_set_style_arc_color(wEgtArc, C_GREEN, LV_PART_INDICATOR);

    wEgtLabel = makeLabel(wEgtArc, "EGT --- C", &lv_font_montserrat_22, C_GREEN,
                          LV_ALIGN_CENTER, 0, 0);

    // Vitesse centrale
    wSpeedLabel = makeLabel(p, "0", &lv_font_montserrat_48, C_WHITE,
                            LV_ALIGN_CENTER, 0, 172);
    makeLabel(p, "km/h", &lv_font_montserrat_14, C_DIM, LV_ALIGN_CENTER, 0, 210);

    // Chronos (3 lignes max : courant / meilleur / precedent)
    wLapCur = makeLabel(p, "TOUR      --:--.-", &lv_font_montserrat_20, C_WHITE,
                        LV_ALIGN_BOTTOM_LEFT, 24, -100);
    wLapBest = makeLabel(p, "MEILLEUR  --:--.-", &lv_font_montserrat_20, C_GREEN,
                         LV_ALIGN_BOTTOM_LEFT, 24, -66);
    wLapLast = makeLabel(p, "PREC.     --:--.-", &lv_font_montserrat_20, C_DIM,
                         LV_ALIGN_BOTTOM_LEFT, 24, -32);

    // REC + statut
    wRecDot = lv_obj_create(p);
    lv_obj_remove_style_all(wRecDot);
    lv_obj_set_size(wRecDot, 12, 12);
    lv_obj_set_style_radius(wRecDot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(wRecDot, C_GRAY, 0);
    lv_obj_set_style_bg_opa(wRecDot, LV_OPA_COVER, 0);
    lv_obj_align(wRecDot, LV_ALIGN_TOP_LEFT, 18, 14);

    wStatusLine = makeLabel(p, "GPS --  BAT --", &lv_font_montserrat_14, C_DIM,
                            LV_ALIGN_TOP_RIGHT, -16, 10);
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
    for (int i = 0; i < 5; i++) {
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
    lv_arc_set_value(wEgtArc, hasEgt ? (int)(f.egt_c > 700 ? 700 : f.egt_c) : 0);
    lv_obj_set_style_arc_color(wEgtArc, zc, LV_PART_INDICATOR);
    if (hasEgt) lv_label_set_text_fmt(wEgtLabel, "EGT %.0f C", f.egt_c);
    else lv_label_set_text(wEgtLabel, "EGT --- C");
    lv_obj_set_style_text_color(wEgtLabel, zc, 0);

    lv_label_set_text_fmt(wSpeedLabel, "%.0f", f.speed_kmh);

    char cur[16], best[16], last[16];
    fmtLap(c.current_lap_ms, cur, sizeof(cur));
    lv_label_set_text_fmt(wLapCur, "TOUR      %s", cur);
    fmtLap(c.best_lap_ms, best, sizeof(best));
    lv_label_set_text_fmt(wLapBest, "MEILLEUR  %s", best);
    fmtLap(c.last_lap_ms, last, sizeof(last));
    lv_label_set_text_fmt(wLapLast, "PREC.     %s", last);

    lv_obj_set_style_bg_color(wRecDot, f.recording ? C_RED : C_GRAY, 0);

    GpsFix g = gps_get();
    lv_label_set_text_fmt(wStatusLine, "GPS %s sats %d  SD %s",
                          g.valid ? "OK" : "--", g.sats, sd_ready() ? "OK" : "--");
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
    switch (g_pageIdx) {
    case PAGE_RACE: updateRacePage(f, c); break;
    case PAGE_TELEM: updateTelemPage(f, c); break;
    case PAGE_ENGINE: updateEnginePage(f); break;
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
    buildSettingsPage();
    buildSessionsPage();
    showPage(PAGE_RACE);

    lv_timer_create(uiTimer, 100, nullptr);

    xTaskCreatePinnedToCore(uiTask, "ui", 16384, nullptr, 2, nullptr, 1);
}
