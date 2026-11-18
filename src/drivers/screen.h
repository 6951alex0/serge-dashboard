#pragma once
// Ecran + tactile : initialisation profilee pour l'interface LVGL.

#include <lvgl.h>

bool screen_begin();
void screen_lvgl_register();   // display + indev LVGL (apres lv_init())
