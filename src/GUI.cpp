#include "GUI.h"
#include "ft81x.h"
#include "outputs.h"
#include "hal.h"
#include "hal_lcd.h"
#include "telemetry.h"

// answer here with fucking lame ass bull shit atsamd21g is that there's not enough memory to even support LVGL
// however, EVE display is supposed to mitigate this. May be left with using EVE drivers only, no LVGL support. BFD.
// test with existing proven library, then switch to crystalfontz driver. Works with a fucking arduino uno. This can work.

void GUI::init() {

    hal_lcd_init();

}



bool GUI::begin() {

    
    return true;
}

void GUI::drawTelemetry(void) {

    
}


void GUI::update(void) {

   

}

void GUI::clearScreen(void) {
    // lv_obj_clean(scrn_act);
}

// void GUI::loadScreen(gui_screen_t scrn) {

//     scrn_act = lv_obj_create(NULL);

//     switch(scrn) {
//         case GUI_SCREEN_LOGIN:          loadLoginScreen(scrn_act);          break;
//         case GUI_SCREEN_HOME:           loadHomeScreen(scrn_act);           break;
//         case GUI_SCREEN_PLOT:           loadPlotScreen(scrn_act);           break;
//         case GUI_SCREEN_TELEMETRY:      loadTelemetryScreen(scrn_act);      break;
//         case GUI_SCREEN_TERMINAL:       loadTerminalScreen(scrn_act);       break;
//     }

//     lv_scr_load_anim(scrn_act, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);

// }

// void GUI::loadLoginScreen(lv_obj_t * scrn) {


// }

// void GUI::loadHomeScreen(lv_obj_t * scrn) {

//     lv_style_t matrixStyle;

//     lv_style_init(&matrixStyle);

//     lv_style_set_radius(&matrixStyle, 10);
//     lv_style_set_border_width(&matrixStyle, 2);
//     lv_style_set_border_color(&matrixStyle, theme_primary_color);

//     lv_style_set_text_color(&matrixStyle, theme_primary_color);
//     lv_style_set_pad_all(&matrixStyle, 20);
    
//     // for(uint8_t i=0; i<POWER_OUTPUT_CHANNELS; i++) {

//     //     lv_obj_t *btn = lv_btn_create(scrn);
//     //     lv_obj_t * label = lv_label_create(btn);

//     //     lv_label_set_text(label, output[i].name);
//     //     lv_obj_center(label);
//     //     lv_obj_add_style(btn, &matrixStyle, LV_PART_MAIN | LV_STATE_DEFAULT);

//     //     lv_obj_align(btn, LV_ALIGN_TOP_RIGHT, 0, lv_pct(i * 8));

//     //     output[i].lv_obj = btn;
//     // }

// }

// void GUI::loadPlotScreen(lv_obj_t * scrn) {
// }

// void GUI::loadTelemetryScreen(lv_obj_t * scrn) {
// }

// void GUI::loadTerminalScreen(lv_obj_t * scrn) {
// }




GUI gui("GUI");