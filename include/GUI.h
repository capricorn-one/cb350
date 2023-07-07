#ifndef GUI_H
#define GUI_H

#include <lvgl.h>
#include "moto_task.h"

typedef enum {
    GUI_SCREEN_LOGIN = 0,
    GUI_SCREEN_HOME = 1,
    GUI_SCREEN_PLOT = 2,
    GUI_SCREEN_TELEMETRY = 3,
    GUI_SCREEN_TERMINAL = 4
} gui_screen_t;

class GUI : public moto_task {

public:

    GUI(const char *name);
    ~GUI() {}

    void init(void);
    
    // void loadScreen(gui_screen_t scrn);

    // void drawButton(const char *name);

    lv_color_t theme_primary_color, theme_secondary_color;

protected:

    bool begin(void);
    void update(void);
    void exit(void) {};
    
private:

    void clearScreen(void);

    // void loadLoginScreen(lv_obj_t * scrn);
    // void loadHomeScreen(lv_obj_t * scrn);
    // void loadPlotScreen(lv_obj_t * scrn);
    // void loadTelemetryScreen(lv_obj_t * scrn);
    // void loadTerminalScreen(lv_obj_t * scrn);

    void drawTelemetry(void);

private:

    const struct device *display_dev;

    lv_obj_t * bar;

    lv_style_t style_bg;
    lv_style_t style_indic;

    lv_obj_t *adc_value[12];
};

extern GUI gui;

#endif