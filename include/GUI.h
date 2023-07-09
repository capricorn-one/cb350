#ifndef GUI_H
#define GUI_H

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

    GUI(const char *name) : moto_task(name, 30) {}
    GUI(const char *name, unsigned long aInterval) : moto_task(name, aInterval) {}

    void init(void);
    
    // void loadScreen(gui_screen_t scrn);

    // void drawButton(const char *name);

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


};

extern GUI gui;

#endif