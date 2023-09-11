#include "moto_task.h"
#include <functional>
#include "hal.h"


moto_task::moto_task(const char *tagName, unsigned long aInterval) : Task(aInterval, TASK_FOREVER, NULL, false) {
    strncpy(taskName, tagName, 16);
}

void moto_task::init(Scheduler *ts) {
    ts->addTask(*this);
    if( begin() ) {
        LOG_INF("Task %s initialized.", taskName);
    } else {
        LOG_ERR("Task %s failed to initialize.", taskName);
    }
}

bool moto_task::OnEnable(void) {
    if(start()) {
        LOG_INF("Task %s started.", taskName);
        return true;
    } else {
        LOG_ERR("Task %s failed to start", taskName);
        return false;
    }
}

bool moto_task::Callback(void) {
    return update();
}

void moto_task::OnDisable(void) {
    LOG_INF("Exiting task %s", taskName);
    exit();
}
