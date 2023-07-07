#include "moto_task.h"
#include <functional>
#include "hal.h"

static Scheduler moto_scheduler;


moto_task::moto_task(const char *tagName, unsigned long aInterval, long aIterations) : Task(aInterval, aIterations, &moto_scheduler, false) {
    initialize_task(tagName, aInterval, aIterations);
}

moto_task::moto_task(const char *tagName) {
    initialize_task(tagName, 100, TASK_FOREVER);
}

void moto_task::initialize_task(const char *tagName, unsigned long aInterval, long aIterations) {
    
    strncpy(taskName, tagName, 16);

}

bool moto_task::OnEnable(void) {
    LOG_INF("Starting task %s", taskName);
    return begin();
}

void moto_task::OnDisable(void) {
    LOG_INF("Exiting task %s", taskName);
    exit();
}

bool moto_task::Callback(void) {
    update();
    return true;
}

void moto_task::start(void) {
    
    // moto_scheduler.startNow();

    moto_scheduler.enableAll();
}

void moto_task::run(void) {
    moto_scheduler.execute();
}