#include "moto_task.h"
#include <functional>
#include "hal.h"

static Scheduler moto_scheduler;

template <typename T>
struct Callback;

template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
   template <typename... Args> 
   static Ret callback(Args... args) {                    
      func(args...);  
   }
   static std::function<Ret(Params...)> func; 
};

template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;

moto_task::moto_task(const char *tagName, unsigned long aInterval, long aIterations) {
    initialize_task(tagName, aInterval, aIterations);
}

moto_task::moto_task(const char *tagName) {
    initialize_task(tagName, 100, TASK_FOREVER);
}

void moto_task::initialize_task(const char *tagName, unsigned long aInterval, long aIterations) {
    
    strncpy(taskName, tagName, 16);

    Callback<void(void)>::func = std::bind(&moto_task::_update, this);
    TaskCallback aCallback = static_cast<TaskCallback>(Callback<void(void)>::callback);

    Callback<bool(void)>::func = std::bind(&moto_task::_begin, this);
    TaskOnEnable aOnEnable = static_cast<TaskOnEnable>(Callback<bool(void)>::callback);

    Callback<void(void)>::func = std::bind(&moto_task::_exit, this);
    TaskOnDisable aOnDisable = static_cast<TaskOnDisable>(Callback<void(void)>::callback);
    
    set(aInterval, aIterations, aCallback, aOnEnable, aOnDisable);

    moto_scheduler.addTask(*this);
}

bool moto_task::_begin(void) {
    LOG_INF("Starting task %s", taskName);
    return begin();
}

void moto_task::_exit(void) {
    LOG_INF("Exiting task %s", taskName);
    exit();
}

void moto_task::_update(void) {
    update();
}

void moto_task::start(void) {
    
    // moto_scheduler.startNow();

    moto_scheduler.enableAll();
}

void moto_task::run(void) {
    moto_scheduler.execute();
}