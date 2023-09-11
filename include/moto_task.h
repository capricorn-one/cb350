#ifndef _MOTO_TASK_H_
#define _MOTO_TASK_H_


#include <stdint.h>
#include <stdbool.h>

// #define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass 
// #define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
// #define _TASK_PRIORITY          // Support for layered scheduling priority
// #define _TASK_TIMEOUT           // Support for overall task timeout 
// #define _TASK_OO_CALLBACKS      // Support for dynamic callback method binding

#include <TaskSchedulerDeclarations.h>

class moto_task : public Task {

public:

    moto_task(const char *tagName, unsigned long aInterval);

    char * tag(void) { return taskName; }

    void init(Scheduler *ts);

protected:

    virtual bool begin(void) = 0;
    virtual bool start(void) = 0;
    virtual bool update(void) = 0;
    virtual void exit(void) = 0;

private:

    char taskName[16];

    bool Callback(void);
    bool OnEnable(void);
    void OnDisable(void);

};

#endif /* _MOTO_TASK_H_ */
