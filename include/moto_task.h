#ifndef _MOTO_TASK_H_
#define _MOTO_TASK_H_


#include <stdint.h>
#include <stdbool.h>

#define _TASK_OO_CALLBACKS

#include <TaskSchedulerDeclarations.h>

class moto_task : public Task {

public:

    moto_task(const char *tagName);
    moto_task(const char *tagName, unsigned long aInterval, long aIterations);
    ~moto_task() {}

    char * tag(void) { return taskName; }

    static void start(void);
    static void run(void);

protected:

    virtual bool begin(void) = 0;
    virtual void update(void) = 0;
    virtual void exit(void) = 0;

private:

    char taskName[16];

    void initialize_task(const char *tagName, unsigned long aInterval, long aIterations);

    bool Callback(void);
    bool OnEnable(void);
    void OnDisable(void);
};

#endif /* _MOTO_TASK_H_ */
