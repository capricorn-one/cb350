#ifndef _MOTO_TASK_H_
#define _MOTO_TASK_H_


#include <stdint.h>
#include <stdbool.h>
#include <TaskScheduler.h>

class moto_task : public Task {

public:

    moto_task(const char *tagName);
    moto_task(const char *tagName, unsigned long aInterval, long aIterations);

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

    bool _begin(void);
    void _update(void);
    void _exit(void);
};

#endif /* _MOTO_TASK_H_ */
