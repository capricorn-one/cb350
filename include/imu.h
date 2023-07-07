#ifndef _MM_IMU_H
#define _MM_IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "moto_task.h"

class mm_imu : public moto_task {

public:

    mm_imu(const char *name);

    float get_accel_x(void) { return accel_x; }
    float get_accel_y(void) { return accel_y; }
    float get_accel_z(void) { return accel_z; }

    float get_gyro_x(void) { return gyro_x; }
    float get_gyro_y(void) { return gyro_y; }
    float get_gyro_z(void) { return gyro_z; }

protected:

    bool begin(void);
    void update(void);
    void exit(void) {}

private:

    float accel_x;
    float accel_y;
    float accel_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    

};

extern mm_imu imu;

#endif // _MM_IMU_H