#ifndef _HAL_IMU_H_
#define _HAL_IMU_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

    bool hal_imu_init(void);
    
    void hal_imu_update(void);

#endif /***** _HAL_IMU_H_ ******/