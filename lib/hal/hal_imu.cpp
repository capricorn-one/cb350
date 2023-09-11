#include "hal_imu.h"
#include "hal.h"
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"

static MPU6050 mpu;
uint8_t devStatus;

bool hal_imu_init(void) {

    IMU_WIRE.begin();

    mpu.initialize();

    pinMode(PIN_IMU_INT, INPUT);

    devStatus = mpu.dmpInitialize();

    return true;
}

void hal_imu_update(void) {

    // mpu.resetFIFO();
    // mpu.getFIFOBytes(mpu.fifoBuffer, 512);
    // mpu.dmpGetQuaternion(&q, mpu.fifoBuffer);
    // mpu.dmpGetGravity(&gravity, &q);
    // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}