#include "imu.h"
#include "hal.h"
#include "hal_imu.h"

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = hal_millis();
	unsigned int ms = now % 1000;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= 1000;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
		 h, min, s, ms);
	return buf;
}

// static int process_mpu6050(const struct device *dev)
// {
// 	struct sensor_value temperature;
// 	struct sensor_value accel[3];
// 	struct sensor_value gyro[3];
// 	int rc = sensor_sample_fetch(dev);

// 	if (rc == 0) {
// 		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
// 					accel);
// 	}
// 	if (rc == 0) {
// 		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
// 					gyro);
// 	}
// 	if (rc == 0) {
// 		rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
// 					&temperature);
// 	}
// 	if (rc == 0) {
// 		printk("[%s]:%g Cel\n"
// 		       "  accel %f %f %f m/s/s\n"
// 		       "  gyro  %f %f %f rad/s\n",
// 		       now_str(),
// 		       sensor_value_to_double(&temperature),
// 		       sensor_value_to_double(&accel[0]),
// 		       sensor_value_to_double(&accel[1]),
// 		       sensor_value_to_double(&accel[2]),
// 		       sensor_value_to_double(&gyro[0]),
// 		       sensor_value_to_double(&gyro[1]),
// 		       sensor_value_to_double(&gyro[2]));
// 	} else {
// 		printk("sample fetch/get failed: %d\n", rc);
// 	}

// 	return rc;
// }


mm_imu::mm_imu(const char *name) : moto_task(name) {
    accel_x = 0.0f;
    accel_y = 0.0f;
    accel_z = 0.0f;

    gyro_x = 0.0f;
    gyro_y = 0.0f;
    gyro_z = 0.0f;
}

bool mm_imu::begin(void) {



}

void mm_imu::update(void) {

    // while (!IS_ENABLED(CONFIG_MPU6050_TRIGGER)) {
	// 	int rc = process_mpu6050(mpu6050);

	// 	if (rc != 0) {
	// 		break;
	// 	}
	// 	k_sleep(K_SECONDS(2));
	// }

    // process_mpu6050(mpu6050);

	hal_imu_update();

}

mm_imu imu;