#include "mraa.hpp"

#include "9dof_block.h"
#include <iostream>
#include <unistd.h>



int main()
{
	IMUBlock imu;

	imu.setGyroEnable(true);
	imu.setAccelRate(9);
	imu.setMagRate(3);
	imu.setMagRes(3);
	imu.setMagMode(0);


	while(true) {
		sensor_t accel = imu.readAccel();
		sensor_t mag = imu.readMag();
		sensor_t rot = imu.readGyro();

		printf("accel %d %d %d\n", accel.x, accel.y, accel.z);
		printf("mag %d %d %d\n", mag.x, mag.y, mag.z);
		printf("rot %d %d %d\n", rot.x, rot.y, rot.z);
		usleep(50000);
	}

	return MRAA_SUCCESS;
}
