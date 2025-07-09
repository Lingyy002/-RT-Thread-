#ifndef __MPU6050_H__
#define __MPU6050_H__

int mpu6050_init(const char *bus_name);
int mpu6050_read_accel(short *accel);
int mpu6050_read_gyro(short *gyro);
int mpu6050_read_temp(short *temp);

void mpu6050_update_yaw(void);
float mpu6050_get_yaw(void);
void mpu6050_angle_reset(void);

#endif
