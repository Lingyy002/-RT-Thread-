#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/i2c.h>
#include <math.h>
#include "mpu6050.h"

#define MPU_I2C_BUS_NAME "i2c1"
#define GYRO_SCALE 131.0f   // MPU6050 ±250°/s对应比例因子
#define ACCEL_SCALE 16384.0f // MPU6050 ±2g对应比例因子
#define COMPLEMENTARY_FILTER_ALPHA 0.98f

static struct rt_thread *mpu_tid = RT_NULL;

// 校准零偏用
static float gz_offset = 0.0f;

// 姿态角，单位度
static float yaw = 0.0f;
static float pitch = 0.0f;
static float roll = 0.0f;

static void mpu_thread(void *param)
{
    if (mpu6050_init(MPU_I2C_BUS_NAME) != RT_EOK) {
        rt_kprintf("MPU6050 init failed!\n");
        return;
    }
    rt_kprintf("MPU6050 initialized.\n");

    short gyro[3], accel[3];

    // 1. 零偏校准陀螺仪Z轴（偏航）
    const int calib_samples = 200;
    float gz_sum = 0.0f;

    rt_kprintf("Calibrating gyro Z offset...\n");
    for (int i = 0; i < calib_samples; i++) {
        if (mpu6050_read_gyro(gyro) == RT_EOK) {
            gz_sum += (float)gyro[2] / GYRO_SCALE;
        }
        rt_thread_mdelay(5);
    }
    gz_offset = gz_sum / calib_samples;
    rt_kprintf("Gyro Z offset = %.3f deg/s\n", gz_offset);

    rt_tick_t last_tick = rt_tick_get();

    while (1) {
        if (mpu6050_read_gyro(gyro) == RT_EOK && mpu6050_read_accel(accel) == RT_EOK) {
            rt_tick_t now_tick = rt_tick_get();
            float dt = (now_tick - last_tick) * 1.0f / RT_TICK_PER_SECOND;
            last_tick = now_tick;

            // 转换陀螺仪读数为deg/s，减去零偏
            float gx = (float)gyro[0] / GYRO_SCALE;
            float gy = (float)gyro[1] / GYRO_SCALE;
            float gz = (float)gyro[2] / GYRO_SCALE - gz_offset;

            // 计算加速度角度（弧度）
            float ax = (float)accel[0] / ACCEL_SCALE;
            float ay = (float)accel[1] / ACCEL_SCALE;
            float az = (float)accel[2] / ACCEL_SCALE;

            // 计算俯仰和滚转角度（deg）
            float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / 3.14159265f;
            float accel_roll  = atan2f(ay, az) * 180.0f / 3.14159265f;

            // 互补滤波融合角度（alpha为陀螺仪权重）
            pitch = COMPLEMENTARY_FILTER_ALPHA * (pitch + gy * dt) + (1 - COMPLEMENTARY_FILTER_ALPHA) * accel_pitch;
            roll  = COMPLEMENTARY_FILTER_ALPHA * (roll  + gx * dt) + (1 - COMPLEMENTARY_FILTER_ALPHA) * accel_roll;

            // Yaw只能靠陀螺仪积分，长期漂移
            yaw += gz * dt;
            if (yaw > 180.0f) yaw -= 360.0f;
            else if (yaw < -180.0f) yaw += 360.0f;

            rt_kprintf("Yaw: %d度, Pitch: %d度, Roll: %d度\n", (int)yaw, (int)pitch, (int)roll);

        } else {
            rt_kprintf("Failed to read gyro or accel\n");
        }
        rt_thread_mdelay(100);
    }
}

int mpu_orientation_test(void)
{
    if (mpu_tid == RT_NULL) {
        mpu_tid = rt_thread_create("mpu_test", mpu_thread, RT_NULL, 2048, 12, 10);
        if (mpu_tid) {
            rt_thread_startup(mpu_tid);
        } else {
            rt_kprintf("Failed to create MPU thread\n");
            return -RT_ERROR;
        }
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(mpu_orientation_test, Start MPU6050 orientation test);
