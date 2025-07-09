#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/i2c.h>
#include "mpu6050.h"

#define MPU_ADDR 0x68

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;
static rt_tick_t last_tick = 0;
static float yaw = 0.0f;
static float yaw_offset = 0.0f;
static float gyro_z_bias = 0.0f;

static int mpu_write_byte(rt_uint8_t reg, rt_uint8_t data)
{
    struct rt_i2c_msg msg;
    rt_uint8_t buf[2] = {reg, data};

    msg.addr = MPU_ADDR;
    msg.flags = RT_I2C_WR;
    msg.buf = buf;
    msg.len = 2;

    return rt_i2c_transfer(i2c_bus, &msg, 1) == 1 ? RT_EOK : -RT_ERROR;
}

static int mpu_read_bytes(rt_uint8_t reg, rt_uint8_t *buf, rt_uint8_t len)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = MPU_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = MPU_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = buf;
    msgs[1].len = len;

    return rt_i2c_transfer(i2c_bus, msgs, 2) == 2 ? RT_EOK : -RT_ERROR;
}

int mpu6050_init(const char *bus_name)
{
    i2c_bus = (struct rt_i2c_bus_device *)rt_i2c_bus_device_find(bus_name);
    if (!i2c_bus)
    {
        rt_kprintf("MPU6050: Can't find I2C bus %s\n", bus_name);
        return -RT_ERROR;
    }

    if (mpu_write_byte(0x6B, 0x00) != RT_EOK)
    {
        rt_kprintf("MPU6050: Failed to wake up\n");
        return -RT_ERROR;
    }

    // 校准陀螺仪z轴零偏
    int samples = 100;
    int sum = 0;
    short gyro[3];
    for (int i = 0; i < samples; i++) {
        if (mpu6050_read_gyro(gyro) != RT_EOK) {
            rt_thread_mdelay(10);
            continue;
        }
        sum += gyro[2];
        rt_thread_mdelay(10);
    }
    gyro_z_bias = sum / (float)samples;

    last_tick = rt_tick_get();
    yaw = 0.0f;
    yaw_offset = 0.0f;

    rt_kprintf("MPU6050 initialized, gyro_z_bias=%.2f\n", gyro_z_bias);

    return RT_EOK;
}

int mpu6050_read_accel(short *accel)
{
    rt_uint8_t buf[6];
    if (mpu_read_bytes(0x3B, buf, 6) != RT_EOK)
        return -RT_ERROR;

    accel[0] = (buf[0] << 8) | buf[1];
    accel[1] = (buf[2] << 8) | buf[3];
    accel[2] = (buf[4] << 8) | buf[5];
    return RT_EOK;
}

int mpu6050_read_gyro(short *gyro)
{
    rt_uint8_t buf[6];
    if (mpu_read_bytes(0x43, buf, 6) != RT_EOK)
        return -RT_ERROR;

    gyro[0] = (buf[0] << 8) | buf[1];
    gyro[1] = (buf[2] << 8) | buf[3];
    gyro[2] = (buf[4] << 8) | buf[5];
    return RT_EOK;
}

int mpu6050_read_temp(short *temp)
{
    rt_uint8_t buf[2];
    if (mpu_read_bytes(0x41, buf, 2) != RT_EOK)
        return -RT_ERROR;

    *temp = (buf[0] << 8) | buf[1];
    return RT_EOK;
}

void mpu6050_update_yaw(void)
{
    short gyro[3];
    if (mpu6050_read_gyro(gyro) != RT_EOK)
        return;

    rt_tick_t now = rt_tick_get();
    float dt = (now - last_tick) * (1.0f / RT_TICK_PER_SECOND);
    last_tick = now;

    float gz = gyro[2] - gyro_z_bias;
    float deg_per_sec = gz / 131.0f;

    yaw += deg_per_sec * dt;
}

float mpu6050_get_yaw(void)
{
    return yaw - yaw_offset;
}

void mpu6050_angle_reset(void)
{
    yaw_offset = yaw;
}
