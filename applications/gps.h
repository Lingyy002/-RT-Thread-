#ifndef __GPS_H__
#define __GPS_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/serial.h>

// GPS串口名称
#define GPS_UART_NAME "uart2"

// GPS 线程配置
#define GPS_THREAD_STACK_SIZE 1024
#define GPS_THREAD_PRIORITY   20
#define GPS_THREAD_TICK       10

// GPS线程声明
extern rt_thread_t gps_thread;

// GPS 数据接收回调函数声明
rt_err_t gps_rx_ind(rt_device_t dev, rt_size_t size);

// GPS 线程入口函数声明
void gps_thread_entry(void *parameter);

// 启动 GPS 线程
void gps_thread_start(void);

// 发送起点坐标到API
void send_start_coordinates_to_api(const char *lat, const char *lon);

#endif /* __GPS_H__ */
