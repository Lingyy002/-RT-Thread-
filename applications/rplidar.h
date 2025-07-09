#ifndef __RPLIDAR_H__
#define __RPLIDAR_H__

#include <rtthread.h>
#include <rtdevice.h>

// 类型定义
typedef unsigned char  _u8;
typedef unsigned int   _u32;
typedef unsigned short _u16;

// RPLIDAR 指令宏
#define RPLIDAR_CMD_SYNC_BYTE                0xA5
#define RPLIDAR_CMD_STOP                     0x25
#define RPLIDAR_CMD_RESET                    0x40
#define RPLIDAR_CMD_SCAN                     0x20
#define RPLIDAR_CMD_FORCE_SCAN               0x21
#define RPLIDAR_CMD_GET_DEVICE_INFO          0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH        0x52

// 应答头同步字节
#define RPLIDAR_ANS_SYNC_BYTE1               0xA5
#define RPLIDAR_ANS_SYNC_BYTE2               0x5A

// 应答数据类型
#define RPLIDAR_ANS_TYPE_MEASUREMENT         0x81
#define RPLIDAR_ANS_TYPE_DEVINFO             0x04
#define RPLIDAR_ANS_TYPE_DEVHEALTH           0x06

// 应答头长度掩码
#define RPLIDAR_ANS_HEADER_SIZE_MASK         0x3FFFFFFF

// 扫描数据位域处理宏
#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT         (0x1)
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT        (0x1 << 0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT     1

// RPLIDAR 通用返回状态
typedef enum {
    RESULT_OK = 0,
    RESULT_FAIL_BIT = 0x80000000,
    RESULT_ALREADY_DONE = 0x20,
    RESULT_INVALID_DATA = 0x8000,
    RESULT_OPERATION_FAIL = 0x8001,
    RESULT_OPERATION_TIMEOUT = 0x8002,
    RESULT_OPERATION_STOP = 0x8003,
    RESULT_OPERATION_NOT_SUPPORT = 0x8004,
    RESULT_FORMAT_NOT_SUPPORT = 0x8005,
    RESULT_INSUFFICIENT_MEMORY = 0x8006,
    RESULT_OPERATION_ABORTED = 0x8007
} u_result;

// ========== 数据结构定义 ==========
#pragma pack(1)

typedef struct {
    _u8  syncByte1;
    _u8  syncByte2;
    _u32 size_q30_subtype;
    _u8  type;
} rplidar_ans_header_t;

typedef struct {
    _u8 model;
    _u16 firmware_version;
    _u8 hardware_version;
    _u8 serialnum[16];
} rplidar_response_device_info_t;

typedef struct {
    _u8 status;
    _u16 error_code;
} rplidar_response_device_health_t;

typedef struct {
    _u8    sync_quality;
    _u16   angle_q6_checkbit;
    _u16   distance_q2;
} rplidar_response_measurement_node_t;

#pragma pack()

// ========== 函数声明 ==========

rt_device_t rp_lidar_create(const char* lidar_name);
rt_err_t rp_lidar_init(rt_device_t lidar);
rt_err_t rp_lidar_scan(rt_device_t lidar, _u32 timeout);
rt_err_t rp_lidar_stop(rt_device_t lidar);
rt_err_t rp_lidar_reset(rt_device_t lidar);

rt_err_t rp_lidar_get_health(rt_device_t lidar, rplidar_response_device_health_t* health, _u32 timeout);
rt_err_t rp_lidar_get_device_info(rt_device_t lidar, rplidar_response_device_info_t* info, _u32 timeout);
rt_err_t rp_lidar_get_scan_data(rt_device_t lidar, rplidar_response_measurement_node_t* node, _u32 timeout);

u_result rp_lidar_wait_scan_data(rt_device_t lidar, rplidar_response_measurement_node_t* node, _u32 timeout);
u_result rp_lidar_wait_resp_header(rt_device_t lidar, rplidar_ans_header_t* header, _u32 timeout);
u_result rp_lidar_recev_data(rt_device_t lidar, _u8* buffer, size_t len, _u32 timeout);

#endif // __RPLIDAR_H__
