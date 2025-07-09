#include <rtthread.h>
#include <stdlib.h>
#include <string.h>
#include "lora.h"
#include "avoid.h"
#include "gps.h"
// 发送起点坐标（示例硬编码，可根据实际需求替换）
static void send_start_point(void)
{
    // 假设起点经纬度字符串
    const char *start_coords = "start:35.6895,139.6917";  // 东京坐标示例
    rt_kprintf("[LoRa] Sending start point: %s\n", start_coords);
    lora_send_string(start_coords);
}

// 终点坐标存储（可拓展为结构体）
static char end_coords[64] = {0};

// LoRa 数据接收回调函数
static void lora_data_callback(const char *data)
{
    rt_kprintf("[LoRa] Received: %s\n", data);

    // 判断是否是终点坐标格式 "end:lat,lon"
    if (strncmp(data, "end:", 4) == 0)
    {
        // 保存终点字符串
        rt_strncpy(end_coords, data + 4, sizeof(end_coords) - 1);
        rt_kprintf("[LoRa] End point received: %s\n", end_coords);

        // 启动避障系统（启动扫描和避障线程）
        start_avoid(0, RT_NULL);

        // 进入前进状态开始巡线避障
        switch_move_state_threaded(MOVE_FORWARD);
    }
    else if (strcmp(data, "stop") == 0)
    {
        switch_move_state_threaded(MOVE_STOP);
    }
    else if (strcmp(data, "left") == 0)
    {
        switch_move_state_threaded(MOVE_LEFT);
    }
    else if (strcmp(data, "right") == 0)
    {
        switch_move_state_threaded(MOVE_RIGHT);
    }
    else
    {
        rt_kprintf("[LoRa] Unknown command: %s\n", data);
    }
}

int main(void)
{
    // 初始化电机控制、避障相关资源
    ensure_initialized();

    // 初始化LoRa UART和配置线程
    lora_uart_init();
    lora_start_config_thread();

    // 注册LoRa接收回调
    lora_set_receive_callback(lora_data_callback);

    // 发送起点坐标给外部（如基站或其他节点）
    send_start_point();

    rt_kprintf("System initialized and ready.\n");

    // 主线程进入死循环
    while (1)
    {
        rt_thread_mdelay(1000);
    }

    return 0;
}
