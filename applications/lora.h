#ifndef __LORA_MODULE_H__
#define __LORA_MODULE_H__

#include <rtthread.h>

/// 初始化 LoRa 模块 UART（在系统启动时调用一次）
void lora_uart_init(void);

/// 启动 AT 指令发送线程（等待 shell 命令触发）
void lora_start_config_thread(void);

/// 通过 LoRa 发送一串字符串数据
void lora_send_string(const char *str);

#endif // __LORA_MODULE_H__
