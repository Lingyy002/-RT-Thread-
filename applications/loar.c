#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>

#define LORA_UART_NAME "uart5"
#define UART_RX_BUF_SIZE 256

static rt_device_t uart_dev = RT_NULL;
static char uart_rx_buf[UART_RX_BUF_SIZE];
static volatile rt_size_t uart_rx_len = 0;

static struct rt_semaphore start_sem;

/// UART 接收中断回调，将串口数据写入缓冲区
static rt_err_t uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    char buf[64];
    rt_size_t len = rt_device_read(dev, 0, buf, sizeof(buf) - 1);
    if (len > 0)
    {
        buf[len] = '\0';

        if (uart_rx_len + len < UART_RX_BUF_SIZE)
        {
            memcpy(uart_rx_buf + uart_rx_len, buf, len);
            uart_rx_len += len;
            uart_rx_buf[uart_rx_len] = '\0';
        }
        else
        {
            uart_rx_len = 0;
            memcpy(uart_rx_buf, buf, len);
            uart_rx_len = len;
            uart_rx_buf[uart_rx_len] = '\0';
        }
    }
    return RT_EOK;
}

/// UART 发送字符串
void uart_send_string(const char *str)
{
    if (uart_dev == RT_NULL) return;
    rt_device_write(uart_dev, 0, str, strlen(str));
}

/// 串口接收打印线程：以 HEX 格式打印接收到的数据
static void uart_rx_thread_entry(void *parameter)
{
    while (1)
    {
        if (uart_rx_len > 0)
        {
            rt_kprintf("[UART RX HEX]: ");
            for (rt_size_t i = 0; i < uart_rx_len; i++)
            {
                rt_kprintf("%02X ", (unsigned char)uart_rx_buf[i]);
            }
            rt_kprintf("\n");

            uart_rx_len = 0;
            uart_rx_buf[0] = '\0';
        }
        rt_thread_mdelay(200);
    }
}

/// 发送 AT 命令线程，等待 shell 命令触发信号量后开始发送
static void uart_send_cmds_thread(void *parameter)
{
    while (1)
    {
        // 等待 shell 触发信号量
        rt_sem_take(&start_sem, RT_WAITING_FOREVER);

        rt_thread_mdelay(1500);

        // 进入 AT 模式
        uart_send_string("+++\r\n");
        rt_kprintf("[UART TX]: +++\n");
        rt_thread_mdelay(1000);

        const char *cmds[] = {
            "AT+MODE1\r\n",
            "AT+CHANNEL01\r\n",
            "AT+MAC00,01\r\n",
            "AT+LEVEL1\r\n",
            "AT+RESET\r\n"
        };

        for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
        {
            uart_send_string(cmds[i]);
            rt_kprintf("[UART TX]: %s", cmds[i]);
            rt_thread_mdelay(500);
        }

        rt_kprintf("[SYSTEM]: Command sequence done, wait for next start_send.\n");
    }
}

/// shell 命令：start_send，触发发送线程
static rt_err_t cmd_start_send(int argc, char** argv)
{
    rt_sem_release(&start_sem);
    rt_kprintf("[SHELL]: start_send command received, sending AT commands...\n");
    return 0;
}
MSH_CMD_EXPORT(cmd_start_send, trigger LoRa AT command sending);

/// 初始化函数，配置 UART3 并启动线程
int uart3_test_init(void)
{
    uart_dev = rt_device_find(LORA_UART_NAME);
    if (uart_dev == RT_NULL)
    {
        rt_kprintf("Can't find device %s\n", LORA_UART_NAME);
        return -1;
    }

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_9600;
    rt_device_control(uart_dev, RT_DEVICE_CTRL_CONFIG, &config);

    rt_device_open(uart_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(uart_dev, uart_rx_ind);

    rt_sem_init(&start_sem, "start_sem", 0, RT_IPC_FLAG_FIFO);

    rt_thread_t tid;
    tid = rt_thread_create("uart_rx", uart_rx_thread_entry, RT_NULL, 1024, 15, 10);
    if (tid) rt_thread_startup(tid);

    tid = rt_thread_create("uart_tx", uart_send_cmds_thread, RT_NULL, 1024, 20, 10);
    if (tid) rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(uart3_test_init);
