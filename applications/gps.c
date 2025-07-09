#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/serial.h>
#include <stdlib.h>
#include <string.h>
#include <board.h>

#define GPS_UART_NAME "uart2"
#define GPS_THREAD_STACK_SIZE 1024
#define GPS_THREAD_PRIORITY   20
#define GPS_THREAD_TICK       10

static rt_device_t gps_serial = RT_NULL;
static rt_thread_t gps_thread = RT_NULL;

// 简易逗号分割函数，将字符串按逗号切割，返回字段数量
static int simple_split(char *str, char *fields[], int max_fields)
{
    int count = 0;
    char *p = str;
    fields[count++] = p;

    while (*p && count < max_fields)
    {
        if (*p == ',')
        {
            *p = '\0';
            fields[count++] = p + 1;
        }
        p++;
    }
    return count;
}

// 简单浮点转字符串，保留6位小数
static void ftoa(double val, char *buf, int buf_size)
{
    int int_part = (int)val;
    int frac_part = (int)((val - int_part) * 1000000);
    if (frac_part < 0) frac_part = -frac_part;
    rt_snprintf(buf, buf_size, "%d.%06d", int_part, frac_part);
}

// 串口接收回调，只打印经纬度
static rt_err_t gps_rx_ind(rt_device_t dev, rt_size_t size)
{
    char ch;
    static char line_buf[128];
    static int line_index = 0;

    if (rt_device_read(dev, 0, &ch, 1) == 1)
    {
        if (line_index < (int)sizeof(line_buf) - 1)
        {
            line_buf[line_index++] = ch;

            if (ch == '\n')
            {
                line_buf[line_index] = '\0';

                // 只处理 GNRMC 报文
                if (rt_strncmp(line_buf, "$GNRMC", 6) == 0)
                {
                    char *fields[16];
                    int field_count = simple_split(line_buf, fields, 16);

                    if (field_count >= 7 && fields[2][0] == 'A') // 定位有效
                    {
                        double lat = atof(fields[3]);
                        double lon = atof(fields[5]);

                        if (lat != 0.0 && lon != 0.0)
                        {
                            int lat_deg = (int)(lat / 100);
                            double lat_min = lat - lat_deg * 100;
                            double latitude = lat_deg + lat_min / 60.0;
                            if (fields[4][0] == 'S') latitude = -latitude;

                            int lon_deg = (int)(lon / 100);
                            double lon_min = lon - lon_deg * 100;
                            double longitude = lon_deg + lon_min / 60.0;
                            if (fields[6][0] == 'W') longitude = -longitude;

                            char lat_str[20], lon_str[20];
                            ftoa(latitude, lat_str, sizeof(lat_str));
                            ftoa(longitude, lon_str, sizeof(lon_str));

                            rt_kprintf("Latitude: %s, Longitude: %s\n", lat_str, lon_str);
                        }
                    }
                }

                line_index = 0;
            }
        }
        else
        {
            line_index = 0; // 超长丢弃
        }
    }

    return RT_EOK;
}

// GPS线程入口，只做初始化
static void gps_thread_entry(void *parameter)
{
    gps_serial = rt_device_find(GPS_UART_NAME);
    if (gps_serial == RT_NULL)
    {
        rt_kprintf("[gps] Cannot find device %s!\n", GPS_UART_NAME);
        return;
    }

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_9600;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity = PARITY_NONE;
    config.bufsz = 128;

    rt_device_control(gps_serial, RT_DEVICE_CTRL_CONFIG, &config);
    rt_device_open(gps_serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(gps_serial, gps_rx_ind);

    rt_kprintf("[gps] UART opened. Waiting for GPS data...\n");

    while (1)
    {
        rt_thread_mdelay(1000); // 空循环防止线程退出
    }
}

// 启动线程
void gps_thread_start(void)
{
    if (gps_thread == RT_NULL)
    {
        gps_thread = rt_thread_create("gps_thread",
                                      gps_thread_entry, RT_NULL,
                                      GPS_THREAD_STACK_SIZE,
                                      GPS_THREAD_PRIORITY,
                                      GPS_THREAD_TICK);
        if (gps_thread != RT_NULL)
        {
            rt_thread_startup(gps_thread);
            rt_kprintf("[gps] Thread started.\n");
        }
        else
        {
            rt_kprintf("[gps] Thread create failed!\n");
        }
    }
}

MSH_CMD_EXPORT(gps_thread_start, Start GPS UART receive thread and print lat/lon);
