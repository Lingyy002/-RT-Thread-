#include "rplidar.h"
#include <stdio.h>
#include <string.h>

#define DBG_SECTION_NAME  "rplidar"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

#define DEG_TO_RAD(x) ((x) * 3.1415926f / 180.0f)

rt_device_t rp_lidar_create(const char* lidar_name)
{
    RT_ASSERT(lidar_name != "");

    rt_device_t lidar = rt_device_find(lidar_name);
    if (!lidar)
    {
        LOG_E("Find %s failed!", lidar_name);
        return RT_NULL;
    }
    return lidar;
}

rt_err_t rp_lidar_init(rt_device_t lidar)
{
    RT_ASSERT(lidar != RT_NULL);

    rt_err_t ret = rt_device_open(lidar, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    if (ret != RT_EOK)
    {
        LOG_E("Open device failed!");
        return ret;
    }

    // 安全判断是不是串口设备再配置波特率
    if (lidar->type == RT_Device_Class_Char && lidar->flag & RT_DEVICE_FLAG_RDWR)
    {
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
        config.baud_rate = BAUD_RATE_115200;

        ret = rt_device_control(lidar, RT_DEVICE_CTRL_CONFIG, &config);
        if (ret != RT_EOK)
        {
            LOG_E("Set baudrate failed!");
            return ret;
        }
    }

    return RT_EOK;
}

u_result rp_lidar_recev_data(rt_device_t lidar, _u8* buffer, size_t len, _u32 timeout)
{
    RT_ASSERT(lidar != RT_NULL);

    int  recvPos = 0;
    _u32 startTs = rt_tick_get();
    _u32 waitTime;

    LOG_I("%d bytes to receive", len);
    while ((waitTime = rt_tick_get() - startTs) <= rt_tick_from_millisecond(timeout))
    {
        rt_uint8_t ch;
        rt_device_read(lidar, 0, &ch, 1);
        buffer[recvPos] = ch;
        recvPos++;
        if (recvPos == len)
        {
            LOG_I("Received content");
            return RESULT_OK;
        }
    }
    return RESULT_OPERATION_TIMEOUT;
}

u_result rp_lidar_wait_resp_header(rt_device_t lidar, rplidar_ans_header_t * header, _u32 timeout)
{
    RT_ASSERT(lidar != RT_NULL);

    int  recvPos = 0;
    _u8  recvBuffer[sizeof(rplidar_ans_header_t)];

    _u32 startTs = rt_tick_get();
    _u32 waitTime;

    while ((waitTime = rt_tick_get() - startTs) <= rt_tick_from_millisecond(timeout))
    {
        size_t remainSize = sizeof(rplidar_ans_header_t) - recvPos;
        LOG_I("%d bytes to receive", remainSize);
        for(size_t i = 0; i < remainSize; i++)
        {
            rt_uint8_t ch;
            if(rt_device_read(lidar, 0, &ch, 1) != 1)
            {
                return RESULT_OPERATION_TIMEOUT;
            };
            recvBuffer[recvPos] = ch;

            switch (recvPos)
            {
                case 0:
                    if (recvBuffer[recvPos] != RPLIDAR_ANS_SYNC_BYTE1)
                    {
                        continue;
                    }
                    break;
                case 1:
                    if (recvBuffer[recvPos] != RPLIDAR_ANS_SYNC_BYTE2)
                    {
                        recvPos = 0;
                        continue;
                    }
                    break;
            }
            recvPos++;
            if (recvPos == sizeof(rplidar_ans_header_t))
            {
                LOG_I("Received header");
                _u8* header_temp = (_u8*) header;
                memcpy(header_temp, recvBuffer, sizeof(rplidar_ans_header_t));
                return RESULT_OK;
            }
        }
    }

    return RESULT_OPERATION_TIMEOUT;
}

u_result rp_lidar_wait_scan_data(rt_device_t lidar, rplidar_response_measurement_node_t * node, _u32 timeout)
{
    RT_ASSERT(lidar != RT_NULL);

    int  recvPos = 0;
    _u8  recvBuffer[sizeof(rplidar_response_measurement_node_t)];

    _u32 startTs = rt_tick_get();
    _u32 waitTime;

    while ((waitTime = rt_tick_get() - startTs) <= rt_tick_from_millisecond(timeout))
    {
        size_t remainSize = sizeof(rplidar_response_measurement_node_t) - recvPos;
        for(size_t i = 0; i < remainSize; i++)
        {
            rt_uint8_t ch;
            if(rt_device_read(lidar, 0, &ch, 1) != 1)
            {
                return RESULT_OPERATION_TIMEOUT;
            };
            recvBuffer[recvPos] = ch;
            switch (recvPos)
            {
                case 0:
                {
                    _u8 tmp = (recvBuffer[recvPos]>>1);
                    if ( (tmp ^ recvBuffer[recvPos]) & 0x1 )
                    {
                        // pass
                    }
                    else
                    {
                        continue;
                    }
                }
                break;
                case 1:
                    if (recvBuffer[recvPos] & RPLIDAR_RESP_MEASUREMENT_CHECKBIT)
                    {
                        // pass
                    }
                    else
                    {
                        recvPos = 0;
                        continue;
                    }
                    break;
            }
            recvPos++;
            if (recvPos == sizeof(rplidar_response_measurement_node_t))
            {
                _u8* node_temp = (_u8*) node;
                memcpy(node_temp, recvBuffer, sizeof(rplidar_response_measurement_node_t));
                return RESULT_OK;
            }
        }
    }

    return RESULT_OPERATION_TIMEOUT;
}

rt_err_t rp_lidar_get_health(rt_device_t lidar, rplidar_response_device_health_t* health, _u32 timeout)
{
    RT_ASSERT(lidar != RT_NULL);

    rt_err_t res;

    char health_cmd[] = {RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_GET_DEVICE_HEALTH};
    rt_device_write(lidar, 0, (void*)health_cmd , (sizeof(health_cmd)));

    rplidar_ans_header_t* header = (rplidar_ans_header_t*) rt_malloc(sizeof(rplidar_ans_header_t));
    if(header == RT_NULL)
    {
        LOG_E("Out of memory");
        return -RT_ERROR;
    }

    res = rp_lidar_wait_resp_header(lidar, header, timeout);
    if(res != RESULT_OK)
    {
        LOG_E("Read Timout");
        return RESULT_OPERATION_TIMEOUT;
    }

    res = rp_lidar_recev_data(lidar, (_u8*) health, sizeof(rplidar_response_device_health_t), 1000);
    if(res != RESULT_OK)
    {
        return RESULT_OPERATION_TIMEOUT;
    }

    return RT_EOK;
}

rt_err_t rp_lidar_get_device_info(rt_device_t lidar, rplidar_response_device_info_t* info, _u32 timeout)
{
    RT_ASSERT(lidar != RT_NULL);

    rt_err_t res;

    char info_cmd[] = {RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_GET_DEVICE_INFO};
    rt_device_write(lidar, 0, (void*)info_cmd , (sizeof(info_cmd)));

    rplidar_ans_header_t* header = (rplidar_ans_header_t*) rt_malloc(sizeof(rplidar_ans_header_t));
    if(header == RT_NULL)
    {
        LOG_E("Out of memory");
        return -RT_ERROR;
    }

    res = rp_lidar_wait_resp_header(lidar, header, timeout);
    if(res != RESULT_OK)
    {
        LOG_E("Read Timout");
        return RESULT_OPERATION_TIMEOUT;
    }

    res = rp_lidar_recev_data(lidar, (_u8*) info, sizeof(rplidar_response_device_info_t), 1000);
    if(res != RESULT_OK)
    {
        return RESULT_OPERATION_TIMEOUT;
    }

    return RT_EOK;
}

rt_err_t rp_lidar_get_scan_data(rt_device_t lidar, rplidar_response_measurement_node_t* node, _u32 timeout)
{
    RT_ASSERT(lidar != RT_NULL);

    rt_err_t res = rp_lidar_wait_scan_data(lidar, node, 1000);
    if(res != RESULT_OK)
    {
        return RESULT_OPERATION_TIMEOUT;
    }
    return RT_EOK;
}

rt_err_t rp_lidar_stop(rt_device_t lidar)
{
    RT_ASSERT(lidar != RT_NULL);

    rt_err_t res = -RT_ERROR;

    char stop_cmd[] = {RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_STOP};
    if( rt_device_write(lidar, 0, (void*)stop_cmd , sizeof(stop_cmd)) == sizeof(stop_cmd) )
    {
        res = RT_EOK;
    }

    return res;
}

rt_err_t rp_lidar_reset(rt_device_t lidar)
{
    RT_ASSERT(lidar != RT_NULL);

    rt_err_t res = -RT_ERROR;

    char reset_cmd[] = {RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_RESET};
    if( rt_device_write(lidar, 0, (void*)reset_cmd , sizeof(reset_cmd)) == sizeof(reset_cmd) )
    {
        res = RT_EOK;
    }

    return res;
}

rt_err_t rp_lidar_scan(rt_device_t lidar, _u32 timeout)
{
    RT_ASSERT(lidar != RT_NULL);

    rt_err_t res = -RT_ERROR;

    char scan_cmd[] = {RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SCAN};
    if( rt_device_write(lidar, 0, (void*)scan_cmd , sizeof(scan_cmd)) == sizeof(scan_cmd) )
    {
        res = RT_EOK;
    }

    rplidar_ans_header_t* header = (rplidar_ans_header_t*) rt_malloc(sizeof(rplidar_ans_header_t));
    if(header == RT_NULL)
    {
        LOG_E("Out of memory");
        return -RT_ERROR;
    }

    res = rp_lidar_wait_resp_header(lidar, header, timeout);
    if(res != RESULT_OK)
    {
        LOG_E("Read Timeout");
        return RESULT_OPERATION_TIMEOUT;
    }
    if (header->type != RPLIDAR_ANS_TYPE_MEASUREMENT) {
        return RESULT_INVALID_DATA;
    }

    _u32 header_size = (header->size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
    if (header_size < sizeof(rplidar_response_measurement_node_t)) {
        return RESULT_INVALID_DATA;
    }

    return res;
}

#define RPLIDAR_THREAD_STACK_SIZE 2048
#define RPLIDAR_THREAD_PRIORITY   10
#define RPLIDAR_THREAD_TIMESLICE  10

static rt_thread_t rplidar_thread = RT_NULL;

static void rplidar_thread_entry(void *parameter)
{
    rt_device_t lidar = rp_lidar_create("rplidar");
    if (lidar == RT_NULL)
    {
        rt_kprintf("Failed to find LIDAR device\n");
        return;
    }

    if (rp_lidar_init(lidar) != RT_EOK)
    {
        rt_kprintf("Failed to init LIDAR device\n");
        return;
    }

    rplidar_response_device_info_t info;
    if (rp_lidar_get_device_info(lidar, &info, 1000) == RT_EOK)
    {
        rt_kprintf("LIDAR serial: ");
        for (int i = 0; i < 16; i++)
            rt_kprintf("%02X", info.serialnum[i]);
        rt_kprintf("\n");
    }
    else
    {
        rt_kprintf("Failed to get device info\n");
    }

    if (rp_lidar_scan(lidar, 1000) != RT_EOK)
    {
        rt_kprintf("Start scan failed\n");
        return;
    }

    rt_kprintf("Scanning...\n");

    int cnt = 0;
    while (1)
    {
        rplidar_response_measurement_node_t node;
        if (rp_lidar_get_scan_data(lidar, &node, 1000) == RT_EOK)
        {
            float angle = (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
            float distance = node.distance_q2 / 4.0f;

            cnt++;
            if (cnt >= 50)
            {
                cnt = 0;
                rt_kprintf("Angle=%d.%02d deg, Distance=%d.%02d mm\n",
                           (int)angle, (int)(angle * 100) % 100,
                           (int)distance, (int)(distance * 100) % 100);
            }
        }
        else
        {
            rt_kprintf("Timeout receiving scan data\n");
        }

        rt_thread_mdelay(10);
    }
}

int rplidar_thread_init(void)
{
    if (rplidar_thread == RT_NULL)
    {
        rplidar_thread = rt_thread_create("rplidar",
                                         rplidar_thread_entry,
                                         RT_NULL,
                                         RPLIDAR_THREAD_STACK_SIZE,
                                         RPLIDAR_THREAD_PRIORITY,
                                         RPLIDAR_THREAD_TIMESLICE);
        if (rplidar_thread != RT_NULL)
        {
            rt_thread_startup(rplidar_thread);
        }
        else
        {
            rt_kprintf("Create rplidar thread failed!\n");
            return -RT_ERROR;
        }
    }
    return RT_EOK;
}

MSH_CMD_EXPORT(rplidar_thread_init, Start RPLIDAR thread scanning and printing);
