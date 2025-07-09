#include <rtthread.h>
#include <rtdevice.h>
#include "pulse.h"

#define PULSE_ENCODER1_DEV_NAME "pulse2" // MA
#define PULSE_ENCODER2_DEV_NAME "pulse3" // MB
#define PULSE_ENCODER3_DEV_NAME "pulse5" // MC
#define PULSE_ENCODER4_DEV_NAME "pulse4" // MD

static rt_device_t encoder_devs[4] = {RT_NULL};
static int32_t last_counts[4] = {0};

int init_encoders(void)
{
    const char* names[] = {
        PULSE_ENCODER1_DEV_NAME,
        PULSE_ENCODER2_DEV_NAME,
        PULSE_ENCODER3_DEV_NAME,
        PULSE_ENCODER4_DEV_NAME
    };

    for (int i = 0; i < 4; i++)
    {
        encoder_devs[i] = rt_device_find(names[i]);
        if (!encoder_devs[i]) return -RT_ERROR;
        if (rt_device_open(encoder_devs[i], RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
            return -RT_ERROR;
        rt_device_read(encoder_devs[i], 0, &last_counts[i], 1);
    }
    return RT_EOK;
}

int get_encoder_delta(int index)
{
    if (index < 0 || index >= 4 || !encoder_devs[index])
        return -RT_ERROR;

    int32_t current = 0;
    rt_device_read(encoder_devs[index], 0, &current, 1);
    int32_t delta = current - last_counts[index];
    last_counts[index] = current;
    return delta;
}
