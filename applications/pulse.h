#ifndef __PULSE_H__
#define __PULSE_H__

#include <rtdevice.h>

int init_encoders(void);
int get_encoder_delta(int index);  // 返回速度

#endif // __PULSE_H__
