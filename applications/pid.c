#include "pid.h"

// 你之前代码中的 KP 范围也可以放这里或外部传入，这里先写死
#define KP_MIN 0.3f
#define KP_MAX 0.7f

void pid_init(PID_Incremental* pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->setpoint = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output = 0.0f;
}

int pid_incremental_calculate(PID_Incremental* pid, float feedback)
{
    float error = pid->setpoint - feedback;
    float abs_error = error > 0 ? error : -error;

    // 自适应 Kp
    if (abs_error > 2000)
        pid->kp = KP_MAX;
    else if (abs_error < 500)
        pid->kp = KP_MIN;
    else
        pid->kp = KP_MIN + (KP_MAX - KP_MIN) * ((abs_error - 500.0f) / 1500.0f);

    float delta_output = pid->kp * (error - pid->prev_error)
                       + pid->ki * error
                       + pid->kd * (error - 2 * pid->prev_error + pid->prev_prev_error);

    // 限制单次增量，避免遇阻爆冲
    if (delta_output > 300.0f) delta_output = 300.0f;
    else if (delta_output < -300.0f) delta_output = -300.0f;

    pid->output += delta_output;

    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;

    // 限制输出范围
    if (pid->output > 7000.0f) pid->output = 7000.0f;
    if (pid->output < -7000.0f) pid->output = -7000.0f;

    return (int)(pid->output);
}

void pid_reset(PID_Incremental* pid)
{
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output = 0.0f;
}
