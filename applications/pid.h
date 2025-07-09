#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;

    float setpoint;
    float prev_error;
    float prev_prev_error;
    float output;
} PID_Incremental;

void pid_init(PID_Incremental* pid, float kp, float ki, float kd);
int pid_incremental_calculate(PID_Incremental* pid, float feedback);

/**
 * @brief 重置 PID 内部状态（误差历史和输出）
 */
void pid_reset(PID_Incremental* pid);

#ifdef __cplusplus
}
#endif

#endif
