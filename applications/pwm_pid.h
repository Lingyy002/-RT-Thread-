#ifndef PWM_PID_H
#define PWM_PID_H

// 运动状态枚举
typedef enum {
    MOVE_STOP = 0,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    ROTATE_LEFT,
    ROTATE_RIGHT
} MoveState;

// 控制运动状态的函数声明
void set_move_state(MoveState state);

// 电机速度定义
#define MOVE_SPEED 2000
#define SLOW_SPEED 1500

#endif // PWM_PID_H
