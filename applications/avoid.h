#ifndef __AVOID_H__
#define __AVOID_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

// 启动避障系统（由 shell 或 main 调用）
int start_avoid(int argc, char *argv[]);
void switch_move_state_threaded(MoveState new_state);

// 控制状态定义
typedef enum {
    MOVE_STOP = 0,
    MOVE_FORWARD,
    MOVE_LEFT,
    MOVE_RIGHT
} MoveState;

typedef enum {
    AVOID_NONE = 0,
    AVOID_LEFT,
    AVOID_RIGHT
} AvoidState;

// 主初始化函数（外部调用确保一次性初始化）
void ensure_initialized(void);


// 强制停车与同步等待
void force_stop_and_wait(void);

// MPU6050测试接口
int mpu_test(int argc, char *argv[]);

#ifdef __cplusplus
}
#endif

#endif /* __AVOID_H__ */
