//#include <rtthread.h>
//#include <rtdevice.h>
//#include <stdlib.h>
//#include <string.h>
//#include "pulse.h"
//#include "pid.h"
//#include "rplidar.h"
//#include "mpu6050.h"
//
//#define MOTOR_COUNT 4
//#define PWM_PERIOD 10000
//#define PWM_STEP_LIMIT 100
//#define SPEED_RAMP_STEP 600
//#define MOVE_SPEED 1500
//#define SLOW_SPEED 1500
//#define CONFIRM_COUNT 2
//#define CLEAR_CONFIRM_COUNT 2
//#define PWM_MAX 8000
//#define PWM_SYNC_THRESHOLD 50
//#define PWM_SYNC_TIMEOUT_MS 1000
//
//typedef struct {
//    const char* dev_name;
//    int pwm_forward_channel;
//    int pwm_reverse_channel;
//    struct rt_device_pwm *dev;
//} motor_pwm_cfg_t;
//
//// 麦轮车电机顺序及方向 左上 右上 左下 右下
//static motor_pwm_cfg_t motor_cfg[MOTOR_COUNT] = {
//    {"pwm1", 1, 2, RT_NULL},
//    {"pwm1", 3, 4, RT_NULL},
//    {"pwm9", 2, 1, RT_NULL},
//    {"pwm12", 2, 1, RT_NULL},
//};
//static int motor_dir[MOTOR_COUNT] = {-1, +1, -1, +1};
//
//typedef struct { float q, r, x, p, k; } KalmanFilter;
//static KalmanFilter kf[MOTOR_COUNT];
//static PID_Incremental pid[MOTOR_COUNT];
//
//static int target_speeds[MOTOR_COUNT] = {0};
//static int current_speeds[MOTOR_COUNT] = {0};
//static int last_pwm[MOTOR_COUNT] = {0};
//
//static rt_thread_t control_tid = RT_NULL;
//static rt_thread_t avoid_tid = RT_NULL;
//static rt_mutex_t speed_mutex = RT_NULL;
//static rt_mutex_t ramp_mutex = RT_NULL;
//static rt_bool_t system_initialized = RT_FALSE;
//
//typedef enum {
//    MOVE_STOP = 0,
//    MOVE_FORWARD,
//    MOVE_LEFT,
//    MOVE_RIGHT,
//    ROTATE_RIGHT    // 新增原地右转状态
//} MoveState;
//
//typedef enum { AVOID_NONE = 0, AVOID_LEFT, AVOID_RIGHT } AvoidState;
//
//static MoveState current_move_state = MOVE_STOP;
//static AvoidState avoid_state = AVOID_NONE;
//static rt_thread_t move_thread = RT_NULL;
//static rt_bool_t first_start = RT_TRUE;
//
//typedef struct {
//    const char* name;
//    float start_angle;
//    float end_angle;
//    float min_dist;
//} direction_t;
//
//static direction_t directions[] = {
//    {"Front", 160.0f, 200.0f, 10000.0f},
//    {"Right", 250.0f, 300.0f, 10000.0f},
//    {"Back", 320.0f, 20.0f, 10000.0f},
//    {"Left", 50.0f, 120.0f, 10000.0f}
//};
//#define DIRECTION_COUNT (sizeof(directions) / sizeof(directions[0]))
//
//static void kalman_init(KalmanFilter *f, float q, float r) {
//    f->q = q; f->r = r; f->x = 0; f->p = 1; f->k = 0;
//}
//
//static float kalman_update(KalmanFilter *f, float z) {
//    f->p += f->q;
//    f->k = f->p / (f->p + f->r);
//    f->x += f->k * (z - f->x);
//    f->p *= (1 - f->k);
//    return f->x;
//}
//
//static int get_stable_speed(int index) {
//    return (int)kalman_update(&kf[index], (float)(get_encoder_delta(index) * motor_dir[index]));
//}
//
//static void apply_pwm(int index, int pwm) {
//    if (index < 0 || index >= MOTOR_COUNT) return;
//    motor_pwm_cfg_t* cfg = &motor_cfg[index];
//    if (!cfg->dev) return;
//
//    int abs_pwm = pwm > 0 ? pwm : -pwm;
//    if (abs_pwm > PWM_PERIOD) abs_pwm = PWM_PERIOD;
//
//    rt_pwm_set(cfg->dev, cfg->pwm_forward_channel, PWM_PERIOD, (pwm > 0) ? abs_pwm : 0);
//    rt_pwm_set(cfg->dev, cfg->pwm_reverse_channel, PWM_PERIOD, (pwm < 0) ? abs_pwm : 0);
//
//    if (pwm == 0) {
//        rt_pwm_disable(cfg->dev, cfg->pwm_forward_channel);
//        rt_pwm_disable(cfg->dev, cfg->pwm_reverse_channel);
//    } else {
//        rt_pwm_enable(cfg->dev, cfg->pwm_forward_channel);
//        rt_pwm_enable(cfg->dev, cfg->pwm_reverse_channel);
//    }
//}
//
//static void control_thread(void *parameter) {
//    while (1) {
//        int local_target[MOTOR_COUNT];
//        rt_mutex_take(ramp_mutex, RT_WAITING_FOREVER);
//        rt_mutex_take(speed_mutex, RT_WAITING_FOREVER);
//        memcpy(local_target, target_speeds, sizeof(target_speeds));
//        rt_mutex_release(speed_mutex);
//
//        for (int i = 0; i < MOTOR_COUNT; i++) {
//            int delta = local_target[i] - current_speeds[i];
//            if (delta > SPEED_RAMP_STEP) delta = SPEED_RAMP_STEP;
//            else if (delta < -SPEED_RAMP_STEP) delta = -SPEED_RAMP_STEP;
//            current_speeds[i] += delta;
//        }
//        rt_mutex_release(ramp_mutex);
//
//        int yaw_correction = 0;
//        static float smooth_yaw = 0;
//
//        if (current_move_state == MOVE_FORWARD) {
//            float raw_yaw = mpu6050_get_yaw();
//            smooth_yaw = 0.9f * smooth_yaw + 0.1f * raw_yaw;
//
//            if (smooth_yaw > -1.0f && smooth_yaw < 1.0f) {
//                yaw_correction = 0;
//            } else {
//                yaw_correction = (int)(smooth_yaw * 10);
//            }
//
//            const int max_correction = 100;
//            if (yaw_correction > max_correction) yaw_correction = max_correction;
//            else if (yaw_correction < -max_correction) yaw_correction = -max_correction;
//
//            rt_kprintf("[YawCorr] raw=%d°, smooth=%d°, correction=%d\n",
//                       (int)raw_yaw, (int)smooth_yaw, yaw_correction);
//        } else {
//            smooth_yaw = 0;
//        }
//
//        for (int i = 0; i < MOTOR_COUNT; i++) {
//            int speed = get_stable_speed(i);
//            pid[i].setpoint = (float)current_speeds[i];
//            int pwm = pid_incremental_calculate(&pid[i], (float)speed);
//
//            if (pwm > PWM_MAX) pwm = PWM_MAX;
//            else if (pwm < -PWM_MAX) pwm = -PWM_MAX;
//
//            if (pwm - last_pwm[i] > PWM_STEP_LIMIT) pwm = last_pwm[i] + PWM_STEP_LIMIT;
//            else if (pwm - last_pwm[i] < -PWM_STEP_LIMIT) pwm = last_pwm[i] - PWM_STEP_LIMIT;
//
//            if (current_move_state == MOVE_FORWARD) {
//                if (i == 0 || i == 2) pwm += yaw_correction;
//                if (i == 1 || i == 3) pwm -= yaw_correction;
//            }
//
//            if (pwm > PWM_MAX) pwm = PWM_MAX;
//            else if (pwm < -PWM_MAX) pwm = -PWM_MAX;
//
//            rt_kprintf("[PWM] Motor %d: pwm=%d last_pwm=%d\n", i, pwm, last_pwm[i]);
//
//            last_pwm[i] = pwm;
//            apply_pwm(i, pwm);
//        }
//
//        rt_thread_mdelay(15);
//    }
//}
//
//static int angle_in_range(float angle, float start, float end) {
//    return (start <= end) ? (angle >= start && angle <= end)
//                          : (angle >= start || angle <= end);
//}
//
//static void force_stop_and_wait(void) {
//    rt_mutex_take(ramp_mutex, RT_WAITING_FOREVER);
//    rt_mutex_take(speed_mutex, RT_WAITING_FOREVER);
//    for (int i = 0; i < MOTOR_COUNT; i++) {
//        target_speeds[i] = 0;
//        current_speeds[i] = 0;
//        pid_reset(&pid[i]);
//    }
//    rt_mutex_release(speed_mutex);
//    for (int i = 0; i < MOTOR_COUNT; i++) {
//        apply_pwm(i, 0);
//        last_pwm[i] = 0;
//    }
//    int waited = 0;
//    while (waited < 1500) {
//        int all_stopped = 1;
//        for (int i = 0; i < MOTOR_COUNT; i++) {
//            if (abs(get_stable_speed(i)) > 50) {
//                all_stopped = 0; break;
//            }
//        }
//        if (all_stopped) break;
//        rt_thread_mdelay(100);
//        waited += 100;
//    }
//    waited = 0;
//    while (waited < PWM_SYNC_TIMEOUT_MS) {
//        int min = last_pwm[0], max = last_pwm[0];
//        for (int i = 1; i < MOTOR_COUNT; i++) {
//            if (last_pwm[i] < min) min = last_pwm[i];
//            if (last_pwm[i] > max) max = last_pwm[i];
//        }
//        if ((max - min) <= PWM_SYNC_THRESHOLD) break;
//        rt_thread_mdelay(50);
//        waited += 50;
//    }
//    rt_mutex_release(ramp_mutex);
//}
//
//static void move_thread_func(void *param) {
//    MoveState state = (MoveState)(rt_uint32_t)param;
//    int target[MOTOR_COUNT] = {0};
//    if (state == MOVE_FORWARD) {
//        for (int i = 0; i < MOTOR_COUNT; i++) target[i] = MOVE_SPEED;
//    } else if (state == MOVE_LEFT) {
//        target[0] = -MOVE_SPEED; target[1] = MOVE_SPEED;
//        target[2] = MOVE_SPEED; target[3] = -MOVE_SPEED;
//    } else if (state == MOVE_RIGHT) {
//        target[0] = MOVE_SPEED; target[1] = -MOVE_SPEED;
//        target[2] = -MOVE_SPEED; target[3] = MOVE_SPEED;
//    } else if (state == ROTATE_RIGHT) {
//        target[0] = MOVE_SPEED; target[1] = -MOVE_SPEED;
//        target[2] = MOVE_SPEED; target[3] = -MOVE_SPEED;
//    }
//
//    rt_mutex_take(speed_mutex, RT_WAITING_FOREVER);
//    memcpy(target_speeds, target, sizeof(target));
//    rt_mutex_release(speed_mutex);
//
//    while (1) {
//        rt_thread_mdelay(1000);
//    }
//}
//
//static void switch_move_state_threaded(MoveState new_state) {
//    if (new_state == current_move_state) return;
//    if (!first_start) {
//        if (move_thread) {
//            rt_thread_delete(move_thread);
//            move_thread = RT_NULL;
//        }
//        force_stop_and_wait();
//    }
//    first_start = RT_FALSE;
//    move_thread = rt_thread_create("move", move_thread_func, (void *)(rt_uint32_t)new_state, 2048, 11, 10);
//    if (move_thread) {
//        rt_thread_startup(move_thread);
//        current_move_state = new_state;
//    }
//}
//
//static void switch_move_state_safe(MoveState new_state) {
//    switch_move_state_threaded(new_state);
//}
//
//static void obstacle_avoidance_thread(void *param)
//{
//    rt_thread_mdelay(500);
//
//    mpu6050_init("i2c1");
//    mpu6050_angle_reset();
//
//    // 初始化麦轮车：先前进10秒
//    rt_kprintf("[Init] MOVE_FORWARD for 10s...\n");
//    switch_move_state_threaded(MOVE_FORWARD);
//    rt_thread_mdelay(10000);
//
//    // 停止，归零角度
//    rt_kprintf("[Init] MOVE_STOP and reset yaw\n");
//    switch_move_state_threaded(MOVE_STOP);
//    rt_thread_mdelay(300);
//    mpu6050_angle_reset();
//
//    // 右转直到偏航角约90°
//    rt_kprintf("[Init] ROTATE_RIGHT until yaw ≈ 90°...\n");
//    switch_move_state_threaded(ROTATE_RIGHT);
//
//    while (1)
//    {
//        mpu6050_update_yaw();
//        float yaw = mpu6050_get_yaw();
//        rt_kprintf("[Init] yaw = %d°\n", (int)yaw);
//
//        if (yaw <= -85.0f && yaw >= -95.0f)
//        {
//            rt_kprintf("[Init] Reached ~90°, stopping rotation\n");
//
//            switch_move_state_threaded(MOVE_STOP);
//            rt_thread_mdelay(300);
//
//            mpu6050_angle_reset();
//
//            switch_move_state_threaded(MOVE_FORWARD);
//            break;
//        }
//
//        rt_thread_mdelay(50);
//    }
//
//    // 下面开始正常避障循环
//    int front_block_count = 0, left_block_count = 0, right_block_count = 0, clear_front_count = 0;
//
//    rt_device_t lidar = rp_lidar_create("rplidar");
//    if (!lidar || rp_lidar_init(lidar) != RT_EOK || rp_lidar_scan(lidar, 1000) != RT_EOK)
//    {
//        rt_kprintf("Lidar init failed\n");
//        return;
//    }
//
//    avoid_state = AVOID_NONE;
//
//    while (1)
//    {
//        for (int i = 0; i < DIRECTION_COUNT; i++)
//            directions[i].min_dist = 10000.0f;
//
//        for (int i = 0; i < 1000; i++)
//        {
//            rplidar_response_measurement_node_t node;
//            if (rp_lidar_get_scan_data(lidar, &node, 1000) != RT_EOK)
//                continue;
//
//            float angle = (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
//            float dist = node.distance_q2 / 4.0f;
//
//            for (int d = 0; d < DIRECTION_COUNT; d++)
//            {
//                if (angle_in_range(angle, directions[d].start_angle, directions[d].end_angle) &&
//                    dist > 100 && dist < directions[d].min_dist)
//                {
//                    directions[d].min_dist = dist;
//                }
//            }
//        }
//
//        front_block_count = (directions[0].min_dist < 800) ? front_block_count + 1 : 0;
//        right_block_count = (directions[1].min_dist < 800) ? right_block_count + 1 : 0;
//        left_block_count = (directions[3].min_dist < 800) ? left_block_count + 1 : 0;
//        clear_front_count = (directions[0].min_dist > 800) ? clear_front_count + 1 : 0;
//
//        mpu6050_update_yaw();
//
//        switch (avoid_state)
//        {
//        case AVOID_NONE:
//            if (front_block_count >= CONFIRM_COUNT)
//            {
//                rt_kprintf("[Avoid] Obstacle! F=%d L=%d R=%d\n",
//                           (int)directions[0].min_dist,
//                           (int)directions[3].min_dist,
//                           (int)directions[1].min_dist);
//                switch_move_state_threaded(MOVE_STOP);
//                rt_thread_mdelay(300);
//                if (left_block_count < CONFIRM_COUNT)
//                {
//                    rt_kprintf("[Avoid] → MOVE_LEFT\n");
//                    avoid_state = AVOID_LEFT;
//                    switch_move_state_threaded(MOVE_LEFT);
//                }
//                else if (right_block_count < CONFIRM_COUNT)
//                {
//                    rt_kprintf("[Avoid] → MOVE_RIGHT\n");
//                    avoid_state = AVOID_RIGHT;
//                    switch_move_state_threaded(MOVE_RIGHT);
//                }
//                else
//                {
//                    rt_kprintf("[Avoid] → STOP\n");
//                    avoid_state = AVOID_NONE;
//                    switch_move_state_threaded(MOVE_STOP);
//                }
//            }
//            else if (current_move_state != MOVE_FORWARD)
//            {
//                avoid_state = AVOID_NONE;
//                switch_move_state_threaded(MOVE_FORWARD);
//            }
//            break;
//
//        case AVOID_LEFT:
//            if (clear_front_count >= CLEAR_CONFIRM_COUNT)
//            {
//                avoid_state = AVOID_NONE;
//                switch_move_state_threaded(MOVE_STOP);
//                rt_thread_mdelay(300);
//                switch_move_state_threaded(MOVE_FORWARD);
//            }
//            else if (front_block_count >= CONFIRM_COUNT && left_block_count >= CONFIRM_COUNT)
//            {
//                rt_kprintf("[Avoid] LEFT blocked too → switch to RIGHT\n");
//                avoid_state = AVOID_RIGHT;
//                switch_move_state_threaded(MOVE_STOP);
//                rt_thread_mdelay(300);
//                switch_move_state_threaded(MOVE_RIGHT);
//            }
//            break;
//
//        case AVOID_RIGHT:
//            if (clear_front_count >= CLEAR_CONFIRM_COUNT)
//            {
//                avoid_state = AVOID_NONE;
//                switch_move_state_threaded(MOVE_STOP);
//                rt_thread_mdelay(300);
//                switch_move_state_threaded(MOVE_FORWARD);
//            }
//            break;
//        }
//
//        rt_thread_mdelay(50);
//    }
//}
//
//
//
//static void ensure_initialized(void) {
//    if (system_initialized) return;
//    speed_mutex = rt_mutex_create("spd_mtx", RT_IPC_FLAG_FIFO);
//    ramp_mutex  = rt_mutex_create("ramp_mtx", RT_IPC_FLAG_FIFO);
//    for (int i = 0; i < MOTOR_COUNT; i++) {
//        motor_cfg[i].dev = (struct rt_device_pwm *)rt_device_find(motor_cfg[i].dev_name);
//        if (!motor_cfg[i].dev) rt_kprintf("PWM device %s not found\n", motor_cfg[i].dev_name);
//        pid_init(&pid[i], 0.6f, 0.03f, 0.08f);
//        kalman_init(&kf[i], 30.0f, 300.0f);
//    }
//    init_encoders();
//    control_tid = rt_thread_create("ctrl", control_thread, RT_NULL, 2048, 10, 10);
//    if (control_tid) rt_thread_startup(control_tid);
//    system_initialized = RT_TRUE;
//}
//
//int start_avoid(int argc, char *argv[])
//{
//    ensure_initialized();
//    rt_thread_mdelay(500);
//
//    if (avoid_tid == RT_NULL)
//    {
//        avoid_tid = rt_thread_create("avoid", obstacle_avoidance_thread, RT_NULL, 4096, 15, 10);
//        if (avoid_tid)
//            rt_thread_startup(avoid_tid);
//    }
//
//    switch_move_state_safe(MOVE_FORWARD);
//    avoid_state = AVOID_NONE;
//
//    rt_kprintf("Avoid system started.\n");
//    return 0;
//}
//MSH_CMD_EXPORT(start_avoid, Start lidar-based obstacle avoidance);
//
//static rt_thread_t mpu_test_tid = RT_NULL;
//static void mpu6050_test_thread(void *param)
//{
//    mpu6050_init("i2c1");
//    while (1)
//    {
//        float yaw = mpu6050_get_yaw();
//        rt_kprintf("[MPU] yaw=%d deg\n", (int)yaw);
//        rt_thread_mdelay(200);
//    }
//}
//
//int start_mpu6050_test(int argc, char *argv[])
//{
//    if (mpu_test_tid == RT_NULL)
//    {
//        mpu_test_tid = rt_thread_create("mpu", mpu6050_test_thread, RT_NULL, 1024, 12, 10);
//        if (mpu_test_tid) rt_thread_startup(mpu_test_tid);
//    }
//    return 0;
//}
//MSH_CMD_EXPORT(start_mpu6050_test, Start MPU6050 yaw debug test);
