#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>

#include "pulse.h"
#include "pid.h"

#define MOTOR_COUNT 4
#define PWM_PERIOD 10000
#define PWM_STEP_LIMIT 500
#define SPEED_RAMP_STEP 200

typedef struct {
    const char* dev_name;
    int pwm_forward_channel;
    int pwm_reverse_channel;
    struct rt_device_pwm *dev;
} motor_pwm_cfg_t;

static motor_pwm_cfg_t motor_cfg[MOTOR_COUNT] = {
    {"pwm1",  1, 2, RT_NULL},
    {"pwm1",  3, 4, RT_NULL},
    {"pwm9",  2, 1, RT_NULL},
    {"pwm12", 2, 1, RT_NULL},
};

static int motor_dir[MOTOR_COUNT] = {-1, +1, -1, +1};

typedef struct {
    float q, r, x, p, k;
} KalmanFilter;

static KalmanFilter kf[MOTOR_COUNT];
static PID_Incremental pid[MOTOR_COUNT];

static int target_speeds[MOTOR_COUNT] = {0};
static int current_speeds[MOTOR_COUNT] = {0};
static int last_pwm[MOTOR_COUNT] = {0};

static rt_thread_t control_tid = RT_NULL;
static rt_bool_t system_initialized = RT_FALSE;

static void kalman_init(KalmanFilter *f, float q, float r) {
    f->q = q; f->r = r; f->x = 0; f->p = 1; f->k = 0;
}

static float kalman_update(KalmanFilter *f, float z) {
    f->p += f->q;
    f->k = f->p / (f->p + f->r);
    f->x += f->k * (z - f->x);
    f->p *= (1 - f->k);
    return f->x;
}

static int get_stable_speed(int index) {
    int raw = get_encoder_delta(index);
    raw *= motor_dir[index];
    float filtered = kalman_update(&kf[index], (float)raw);
    return (int)filtered;
}

static int limit_pwm_change(int index, int new_pwm) {
    int delta = new_pwm - last_pwm[index];
    if (delta > PWM_STEP_LIMIT) delta = PWM_STEP_LIMIT;
    else if (delta < -PWM_STEP_LIMIT) delta = -PWM_STEP_LIMIT;
    last_pwm[index] += delta;
    return last_pwm[index];
}

static void apply_pwm(int index, int pwm) {
    if (index < 0 || index >= MOTOR_COUNT) return;
    motor_pwm_cfg_t* cfg = &motor_cfg[index];
    if (!cfg->dev) return;

    int abs_pwm = pwm > 0 ? pwm : -pwm;
    if (abs_pwm > PWM_PERIOD) abs_pwm = PWM_PERIOD;

    rt_pwm_set(cfg->dev, cfg->pwm_forward_channel, PWM_PERIOD, (pwm > 0) ? abs_pwm : 0);
    rt_pwm_set(cfg->dev, cfg->pwm_reverse_channel, PWM_PERIOD, (pwm < 0) ? abs_pwm : 0);

    rt_pwm_enable(cfg->dev, cfg->pwm_forward_channel);
    rt_pwm_enable(cfg->dev, cfg->pwm_reverse_channel);
}

static void control_thread(void *parameter) {
    int speed[MOTOR_COUNT];
    int pwm[MOTOR_COUNT];

    while (1) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (current_speeds[i] < target_speeds[i]) {
                current_speeds[i] += SPEED_RAMP_STEP;
                if (current_speeds[i] > target_speeds[i]) current_speeds[i] = target_speeds[i];
            } else if (current_speeds[i] > target_speeds[i]) {
                current_speeds[i] -= SPEED_RAMP_STEP;
                if (current_speeds[i] < target_speeds[i]) current_speeds[i] = target_speeds[i];
            }
        }

        for (int i = 0; i < MOTOR_COUNT; i++) {
            speed[i] = get_stable_speed(i);
            pid[i].setpoint = (float)current_speeds[i];
            pwm[i] = pid_incremental_calculate(&pid[i], (float)speed[i]);
            pwm[i] = limit_pwm_change(i, pwm[i]);
            apply_pwm(i, pwm[i]);
        }

        rt_kprintf("Target: %d %d %d %d | Speed: %d %d %d %d | PWM: %d %d %d %d\n",
            target_speeds[0], target_speeds[1], target_speeds[2], target_speeds[3],
            speed[0], speed[1], speed[2], speed[3],
            pwm[0], pwm[1], pwm[2], pwm[3]);

        rt_thread_mdelay(15);
    }
}

static void ensure_initialized(void) {
    if (system_initialized) return;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_cfg[i].dev = (struct rt_device_pwm *)rt_device_find(motor_cfg[i].dev_name);
        if (!motor_cfg[i].dev) {
            rt_kprintf("PWM device %s not found\n", motor_cfg[i].dev_name);
            return;
        }
        pid_init(&pid[i], 0.6f, 0.03f, 0.08f);
        kalman_init(&kf[i], 30.0f, 300.0f);
        current_speeds[i] = 0;
        target_speeds[i] = 0;
    }

    if (init_encoders() != RT_EOK) {
        rt_kprintf("Encoder init failed\n");
        return;
    }

    control_tid = rt_thread_create("ctrl", control_thread, RT_NULL, 2048, 10, 10);
    if (!control_tid) {
        rt_kprintf("Control thread create failed\n");
        return;
    }

    rt_thread_startup(control_tid);
    system_initialized = RT_TRUE;
}

// ==== 控制运动状态 ====
typedef enum {
    MOVE_STOP = 0,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    ROTATE_LEFT,
    ROTATE_RIGHT
} MoveState;

#define MOVE_SPEED 6000
#define ROTATE_SPEED 3000

static void set_move_state(MoveState state) {
    ensure_initialized();
    switch (state) {
        case MOVE_FORWARD:
            for (int i = 0; i < 4; i++) target_speeds[i] = MOVE_SPEED;
            break;
        case MOVE_BACKWARD:
            for (int i = 0; i < 4; i++) target_speeds[i] = -MOVE_SPEED;
            break;
        case MOVE_LEFT:
            target_speeds[0] = -MOVE_SPEED; target_speeds[1] = MOVE_SPEED;
            target_speeds[2] = MOVE_SPEED;  target_speeds[3] = -MOVE_SPEED;
            break;
        case MOVE_RIGHT:
            target_speeds[0] = MOVE_SPEED;  target_speeds[1] = -MOVE_SPEED;
            target_speeds[2] = -MOVE_SPEED; target_speeds[3] = MOVE_SPEED;
            break;
        case ROTATE_LEFT:
            target_speeds[0] = -ROTATE_SPEED; target_speeds[1] = ROTATE_SPEED;
            target_speeds[2] = -ROTATE_SPEED; target_speeds[3] = ROTATE_SPEED;
            break;
        case ROTATE_RIGHT:
            target_speeds[0] = ROTATE_SPEED;  target_speeds[1] = -ROTATE_SPEED;
            target_speeds[2] = ROTATE_SPEED;  target_speeds[3] = -ROTATE_SPEED;
            break;
        default:
            for (int i = 0; i < 4; i++) target_speeds[i] = 0;
            break;
    }
    rt_kprintf("Set speeds: %d %d %d %d\n", target_speeds[0], target_speeds[1], target_speeds[2], target_speeds[3]);
}

int move_stop(int argc, char *argv[]) {
    set_move_state(MOVE_STOP); return 0;
} MSH_CMD_EXPORT(move_stop, Stop);

int move_forward(int argc, char *argv[]) {
    set_move_state(MOVE_FORWARD); return 0;
} MSH_CMD_EXPORT(move_forward, Forward);

int move_backward(int argc, char *argv[]) {
    set_move_state(MOVE_BACKWARD); return 0;
} MSH_CMD_EXPORT(move_backward, Backward);

int move_left(int argc, char *argv[]) {
    set_move_state(MOVE_LEFT); return 0;
} MSH_CMD_EXPORT(move_left, Left);

int move_right(int argc, char *argv[]) {
    set_move_state(MOVE_RIGHT); return 0;
} MSH_CMD_EXPORT(move_right, Right);

int rotate_left(int argc, char *argv[]) {
    set_move_state(ROTATE_LEFT); return 0;
} MSH_CMD_EXPORT(rotate_left, Rotate Left);

int rotate_right(int argc, char *argv[]) {
    set_move_state(ROTATE_RIGHT); return 0;
} MSH_CMD_EXPORT(rotate_right, Rotate Right);
