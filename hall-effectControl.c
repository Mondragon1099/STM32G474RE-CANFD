/**
 ******************************************************************************
 * @file    main.c
 * @brief   Solar Boat — Sensor Hub + FDCAN + Steering-Driven Dual Stepper
 *          STM32G474RET6 @ 170 MHz (NUCLEO-G474RE, solarboat_output.ioc)
 *
 * ── What this firmware does ───────────────────────────────────────────────
 *   One main() loop, two concerns:
 *
 *   1) Sensor hub + CAN FD telemetry  (from the sensor-hub build)
 *        - 3 x AS5600 magnetic angle sensors (I2C2/3/4)
 *        - 2 x ADXL345 vibration sensors      (I2C3, addr 0xA6 / 0x3A)
 *        - 2 x Hall-effect motor RPM         (PA0/PA1 EXTI)
 *        - 1 x Throttle ADC                  (PC1 → ADC1_IN7)
 *        All values are framed and broadcast on FDCAN1.
 *
 *   2) Closed-loop dual stepper control  (from the stepper-only build)
 *        - 2 x DM860I drivers @ 3200 microsteps/rev
 *        - Velocity-based pulse-rate commands with asymmetric ramping
 *        - AS5600 #1 (steering wheel) is the setpoint
 *        - AS5600 #2/#3 (rudder feedback) close the loop via P + deadband
 *
 * ── Pin map (from solarboat_output.ioc) ───────────────────────────────────
 *   AS5600 #1 (steering wheel)    I2C3   PC8 SCL   PC9 SDA
 *   AS5600 #2 (rudder left  FB)   I2C4   PC6 SCL   PC7 SDA
 *   AS5600 #3 (rudder right FB)   I2C2   PA9 SCL   PA8 SDA
 *   ADXL345 #1 (vibration L)      I2C3   addr 0xA6  (0x53 << 1)
 *   ADXL345 #2 (vibration R)      I2C3   addr 0x3A  (0x1D << 1)
 *
 *   Hall #1                       PA0  (EXTI0, pull-up, rising+falling)
 *   Hall #2                       PA1  (EXTI1, pull-up, rising+falling)
 *   Throttle                      PC1 → ADC1_IN7
 *
 *   Stepper Left  PUL              PB6  → TIM8  CH1   (AF5)
 *   Stepper Left  DIR              PB4  GPIO output
 *   Stepper Left  EN  (active LOW) PB5  GPIO output
 *
 *   Stepper Right PUL              PB15 → TIM15 CH2   (AF14)
 *   Stepper Right DIR              PB10 GPIO output
 *   Stepper Right EN  (active LOW) PC3  GPIO output
 *
 *   FDCAN1                         PA11 RX / PA12 TX
 *   LPUART1 (NUCLEO VCP)           PA2  TX / PA3  RX
 *   TIM2 (internal)                32-bit, 1 µs tick, free-running for Hall.
 *
 * ── CAN FD TX frames (built via can_frames.c) ─────────────────────────────
 *   ID 0x040 — steering              (AS5600 #1)            @ 500 Hz  (2 ms)
 *   ID 0x122 — stepper_FeedbackLeft  (AS5600 #2)            @  10 Hz  (100 ms)
 *   ID 0x123 — stepper_FeedbackRight (AS5600 #3)            @  10 Hz  (100 ms)
 *   ID 0x2C8 — throttle_Input        (ADC1)                 @  50 Hz  (20 ms)
 *   ID 0x420 — motor_RpmLeft         (Hall #1)              @ 1000 Hz (1 ms)
 *   ID 0x421 — motor_RpmRight        (Hall #2)              @ 1000 Hz (1 ms)
 *   ID 0x480 — motor_VibrationLeft   (ADXL #1, unsafe flag) @ 100 Hz  (10 ms)
 *   ID 0x481 — motor_VibrationRight  (ADXL #2, unsafe flag) @ 100 Hz  (10 ms)
 *
 *   Sensor read cadence and CAN tx cadence are decoupled. AS5600 #1/#2/#3
 *   are all read at 500 Hz so the control loop has fresh feedback every
 *   tick; the feedback CAN frames just sub-sample to 10 Hz.
 *
 * ── Steering-to-stepper control law ───────────────────────────────────────
 *   Both wheel and rudder zero references are captured at boot, on the
 *   first valid I²C read. All subsequent angles are wrap-aware signed
 *   deltas relative to those references.
 *
 *       wheel_delta = angle_diff(wheel, wheel_zero)
 *       target      = clamp(wheel_delta * RUDDER_DEG_MAX / WHEEL_DEG_RANGE,
 *                           ±RUDDER_DEG_MAX)
 *       actual      = clamp(angle_diff(mean(fb_L, fb_R), rudder_zero),
 *                           ±RUDDER_DEG_MAX)
 *       error       = target - actual                       [degrees]
 *       cmd_pps     = clamp(STEER_KP * error, ±STEP_RATE_MAX)
 *       stepper_run_both(cmd_pps)
 *
 *   With STEER_KP = 300 p/s per degree:
 *       1°  error → 300 pps   (~5.6 RPM motor)
 *       10° error → 3000 pps  (~56 RPM)
 *       35° error → 10500 pps (~197 RPM — past this it saturates)
 *
 *   |error| < STEER_DEADBAND_DEG  →  cmd = 0  (motors ramp to stop)
 *
 *   Both motors receive the *same* signed pulse-rate command. Physical
 *   mirroring is handled inside the stepper struct (invert_dir on the
 *   right motor), not here.
 *
 * ── Pulse arithmetic (3200 steps/rev, DM860I 1/16 microstep) ──────────────
 *   1 pulse        = 0.1125° at the motor shaft
 *   STEP_RATE_MAX  = 60000 p/s ≈ 1125 RPM
 *   STEP_RATE_MIN  =   200 p/s ≈ 3.75 RPM
 *   ARR            = (1e6 / rate_pps) - 1   (1 MHz timer tick after PSC=169)
 *   CCR            = ARR / 2                (50 % duty, DM860I needs ≥ 2.5 µs)
 *
 ******************************************************************************
 */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "can_frames.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* =========================================================================
 * PERIPHERAL HANDLES
 * ========================================================================= */
COM_InitTypeDef     BspCOMInit;
ADC_HandleTypeDef   hadc1;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef   hi2c2;       /* AS5600 #3                                 */
I2C_HandleTypeDef   hi2c3;       /* AS5600 #1 + ADXL #1 + ADXL #2             */
I2C_HandleTypeDef   hi2c4;       /* AS5600 #2                                 */
TIM_HandleTypeDef   htim2;       /* free-running 1 µs counter (Hall timing)   */
TIM_HandleTypeDef   htim8;       /* left  stepper PWM (PB6  TIM8  CH1)        */
TIM_HandleTypeDef   htim15;      /* right stepper PWM (PB15 TIM15 CH2)        */

/* USER CODE BEGIN PV */

/* =========================================================================
 * SENSOR STATE
 * ========================================================================= */

/* --- AS5600 #1 steering wheel  (I2C3) --- */
static uint8_t steering_data[2];
static float   steering_deg = 0.0f;

/* --- AS5600 #2 stepper_FeedbackLeft  (I2C4) --- */
static uint8_t stepper_feedbackLeft_data[2];
static float   stepper_feedbackLeft_deg = 0.0f;

/* --- AS5600 #3 stepper_FeedbackRight  (I2C2) --- */
static uint8_t stepper_feedbackRight_data[2];
static float   stepper_feedbackRight_deg = 0.0f;

/* --- Throttle 0-5 V (ADC1 IN7) --- */
uint16_t throttle_raw = 0;
float    throttle_out = 0.0f;

/* --- ADXL345 #1  motor_VibrationLeft --- */
#define ADXL_SAMPLES   10
#define ADXL_THRESH_X  0.08f
#define ADXL_THRESH_Y  0.08f
#define ADXL_THRESH_Z  0.08f
uint8_t  adxl1_id;
uint8_t  adxl1_data[6];
int16_t  ax1, ay1, az1;
float    accel_x1[ADXL_SAMPLES], accel_y1[ADXL_SAMPLES], accel_z1[ADXL_SAMPLES];
float    mean_x1 = 0.0f, mean_y1 = 0.0f, mean_z1 = 0.0f;
float    sum_x1  = 0.0f, sum_y1  = 0.0f, sum_z1  = 0.0f;
uint8_t  adxl_power_ctl = 0x08;
float    adxl_cal_val   = 0.0039f;
int      adxl1_index    = 0;

/* --- ADXL345 #2  motor_VibrationRight --- */
uint8_t  adxl2_id;
uint8_t  adxl2_data[6];
int16_t  ax2, ay2, az2;
float    accel_x2[ADXL_SAMPLES], accel_y2[ADXL_SAMPLES], accel_z2[ADXL_SAMPLES];
float    mean_x2 = 0.0f, mean_y2 = 0.0f, mean_z2 = 0.0f;
float    sum_x2  = 0.0f, sum_y2  = 0.0f, sum_z2  = 0.0f;
int      adxl2_index    = 0;

/* --- Hall sensors  motor_RpmLeft / motor_RpmRight --- */
#define AVG_SAMPLES           5
#define PULSES_PER_REVOLUTION 4
#define RPM_TIMEOUT           3000000U   /* 3 s in µs                         */
#define TICKS_PER_SECOND      1000714.0f /* TIM2 calibrated                   */

volatile uint32_t current_time           = 0;

volatile uint32_t pulse_1_count          = 0;
volatile uint32_t last_time_1            = 0;
volatile float    rpm_1                  = 0.0f;
volatile float    rpm_1_avg              = 0.0f;
volatile uint8_t  new_rpm_1_ready        = 0;
volatile float    rpm_1_buffer[AVG_SAMPLES] = {0};
volatile uint8_t  rpm_1_buffer_index     = 0;

volatile uint32_t pulse_2_count          = 0;
volatile uint32_t last_time_2            = 0;
volatile float    rpm_2                  = 0.0f;
volatile float    rpm_2_avg              = 0.0f;
volatile uint8_t  new_rpm_2_ready        = 0;
volatile float    rpm_2_buffer[AVG_SAMPLES] = {0};
volatile uint8_t  rpm_2_buffer_index     = 0;

/* --- FDCAN filter --- */
FDCAN_FilterTypeDef sFilterConfig;

/* =========================================================================
 * STEPPER CONFIGURATION
 * ========================================================================= */
#define STEP_TIM_PSC               169U
#define STEP_TIM_CLK_HZ            1000000.0f    /* 170 MHz / (169+1) = 1 MHz */
#define STEPS_PER_REV              3200UL

#define STEP_RATE_MAX              60000.0f      /* p/s — hardware ceiling    */
#define STEP_RATE_MIN              200.0f        /* p/s — floor (below = stop)*/

#define RAMP_UP_SLEW               800.0f        /* p/s added  per ctrl tick  */
#define RAMP_DOWN_SLEW             2000.0f       /* p/s removed per ctrl tick */
#define DIR_CHANGE_RAMP_HZ         2000.0f       /* p/s threshold before flip */
#define STEPPER_ENABLE_SETTLE_MS   5U            /* DM860I ENA→pulse settle   */
#define DIR_CHANGE_SETTLE_MS       2U            /* DM860I DIR→pulse settle   */

#define RPM_TO_PPS(rpm)  ((float)(rpm) * (float)STEPS_PER_REV / 60.0f)
#define PPS_TO_RPM(pps)  ((float)(pps) * 60.0f / (float)STEPS_PER_REV)

/* One stepper instance bundles HW config + ramp state-machine state.        */
typedef struct {
    /* Hardware */
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
    GPIO_TypeDef      *en_port;
    uint16_t           en_pin;
    uint8_t            invert_dir;   /* 1 = flip DIR for physical mirroring   */

    /* Runtime (zeroed by stepper_init) */
    uint8_t  enabled;
    uint8_t  enable_wait;
    uint32_t enable_tick;
    uint8_t  dir;                    /* 1=positive, 0=negative                */
    uint8_t  dir_chg;
    uint32_t dir_tick;
    float    speed;                  /* current pulse rate, magnitude (p/s)   */
    float    rate_out;               /* signed live rate (p/s) for telemetry  */
} stepper_t;

stepper_t stepper_left = {
    .htim       = &htim8,
    .channel    = TIM_CHANNEL_1,
    .dir_port   = GPIOB,
    .dir_pin    = DIR_L_Pin,        /* PB4 */
    .en_port    = GPIOB,
    .en_pin     = EN_L_Pin,         /* PB5 */
    .invert_dir = 0
};

stepper_t stepper_right = {
    .htim       = &htim15,
    .channel    = TIM_CHANNEL_2,
    .dir_port   = GPIOB,
    .dir_pin    = DIR_R_Pin,        /* PB10 */
    .en_port    = GPIOC,
    .en_pin     = EN_R_Pin,         /* PC3 */
    .invert_dir = 1                 /* right motor faces opposite way         */
};

/* =========================================================================
 * STEERING-TO-RUDDER CONTROL PARAMETERS
 * ========================================================================= */
#define WHEEL_DEG_RANGE      35.0f      /* full wheel travel from centre, deg */
#define RUDDER_DEG_MAX       35.0f      /* full rudder travel from centre, deg*/
#define STEER_DEADBAND_DEG    1.0f      /* error magnitude below this → cmd=0 */
#define STEER_KP            300.0f      /* pps per degree of rudder error     */

/* Boot-zero references; -1 sentinel = not yet captured.                     */
static float steering_zero_deg  = -1.0f;
static float rudder_zero_deg    = -1.0f;

/* Computed control state (exposed for the status print).                    */
static float target_rudder_deg  =  0.0f;
static float actual_rudder_deg  =  0.0f;
static float steering_error_deg =  0.0f;
static float steer_cmd_pps      =  0.0f;

/* =========================================================================
 * SCHEDULER TICK INTERVALS
 * ========================================================================= */
#define RATE_STEER_CTRL_MS         2U   /* 500 Hz: read all AS5600 + ctrl + tx */
#define RATE_STEPPER_FB_TX_MS    100U   /* 10 Hz: rudder feedback CAN tx       */
#define RATE_THROTTLE_MS          20U   /* 50 Hz                              */
#define RATE_MOTOR_VIBRATION_MS   10U   /* 100 Hz frames, 1 ms inner sample   */
#define RATE_MOTOR_RPM_MS          1U   /* 1000 Hz                            */
#define RATE_STATUS_PRINT_MS     100U   /* 10 Hz UART status                  */

#define ADXL_SAMPLE_INTERVAL_MS (RATE_MOTOR_VIBRATION_MS / ADXL_SAMPLES)  /* 1 ms */

static uint32_t last_tick_steer_ctrl   = 0;
static uint32_t last_tick_fb_left_tx   = 0;
static uint32_t last_tick_fb_right_tx  = 0;
static uint32_t last_tick_throttle     = 0;
static uint32_t last_tick_vib_left     = 0;
static uint32_t last_tick_vib_right    = 0;
static uint32_t last_tick_rpm_left     = 0;
static uint32_t last_tick_rpm_right    = 0;
static uint32_t last_tick_status       = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
static void MX_FDCAN1_Init(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* =========================================================================
 * USER CODE BEGIN 0
 *   Helpers, sensor reads, ISR, stepper driver functions, control law.
 * ========================================================================= */

/* ---- Small math helpers ---- */
static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float angle_wrap_360(float deg)
{
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg <    0.0f) deg += 360.0f;
    return deg;
}

/* Wrap-aware signed difference target-current in [-180, 180]. */
static float angle_diff_deg(float target, float current)
{
    float d = target - current;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/* Circular mean of two angles (handles wraparound near 0°/360°). */
static float circular_mean_deg(float a_deg, float b_deg)
{
    float a_rad = a_deg * (float)M_PI / 180.0f;
    float b_rad = b_deg * (float)M_PI / 180.0f;
    float x = cosf(a_rad) + cosf(b_rad);
    float y = sinf(a_rad) + sinf(b_rad);
    return angle_wrap_360(atan2f(y, x) * 180.0f / (float)M_PI);
}

/* ---- One-shot AS5600 read.
 *      Returns degrees in [0, 360) on success, -1.0 on I²C error.
 *      Single 2-byte register read at 0x0E (ANGLE high byte); the AS5600
 *      auto-increments to 0x0F for the low byte. Result masked to 12 bits.
 *      5 ms timeout — at 500 Hz we cannot afford to block on a wedged bus. */
static float as5600_read_deg(I2C_HandleTypeDef *hi2c, uint8_t *buf)
{
    const uint8_t addr = (0x36 << 1);
    if (HAL_I2C_Mem_Read(hi2c, addr, 0x0E, I2C_MEMADD_SIZE_8BIT,
                         buf, 2, 5 /* ms */) != HAL_OK) {
        return -1.0f;
    }
    uint16_t raw = (((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;
    return (raw * 360.0f) / 4096.0f;
}

/* ---- One-shot ADC poll (throttle). ---- */
static uint16_t read_adc(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return v;
}

/* =========================================================================
 * STEPPER LOW-LEVEL HELPERS
 * ========================================================================= */

/* Set pulse rate in p/s. rate_pps < STEP_RATE_MIN → silence (CCR=0).
 * Never clears BDTR.MOE — that's left set permanently by stepper_init(),
 * which avoids the re-enable glitch.                                       */
static void stepper_write_rate(stepper_t *s, float rate_pps)
{
    if (rate_pps < STEP_RATE_MIN) {
        __HAL_TIM_SET_COMPARE(s->htim, s->channel, 0U);
        s->htim->Instance->EGR = TIM_EGR_UG;
        return;
    }
    if (rate_pps > STEP_RATE_MAX) rate_pps = STEP_RATE_MAX;

    uint32_t arr = (uint32_t)((STEP_TIM_CLK_HZ / rate_pps) - 1.0f);
    if (arr < 10U)    arr = 10U;
    if (arr > 65535U) arr = 65535U;

    __HAL_TIM_SET_AUTORELOAD(s->htim, arr);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, arr / 2U);   /* 50 % duty      */
    s->htim->Instance->EGR = TIM_EGR_UG;
}

/* Write logical direction (transparently inverts for mirrored motors). */
static void stepper_write_dir(stepper_t *s, uint8_t positive)
{
    uint8_t actual = s->invert_dir ? (uint8_t)(!positive) : positive;
    HAL_GPIO_WritePin(s->dir_port, s->dir_pin,
                      actual ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* DM860I ENA is active LOW: enable=1 → pin LOW → driver on. */
static void stepper_write_enable(stepper_t *s, uint8_t enable)
{
    HAL_GPIO_WritePin(s->en_port, s->en_pin,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* =========================================================================
 * STEPPER PUBLIC API  (velocity-based ramp; same as the standalone build)
 * ========================================================================= */

void stepper_init(stepper_t *s)
{
    stepper_write_rate(s, 0.0f);
    stepper_write_enable(s, 0);

    s->enabled      = 0U;
    s->enable_wait  = 0U;
    s->enable_tick  = 0U;
    s->dir          = 1U;
    s->dir_chg      = 0U;
    s->dir_tick     = 0U;
    s->speed        = 0.0f;
    s->rate_out     = 0.0f;

    /* Permanent output-stage enable. CCR=0 silences without clearing MOE. */
    s->htim->Instance->BDTR |= TIM_BDTR_MOE;
}

void stepper_hard_stop(stepper_t *s)
{
    stepper_write_rate(s, 0.0f);
    stepper_write_enable(s, 0);
    s->enabled      = 0U;
    s->enable_wait  = 0U;
    s->speed        = 0.0f;
    s->rate_out     = 0.0f;
}

/* Tick toward signed cmd_pps. Must be called every RATE_STEER_CTRL_MS ms.
 * Handles enable/dir settle gates, direction reversals, asymmetric ramp.   */
void stepper_run(stepper_t *s, float cmd_pps)
{
    uint32_t now      = HAL_GetTick();
    uint8_t  want_dir = (cmd_pps >= 0.0f) ? 1U : 0U;
    float    target   = fabsf(cmd_pps);
    if (target > STEP_RATE_MAX) target = STEP_RATE_MAX;

    /* Gate 1: enable settle */
    if (s->enable_wait) {
        if ((now - s->enable_tick) < STEPPER_ENABLE_SETTLE_MS) {
            stepper_write_rate(s, 0.0f);
            s->rate_out = 0.0f;
            return;
        }
        s->enable_wait = 0U;
        s->enabled     = 1U;
        if (s->speed < STEP_RATE_MIN) s->speed = STEP_RATE_MIN;
    }

    /* Gate 2: direction-change settle */
    if (s->dir_chg) {
        if ((now - s->dir_tick) < DIR_CHANGE_SETTLE_MS) {
            stepper_write_rate(s, 0.0f);
            s->rate_out = 0.0f;
            return;
        }
        s->dir_chg = 0U;
        s->speed   = STEP_RATE_MIN;
    }

    /* Direction reversal: ramp down to threshold, then flip. */
    if ((target >= STEP_RATE_MIN) && (want_dir != s->dir)) {
        if (s->speed > DIR_CHANGE_RAMP_HZ) {
            s->speed -= RAMP_DOWN_SLEW;
            if (s->speed < DIR_CHANGE_RAMP_HZ) s->speed = DIR_CHANGE_RAMP_HZ;

            if (!s->enabled && !s->enable_wait) {
                stepper_write_dir(s, s->dir);
                stepper_write_enable(s, 1);
                s->enable_wait = 1U;
                s->enable_tick = now;
                s->rate_out    = 0.0f;
                return;
            }
            stepper_write_rate(s, s->speed);
            s->rate_out = s->dir ? s->speed : -s->speed;
            return;
        }
        stepper_write_rate(s, 0.0f);
        s->dir = want_dir;
        stepper_write_dir(s, s->dir);
        s->dir_chg  = 1U;
        s->dir_tick = now;
        s->speed    = 0.0f;
        s->rate_out = 0.0f;
        return;
    }

    /* Normal ramp */
    if (target < STEP_RATE_MIN) {
        if (s->speed > 0.0f) {
            s->speed -= RAMP_DOWN_SLEW;
            if (s->speed <= STEP_RATE_MIN) {
                stepper_hard_stop(s);
                return;
            }
        } else {
            stepper_hard_stop(s);
            return;
        }
    }
    else if (s->speed < target) {
        s->speed += RAMP_UP_SLEW;
        if (s->speed > target) s->speed = target;
    }
    else if (s->speed > target) {
        s->speed -= RAMP_DOWN_SLEW;
        if (s->speed < target) s->speed = target;
        if (s->speed < STEP_RATE_MIN) {
            stepper_hard_stop(s);
            return;
        }
    }

    /* Enable driver if first non-zero command after idle. */
    if (!s->enabled && !s->enable_wait) {
        stepper_write_dir(s, s->dir);
        stepper_write_enable(s, 1);
        s->enable_wait = 1U;
        s->enable_tick = now;
        s->rate_out    = 0.0f;
        return;
    }

    stepper_write_rate(s, s->speed);
    s->rate_out = s->dir ? s->speed : -s->speed;
}

void stepper_run_both(float cmd_pps)
{
    stepper_run(&stepper_left,  cmd_pps);
    stepper_run(&stepper_right, cmd_pps);
}

void stepper_stop_both(void)
{
    stepper_hard_stop(&stepper_left);
    stepper_hard_stop(&stepper_right);
}

/* =========================================================================
 * STEERING CONTROL TICK  (500 Hz)
 *
 * Reads all three AS5600s on this tick, captures boot-zero references on
 * first valid read, runs the P-with-deadband control law, and commands
 * both steppers via stepper_run_both().
 * ========================================================================= */
static void steering_control_tick(void)
{
    /* Read all 3 magnetic encoders. Only latch valid (>=0) readings. */
    float v;

    v = as5600_read_deg(&hi2c3, steering_data);
    if (v >= 0.0f) steering_deg = v;

    v = as5600_read_deg(&hi2c4, stepper_feedbackLeft_data);
    if (v >= 0.0f) stepper_feedbackLeft_deg = v;

    v = as5600_read_deg(&hi2c2, stepper_feedbackRight_data);
    if (v >= 0.0f) stepper_feedbackRight_deg = v;

    /* Boot-zero capture (first tick where we have any reading). */
    if (steering_zero_deg < 0.0f) {
        steering_zero_deg = steering_deg;
    }
    if (rudder_zero_deg < 0.0f) {
        rudder_zero_deg = circular_mean_deg(stepper_feedbackLeft_deg,
                                            stepper_feedbackRight_deg);
    }

    /* Wheel command → target rudder angle. */
    float wheel_delta = angle_diff_deg(steering_deg, steering_zero_deg);
    target_rudder_deg = clampf(
        wheel_delta * (RUDDER_DEG_MAX / WHEEL_DEG_RANGE),
        -RUDDER_DEG_MAX, +RUDDER_DEG_MAX);

    /* Sensor fusion (circular mean) → actual rudder angle. */
    float fb_mean = circular_mean_deg(stepper_feedbackLeft_deg,
                                      stepper_feedbackRight_deg);
    actual_rudder_deg = clampf(
        angle_diff_deg(fb_mean, rudder_zero_deg),
        -RUDDER_DEG_MAX, +RUDDER_DEG_MAX);

    /* P controller with deadband. */
    steering_error_deg = target_rudder_deg - actual_rudder_deg;
    if (fabsf(steering_error_deg) < STEER_DEADBAND_DEG) {
        steer_cmd_pps = 0.0f;
    } else {
        steer_cmd_pps = clampf(
            steering_error_deg * STEER_KP,
            -STEP_RATE_MAX, +STEP_RATE_MAX);
    }

    /* Drive both steppers with the same signed pulse-rate command. */
    stepper_run_both(steer_cmd_pps);
}

/* =========================================================================
 * HALL-SENSOR EXTI ISR
 *   PA0 → motor_RpmLeft   PA1 → motor_RpmRight
 *   Computes instantaneous RPM from inter-edge time and pushes into a
 *   moving-average ring buffer.
 * ========================================================================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t time_diff;
    current_time = __HAL_TIM_GET_COUNTER(&htim2);

    if (GPIO_Pin == GPIO_PIN_0) {
        time_diff = (current_time >= last_time_1)
                  ? (current_time - last_time_1)
                  : (0xFFFFFFFFU - last_time_1 + current_time);

        if (time_diff > 5000U) {
            rpm_1 = (60.0f * TICKS_PER_SECOND) /
                    ((float)time_diff * (float)PULSES_PER_REVOLUTION);
            rpm_1_buffer[rpm_1_buffer_index] = rpm_1;
            rpm_1_buffer_index = (rpm_1_buffer_index + 1) % AVG_SAMPLES;

            float sum = 0.0f;
            for (int i = 0; i < AVG_SAMPLES; i++) sum += rpm_1_buffer[i];
            rpm_1_avg       = sum / (float)AVG_SAMPLES;
            last_time_1     = current_time;
            pulse_1_count++;
            new_rpm_1_ready = 1;
        }
    }
    else if (GPIO_Pin == GPIO_PIN_1) {
        time_diff = (current_time >= last_time_2)
                  ? (current_time - last_time_2)
                  : (0xFFFFFFFFU - last_time_2 + current_time);

        if (time_diff > 5000U) {
            rpm_2 = (60.0f * TICKS_PER_SECOND) /
                    ((float)time_diff * (float)PULSES_PER_REVOLUTION);
            rpm_2_buffer[rpm_2_buffer_index] = rpm_2;
            rpm_2_buffer_index = (rpm_2_buffer_index + 1) % AVG_SAMPLES;

            float sum = 0.0f;
            for (int i = 0; i < AVG_SAMPLES; i++) sum += rpm_2_buffer[i];
            rpm_2_avg       = sum / (float)AVG_SAMPLES;
            last_time_2     = current_time;
            pulse_2_count++;
            new_rpm_2_ready = 1;
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  Application entry point.
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    MX_GPIO_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_I2C4_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM8_Init();
    MX_TIM15_Init();
    MX_FDCAN1_Init();

    /* USER CODE BEGIN 2 */

    /* µs counter for Hall ISR timing */
    HAL_TIM_Base_Start(&htim2);

    /* Stepper PWM outputs — stay silent (CCR=0) until first stepper_run() */
    HAL_TIM_PWM_Start(&htim8,  TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
    stepper_init(&stepper_left);
    stepper_init(&stepper_right);

    /* ADXL345 #1 init (addr 0xA6) */
    HAL_I2C_Mem_Read (&hi2c3, 0xA6, 0x00, I2C_MEMADD_SIZE_8BIT,
                      &adxl1_id, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0xA6, 0x2D, I2C_MEMADD_SIZE_8BIT,
                      &adxl_power_ctl, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0xA6, 0x31, I2C_MEMADD_SIZE_8BIT,
                      &adxl_power_ctl, 1, HAL_MAX_DELAY);

    /* ADXL345 #2 init (addr 0x3A) */
    HAL_I2C_Mem_Read (&hi2c3, 0x3A, 0x00, I2C_MEMADD_SIZE_8BIT,
                      &adxl2_id, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0x3A, 0x2D, I2C_MEMADD_SIZE_8BIT,
                      &adxl_power_ctl, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0x3A, 0x31, I2C_MEMADD_SIZE_8BIT,
                      &adxl_power_ctl, 1, HAL_MAX_DELAY);

    /* FDCAN filter + start (accept-all goes to FIFO0; we only TX) */
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x000;
    sFilterConfig.FilterID2    = 0x000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
        Error_Handler();

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
            FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
        Error_Handler();

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        Error_Handler();

    /* UART console */
    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
        Error_Handler();

    printf("Solar Boat ready.\r\n");
    printf("Hold the steering wheel at centre on power-up so the zero\r\n");
    printf("references capture cleanly. WHL_RANGE=%.1f deg  RUDDER_MAX=%.1f deg\r\n",
           WHEEL_DEG_RANGE, RUDDER_DEG_MAX);
    printf("STEPS_PER_REV=%lu  STEP_RATE_MAX=%.0f p/s (%.1f RPM)\r\n",
           STEPS_PER_REV, STEP_RATE_MAX, PPS_TO_RPM(STEP_RATE_MAX));

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* ── 500 Hz: read all AS5600s, run control law, drive steppers,
         *           send steering CAN frame (ID 0x040). */
        if (now - last_tick_steer_ctrl >= RATE_STEER_CTRL_MS)
        {
            steering_control_tick();
            can_tx_steering1(steering_deg);
            last_tick_steer_ctrl = now;
        }

        /* ── 10 Hz: ID 0x122  stepper_FeedbackLeft  (AS5600 #2) */
        if (now - last_tick_fb_left_tx >= RATE_STEPPER_FB_TX_MS)
        {
            can_tx_steering2(stepper_feedbackLeft_deg);
            last_tick_fb_left_tx = now;
        }

        /* ── 10 Hz: ID 0x123  stepper_FeedbackRight (AS5600 #3) */
        if (now - last_tick_fb_right_tx >= RATE_STEPPER_FB_TX_MS)
        {
            can_tx_steering3(stepper_feedbackRight_deg);
            last_tick_fb_right_tx = now;
        }

        /* ── 50 Hz: ID 0x2C8 throttle_Input (ADC1) */
        if (now - last_tick_throttle >= RATE_THROTTLE_MS)
        {
            throttle_raw = read_adc();
            throttle_out = (throttle_raw * 3.3f) / 4096.0f;
            can_tx_throttle(throttle_out);
            last_tick_throttle = now;
        }

        /* ── 100 Hz frame / 1 ms sample: ID 0x480 motor_VibrationLeft */
        if (now - last_tick_vib_left >= ADXL_SAMPLE_INTERVAL_MS)
        {
            HAL_I2C_Mem_Read(&hi2c3, 0xA6, 0x32, I2C_MEMADD_SIZE_8BIT,
                             adxl1_data, 6, HAL_MAX_DELAY);
            ax1 = (int16_t)((adxl1_data[1] << 8) | adxl1_data[0]);
            ay1 = (int16_t)((adxl1_data[3] << 8) | adxl1_data[2]);
            az1 = (int16_t)((adxl1_data[5] << 8) | adxl1_data[4]);

            if (adxl1_index < ADXL_SAMPLES) {
                accel_x1[adxl1_index] = ax1 * adxl_cal_val;
                accel_y1[adxl1_index] = ay1 * adxl_cal_val;
                accel_z1[adxl1_index] = az1 * adxl_cal_val;
                adxl1_index++;
            } else {
                mean_x1 = 0.0f; mean_y1 = 0.0f; mean_z1 = 0.0f;
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_x1 += accel_x1[i];
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_y1 += accel_y1[i];
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_z1 += accel_z1[i];
                mean_x1 /= ADXL_SAMPLES;
                mean_y1 /= ADXL_SAMPLES;
                mean_z1 /= ADXL_SAMPLES;

                sum_x1 = 0.0f; sum_y1 = 0.0f; sum_z1 = 0.0f;
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_x1 += (accel_x1[i] - mean_x1) * (accel_x1[i] - mean_x1);
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_y1 += (accel_y1[i] - mean_y1) * (accel_y1[i] - mean_y1);
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_z1 += (accel_z1[i] - mean_z1) * (accel_z1[i] - mean_z1);

                uint8_t motor_vibrationLeft_unsafe =
                    (sqrtf(sum_x1 / (float)ADXL_SAMPLES) > ADXL_THRESH_X) ||
                    (sqrtf(sum_y1 / (float)ADXL_SAMPLES) > ADXL_THRESH_Y) ||
                    (sqrtf(sum_z1 / (float)ADXL_SAMPLES) > ADXL_THRESH_Z);
                can_tx_adxl1(motor_vibrationLeft_unsafe);

                adxl1_index = 0;
                sum_x1 = 0.0f; sum_y1 = 0.0f; sum_z1 = 0.0f;
                mean_x1 = 0.0f; mean_y1 = 0.0f; mean_z1 = 0.0f;
            }
            last_tick_vib_left = now;
        }

        /* ── 100 Hz frame / 1 ms sample: ID 0x481 motor_VibrationRight */
        if (now - last_tick_vib_right >= ADXL_SAMPLE_INTERVAL_MS)
        {
            HAL_I2C_Mem_Read(&hi2c3, 0x3A, 0x32, I2C_MEMADD_SIZE_8BIT,
                             adxl2_data, 6, HAL_MAX_DELAY);
            ax2 = (int16_t)((adxl2_data[1] << 8) | adxl2_data[0]);
            ay2 = (int16_t)((adxl2_data[3] << 8) | adxl2_data[2]);
            az2 = (int16_t)((adxl2_data[5] << 8) | adxl2_data[4]);

            if (adxl2_index < ADXL_SAMPLES) {
                accel_x2[adxl2_index] = ax2 * adxl_cal_val;
                accel_y2[adxl2_index] = ay2 * adxl_cal_val;
                accel_z2[adxl2_index] = az2 * adxl_cal_val;
                adxl2_index++;
            } else {
                mean_x2 = 0.0f; mean_y2 = 0.0f; mean_z2 = 0.0f;
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_x2 += accel_x2[i];
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_y2 += accel_y2[i];
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_z2 += accel_z2[i];
                mean_x2 /= ADXL_SAMPLES;
                mean_y2 /= ADXL_SAMPLES;
                mean_z2 /= ADXL_SAMPLES;

                sum_x2 = 0.0f; sum_y2 = 0.0f; sum_z2 = 0.0f;
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_x2 += (accel_x2[i] - mean_x2) * (accel_x2[i] - mean_x2);
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_y2 += (accel_y2[i] - mean_y2) * (accel_y2[i] - mean_y2);
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_z2 += (accel_z2[i] - mean_z2) * (accel_z2[i] - mean_z2);

                uint8_t motor_vibrationRight_unsafe =
                    (sqrtf(sum_x2 / (float)ADXL_SAMPLES) > ADXL_THRESH_X) ||
                    (sqrtf(sum_y2 / (float)ADXL_SAMPLES) > ADXL_THRESH_Y) ||
                    (sqrtf(sum_z2 / (float)ADXL_SAMPLES) > ADXL_THRESH_Z);
                can_tx_adxl2(motor_vibrationRight_unsafe);

                adxl2_index = 0;
                sum_x2 = 0.0f; sum_y2 = 0.0f; sum_z2 = 0.0f;
                mean_x2 = 0.0f; mean_y2 = 0.0f; mean_z2 = 0.0f;
            }
            last_tick_vib_right = now;
        }

        /* ── 1000 Hz: ID 0x420 motor_RpmLeft (Hall #1) */
        if (now - last_tick_rpm_left >= RATE_MOTOR_RPM_MS)
        {
            current_time = __HAL_TIM_GET_COUNTER(&htim2);
            uint32_t time_since = (current_time >= last_time_1)
                                ? (current_time - last_time_1)
                                : (0xFFFFFFFFU - last_time_1 + current_time);
            if (time_since > RPM_TIMEOUT) {
                rpm_1     = 0.0f;
                rpm_1_avg = 0.0f;
            }
            can_tx_rpm(rpm_1_avg);
            last_tick_rpm_left = now;
        }

        /* ── 1000 Hz: ID 0x421 motor_RpmRight (Hall #2) */
        if (now - last_tick_rpm_right >= RATE_MOTOR_RPM_MS)
        {
            current_time = __HAL_TIM_GET_COUNTER(&htim2);
            uint32_t time_since = (current_time >= last_time_2)
                                ? (current_time - last_time_2)
                                : (0xFFFFFFFFU - last_time_2 + current_time);
            if (time_since > RPM_TIMEOUT) {
                rpm_2     = 0.0f;
                rpm_2_avg = 0.0f;
            }
            can_tx_rpm2(rpm_2_avg);
            last_tick_rpm_right = now;
        }

        /* ── 10 Hz: consolidated status print
         *    Per-sensor printfs at higher rates flood the UART at 115200,
         *    so everything is summarised here once every 100 ms. */
        if (now - last_tick_status >= RATE_STATUS_PRINT_MS)
        {
            printf("WHL=%6.1f  TGT=%5.1f  FB=%5.1f  ERR=%5.1f  CMD=%6.0fp/s | "
                   "L=%6.0fp/s(%5.1fRPM)  R=%6.0fp/s(%5.1fRPM) | "
                   "THR=%.2fV  RPM_L=%5.1f  RPM_R=%5.1f\r\n",
                   steering_deg,
                   target_rudder_deg,
                   actual_rudder_deg,
                   steering_error_deg,
                   steer_cmd_pps,
                   stepper_left.rate_out,  PPS_TO_RPM(fabsf(stepper_left.rate_out)),
                   stepper_right.rate_out, PPS_TO_RPM(fabsf(stepper_right.rate_out)),
                   throttle_out, rpm_1_avg, rpm_2_avg);
            last_tick_status = now;
        }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/* =========================================================================
 * SYSTEM CLOCK CONFIGURATION
 *   HSE 24 MHz → PLLM/6 → PLLN×85 → PLLR/2 → SYSCLK 170 MHz
 * ========================================================================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV6;
    RCC_OscInitStruct.PLL.PLLN       = 85;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

/* =========================================================================
 * PERIPHERAL INIT FUNCTIONS
 * ========================================================================= */

static void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef   multimode = {0};
    ADC_ChannelConfTypeDef sConfig   = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation      = 0;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
        Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_7;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_FD_NO_BRS;
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission   = ENABLE;
    hfdcan1.Init.TransmitPause        = ENABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;
    hfdcan1.Init.NominalPrescaler     = 2;
    hfdcan1.Init.NominalSyncJumpWidth = 39;
    hfdcan1.Init.NominalTimeSeg1      = 130;
    hfdcan1.Init.NominalTimeSeg2      = 39;
    hfdcan1.Init.DataPrescaler        = 17;
    hfdcan1.Init.DataSyncJumpWidth    = 6;
    hfdcan1.Init.DataTimeSeg1         = 13;
    hfdcan1.Init.DataTimeSeg2         = 6;
    hfdcan1.Init.StdFiltersNbr        = 1;
    hfdcan1.Init.ExtFiltersNbr        = 0;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_QUEUE_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) Error_Handler();
}

static void MX_I2C2_Init(void)
{
    hi2c2.Instance              = I2C2;
    hi2c2.Init.Timing           = 0x40B285C2;
    hi2c2.Init.OwnAddress1      = 0;
    hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2      = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) Error_Handler();
}

static void MX_I2C3_Init(void)
{
    hi2c3.Instance              = I2C3;
    hi2c3.Init.Timing           = 0x40B285C2;
    hi2c3.Init.OwnAddress1      = 0;
    hi2c3.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2      = 0;
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c3.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) Error_Handler();
}

static void MX_I2C4_Init(void)
{
    hi2c4.Instance              = I2C4;
    hi2c4.Init.Timing           = 0x40B285C2;
    hi2c4.Init.OwnAddress1      = 0;
    hi2c4.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c4.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c4.Init.OwnAddress2      = 0;
    hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c4.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c4.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c4) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = STEP_TIM_PSC;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFFFFFFUL;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        Error_Handler();
}

/* TIM8 — left motor PWM on PB6, CH1, AF5 */
static void MX_TIM8_Init(void)
{
    TIM_ClockConfigTypeDef         sClockSourceConfig  = {0};
    TIM_MasterConfigTypeDef        sMasterConfig        = {0};
    TIM_OC_InitTypeDef             sConfigOC            = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    GPIO_InitTypeDef               GPIO_InitStruct      = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_TIM8_CLK_ENABLE();

    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = STEP_TIM_PSC;
    htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim8.Init.Period            = 65535;
    htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.BreakAFMode      = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.Break2AFMode     = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
        Error_Handler();
}

/* TIM15 — right motor PWM on PB15, CH2, AF14 */
static void MX_TIM15_Init(void)
{
    TIM_ClockConfigTypeDef         sClockSourceConfig  = {0};
    TIM_MasterConfigTypeDef        sMasterConfig        = {0};
    TIM_OC_InitTypeDef             sConfigOC            = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    GPIO_InitTypeDef               GPIO_InitStruct      = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin       = GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_TIM15_CLK_ENABLE();

    htim15.Instance               = TIM15;
    htim15.Init.Prescaler         = STEP_TIM_PSC;
    htim15.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim15.Init.Period            = 65535;
    htim15.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim15.Init.RepetitionCounter = 0;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim15) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
        Error_Handler();
}

/* =========================================================================
 * GPIO INITIALISATION
 *
 *   Outputs:
 *     PB4  DIR_L      stepper left  direction
 *     PB5  EN_L       stepper left  enable (active LOW)
 *     PB10 DIR_R      stepper right direction
 *     PC3  EN_R       stepper right enable (active LOW)
 *
 *   Inputs (EXTI):
 *     PA0  Hall #1    pull-up, rising+falling, EXTI0_IRQn
 *     PA1  Hall #2    pull-up, rising+falling, EXTI1_IRQn
 *
 *   PUL pins (PB6, PB15) are configured as timer alternate functions
 *   inside MX_TIM8_Init / MX_TIM15_Init — not here.
 * ========================================================================= */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Pre-drive stepper DIR/EN pins LOW to avoid a spurious enable pulse
     * as we transition the pin from analog (reset) to push-pull output. */
    HAL_GPIO_WritePin(GPIOB, DIR_L_Pin | EN_L_Pin | DIR_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, EN_R_Pin, GPIO_PIN_RESET);

    /* PB4=DIR_L, PB5=EN_L, PB10=DIR_R */
    GPIO_InitStruct.Pin   = DIR_L_Pin | EN_L_Pin | DIR_R_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PC3=EN_R */
    GPIO_InitStruct.Pin = EN_R_Pin;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PA0=Hall #1, PA1=Hall #2 — pull-up, EXTI rising+falling */
    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
