/**
 ******************************************************************************
 * @file    main.c
 * @brief   Dual closed-loop stepper control — Solar Boat project
 *          Generated against: solarboat_output.ioc (STM32G474RETx, NUCLEO-G474RE)
 *
 * Hardware:
 *   MCU    : STM32G474RET6 @ 170 MHz (24 MHz HSE, PLLM=6, PLLN=85, PLLR=2)
 *   Motors : 2x P-Series IP65 Nema 34, 8.5 Nm, 1000PPR encoder (4000 CPR x4)
 *   Drivers: 2x DM860I  —  DIP-switched to 3200 microsteps/rev (1/16)
 *
 * ── Verified pin map from IOC ────────────────────────────────────────────────
 *   PB6   PUL_L   Left  motor pulse  (TIM8  CH1, AF5)
 *   PB4   DIR_L   Left  motor direction (GPIO output)
 *   PB5   EN_L    Left  motor enable   (GPIO output, active LOW to driver)
 *
 *   PB15  PUL_R   Right motor pulse  (TIM15 CH2, AF14)   ← PB15, NOT PB3
 *   PB10  DIR_R   Right motor direction (GPIO output)
 *   PC3   EN_R    Right motor enable   (GPIO output, active LOW to driver)
 *
 *   TIM2  (internal) — free-running microsecond timebase
 *
 * ── Other peripherals present in IOC (not used here, do not conflict) ───────
 *   PA0   EXTI0  rising/falling interrupt (pull-up)
 *   PA1   EXTI1  rising/falling interrupt (pull-up)
 *   PA10  Zero_Steering  GPIO input (pull-down)
 *   PA11  FDCAN1_RX
 *   PA12  FDCAN1_TX
 *   PA2   LPUART1_TX
 *   PA3   LPUART1_RX
 *   PA4   DAC1_OUT1
 *   PA5   DAC1_OUT2
 *   PA8   I2C2_SDA
 *   PA9   I2C2_SCL
 *   PC1   ADC1_IN7
 *   PC6   I2C4_SCL
 *   PC7   I2C4_SDA
 *   PC8   I2C3_SCL
 *   PC9   I2C3_SDA
 *   PC10  Y1_AM  GPIO input
 *   PC11  Y3_AM  GPIO input
 *   PC12  Y2_AM  GPIO input
 *   PD2   Y4_AM  GPIO input
 *
 * ── DM860I DIP switch guide for 3200 microsteps/rev ─────────────────────────
 *   Microstep bank  (SW5-SW8):  ON  OFF ON  OFF  = 3200 p/rev (1/16)
 *   Current bank    (SW1-SW4):  set to match motor rated current (~6A peak)
 *                               ON  ON  OFF OFF  = 6.1A peak on DM860I
 *
 * ── WHY FILE 2 WAS FASTER — key differences absorbed into this file ─────────
 *   1. Pulse RATE not period: timer ARR = (1MHz / rate_pps) - 1
 *      File 1 used pd_us half-period which gave ~800 p/s at default.
 *      This file uses rate_pps directly — 60000 p/s max → ~1125 RPM motor.
 *   2. Asymmetric ramping: separate slew rates for accel vs decel.
 *      Prevents lost steps on fast acceleration and overshoot on stop.
 *   3. MOE never cleared during normal stop: compare set to 0 instead.
 *      Avoids the BDTR re-enable delay that caused output glitches.
 *   4. Non-blocking enable/direction settle using HAL_GetTick() gates.
 *      No blocking delay_us() calls in the control path.
 *
 * ── Pulse / speed arithmetic ─────────────────────────────────────────────────
 *   3200 pulses = 1 full motor revolution (1/16 microstep, DIP confirmed)
 *   1 pulse     = 0.1125°
 *   ARR         = (1 000 000 / rate_pps) - 1
 *   CCR         = ARR / 2  (50% duty, DM860I needs ≥2.5 µs pulse width)
 *
 *   rate_pps → RPM:  RPM = (rate_pps / 3200) * 60
 *   RPM → rate_pps:  rate_pps = (RPM * 3200) / 60
 *
 *   Examples (3200 steps/rev):
 *     200   p/s →   3.75 RPM  (STEP_RATE_MIN floor)
 *     3200  p/s →  60   RPM
 *     10000 p/s → 187.5 RPM
 *     60000 p/s →1125   RPM  (STEP_RATE_MAX ceiling)
 ******************************************************************************
 */

#include "main.h"
#include <stdio.h>
#include <math.h>

/* ===========================================================================
 * PERIPHERAL HANDLES
 * One handle per timer. HAL uses these structs to track register state.
 *   htim2  = microsecond timebase (TIM2 is 32-bit, never overflows in practice)
 *   htim8  = left  motor PWM output (advanced timer, needs BDTR MOE)
 *   htim15 = right motor PWM output (advanced timer, needs BDTR MOE)
 * =========================================================================== */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
COM_InitTypeDef   BspCOMInit;

/* ===========================================================================
 * TIMER CLOCK
 * System clock = 170 MHz (confirmed in IOC: RCC.AHBFreq_Value=170000000)
 * Prescaler 169  →  timer tick = 170 MHz / (169+1) = 1 MHz = 1 tick per µs
 *
 * STEP_TIM_CLK_HZ is used as a float for ARR calculation:
 *   ARR = (STEP_TIM_CLK_HZ / rate_pps) - 1
 * This gives exact integer ticks per pulse period.
 * =========================================================================== */
#define STEP_TIM_PSC       169U
#define STEP_TIM_CLK_HZ    1000000.0f     /* timer ticks/sec after prescaler  */

/* ===========================================================================
 * STEPS PER REVOLUTION
 * Must match DM860I DIP switch setting exactly.
 * 3200 = 1/16 microstepping.
 * Changing this line is the only edit needed if you re-DIP-switch the driver.
 * =========================================================================== */
#define STEPS_PER_REV      3200UL

/* ===========================================================================
 * PULSE RATE LIMITS (pulses per second)
 *
 * These are frequency-domain limits — the number of step pulses per second.
 * The timer ARR is computed from these: ARR = (1 000 000 / rate_pps) - 1
 *
 * STEP_RATE_MAX = 60000 p/s  →  60000/3200*60 = 1125 RPM motor
 *                              Do not exceed without oscilloscope verification.
 * STEP_RATE_MIN = 200   p/s  →  200/3200*60   = 3.75 RPM
 *                              Below this, torque ripple becomes audible.
 *
 * RPM_TO_PPS macro converts RPM to pulse rate for readable call sites:
 *   stepper_run(&stepper_left, RPM_TO_PPS(30));  // 30 RPM = 1600 p/s
 * =========================================================================== */
#define STEP_RATE_MAX      60000.0f       /* p/s — absolute ceiling           */
#define STEP_RATE_MIN      200.0f         /* p/s — floor, below = stop        */

#define RPM_TO_PPS(rpm)    ((float)(rpm) * (float)STEPS_PER_REV / 60.0f)
#define PPS_TO_RPM(pps)    ((float)(pps) * 60.0f / (float)STEPS_PER_REV)

/* ===========================================================================
 * RAMP PARAMETERS
 * Asymmetric slew rates applied once per RATE_STEPPER_MS tick.
 *
 * RAMP_UP_SLEW   : pulse/s added    per control tick (acceleration)
 * RAMP_DOWN_SLEW : pulse/s removed  per control tick (deceleration)
 *
 * Making decel faster than accel prevents overshoot when stopping.
 * Increase RAMP_UP_SLEW for snappier response; decrease for smoother starts.
 *
 * At 2 ms tick rate:
 *   RAMP_UP_SLEW=800   → 0→60000 p/s in ~150 ticks = ~300 ms
 *   RAMP_DOWN_SLEW=2000 → 60000→0 p/s in ~30 ticks  = ~60 ms
 * =========================================================================== */
#define RAMP_UP_SLEW          800.0f     /* p/s per tick — acceleration rate  */
#define RAMP_DOWN_SLEW        2000.0f    /* p/s per tick — deceleration rate  */
#define DIR_CHANGE_RAMP_HZ    2000.0f    /* p/s threshold before DIR flip     */

/* ===========================================================================
 * SETTLE TIMES (milliseconds)
 * DM860I ENA→pulse settle: driver datasheet requires 5 ms minimum.
 *   STEPPER_ENABLE_SETTLE_MS = 5 gives headroom.
 * DM860I DIR→pulse settle: datasheet requires 5 µs minimum.
 *   DIR_CHANGE_SETTLE_MS = 2 ms is >>spec but avoids any race.
 * Both are enforced via HAL_GetTick() — non-blocking, no delay_us().
 * =========================================================================== */
#define STEPPER_ENABLE_SETTLE_MS   5U    /* ms after ENA before first pulse   */
#define DIR_CHANGE_SETTLE_MS       2U    /* ms after DIR change before pulse  */

/* ===========================================================================
 * STEPPER STATE STRUCT
 * All runtime state for one motor+driver pair in a single struct.
 * Using a struct (rather than parallel arrays of globals) means:
 *   - adding a third motor = add one stepper_t instance
 *   - all control functions take stepper_t* so they work on any motor
 *
 * Fields:
 *   htim              HAL timer handle for this motor's PUL output
 *   channel           TIM_CHANNEL_x for the PWM output
 *   dir_port/pin      GPIO for DIR signal
 *   en_port/pin       GPIO for ENA signal  (active LOW on DM860I)
 *   invert_dir        1 = physically mirrored motor; flips DIR pin logic
 *   enabled           1 = driver is currently energised
 *   enable_wait       1 = waiting for enable settle time to expire
 *   enable_tick       HAL_GetTick() value when enable was asserted
 *   dir               current direction: 1=positive, 0=negative
 *   dir_chg           1 = waiting for direction-change settle to expire
 *   dir_tick          HAL_GetTick() value when direction changed
 *   speed             current live pulse rate (p/s), magnitude only
 *   rate_out          signed output rate (+positive, -negative) for telemetry
 * =========================================================================== */
typedef struct {
    /* Hardware */
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
    GPIO_TypeDef      *en_port;
    uint16_t           en_pin;
    uint8_t            invert_dir;

    /* Runtime state — zeroed in stepper_init(), do not write directly */
    uint8_t  enabled;
    uint8_t  enable_wait;
    uint32_t enable_tick;
    uint8_t  dir;            /* 1 = positive direction, 0 = negative          */
    uint8_t  dir_chg;
    uint32_t dir_tick;
    float    speed;          /* current live pulse rate p/s (magnitude)        */
    float    rate_out;       /* signed output p/s — use for logging/CAN        */
} stepper_t;

/* ===========================================================================
 * MOTOR INSTANCES
 * Pin assignments from IOC:
 *   PB6 =PUL_L  PB4 =DIR_L  PB5 =EN_L   → left  motor, TIM8  CH1
 *   PB15=PUL_R  PB10=DIR_R  PC3 =EN_R   → right motor, TIM15 CH2
 *
 * invert_dir=1 on right motor: motors face each other on a differential
 * drive, so the same logical "positive" command rotates them symmetrically.
 *
 * Runtime fields (enabled, speed, etc.) are zeroed by stepper_init().
 * =========================================================================== */
stepper_t stepper_left = {
    .htim       = &htim8,
    .channel    = TIM_CHANNEL_1,
    .dir_port   = GPIOB,
    .dir_pin    = DIR_L_Pin,      /* PB4  */
    .en_port    = GPIOB,
    .en_pin     = EN_L_Pin,       /* PB5  */
    .invert_dir = 0
};

stepper_t stepper_right = {
    .htim       = &htim15,
    .channel    = TIM_CHANNEL_2,
    .dir_port   = GPIOB,
    .dir_pin    = DIR_R_Pin,      /* PB10 */
    .en_port    = GPIOC,
    .en_pin     = EN_R_Pin,       /* PC3  */
    .invert_dir = 1
};

/* ===========================================================================
 * FORWARD DECLARATIONS
 * =========================================================================== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
void Error_Handler(void);

/* ===========================================================================
 * LOW-LEVEL: WRITE PULSE RATE TO TIMER
 * The only function that touches ARR/CCR/EGR.
 * All higher-level functions go through here.
 *
 * rate_pps < STEP_RATE_MIN: silences output by setting CCR=0.
 *   MOE is NOT cleared — avoids the re-enable glitch that affected File 1.
 *   The pin simply stays LOW because CCR=0 means counter never reaches compare.
 *
 * rate_pps ≥ STEP_RATE_MIN:
 *   ARR = (STEP_TIM_CLK_HZ / rate_pps) - 1
 *   CCR = ARR / 2  →  50% duty cycle
 *   EGR UG bit forces immediate register load (no wait for next overflow).
 * =========================================================================== */
static void stepper_write_rate(stepper_t *s, float rate_pps)
{
    if (rate_pps < STEP_RATE_MIN)
    {
        /* Silence: CCR=0 keeps pin LOW without disabling the timer output */
        __HAL_TIM_SET_COMPARE(s->htim, s->channel, 0U);
        s->htim->Instance->EGR = TIM_EGR_UG;
        return;
    }

    /* Clamp to hardware maximum */
    if (rate_pps > STEP_RATE_MAX) rate_pps = STEP_RATE_MAX;

    /* ARR = ticks per full pulse period */
    uint32_t arr = (uint32_t)((STEP_TIM_CLK_HZ / rate_pps) - 1.0f);

    /* Guard against out-of-range values */
    if (arr < 10U)    arr = 10U;     /* min ~100 kHz — above DM860I max anyway */
    if (arr > 65535U) arr = 65535U;  /* 16-bit timer limit                      */

    __HAL_TIM_SET_AUTORELOAD(s->htim, arr);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, arr / 2U);  /* 50% duty        */
    s->htim->Instance->EGR = TIM_EGR_UG;                   /* immediate update */
}

/* ===========================================================================
 * LOW-LEVEL: WRITE DIRECTION PIN
 * Applies invert_dir transparently so all callers use logical direction.
 * positive=1 → "forward" for this motor (accounting for physical mounting).
 * positive=0 → "reverse".
 * =========================================================================== */
static void stepper_write_dir(stepper_t *s, uint8_t positive)
{
    uint8_t actual = s->invert_dir ? (uint8_t)(!positive) : positive;
    HAL_GPIO_WritePin(s->dir_port, s->dir_pin,
                      actual ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ===========================================================================
 * LOW-LEVEL: WRITE ENABLE PIN
 * DM860I ENA is active LOW:
 *   enable=1  →  GPIO LOW  →  driver on, coils energised, shaft locked
 *   enable=0  →  GPIO HIGH →  driver off, shaft free
 * =========================================================================== */
static void stepper_write_enable(stepper_t *s, uint8_t enable)
{
    HAL_GPIO_WritePin(s->en_port, s->en_pin,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* ===========================================================================
 * API: stepper_init()
 * Must be called once per motor after peripheral init and before any motion.
 * Zeros all runtime state, arms the timer, and leaves the driver disabled.
 * Safe to call again at any time to perform a soft reset of one motor.
 * =========================================================================== */
void stepper_init(stepper_t *s)
{
    /* Silence output and disable driver */
    stepper_write_rate(s, 0.0f);
    stepper_write_enable(s, 0);

    /* Zero all runtime state */
    s->enabled      = 0U;
    s->enable_wait  = 0U;
    s->enable_tick  = 0U;
    s->dir          = 1U;    /* default: positive direction */
    s->dir_chg      = 0U;
    s->dir_tick     = 0U;
    s->speed        = 0.0f;
    s->rate_out     = 0.0f;

    /* Enable advanced-timer output stage (MOE) — left set permanently.
     * We never clear MOE after this; CCR=0 is used to silence instead.
     * This avoids the re-enable glitch that clearing MOE causes. */
    s->htim->Instance->BDTR |= TIM_BDTR_MOE;
}

/* ===========================================================================
 * API: stepper_hard_stop()
 * Immediately silences the pulse output and disables the driver.
 * Use for faults, E-stop, or end-of-sequence shutdown.
 * After a hard stop, call stepper_init() or stepper_run() to restart.
 * =========================================================================== */
void stepper_hard_stop(stepper_t *s)
{
    stepper_write_rate(s, 0.0f);
    stepper_write_enable(s, 0);

    s->enabled      = 0U;
    s->enable_wait  = 0U;
    s->speed        = 0.0f;
    s->rate_out     = 0.0f;
}

/* ===========================================================================
 * API: stepper_run()
 * The primary per-tick control function. Call this every RATE_STEPPER_MS ms.
 *
 * cmd_pps: signed target pulse rate in pulses/second.
 *   Positive = forward (dir=1), negative = reverse (dir=0).
 *   Magnitude is clamped to [0, STEP_RATE_MAX].
 *   Use RPM_TO_PPS(n) for readable call sites.
 *
 * What it does each call:
 *   1. Non-blocking enable settle gate — outputs 0 until driver is ready
 *   2. Non-blocking direction-change settle gate — outputs 0 during DIR setup
 *   3. Direction reversal: ramps down to DIR_CHANGE_RAMP_HZ then flips DIR
 *   4. Asymmetric ramp: accelerates at RAMP_UP_SLEW, decelerates at RAMP_DOWN_SLEW
 *   5. Writes the new rate to the timer via stepper_write_rate()
 *   6. Updates s->rate_out (signed) for telemetry / CAN / printf
 *
 * The function is designed to be called repeatedly from a timed loop.
 * It is fully non-blocking — never calls HAL_Delay() or delay_us().
 * =========================================================================== */
void stepper_run(stepper_t *s, float cmd_pps)
{
    uint32_t now    = HAL_GetTick();
    uint8_t  want_dir = (cmd_pps >= 0.0f) ? 1U : 0U;
    float    target   = fabsf(cmd_pps);

    if (target > STEP_RATE_MAX) target = STEP_RATE_MAX;

    /* -----------------------------------------------------------------------
     * Gate 1: Enable settle
     * The DM860I needs STEPPER_ENABLE_SETTLE_MS after ENA goes active before
     * the first pulse. While waiting, output 0 and return immediately.
     * ----------------------------------------------------------------------- */
    if (s->enable_wait)
    {
        if ((now - s->enable_tick) < STEPPER_ENABLE_SETTLE_MS)
        {
            stepper_write_rate(s, 0.0f);
            s->rate_out = 0.0f;
            return;
        }
        /* Settle time expired — driver is now ready */
        s->enable_wait = 0U;
        s->enabled     = 1U;
        if (s->speed < STEP_RATE_MIN) s->speed = STEP_RATE_MIN;
    }

    /* -----------------------------------------------------------------------
     * Gate 2: Direction-change settle
     * After flipping the DIR pin, hold output at 0 for DIR_CHANGE_SETTLE_MS.
     * ----------------------------------------------------------------------- */
    if (s->dir_chg)
    {
        if ((now - s->dir_tick) < DIR_CHANGE_SETTLE_MS)
        {
            stepper_write_rate(s, 0.0f);
            s->rate_out = 0.0f;
            return;
        }
        /* Settle expired — resume at minimum speed in new direction */
        s->dir_chg = 0U;
        s->speed   = STEP_RATE_MIN;
    }

    /* -----------------------------------------------------------------------
     * Direction reversal handling
     * If target direction differs from current and we want to move:
     *   - Ramp down to DIR_CHANGE_RAMP_HZ first
     *   - Once below threshold, cut output, flip DIR pin, start settle timer
     * This prevents the driver from seeing a mid-motion DIR change which
     * causes missed steps and fault conditions on the DM860I.
     * ----------------------------------------------------------------------- */
    if ((target >= STEP_RATE_MIN) && (want_dir != s->dir))
    {
        if (s->speed > DIR_CHANGE_RAMP_HZ)
        {
            /* Still too fast — decelerate toward threshold */
            s->speed -= RAMP_DOWN_SLEW;
            if (s->speed < DIR_CHANGE_RAMP_HZ) s->speed = DIR_CHANGE_RAMP_HZ;

            /* Ensure driver is enabled while decelerating */
            if (!s->enabled && !s->enable_wait)
            {
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

        /* Below threshold — safe to flip direction */
        stepper_write_rate(s, 0.0f);
        s->dir = want_dir;
        stepper_write_dir(s, s->dir);
        s->dir_chg  = 1U;
        s->dir_tick = now;
        s->speed    = 0.0f;
        s->rate_out = 0.0f;
        return;
    }

    /* -----------------------------------------------------------------------
     * Normal ramp: accelerate, decelerate, or stop
     * Three cases:
     *   target < STEP_RATE_MIN  → ramp down to 0, then hard stop
     *   speed  < target         → ramp up   at RAMP_UP_SLEW
     *   speed  > target         → ramp down at RAMP_DOWN_SLEW
     * ----------------------------------------------------------------------- */
    if (target < STEP_RATE_MIN)
    {
        /* Commanded stop — decelerate then disable driver */
        if (s->speed > 0.0f)
        {
            s->speed -= RAMP_DOWN_SLEW;
            if (s->speed <= STEP_RATE_MIN)
            {
                stepper_hard_stop(s);
                return;
            }
        }
        else
        {
            stepper_hard_stop(s);
            return;
        }
    }
    else if (s->speed < target)
    {
        /* Accelerate */
        s->speed += RAMP_UP_SLEW;
        if (s->speed > target) s->speed = target;
    }
    else if (s->speed > target)
    {
        /* Decelerate */
        s->speed -= RAMP_DOWN_SLEW;
        if (s->speed < target) s->speed = target;
        if (s->speed < STEP_RATE_MIN)
        {
            stepper_hard_stop(s);
            return;
        }
    }

    /* -----------------------------------------------------------------------
     * Enable driver if not already enabled
     * This handles the case where stepper_run() is called with a non-zero
     * target on a motor that was previously stopped (driver disabled).
     * Insert the enable settle gate and return — motion begins next tick.
     * ----------------------------------------------------------------------- */
    if (!s->enabled && !s->enable_wait)
    {
        stepper_write_dir(s, s->dir);
        stepper_write_enable(s, 1);
        s->enable_wait = 1U;
        s->enable_tick = now;
        s->rate_out    = 0.0f;
        return;
    }

    /* -----------------------------------------------------------------------
     * Output: write computed speed to timer and update signed rate_out
     * rate_out sign convention: positive = forward, negative = reverse.
     * Use s->rate_out anywhere you need to know what the motor is doing.
     * ----------------------------------------------------------------------- */
    stepper_write_rate(s, s->speed);
    s->rate_out = s->dir ? s->speed : -s->speed;
}

/* ===========================================================================
 * API: stepper_run_both()
 * Convenience wrapper: calls stepper_run() on both motors with the same
 * signed pulse rate. Both motors ramp independently so if one was already
 * moving, its ramp state is preserved.
 *
 * cmd_pps > 0  → both motors forward
 * cmd_pps < 0  → both motors reverse
 * cmd_pps = 0  → both motors ramp to stop
 * =========================================================================== */
void stepper_run_both(float cmd_pps)
{
    stepper_run(&stepper_left,  cmd_pps);
    stepper_run(&stepper_right, cmd_pps);
}

/* ===========================================================================
 * API: stepper_stop_both()
 * Immediately kills output and disables both drivers.
 * Use for E-stop or end-of-sequence. Not ramped — instant.
 * =========================================================================== */
void stepper_stop_both(void)
{
    stepper_hard_stop(&stepper_left);
    stepper_hard_stop(&stepper_right);
}

/* ===========================================================================
 * MAIN
 * Init order: HAL → clock → GPIO → timers → TIM2 start → stepper_init →
 *             peripherals → motion loop.
 *
 * The motion loop demonstrates the correct usage pattern:
 *   stepper_run() must be called on a regular tick (every RATE_STEPPER_MS ms).
 *   HAL_Delay() is used here for simplicity; replace with a timer-based
 *   scheduler (SysTick, RTOS task, or TIM interrupt) in production.
 * =========================================================================== */

/* How often stepper_run() is called in the motion loop (milliseconds).
 * Must match the assumption baked into RAMP_UP_SLEW and RAMP_DOWN_SLEW.
 * 2 ms = 500 Hz update rate — matches File 2's RATE_STEERING_CTRL_MS.     */
#define RATE_STEPPER_MS   2U

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM8_Init();
    MX_TIM15_Init();

    /* Start TIM2 free-running microsecond counter */
    HAL_TIM_Base_Start(&htim2);

    /* Start PWM timers — outputs stay silent (CCR=0) until stepper_run() fires */
    HAL_TIM_PWM_Start(&htim8,  TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

    /* Initialise both motors — zeros state, sets MOE, leaves drivers disabled */
    stepper_init(&stepper_left);
    stepper_init(&stepper_right);

    /* Optional UART debug via NUCLEO VCP */
    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    BSP_COM_Init(COM1, &BspCOMInit);

    printf("Solar Boat stepper ready.\r\n");
    printf("STEPS_PER_REV=%lu  STEP_RATE_MAX=%.0f p/s = %.1f RPM\r\n",
           STEPS_PER_REV, STEP_RATE_MAX, PPS_TO_RPM(STEP_RATE_MAX));

    /* =========================================================
     * MOTION SEQUENCE
     *
     * stepper_run() / stepper_run_both() must be called every
     * RATE_STEPPER_MS milliseconds. The ramp state machine inside
     * tracks the acceleration profile across calls.
     *
     * Calling it once and then blocking with HAL_Delay() will NOT
     * produce a smooth ramp — it will jump to the target speed.
     * The while() loops below are the correct pattern.
     *
     * API summary:
     *   stepper_run(&s, cmd_pps)     tick one motor toward cmd_pps
     *   stepper_run_both(cmd_pps)    tick both motors toward cmd_pps
     *   stepper_hard_stop(&s)        instant stop, driver off
     *   stepper_stop_both()          instant stop both, drivers off
     *   RPM_TO_PPS(n)                convert RPM to p/s
     *   PPS_TO_RPM(pps)              convert p/s to RPM
     *   s.rate_out                   signed live rate for logging
     *
     * RPM reference (3200 steps/rev):
     *   RPM_TO_PPS(10)  =  533 p/s
     *   RPM_TO_PPS(30)  = 1600 p/s
     *   RPM_TO_PPS(60)  = 3200 p/s
     *   RPM_TO_PPS(100) = 5333 p/s
     *   RPM_TO_PPS(300) = 16000 p/s
     *   RPM_TO_PPS(600) = 32000 p/s
     * ========================================================= */

    uint32_t phase_start = HAL_GetTick();
    uint32_t last_tick   = HAL_GetTick();
    uint32_t last_print  = HAL_GetTick();
    uint8_t  phase       = 0;

    /* Target for current phase — change each phase transition below */
    float target_pps = 0.0f;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* --- Stepper tick: must run every RATE_STEPPER_MS ms --- */
        if (now - last_tick >= RATE_STEPPER_MS)
        {
            stepper_run_both(target_pps);
            last_tick = now;
        }

        /* --- Motion phase sequencer --- */
        switch (phase)
        {
            case 0:   /* Ramp up to 30 RPM forward, hold 3 s */
                target_pps = RPM_TO_PPS(400);
                if (now - phase_start >= 3000U) { phase = 1; phase_start = now; }
                break;

            case 1:   /* Ramp up to 100 RPM forward, hold 3 s */
                target_pps = RPM_TO_PPS(-400);
                if (now - phase_start >= 3000U) { phase = 2; phase_start = now; }
                break;

            case 2:   /* Ramp down to 0, stop 500 ms */
                target_pps = 0.0f;
                if (now - phase_start >= 500U)  { phase = 3; phase_start = now; }
                break;

            case 3:   /* Reverse at 30 RPM for 3 s */
                target_pps = RPM_TO_PPS(0);
                if (now - phase_start >= 3000U) { phase = 4; phase_start = now; }
                break;

            case 4:   /* Ramp down, stop, disable */
                target_pps = 0.0f;
                /* Wait until both motors have actually stopped */
                if (stepper_left.speed  < STEP_RATE_MIN &&
                    stepper_right.speed < STEP_RATE_MIN)
                {
                    stepper_stop_both();
                    printf("Sequence complete. L=%.1f R=%.1f p/s\r\n",
                           stepper_left.rate_out, stepper_right.rate_out);
                    phase = 5;
                }
                break;

            case 5:   /* Idle */
            default:
                break;
        }

        /* --- Debug print @ 10 Hz --- */
        if (now - last_print >= 100U)
        {
            printf("PH=%u L=%.0fp/s(%.1fRPM) R=%.0fp/s(%.1fRPM)\r\n",
                   phase,
                   stepper_left.rate_out,  PPS_TO_RPM(fabsf(stepper_left.rate_out)),
                   stepper_right.rate_out, PPS_TO_RPM(fabsf(stepper_right.rate_out)));
            last_print = now;
        }
    }

    /* =========================================================
     * END MOTION SEQUENCE
     * ========================================================= */
}

/* ===========================================================================
 * SYSTEM CLOCK CONFIGURATION
 * Matches IOC exactly:
 *   HSE = 24 MHz  (RCC.HSE_VALUE=24000000)
 *   PLLM = DIV6   →  VCO input  =  4 MHz
 *   PLLN = 85     →  VCO output = 340 MHz
 *   PLLR = DIV2   →  SYSCLK    = 170 MHz
 *   All bus dividers = 1  →  HCLK = APB1 = APB2 = 170 MHz
 * =========================================================================== */
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

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

/* ===========================================================================
 * GPIO INITIALISATION
 * Configures DIR and ENA output pins only.
 * PUL pins (PB6, PB15) are configured as timer AF inside MX_TIM8/15_Init.
 *
 * Pin map:
 *   PB4  DIR_L   output, push-pull, no pull, low speed
 *   PB5  EN_L    output, push-pull, no pull, low speed
 *   PB10 DIR_R   output, push-pull, no pull, low speed
 *   PC3  EN_R    output, push-pull, no pull, low speed
 *
 * All outputs pre-driven LOW before configuring the pin mode to avoid
 * spurious enable pulses during the GPIO_Init call.
 * =========================================================================== */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Pre-drive LOW (ENA high = disabled on DM860I; driven LOW here first
     * so the transition to output mode doesn't glitch the driver) */
    HAL_GPIO_WritePin(GPIOB, DIR_L_Pin | EN_L_Pin | DIR_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, EN_R_Pin, GPIO_PIN_RESET);

    /* GPIOB: PB4=DIR_L, PB5=EN_L, PB10=DIR_R */
    GPIO_InitStruct.Pin   = DIR_L_Pin | EN_L_Pin | DIR_R_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* DIR/ENA edges don't need to be fast */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* GPIOC: PC3=EN_R */
    GPIO_InitStruct.Pin = EN_R_Pin;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* ===========================================================================
 * TIM2 INITIALISATION — FREE-RUNNING MICROSECOND COUNTER
 * 32-bit timer, prescaler 169 → 1 tick = 1 µs.
 * Used by the Hall-sensor ISR and any future delay_us() needs.
 * Period = 0xFFFFFFFF → wraps after ~4295 s, negligible in practice.
 * =========================================================================== */
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
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

/* ===========================================================================
 * TIM8 INITIALISATION — LEFT MOTOR PWM (PB6, CH1, AF5)
 * TIM8 is an advanced-control timer.
 * MOE (BDTR.MOE) is permanently set by stepper_init() after this function.
 * GPIO alternate function configured here; no separate MspInit needed.
 * =========================================================================== */
static void MX_TIM8_Init(void)
{
    TIM_ClockConfigTypeDef         sClockSourceConfig  = {0};
    TIM_MasterConfigTypeDef        sMasterConfig        = {0};
    TIM_OC_InitTypeDef             sConfigOC            = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    GPIO_InitTypeDef               GPIO_InitStruct      = {0};

    /* PB6 → TIM8_CH1 (AF5 on STM32G474) */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;  /* fast edges for clean pulses */
    GPIO_InitStruct.Alternate = GPIO_AF5_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_TIM8_CLK_ENABLE();

    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = STEP_TIM_PSC;   /* 169 → 1 MHz tick */
    htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim8.Init.Period            = 65535;           /* overwritten by stepper_write_rate */
    htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim8) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) Error_Handler();

    /* PWM mode 1: output HIGH while CNT < CCR */
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;                    /* CCR set by stepper_write_rate */
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    /* BDTR: no break, no dead-time. MOE set permanently by stepper_init(). */
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
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();
}

/* ===========================================================================
 * TIM15 INITIALISATION — RIGHT MOTOR PWM (PB15, CH2, AF14)
 * Structure mirrors TIM8 exactly; instance, pin, and AF number differ.
 * IOC confirms: PB15 = PUL_R (S_TIM15_CH2). NOT PB3.
 * =========================================================================== */
static void MX_TIM15_Init(void)
{
    TIM_ClockConfigTypeDef         sClockSourceConfig  = {0};
    TIM_MasterConfigTypeDef        sMasterConfig        = {0};
    TIM_OC_InitTypeDef             sConfigOC            = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    GPIO_InitTypeDef               GPIO_InitStruct      = {0};

    /* PB15 → TIM15_CH2 (AF14 on STM32G474) */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin       = GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM15;
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
    if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();
}

/* ===========================================================================
 * ERROR HANDLER
 * Disables interrupts and halts. Attach debugger to find the call site.
 * Production: log fault via CoreDebug registers and trigger IWDG reset.
 * =========================================================================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
