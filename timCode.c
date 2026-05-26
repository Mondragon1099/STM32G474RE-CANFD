/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   Dual closed-loop stepper control - Solar Boat project
 *
 *   This file MERGES two earlier versions:
 *     - File 1 contributed the position-control API:
 *           stepper_set_target_deg() / stepper_move_deg() /
 *           stepper_get_output_deg() / stepper_at_target() /
 *           stepper_position_tick()
 *       which drives a clamped P controller toward a target output-shaft
 *       angle, producing a natural trapezoidal velocity profile.
 *     - File 2 contributed the CubeMX USER CODE block layout, the TIM3
 *       quadrature encoder readback, and the rest of the peripheral
 *       wiring (I2C3, ADC1/5, DAC1, FDCAN1).
 *
 *   The position controller is OPEN-LOOP on encoder feedback - it tracks
 *   commanded position only.  The encoder reading is shown in the debug
 *   printf next to the commanded angle so you can compare them by eye.
 *   Real feedback closure can be layered on later once you trust the
 *   measurement.
 *
 *   Demo loop: cycles through a hardcoded list of target angles, dwelling
 *   1 s at each.  Edit demo_targets[] in main() to test other angles.
 *
 * Notes for CubeMX regeneration:
 *   - Custom includes  -> USER CODE BEGIN Includes
 *   - Custom typedefs  -> USER CODE BEGIN PTD
 *   - Custom macros    -> USER CODE BEGIN PD / PM
 *   - Custom globals   -> USER CODE BEGIN PV
 *   - Custom helpers   -> USER CODE BEGIN 0
 *   - Custom startup   -> USER CODE BEGIN 2
 *   - Custom main loop -> USER CODE BEGIN WHILE / 3
 *   - The TIM8/TIM15 Prescaler=169 edits in the auto-generated init bodies
 *     are NOT in USER CODE blocks; if you regenerate from .ioc, set the
 *     prescaler to 169 in the CubeMX TIM config or move the override into
 *     USER CODE BEGIN TIMx_Init 2 (write PSC then pulse EGR.UG).
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    /* Hardware */
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
    GPIO_TypeDef      *en_port;
    uint16_t           en_pin;
    uint8_t            invert_dir;

    /* Runtime state */
    uint8_t  enabled;
    uint8_t  enable_wait;
    uint32_t enable_tick;
    uint8_t  dir;
    uint8_t  dir_chg;
    uint32_t dir_tick;
    float    speed;
    float    rate_out;
} stepper_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- Timer / pulse-rate base settings --- */
#define STEP_TIM_PSC               169U
#define STEP_TIM_CLK_HZ            1000000.0f
#define STEPS_PER_REV              3200UL

#define STEP_RATE_MAX              60000.0f
#define STEP_RATE_MIN              200.0f

#define RAMP_UP_SLEW               800.0f
#define RAMP_DOWN_SLEW             2000.0f
#define DIR_CHANGE_RAMP_HZ         2000.0f

#define STEPPER_ENABLE_SETTLE_MS   5U
#define DIR_CHANGE_SETTLE_MS       2U

#define RATE_STEPPER_MS            2U

/* --- Mechanical / position control --- */
/* GEAR_RATIO  : motor revolutions per output revolution (20:1 reducer here).
 *               Used by BOTH the position math and the encoder display so
 *               commanded degrees and measured degrees are in the same units.
 * POS_PEAK_PPS: ceiling speed during a goto - error gets clamped here.
 * POS_KP      : P gain (pps per step of error).  Higher = sharper decel into
 *               target, more chance of overshoot.
 * POS_DEADBAND_STEPS: error magnitude that counts as "arrived".
 *               100 steps / STEPS_PER_OUT_DEG ~ 0.56 deg at the output. */
#define GEAR_RATIO                 20.0f
#define STEPS_PER_OUT_DEG          ((float)STEPS_PER_REV * GEAR_RATIO / 360.0f)
#define POS_PEAK_PPS               RPM_TO_PPS(400.0f)
#define POS_KP                     30.0f
#define POS_DEADBAND_STEPS         100.0f

/* --- Encoder readback --- */
/* ENCODER_SIGN flips the sign if the encoder counts opposite to commanded
 * direction.  Set to +1 or -1 once verified on bench. */
#define ENCODER_SIGN               -1.0f
#define ENCODER_COUNTS_PER_REV     4000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RPM_TO_PPS(rpm)    ((float)(rpm) * (float)STEPS_PER_REV / 60.0f)
#define PPS_TO_RPM(pps)    ((float)(pps) * 60.0f / (float)STEPS_PER_REV)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc5;

DAC_HandleTypeDef hdac1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
stepper_t stepper_left = {
    .htim       = &htim8,
    .channel    = TIM_CHANNEL_1,
    .dir_port   = GPIOB,
    .dir_pin    = DIR_L_Pin,
    .en_port    = GPIOB,
    .en_pin     = EN_L_Pin,
    .invert_dir = 0
};

stepper_t stepper_right = {
    .htim       = &htim15,
    .channel    = TIM_CHANNEL_2,
    .dir_port   = GPIOB,
    .dir_pin    = DIR_R_Pin,
    .en_port    = GPIOC,
    .en_pin     = EN_R_Pin,
    .invert_dir = 1
};

/* Loop scheduling */
static uint32_t last_tick   = 0;
static uint32_t last_print  = 0;

/* Encoder zero reference */
static int32_t  encoder_zero_count = 0;

/* Position-control state.  Units = MOTOR steps (not output degrees) to keep
 * arithmetic precise.  Use stepper_set_target_deg() / stepper_get_output_deg()
 * for human-readable I/O. */
static float    pos_target_steps  = 0.0f;
static float    pos_current_steps = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM8_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC5_Init(void);
/* USER CODE BEGIN PFP */
static void stepper_write_rate(stepper_t *s, float rate_pps);
static void stepper_write_dir(stepper_t *s, uint8_t positive);
static void stepper_write_enable(stepper_t *s, uint8_t enable);
void stepper_init(stepper_t *s);
void stepper_hard_stop(stepper_t *s);
void stepper_run(stepper_t *s, float cmd_pps);
void stepper_run_both(float cmd_pps);
void stepper_stop_both(void);

static void  encoder_zero(void);
static float encoder_get_deg(void);

void    stepper_set_target_deg(float output_deg);
void    stepper_move_deg(float output_deg_delta);
float   stepper_get_output_deg(void);
uint8_t stepper_at_target(void);
void    stepper_position_tick(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void stepper_write_rate(stepper_t *s, float rate_pps)
{
    if (rate_pps < STEP_RATE_MIN)
    {
        __HAL_TIM_SET_COMPARE(s->htim, s->channel, 0U);
        s->htim->Instance->EGR = TIM_EGR_UG;
        return;
    }

    if (rate_pps > STEP_RATE_MAX) rate_pps = STEP_RATE_MAX;

    uint32_t arr = (uint32_t)((STEP_TIM_CLK_HZ / rate_pps) - 1.0f);

    if (arr < 10U)    arr = 10U;
    if (arr > 65535U) arr = 65535U;

    __HAL_TIM_SET_AUTORELOAD(s->htim, arr);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, arr / 2U);
    s->htim->Instance->EGR = TIM_EGR_UG;
}

static void stepper_write_dir(stepper_t *s, uint8_t positive)
{
    uint8_t actual = s->invert_dir ? (uint8_t)(!positive) : positive;
    HAL_GPIO_WritePin(s->dir_port, s->dir_pin,
                      actual ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stepper_write_enable(stepper_t *s, uint8_t enable)
{
    HAL_GPIO_WritePin(s->en_port, s->en_pin,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

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

void stepper_run(stepper_t *s, float cmd_pps)
{
    uint32_t now      = HAL_GetTick();
    uint8_t  want_dir = (cmd_pps >= 0.0f) ? 1U : 0U;
    float    target   = fabsf(cmd_pps);

    if (target > STEP_RATE_MAX) target = STEP_RATE_MAX;

    if (s->enable_wait)
    {
        if ((now - s->enable_tick) < STEPPER_ENABLE_SETTLE_MS)
        {
            stepper_write_rate(s, 0.0f);
            s->rate_out = 0.0f;
            return;
        }
        s->enable_wait = 0U;
        s->enabled     = 1U;
        if (s->speed < STEP_RATE_MIN) s->speed = STEP_RATE_MIN;
    }

    if (s->dir_chg)
    {
        if ((now - s->dir_tick) < DIR_CHANGE_SETTLE_MS)
        {
            stepper_write_rate(s, 0.0f);
            s->rate_out = 0.0f;
            return;
        }
        s->dir_chg = 0U;
        s->speed   = STEP_RATE_MIN;
    }

    if ((target >= STEP_RATE_MIN) && (want_dir != s->dir))
    {
        if (s->speed > DIR_CHANGE_RAMP_HZ)
        {
            s->speed -= RAMP_DOWN_SLEW;
            if (s->speed < DIR_CHANGE_RAMP_HZ) s->speed = DIR_CHANGE_RAMP_HZ;

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

        stepper_write_rate(s, 0.0f);
        s->dir = want_dir;
        stepper_write_dir(s, s->dir);
        s->dir_chg  = 1U;
        s->dir_tick = now;
        s->speed    = 0.0f;
        s->rate_out = 0.0f;
        return;
    }

    if (target < STEP_RATE_MIN)
    {
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
        s->speed += RAMP_UP_SLEW;
        if (s->speed > target) s->speed = target;
    }
    else if (s->speed > target)
    {
        s->speed -= RAMP_DOWN_SLEW;
        if (s->speed < target) s->speed = target;
        if (s->speed < STEP_RATE_MIN)
        {
            stepper_hard_stop(s);
            return;
        }
    }

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

/* ---------------------------------------------------------------------------
 * Encoder readback - TIM3 in quadrature mode counts edges on PA6/PA7.
 * encoder_zero() captures the current count as the reference.
 * encoder_get_deg() returns degrees at the OUTPUT shaft (after GEAR_RATIO).
 * --------------------------------------------------------------------------- */
static void encoder_zero(void)
{
    encoder_zero_count = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

static float encoder_get_deg(void)
{
    int32_t now   = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t delta = now - encoder_zero_count;
    return ENCODER_SIGN *
           ((float)delta * 360.0f) /
           ((float)ENCODER_COUNTS_PER_REV * GEAR_RATIO);
}

/* ---------------------------------------------------------------------------
 * Position control API
 *
 *   stepper_set_target_deg(d)   absolute target in OUTPUT degrees
 *   stepper_move_deg(d)         relative shift of target by d output degrees
 *   stepper_get_output_deg()    current COMMANDED output position
 *   stepper_at_target()         1 when commanded position is within deadband
 *   stepper_position_tick()     run one control tick (every RATE_STEPPER_MS)
 *
 * The setpoint can be changed at any time, including from inside an ISR.
 * Float writes aren't atomic on Cortex-M, so in the worst case one tick
 * sees a half-updated target and produces one slightly-off command - harmless.
 *
 * IMPORTANT: this is open-loop on the encoder.  stepper_get_output_deg()
 * tracks what was COMMANDED, not what was MEASURED.  The encoder readback
 * (encoder_get_deg) is independent and is only used for display.
 * --------------------------------------------------------------------------- */
void stepper_set_target_deg(float output_deg)
{
    pos_target_steps = output_deg * STEPS_PER_OUT_DEG;
}

void stepper_move_deg(float output_deg_delta)
{
    pos_target_steps += output_deg_delta * STEPS_PER_OUT_DEG;
}

float stepper_get_output_deg(void)
{
    return pos_current_steps / STEPS_PER_OUT_DEG;
}

uint8_t stepper_at_target(void)
{
    return fabsf(pos_target_steps - pos_current_steps) < POS_DEADBAND_STEPS;
}

/* P controller with peak-speed clamp.  Naturally produces a trapezoidal
 * velocity profile: full speed in the middle, ramp-down as it homes in. */
void stepper_position_tick(void)
{
    float error = pos_target_steps - pos_current_steps;
    float cmd_pps;

    if (fabsf(error) < POS_DEADBAND_STEPS) {
        cmd_pps = 0.0f;
    } else {
        cmd_pps = error * POS_KP;
        if (cmd_pps >  POS_PEAK_PPS) cmd_pps =  POS_PEAK_PPS;
        if (cmd_pps < -POS_PEAK_PPS) cmd_pps = -POS_PEAK_PPS;
    }

    stepper_run_both(cmd_pps);

    /* Integrate the post-ramp executed rate.  Using rate_out (not cmd_pps)
     * keeps the tracker honest during enable settle and direction reversals. */
    pos_current_steps += stepper_left.rate_out * (RATE_STEPPER_MS / 1000.0f);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  MX_TIM8_Init();
  MX_DAC1_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  MX_ADC5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);

  HAL_TIM_PWM_Start(&htim8,  TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

  stepper_init(&stepper_left);
  stepper_init(&stepper_right);

  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
      Error_Handler();
  encoder_zero();

  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  BSP_COM_Init(COM1, &BspCOMInit);

  printf("Solar Boat position-control demo.\r\n");
  printf("STEPS_PER_REV=%lu  GEAR=%.1f:1  STEPS_PER_OUT_DEG=%.2f\r\n",
         STEPS_PER_REV, (double)GEAR_RATIO, (double)STEPS_PER_OUT_DEG);
  printf("POS_PEAK_PPS=%.0f (%.1f motor RPM)  POS_KP=%.1f  DEADBAND=%.0f steps\r\n",
         (double)POS_PEAK_PPS, (double)PPS_TO_RPM(POS_PEAK_PPS),
         (double)POS_KP, (double)POS_DEADBAND_STEPS);

  last_tick  = HAL_GetTick();
  last_print = HAL_GetTick();
  /* USER CODE END 2 */

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* ---------------------------------------------------------------------
   * Demo: cycle through a list of target angles, dwelling 1 s at each.
   *
   * To input your own degree elsewhere in the code, call:
   *     stepper_set_target_deg(45.0f);    // go to +45 deg output
   *     stepper_set_target_deg(-12.5f);   // go to -12.5 deg output
   *     stepper_move_deg(10.0f);          // shift target by +10 deg
   *
   * Read back with:
   *     stepper_get_output_deg();   // what was commanded
   *     encoder_get_deg();          // what was measured (open-loop check)
   * --------------------------------------------------------------------- */
  const float demo_targets[] = { 30.0f, -30.0f, 60.0f, -60.0f, 0.0f };
  const uint8_t demo_count   = sizeof(demo_targets) / sizeof(demo_targets[0]);
  uint8_t  demo_idx = 0;
  uint32_t at_target_since = 0;

  /* Kick off with the first target so stepper_at_target() doesn't fire
   * immediately at the start (target == current == 0). */
  stepper_set_target_deg(demo_targets[demo_idx]);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      /* --- Position control tick at fixed rate --- */
      if (now - last_tick >= RATE_STEPPER_MS)
      {
          stepper_position_tick();
          last_tick = now;
      }

      /* --- Advance to next target after dwelling 1 s on current --- */
      if (stepper_at_target())
      {
          if (at_target_since == 0U)
          {
              at_target_since = now;
          }
          else if ((now - at_target_since) >= 1000U)
          {
              demo_idx = (uint8_t)((demo_idx + 1U) % demo_count);
              stepper_set_target_deg(demo_targets[demo_idx]);
              at_target_since = 0U;
          }
      }
      else
      {
          at_target_since = 0U;
      }

      /* --- 10 Hz status: target vs commanded vs measured --- */
      if (now - last_print >= 100U)
      {
          printf("TGT=%+7.2f  CMD=%+7.2f  ENC=%+7.2f  err=%+7.2f  at_tgt=%u\r\n",
                 (double)demo_targets[demo_idx],
                 (double)stepper_get_output_deg(),
                 (double)encoder_get_deg(),
                 (double)(demo_targets[demo_idx] - stepper_get_output_deg()),
                 stepper_at_target());
          last_print = now;
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC5_Init(void)
{

  /* USER CODE BEGIN ADC5_Init 0 */

  /* USER CODE END ADC5_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC5_Init 1 */

  /* USER CODE END ADC5_Init 1 */

  /** Common config
  */
  hadc5.Instance = ADC5;
  hadc5.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc5.Init.Resolution = ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation = 0;
  hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait = DISABLE;
  hadc5.Init.ContinuousConvMode = DISABLE;
  hadc5.Init.NbrOfConversion = 1;
  hadc5.Init.DiscontinuousConvMode = DISABLE;
  hadc5.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc5.Init.DMAContinuousRequests = DISABLE;
  hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc5.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC5_Init 2 */

  /* USER CODE END ADC5_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 39;
  hfdcan1.Init.NominalTimeSeg1 = 130;
  hfdcan1.Init.NominalTimeSeg2 = 39;
  hfdcan1.Init.DataPrescaler = 17;
  hfdcan1.Init.DataSyncJumpWidth = 6;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 6;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x40B285C2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */
  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */
  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 169;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */
  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */
  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */
  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 169;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */
  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_R_Pin|DIR_L_Pin|EN_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : EN_R_Pin */
  GPIO_InitStruct.Pin = EN_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_R_Pin DIR_L_Pin EN_L_Pin */
  GPIO_InitStruct.Pin = DIR_R_Pin|DIR_L_Pin|EN_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Zero_Steering_Pin */
  GPIO_InitStruct.Pin = Zero_Steering_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Zero_Steering_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
