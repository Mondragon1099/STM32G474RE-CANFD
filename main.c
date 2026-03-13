/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Sensor Hub + FDCAN Transmit — STM32G474RE
  *
  * Sensors:
  *   - AS5600  : magnetic steering angle (I2C3, addr 0x36)
  *   - ADXL345 #1 : motor 1 vibration   (I2C3, addr 0x53 / 0xA6)
  *   - ADXL345 #2 : motor 2 vibration   (I2C3, addr 0x1D / 0x3A)
  *   - Hall sensor : motor RPM           (PA0, EXTI0)
  *   - Rotary encoder : incremental angle (TIM3)
  *   - Potentiometer  : fan duty cycle   (ADC1, TIM6 ISR)
  *
  * CAN FD TX frames:
  *   ID 0x000  — AS5600 steering angle (bytes 0-1, deg x10 as uint16)
  *   ID 0x001  — dummy frame (proof of concept, bytes 0-1 = 0xDE, 0xAD)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef hi2c3;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* --- AS5600 steering angle --- */
uint8_t  as5600_data[2];
uint16_t as5600_raw = 0;
float    as5600_deg = 0.0f;

/* --- ADXL345 #1 (motor 1) --- */
/*uint8_t  adxl1_id;
uint8_t  adxl1_data[6];
int16_t  ax1, ay1, az1;
uint8_t  adxl_power_ctl = 0x08;
float    adxl_cal_val   = 0.0039f;*/

/* --- ADXL345 #2 (motor 2) --- */
/*uint8_t  adxl2_id;
uint8_t  adxl2_data[6];
int16_t  ax2, ay2, az2;*/

/* --- Hall-effect RPM --- */
/*volatile uint32_t pulse_count     = 0;
volatile uint32_t last_time       = 0;
volatile uint32_t current_time    = 0;
volatile float    rpm             = 0.0f;
volatile float    rpm_avg         = 0.0f;
volatile uint8_t  new_rpm_ready   = 0;*/

/*#define AVG_SAMPLES           5
#define PULSES_PER_REVOLUTION 4
#define RPM_TIMEOUT           3000000U    3 s in microseconds
#define TICKS_PER_SECOND      1000714.0f  calibrated */

/*volatile float   rpm_buffer[AVG_SAMPLES] = {0};
volatile uint8_t rpm_buffer_index        = 0;*/

/* --- FDCAN TX --- */
FDCAN_FilterTypeDef   sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader_0x000;   /* AS5600 steering angle */
FDCAN_TxHeaderTypeDef TxHeader_0x001;   /* dummy frame           */
uint8_t TxData_0x000[64];
uint8_t TxData_0x001[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN1_Init(void);

/* USER CODE BEGIN 0 */

/* Read ADC once (blocking) */
/*static uint16_t read_adc_once(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return v;
}*/

/* Set fan PWM duty 0.0 – 1.0 */
/*static void fan_set_duty(float u)
{
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(u * (float)arr));
}*/

/* TIM6 overflow ISR — update fan duty from pot */
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        uint16_t adc  = read_adc_once();
        float    duty = (float)adc / 4095.0f;
        fan_set_duty(duty);
    }
}*/

/* PA0 EXTI ISR — Hall sensor rising/falling edge for RPM */
/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        current_time = __HAL_TIM_GET_COUNTER(&htim2);

        uint32_t time_diff;
        if (current_time >= last_time)
            time_diff = current_time - last_time;
        else
            time_diff = (0xFFFFFFFFU - last_time) + current_time;

        if (time_diff > 5000U)  debounce: >5 ms
        {
            rpm = (60.0f * TICKS_PER_SECOND) /
                  ((float)time_diff * (float)PULSES_PER_REVOLUTION);

            rpm_buffer[rpm_buffer_index] = rpm;
            rpm_buffer_index = (rpm_buffer_index + 1) % AVG_SAMPLES;

            float sum = 0.0f;
            for (int i = 0; i < AVG_SAMPLES; i++)
                sum += rpm_buffer[i];
            rpm_avg = sum / (float)AVG_SAMPLES;

            last_time     = current_time;
            pulse_count++;
            new_rpm_ready = 1;
        }
    }
}*/

/* USER CODE END 0 */

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C3_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM6_Init();
    MX_FDCAN1_Init();

    /* USER CODE BEGIN 2 */

    /* --- Free-running microsecond timer --- */
    HAL_TIM_Base_Start(&htim2);

    /* --- ADXL345 #1 (addr 0xA6) --- */
   /* HAL_I2C_Mem_Read (&hi2c3, 0xA6, 0x00, 1, &adxl1_id,    1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0xA6, 0x2D, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0xA6, 0x31, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);*/

    /* --- ADXL345 #2 (addr 0x3A) --- */
    /*HAL_I2C_Mem_Read (&hi2c3, 0x3A, 0x00, 1, &adxl2_id,    1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0x3A, 0x2D, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0x3A, 0x31, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);*/

    /* --- Fan PWM --- */
    /*HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    fan_set_duty(0.0f);
    HAL_TIM_Base_Start_IT(&htim6);*/

    /* --- Quadrature encoder --- */
   /* __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);*/

    /* --- FDCAN filter (required even for TX-only) --- */
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

    /* --- TX header for ID 0x000 (AS5600 steering angle) --- */
    TxHeader_0x000.Identifier          = 0x000;
    TxHeader_0x000.IdType              = FDCAN_STANDARD_ID;
    TxHeader_0x000.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader_0x000.DataLength          = FDCAN_DLC_BYTES_2;
    TxHeader_0x000.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader_0x000.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader_0x000.FDFormat            = FDCAN_FD_CAN;
    TxHeader_0x000.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader_0x000.MessageMarker       = 0;

    /* --- TX header for ID 0x001 (dummy) --- */
    TxHeader_0x001.Identifier          = 0x001;
    TxHeader_0x001.IdType              = FDCAN_STANDARD_ID;
    TxHeader_0x001.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader_0x001.DataLength          = FDCAN_DLC_BYTES_2;
    TxHeader_0x001.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader_0x001.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader_0x001.FDFormat            = FDCAN_FD_CAN;
    TxHeader_0x001.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader_0x001.MessageMarker       = 0;

    /* --- Start FDCAN --- */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        Error_Handler();

    /* --- UART console --- */
    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
        Error_Handler();

    /* USER CODE END 2 */

    /* -----------------------------------------------------------------------
     * Main loop
     * --------------------------------------------------------------------- */
    while (1)
    {
        printf("test\r\n");
    	/*  --- RPM timeout check ---
        current_time = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t time_since_pulse;
        if (current_time >= last_time)
            time_since_pulse = current_time - last_time;
        else
            time_since_pulse = (0xFFFFFFFFU - last_time) + current_time;

        if (time_since_pulse > RPM_TIMEOUT)
        {
            rpm     = 0.0f;
            rpm_avg = 0.0f;
        }
        if (new_rpm_ready)
        {
            printf("RPM: %.1f\r\n", rpm_avg);
            new_rpm_ready = 0;
        }*/

        /* --- AS5600 steering angle --- */
        if (HAL_I2C_Mem_Read(&hi2c3, 0x36 << 1, 0x0E,
                I2C_MEMADD_SIZE_8BIT, &as5600_data[0], 1, HAL_MAX_DELAY) == HAL_OK
         && HAL_I2C_Mem_Read(&hi2c3, 0x36 << 1, 0x0F,
                I2C_MEMADD_SIZE_8BIT, &as5600_data[1], 1, HAL_MAX_DELAY) == HAL_OK)
        {
            as5600_raw = ((uint16_t)as5600_data[0] << 8) | as5600_data[1];
            as5600_deg = (as5600_raw * 360.0f) / 4096.0f;
        }

       /*  --- ADXL345 #1 ---
        HAL_I2C_Mem_Read(&hi2c3, 0xA6, 0x32, 1, adxl1_data, 6, HAL_MAX_DELAY);
        ax1 = (int16_t)((adxl1_data[1] << 8) | adxl1_data[0]);
        ay1 = (int16_t)((adxl1_data[3] << 8) | adxl1_data[2]);
        az1 = (int16_t)((adxl1_data[5] << 8) | adxl1_data[4]);
        float accel_x1 = ax1 * adxl_cal_val;
        float accel_y1 = ay1 * adxl_cal_val;
        float accel_z1 = az1 * adxl_cal_val;

         --- ADXL345 #2 ---
        HAL_I2C_Mem_Read(&hi2c3, 0x3A, 0x32, 1, adxl2_data, 6, HAL_MAX_DELAY);
        ax2 = (int16_t)((adxl2_data[1] << 8) | adxl2_data[0]);
        ay2 = (int16_t)((adxl2_data[3] << 8) | adxl2_data[2]);
        az2 = (int16_t)((adxl2_data[5] << 8) | adxl2_data[4]);
        float accel_x2 = ax2 * adxl_cal_val;
        float accel_y2 = ay2 * adxl_cal_val;
        float accel_z2 = az2 * adxl_cal_val;

         --- Quadrature encoder ---
        int16_t counts    = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        float   angle_deg = ((float)counts * 360.0f) / 16384.0f;*/

        /* --- Debug UART --- */
        uint32_t A = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
        uint32_t B = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
        printf("A=%lu B=%lu\r\n", A, B);
/*        printf("counts: %d  encoder angle: %.2f deg\r\n", counts, angle_deg);*/
        printf("AS5600 angle: %.2f deg\r\n", as5600_deg);
/*        printf("Pulses: %lu  RPM: %.1f\r\n", pulse_count, rpm_avg);*/
/*        printf("ADXL1  x:%.2f  y:%.2f  z:%.2f g\r\n", accel_x1, accel_y1, accel_z1);*/
/*        printf("ADXL2  x:%.2f  y:%.2f  z:%.2f g\r\n", accel_x2, accel_y2, accel_z2);*/

        /* -----------------------------------------------------------------
         * CAN FD payload layout
         *
         * ID 0x000 — AS5600 steering angle
         * [0-1]  as5600_deg x10  uint16  e.g. 2700 = 270.0 deg
         *
         * ID 0x001 — dummy frame (proof of concept)
         * [0-1]  0xDE 0xAD
         * ----------------------------------------------------------------- */

        /* --- Build frame 0x000 — AS5600 steering angle --- */
        memset(TxData_0x000, 0, sizeof(TxData_0x000));
        uint16_t as5600_deg_int = (uint16_t)(as5600_deg * 10.0f);
        TxData_0x000[0] = (as5600_deg_int >> 8) & 0xFF;
        TxData_0x000[1] =  as5600_deg_int        & 0xFF;

        /* --- Build frame 0x001 — dummy --- */
        memset(TxData_0x001, 0, sizeof(TxData_0x001));
        TxData_0x001[0] = 0xDE;
        TxData_0x001[1] = 0xAD;

        /* --- Transmit 0x000 --- */
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x000, TxData_0x000) != HAL_OK)
        {
            printf("TX 0x000 failed\r\n");
        }

        /* --- Transmit 0x001 --- */
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x001, TxData_0x001) != HAL_OK)
        {
            printf("TX 0x001 failed\r\n");
        }

        HAL_Delay(1000);
    }
}

/* =========================================================================
 * SystemClock_Config
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

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

/* =========================================================================
 * Peripheral init functions (unchanged from CubeMX output)
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
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();

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
    hfdcan1.Instance                 = FDCAN1;
    hfdcan1.Init.ClockDivider        = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat         = FDCAN_FRAME_FD_NO_BRS;
    hfdcan1.Init.Mode                = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission  = ENABLE;
    hfdcan1.Init.TransmitPause       = ENABLE;
    hfdcan1.Init.ProtocolException   = DISABLE;
    hfdcan1.Init.NominalPrescaler    = 2;
    hfdcan1.Init.NominalSyncJumpWidth = 39;
    hfdcan1.Init.NominalTimeSeg1     = 130;
    hfdcan1.Init.NominalTimeSeg2     = 39;
    hfdcan1.Init.DataPrescaler       = 17;
    hfdcan1.Init.DataSyncJumpWidth   = 6;
    hfdcan1.Init.DataTimeSeg1        = 13;
    hfdcan1.Init.DataTimeSeg2        = 6;
    hfdcan1.Init.StdFiltersNbr       = 1;
    hfdcan1.Init.ExtFiltersNbr       = 0;
    hfdcan1.Init.TxFifoQueueMode     = FDCAN_TX_QUEUE_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) Error_Handler();
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
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
    TIM_MasterConfigTypeDef      sMasterConfig      = {0};
    TIM_OC_InitTypeDef           sConfigOC          = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 65535;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

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
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 169;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 4294967295U;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig      = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 0;
    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 0;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 0;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 65535;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* LED on PA5 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Hall sensor input on PA0 */
    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* =========================================================================
 * Error handler
 * ========================================================================= */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(100);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert failed: file %s, line %lu\r\n", file, (unsigned long)line);
}
#endif /* USE_FULL_ASSERT */
