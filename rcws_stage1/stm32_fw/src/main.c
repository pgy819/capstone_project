/**
 * main.c — RCWS Stage 1: 500 Hz 2-axis visual-servoing firmware (NUCLEO-F401RE)
 *
 *  Clock  : HSI 16 MHz -> PLL -> SYSCLK 84 MHz (APB1 42 MHz / TIM 84 MHz, APB2 84 MHz)
 *  USART1 : PA9/PA10 115200 8N1, RX DMA2_Stream2 / CH4 circular  (Pi 5 vision packets)
 *  USART2 : PA2/PA3  115200 8N1 (ST-LINK VCP, boot banner only)
 *  USART6 : PC6/PC7  1 Mbps 8N1, TX DMA2_Stream6 / CH5 normal    (Feetech STS3215 bus)
 *  PA4    : half-duplex direction, PA5: LD2 status LED, PC13: B1 commissioning button
 *  TIM2   : 500 Hz (PSC 83, ARR 1999) -> PD servo loop inside TIM2_IRQHandler
 *
 *  NVIC priorities (group 4): USART6/DMA2_S6 = 1, TIM2 = 2, USART1/DMA2_S2 = 3, SysTick = 15
 */
#include "stm32f4xx_hal.h"
#include "protocol.h"
#include "feetech_sts.h"

/* ------------------------------------------------------------------------- */
/* Control constants (STAGE1_SPEC Module B §2)                               */
/* ------------------------------------------------------------------------- */
#define KP_PAN          0.0025f
#define KD_PAN          0.015f
#define KP_TILT         0.0025f
#define KD_TILT         0.015f

#define DIR_PAN         1.0f
#define DIR_TILT        (-1.0f)

#define D_LPF_KEEP      0.6f
#define D_LPF_NEW       0.4f
#define D_DECAY_SUBTICK 0.85f

#define SERVO_CENTER    2048.0f
#define PAN_MIN         512.0f
#define PAN_MAX         3584.0f
#define TILT_MIN        1536.0f
#define TILT_MAX        2560.0f

#define COMMISSION_OLD_ID   1u
#define COMMISSION_NEW_ID   2u
#define DEBOUNCE_MS         50u

/* ------------------------------------------------------------------------- */
/* Peripheral handles                                                         */
/* ------------------------------------------------------------------------- */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef  hdma_usart1_rx;
DMA_HandleTypeDef  hdma_usart6_tx;
TIM_HandleTypeDef  htim2;

/* ------------------------------------------------------------------------- */
/* Servo-loop state (owned by TIM2_IRQHandler)                               */
/* ------------------------------------------------------------------------- */
static float   f_pan_step  = SERVO_CENTER;
static float   f_tilt_step = SERVO_CENTER;
static float   d_ex_lpf = 0.0f;
static float   d_ey_lpf = 0.0f;
static int16_t prev_err_x = 0;
static int16_t prev_err_y = 0;
static uint8_t is_first_track_frame = 1u;

typedef enum { SERVO_TIMEOUT = 0, SERVO_HOLD, SERVO_TRACKING } ServoState_t;
static volatile ServoState_t g_servo_state = SERVO_TIMEOUT;   /* read by LED task */

/* ------------------------------------------------------------------------- */
/* Prototypes                                                                 */
/* ------------------------------------------------------------------------- */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void Commissioning_CheckAndRun(void);
static void Servo_ControlStep(void);
static void Led_Task(void);
void Error_Handler(void);

/* ========================================================================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();          /* DMA clock before UART/DMA linking */
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART6_UART_Init();
    MX_TIM2_Init();

    static const char banner[] = "\r\nRCWS Stage1 FW: 500Hz 2-axis visual servo\r\n";
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)banner, (uint16_t)(sizeof(banner) - 1u), 50u);

    /* PC13 held at boot -> rename servo ID 1 -> 2 (blocking, before TIM2 starts). */
    Commissioning_CheckAndRun();

    if (Protocol_StartRx(&huart1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    while (1) {
        Protocol_Poll();    /* drain USART1 DMA ring -> FSM parser */
        Led_Task();
    }
}

/* ========================================================================= */
/* 500 Hz servo loop                                                         */
/* ========================================================================= */
static void Servo_ControlStep(void)
{
    uint8_t  status;
    int16_t  pkt_err_x, pkt_err_y;
    uint8_t  new_flag;
    uint32_t watchdog;

    /* 1. Snapshot shared data and clear the new-packet flag atomically.
       2. Increment the communication watchdog. */
    __disable_irq();
    status    = g_vision_pkt.status;
    pkt_err_x = g_vision_pkt.err_x;
    pkt_err_y = g_vision_pkt.err_y;
    new_flag  = g_new_pkt_flag;
    g_new_pkt_flag = 0u;
    if (g_comm_watchdog < 0xFFFFFFFFu) {
        g_comm_watchdog++;
    }
    watchdog = g_comm_watchdog;
    __enable_irq();

    /* 3. Safety hold (500 ms comm timeout or target lost) + first-frame reset. */
    if (watchdog > COMM_WATCHDOG_LIMIT ||
        (status != VISION_STATUS_DETECTED && status != VISION_STATUS_COASTING)) {
        d_ex_lpf = 0.0f;
        d_ey_lpf = 0.0f;
        is_first_track_frame = 1u;
        g_servo_state = (watchdog > COMM_WATCHDOG_LIMIT) ? SERVO_TIMEOUT : SERVO_HOLD;
        /* f_pan_step / f_tilt_step held */
    } else {
        /* 4. Multi-rate incremental PD. */
        if (new_flag) {
            float raw_dex, raw_dey;
            if (is_first_track_frame) {
                raw_dex = 0.0f;            /* no derivative kick on re-acquisition */
                raw_dey = 0.0f;
                is_first_track_frame = 0u;
            } else {
                raw_dex = (float)(pkt_err_x - prev_err_x);
                raw_dey = (float)(pkt_err_y - prev_err_y);
            }
            prev_err_x = pkt_err_x;
            prev_err_y = pkt_err_y;
            d_ex_lpf = D_LPF_KEEP * d_ex_lpf + D_LPF_NEW * raw_dex;
            d_ey_lpf = D_LPF_KEEP * d_ey_lpf + D_LPF_NEW * raw_dey;
        } else {
            d_ex_lpf *= D_DECAY_SUBTICK;   /* sub-ticks between camera frames */
            d_ey_lpf *= D_DECAY_SUBTICK;
        }

        f_pan_step  += DIR_PAN  * (KP_PAN  * (float)pkt_err_x + KD_PAN  * d_ex_lpf);
        f_tilt_step += DIR_TILT * (KP_TILT * (float)pkt_err_y + KD_TILT * d_ey_lpf);
        g_servo_state = SERVO_TRACKING;
    }

    /* 5. Mechanical limits. */
    if (f_pan_step < PAN_MIN)   { f_pan_step = PAN_MIN; }
    if (f_pan_step > PAN_MAX)   { f_pan_step = PAN_MAX; }
    if (f_tilt_step < TILT_MIN) { f_tilt_step = TILT_MIN; }
    if (f_tilt_step > TILT_MAX) { f_tilt_step = TILT_MAX; }

    /* 6. 22-byte SYNC WRITE via DMA (non-blocking). */
    STS_SyncWrite_PosSpeed((uint16_t)(f_pan_step + 0.5f), STS_DEFAULT_SPEED,
                           (uint16_t)(f_tilt_step + 0.5f), STS_DEFAULT_SPEED);
}

/* ========================================================================= */
/* Servo ID commissioning (PC13 held at boot)                                */
/* ========================================================================= */
static void Commissioning_CheckAndRun(void)
{
    /* B1 is active-low: must stay LOW for the whole 50 ms debounce window. */
    for (uint32_t i = 0u; i < DEBOUNCE_MS; i++) {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_RESET) {
            return;
        }
        HAL_Delay(1);
    }

    static const char msg[] = "Commissioning: servo ID 1 -> 2 (connect ONLY that servo)\r\n";
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1u), 50u);

    HAL_Delay(100);   /* let the servo finish its own power-up */
    HAL_StatusTypeDef st = STS_Assign_ID(COMMISSION_OLD_ID, COMMISSION_NEW_ID);

    /* 10 Hz blink for 2 s = 20 periods = 40 toggles of 50 ms. */
    for (uint32_t i = 0u; i < 40u; i++) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(50);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    static const char ok[]  = "Commissioning done.\r\n";
    static const char err[] = "Commissioning UART error!\r\n";
    if (st == HAL_OK) {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)ok, (uint16_t)(sizeof(ok) - 1u), 50u);
    } else {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)err, (uint16_t)(sizeof(err) - 1u), 50u);
    }
}

/* LD2: solid = tracking, 1 Hz = target lost (hold), 5 Hz = comm timeout. */
static void Led_Task(void)
{
    static uint32_t last = 0u;
    const uint32_t now = HAL_GetTick();
    const ServoState_t st = g_servo_state;

    if (st == SERVO_TRACKING) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        return;
    }
    const uint32_t half_period = (st == SERVO_HOLD) ? 500u : 100u;
    if ((now - last) >= half_period) {
        last = now;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}

/* ========================================================================= */
/* Clock: HSI 16 MHz / M16 * N336 / P4 = 84 MHz                              */
/* ========================================================================= */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 16;
    osc.PLL.PLLN            = 336;
    osc.PLL.PLLP            = RCC_PLLP_DIV4;   /* 336 / 4 = 84 MHz */
    osc.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;      /* HCLK 84 MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV2;        /* 42 MHz, TIM2 clock = 84 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;        /* 84 MHz (USART1/6) */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================================================================= */
/* Peripheral init                                                           */
/* ========================================================================= */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

    /* PA4 direction, PA5 LD2 */
    g.Pin   = GPIO_PIN_4 | GPIO_PIN_5;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    /* PC13 B1 (external pull-up on Nucleo; internal pull-up as backup) */
    g.Pin   = GPIO_PIN_13;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART6_UART_Init(void)
{
    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = 1000000;   /* 84 MHz / (16 * 5.25) = exactly 1 Mbps */
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;
    huart6.Init.Mode         = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef src = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 84 - 1;      /* 84 MHz -> 1 MHz */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 2000 - 1;    /* 1 MHz / 2000 = 500 Hz */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    src.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &src) != HAL_OK) {
        Error_Handler();
    }
    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================================================================= */
/* HAL MSP callbacks                                                         */
/* ========================================================================= */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        g.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
        g.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &g);

        /* USART1_RX: DMA2_Stream2 / Channel 4, circular (never DMA1 on F401) */
        hdma_usart1_rx.Instance                 = DMA2_Stream2;
        hdma_usart1_rx.Init.Channel             = DMA_CHANNEL_4;
        hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart1_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_usart1_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);

        HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    } else if (huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
        g.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &g);
    } else if (huart->Instance == USART6) {
        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        g.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &g);

        /* USART6_TX: DMA2_Stream6 / Channel 5, normal */
        hdma_usart6_tx.Instance                 = DMA2_Stream6;
        hdma_usart6_tx.Init.Channel             = DMA_CHANNEL_5;
        hdma_usart6_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_usart6_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart6_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart6_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart6_tx.Init.Mode                = DMA_NORMAL;
        hdma_usart6_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
        hdma_usart6_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(huart, hdmatx, hdma_usart6_tx);

        /* Required so HAL_UART_TxCpltCallback fires on UART_FLAG_TC. */
        HAL_NVIC_SetPriority(USART6_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}

/* UART error: USART1 -> deferred DMA restart; USART6 -> release the bus. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Protocol_RequestRxRestart();
    } else if (huart->Instance == USART6) {
        HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
    }
}

/* ========================================================================= */
/* Interrupt handlers (complete vector linkage — GEMINI.md §4.1)             */
/* ========================================================================= */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET &&
        __HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
        Servo_ControlStep();
    }
}

void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void DMA2_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart6_tx);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void USART6_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart6);
}

void HardFault_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

/* ========================================================================= */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Fast LD2 blink (busy-wait; SysTick disabled) */
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        for (volatile uint32_t i = 0; i < 400000u; i++) {
        }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
