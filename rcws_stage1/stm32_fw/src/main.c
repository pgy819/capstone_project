/**
 * main.c — RCWS Stage 1: 500 Hz 2-axis visual-servoing firmware (NUCLEO-F401RE)
 *
 *  Clock  : HSI 16 MHz -> PLL -> SYSCLK 84 MHz (APB1 42 MHz / TIM 84 MHz, APB2 84 MHz)
 *  Pin map inherited from the team firmware (hyunjinscode ball5_2_best.ino):
 *  USART1 : PA9 TX / PA10 RX 1 Mbps 8N1, TX DMA2_Stream7 / CH4 normal  (Feetech STS3215 bus,
 *           Pan = ID 2, Tilt = ID 1)
 *  USART6 : PC6 TX / PC7 RX 115200 8N1, RX DMA2_Stream1 / CH5 circular, TX DMA2_Stream6 / CH5
 *           (Pi 5 link, packet v2: link_proto.h)
 *  USART2 : PA2/PA3  115200 8N1 (ST-LINK VCP, boot banner only)
 *  PA4    : half-duplex direction (harmless with the auto-direction adapter), PA5: LD2 status LED
 *  PC13   : B1 — held at boot: servo ID commissioning; pressed while running: EMERGENCY STOP
 *           (torque off, latched until the black RESET button)
 *  TIM2   : 500 Hz (PSC 83, ARR 1999) -> PD servo loop inside TIM2_IRQHandler
 *  Build env "sysid" (-DRCWS_SYSID): servo identification test mode instead (see sysid.h)
 *
 *  NVIC priorities (group 4): USART1/DMA2_S7 (bus) = 1, TIM2 = 2, USART6/DMA2_S1/DMA2_S6 (Pi) = 3,
 *  SysTick = 15
 */
#include "stm32f4xx_hal.h"
#include "protocol.h"
#include "feetech_sts.h"
#include "servo_guard.h"
#include <string.h>
#ifdef RCWS_SYSID
#include "sysid.h"
#endif

#ifndef RCWS_VCP_BAUD
#define RCWS_VCP_BAUD 115200u   /* sysid env: 921600 */
#endif

/* ------------------------------------------------------------------------- */
/* Control constants (STAGE1_SPEC Module B §2)                               */
/* ------------------------------------------------------------------------- */
#define KP_PAN          0.0025f
#define KD_PAN          0.015f
#define KP_TILT         0.0025f
#define KD_TILT         0.015f

#define DIR_PAN         1.0f
#define DIR_TILT        (-1.0f)

/* Packet v2 carries the angle error in 1/16 servo count. The PD gains above were tuned on the v1
   virtual-pixel error (1920 px across the HFOV, 0.8145 mrad/px), so the error is converted back:
   1 count = 2*pi/4096 rad = 1.534 mrad = 1.883 v1 px. v1 err_y was +down, packet v2 tilt is +up. */
#define V1PX_PER_COUNT  1.883f
#define LINK_TO_V1PX    (V1PX_PER_COUNT / (float)LINK_ERR_SCALE)

#define D_LPF_KEEP      0.6f
#define D_LPF_NEW       0.4f
#define D_DECAY_SUBTICK 0.85f

#define SERVO_CENTER       ((float)LINK_SERVO_CENTER)
#define PAN_HOME           ((float)LINK_PAN_HOME)    /* 2048 = front (taught with SET_HOME) */
#define TILT_HOME          ((float)LINK_TILT_HOME)
#define PAN_TRAVEL_COUNTS  ((float)STS_PAN_TRAVEL)    /* +-125 deg around home (cable limit +-130 deg) */
#define TILT_TRAVEL_COUNTS ((float)STS_TILT_TRAVEL)   /* +-45 deg around home */
/* Goal clamp around home, bounded to the STS3215 single-turn range [0, 4095]:
   Pan [626, 3470], Tilt [1536, 2560]. */
#define PAN_MIN         ((PAN_HOME  > PAN_TRAVEL_COUNTS)  ? (PAN_HOME  - PAN_TRAVEL_COUNTS)  : 0.0f)
#define PAN_MAX         ((PAN_HOME  + PAN_TRAVEL_COUNTS  < (float)STS_POS_MAX) ? (PAN_HOME  + PAN_TRAVEL_COUNTS)  : (float)STS_POS_MAX)
#define TILT_MIN        ((TILT_HOME > TILT_TRAVEL_COUNTS) ? (TILT_HOME - TILT_TRAVEL_COUNTS) : 0.0f)
#define TILT_MAX        ((TILT_HOME + TILT_TRAVEL_COUNTS < (float)STS_POS_MAX) ? (TILT_HOME + TILT_TRAVEL_COUNTS) : (float)STS_POS_MAX)

/* v4 motion shaping and safety (scenarios I, J; servo_guard.h) */
#define DEADBAND_PX         3.0f      /* no correction for |pixel error| <= 3 px */
#define V_MAX_DPS           300.0f    /* goal velocity limit */
#define A_MAX_DPS2          3000.0f   /* goal acceleration limit while following */
#define A_BRAKE_DPS2        6000.0f   /* deceleration to a stop on HOLD (no coasting on the last velocity) */
#define COMM_TORQUE_OFF_LIMIT 5000u   /* 10 s without packets: torque off (Pi gone) */
#define RAMP_TICKS          500u      /* 1 s of slow motion after the torque comes back */
#define STS_RAMP_SPEED      300u      /* servo speed during RAMP [steps/s] ~ 26 deg/s */
#define BOOT_SLOW_TICKS     2500u     /* first 5 s after boot: slow move to home (covers 125 deg at 300 steps/s) */
#define STATUS_PERIOD_MS    20u       /* uplink STATUS 50 Hz */
#define JOG_MAX_DPS         60.0f     /* operator jog (web) rate limit */
#define HOME_GAIN           0.01f     /* HOME: goal velocity = gain * (centre - goal) per tick */
#define HOME_MAX_DPS        30.0f
#define ESTOP_DEBOUNCE_MS   30u

#define COMMISSION_OLD_ID   1u        /* factory ID */
#define COMMISSION_NEW_ID   STS_PAN_ID /* 2 = Pan (team assignment) */
#define DEBOUNCE_MS         50u

/* ------------------------------------------------------------------------- */
/* Peripheral handles                                                         */
/* ------------------------------------------------------------------------- */
UART_HandleTypeDef huart1;          /* servo bus */
UART_HandleTypeDef huart2;          /* ST-LINK VCP */
UART_HandleTypeDef huart6;          /* Pi 5 link */
DMA_HandleTypeDef  hdma_usart1_tx;
DMA_HandleTypeDef  hdma_usart6_rx;
DMA_HandleTypeDef  hdma_usart6_tx;
TIM_HandleTypeDef  htim2;

#ifndef RCWS_SYSID
/* ------------------------------------------------------------------------- */
/* Servo-loop state (owned by TIM2_IRQHandler)                               */
/* ------------------------------------------------------------------------- */
static AxisState_t  ax_pan  = { PAN_HOME, 0.0f };
static AxisState_t  ax_tilt = { TILT_HOME, 0.0f };
static AxisLimits_t lim_pan, lim_tilt;
static Guard_t      g_guard;
static const GuardCfg_t k_guard_cfg = { COMM_WATCHDOG_LIMIT, COMM_TORQUE_OFF_LIMIT, RAMP_TICKS };
static volatile uint8_t g_estop = 0u;          /* set by the main loop (B1), read by TIM2 */
static volatile uint8_t g_estop_sw = 0u;       /* set by TIM2 from the link flag, latched */
static float   d_ex_lpf = 0.0f;
static float   d_ey_lpf = 0.0f;
static float   prev_err_x = 0.0f;
static float   prev_err_y = 0.0f;
static uint8_t is_first_track_frame = 1u;
static uint32_t boot_ticks = 0u;

static volatile GuardState_t g_servo_state = GUARD_HOLD;    /* read by LED / uplink */
static volatile uint8_t      g_comm_lost = 1u;              /* HOLD because of a comm gap */
static volatile uint8_t      g_operator_hold = 0u;
static volatile uint16_t     g_pan_goal_u16 = LINK_PAN_HOME;        /* published by TIM2 for the uplink */
static volatile uint16_t     g_tilt_goal_u16 = LINK_TILT_HOME;
static uint8_t               g_ping_flags = 0u;             /* LINK_S_*_PING_OK | LINK_S_PING_DONE */
static volatile uint8_t      g_home_set = 0u;               /* LINK_S_HOME_SET: SET_HOME written */
static uint8_t               calib_step = 0u;               /* TIM2: 0 idle, 1 pan, 2 tilt, 3 done */
#endif

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
#ifndef RCWS_SYSID
static void Commissioning_CheckAndRun(void);
static void Servo_ControlStep(void);
static void Led_Task(void);
static void Estop_Task(void);
static void Status_Task(void);
static size_t put_str(char *b, size_t n, const char *s);
static size_t put_u16(char *b, size_t n, uint16_t v);
#endif
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

    /* The team firmware ran the servos in wheel mode: back to position mode (RAM, every boot). */
    HAL_Delay(100);   /* servo power-up */
    const HAL_StatusTypeDef mode_st = STS_InitPositionMode();
    (void)mode_st;

#ifdef RCWS_SYSID
    /* System-identification test mode: no vision link, PC drives the excitation over the VCP. */
    SysId_Init();
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }
    while (1) {
        SysId_Poll();
    }
#else
    static const char banner[] = "\r\nRCWS Stage1 FW: 500Hz 2-axis visual servo\r\n";
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)banner, (uint16_t)(sizeof(banner) - 1u), 50u);
    if (mode_st != HAL_OK) {
        static const char m[] = "warning: position-mode write failed on the servo bus\r\n";
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)m, (uint16_t)(sizeof(m) - 1u), 50u);
    }

    /* PC13 held at boot -> rename servo ID 1 -> 2 = Pan (blocking, before TIM2 starts). */
    Commissioning_CheckAndRun();

    /* Self-check: does each ID answer? (reported in every uplink STATUS, shown on the web page) */
    {
        uint16_t p0 = 0u, t0 = 0u;
        const uint8_t ok = STS_BootPing(&p0, &t0);
        g_ping_flags = (uint8_t)(LINK_S_PING_DONE | ((ok & 1u) ? LINK_S_PAN_PING_OK : 0u) |
                                 ((ok & 2u) ? LINK_S_TILT_PING_OK : 0u));
        char m[128];
        size_t n = 0u;
        n = put_str(m, n, "servo self-check: Pan(ID2) ");
        n = (ok & 1u) ? put_u16(m, put_str(m, n, "pos "), p0) : put_str(m, n, "NO ANSWER");
        n = put_str(m, n, ", Tilt(ID1) ");
        n = (ok & 2u) ? put_u16(m, put_str(m, n, "pos "), t0) : put_str(m, n, "NO ANSWER");
        n = put_str(m, n, "  (front = 2048 after Set home)\r\n");
        (void)HAL_UART_Transmit(&huart2, (const uint8_t *)m, (uint16_t)n, 50u);

        /* Start where the servos are: no motion at power-up (the old 2048 boot move could swing a
           not-yet-taught tilt into the frame). A servo outside its travel window means the front
           pose is not taught: stay limp (software ESTOP) until "Set home" or an explicit release. */
        if (ok == 3u) {
            ax_pan.goal  = (float)p0;
            ax_tilt.goal = (float)t0;
            g_pan_goal_u16  = p0;
            g_tilt_goal_u16 = t0;
            if ((float)p0 < PAN_MIN || (float)p0 > PAN_MAX || (float)t0 < TILT_MIN || (float)t0 > TILT_MAX) {
                g_estop_sw = 1u;
                static const char w[] = "servo outside its travel window: torque off. Teach the front pose (web: E-STOP -> Set home)\r\n";
                (void)HAL_UART_Transmit(&huart2, (const uint8_t *)w, (uint16_t)(sizeof(w) - 1u), 50u);
            }
        }
    }

    lim_pan  = Guard_Limits(PAN_MIN, PAN_MAX, V_MAX_DPS, A_MAX_DPS2, A_BRAKE_DPS2);
    lim_tilt = Guard_Limits(TILT_MIN, TILT_MAX, V_MAX_DPS, A_MAX_DPS2, A_BRAKE_DPS2);
    Guard_Init(&g_guard);

    if (Protocol_StartRx(&huart6) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    while (1) {
        Protocol_Poll();    /* drain USART6 DMA ring -> packet v2 frame finder */
        Estop_Task();
        Status_Task();      /* 50 Hz uplink */
        Led_Task();
    }
#endif
}

#ifndef RCWS_SYSID
/* ========================================================================= */
/* 500 Hz servo loop                                                         */
/* ========================================================================= */
static void Servo_ControlStep(void)
{
    uint8_t  mode, flags;
    float    pkt_err_x, pkt_err_y;
    float    pkt_rate_pan, pkt_rate_tilt;
    uint8_t  new_flag;
    uint32_t watchdog;

    /* 1. Snapshot shared data and clear the new-packet flag atomically.
       2. Increment the communication watchdog. */
    __disable_irq();
    mode      = g_vision_pkt.mode;
    flags     = g_vision_pkt.flags;
    pkt_err_x = (float)g_vision_pkt.err_pan;
    pkt_err_y = (float)g_vision_pkt.err_tilt;
    pkt_rate_pan  = (float)g_vision_pkt.rate_pan;
    pkt_rate_tilt = (float)g_vision_pkt.rate_tilt;
    new_flag  = g_new_pkt_flag;
    g_new_pkt_flag = 0u;
    if (g_comm_watchdog < 0xFFFFFFFFu) {
        g_comm_watchdog++;
    }
    watchdog = g_comm_watchdog;
    __enable_irq();

    /* v1 pixel units for the PD below: +right, +down */
    pkt_err_x *= LINK_TO_V1PX;
    pkt_err_y *= -LINK_TO_V1PX;
    if (boot_ticks < 0xFFFFFFFFu) {
        boot_ticks++;
    }

    /* 3. Safety supervisor (servo_guard.h): RUN / HOLD / TORQUE_OFF / RAMP / ESTOP.
          Web "stop tracking" (OPERATOR_HOLD) = target invalid; software ESTOP latches like B1. */
    const uint8_t fresh = (watchdog <= COMM_WATCHDOG_LIMIT) ? 1u : 0u;
    if (fresh && (flags & LINK_F_ESTOP_REQ)) {
        g_estop_sw = 1u;
    } else if (fresh && (flags & LINK_F_ESTOP_CLEAR) && g_estop_sw && !g_estop) {
        g_estop_sw = 0u;                   /* web release: limp, then RAMP back (B1 stays latched) */
        Guard_ClearEstop(&g_guard);
    }
    g_operator_hold = (fresh && (flags & LINK_F_OPERATOR_HOLD)) ? 1u : 0u;
    const uint8_t operator_move = (mode == LINK_MODE_JOG || mode == LINK_MODE_HOME) ? 1u : 0u;
    const uint8_t target_valid =
        (((mode == LINK_MODE_FOLLOW || mode == LINK_MODE_PREDICT) && !g_operator_hold) || operator_move) ? 1u : 0u;
    const GuardState_t st = Guard_Update(&g_guard, &k_guard_cfg, watchdog, target_valid,
                                         (uint8_t)(g_estop | g_estop_sw));
    g_servo_state = st;
    g_comm_lost = (watchdog > COMM_WATCHDOG_LIMIT) ? 1u : 0u;

    if (st != GUARD_RUN) {
        d_ex_lpf = 0.0f;
        d_ey_lpf = 0.0f;
        is_first_track_frame = 1u;   /* no derivative kick when tracking resumes */
    }
    if (st == GUARD_TORQUE_OFF || st == GUARD_ESTOP) {
        /* limp: the goal is not moved; repeat the torque-off frame (a busy bus skips one) */
        ax_pan.v = 0.0f;
        ax_tilt.v = 0.0f;
        /* Web "Set home": only in a software ESTOP (servos limp, placed at the front by hand).
           One WRITE 128 -> address 40 per servo (Feetech mid-position calibration): the servo
           stores the offset itself and reports 2048 here from now on, also after power-off. */
        if (st == GUARD_ESTOP && g_estop_sw && !g_estop && fresh && mode == LINK_MODE_SET_HOME) {
            if (calib_step == 0u) {
                calib_step = 1u;
            }
            if (calib_step == 1u && STS_WriteByteDMA(STS_PAN_ID, STS_ADDR_TORQUE_ENABLE, STS_TORQUE_CALIB_MID)) {
                calib_step = 2u;
            } else if (calib_step == 2u && STS_WriteByteDMA(STS_TILT_ID, STS_ADDR_TORQUE_ENABLE, STS_TORQUE_CALIB_MID)) {
                calib_step = 3u;
                ax_pan.goal  = SERVO_CENTER;   /* the servos are now at 2048: torque comes back without a jump */
                ax_tilt.goal = SERVO_CENTER;
                g_pan_goal_u16  = LINK_SERVO_CENTER;
                g_tilt_goal_u16 = LINK_SERVO_CENTER;
                g_home_set = 1u;
            }
            return;
        }
        if (mode != LINK_MODE_SET_HOME) {
            calib_step = 0u;                   /* a later request may teach the pose again */
        }
        if ((g_guard.ticks_in_state % 25u) == 0u) {
            STS_SyncWrite_Torque(0u);
        }
        return;
    }
    if (st == GUARD_RAMP && g_guard.ticks_in_state < 10u) {
        if ((g_guard.ticks_in_state % 2u) == 0u) {
            STS_SyncWrite_Torque(1u);      /* torque back on, then slow positions below */
        }
        return;
    }

    float v_pan = 0.0f, v_tilt = 0.0f;
    if (st == GUARD_RUN && operator_move) {
        /* web operator: JOG = commanded rate (+right / +up), HOME = slow return to the centre */
        if (mode == LINK_MODE_JOG) {
            const float k = GUARD_TICK_S;                       /* count/s -> count/tick */
            v_pan  = DIR_PAN * pkt_rate_pan * k;
            v_tilt = -DIR_TILT * pkt_rate_tilt * k;             /* tilt: +up = -image y */
            const float jmax = JOG_MAX_DPS * GUARD_STEPS_PER_DEG * GUARD_TICK_S;
            v_pan  = v_pan  > jmax ? jmax : (v_pan  < -jmax ? -jmax : v_pan);
            v_tilt = v_tilt > jmax ? jmax : (v_tilt < -jmax ? -jmax : v_tilt);
        } else {
            const float hmax = HOME_MAX_DPS * GUARD_STEPS_PER_DEG * GUARD_TICK_S;
            v_pan  = HOME_GAIN * (PAN_HOME - ax_pan.goal);
            v_tilt = HOME_GAIN * (TILT_HOME - ax_tilt.goal);
            v_pan  = v_pan  > hmax ? hmax : (v_pan  < -hmax ? -hmax : v_pan);
            v_tilt = v_tilt > hmax ? hmax : (v_tilt < -hmax ? -hmax : v_tilt);
        }
        is_first_track_frame = 1u;   /* no derivative kick when tracking resumes */
    } else if (st == GUARD_RUN) {
        /* 4. Multi-rate incremental PD (velocity request in steps/tick). */
        if (new_flag) {
            float raw_dex, raw_dey;
            if (is_first_track_frame) {
                raw_dex = 0.0f;            /* no derivative kick on re-acquisition */
                raw_dey = 0.0f;
                is_first_track_frame = 0u;
            } else {
                raw_dex = pkt_err_x - prev_err_x;
                raw_dey = pkt_err_y - prev_err_y;
            }
            prev_err_x = pkt_err_x;
            prev_err_y = pkt_err_y;
            d_ex_lpf = D_LPF_KEEP * d_ex_lpf + D_LPF_NEW * raw_dex;
            d_ey_lpf = D_LPF_KEEP * d_ey_lpf + D_LPF_NEW * raw_dey;
        } else {
            d_ex_lpf *= D_DECAY_SUBTICK;   /* sub-ticks between camera frames */
            d_ey_lpf *= D_DECAY_SUBTICK;
        }
        const float ex = Guard_Deadband(pkt_err_x, DEADBAND_PX);
        const float ey = Guard_Deadband(pkt_err_y, DEADBAND_PX);
        v_pan  = DIR_PAN  * (KP_PAN  * ex + KD_PAN  * d_ex_lpf);
        v_tilt = DIR_TILT * (KP_TILT * ey + KD_TILT * d_ey_lpf);
    }

    /* 5. Velocity / acceleration limits, soft position limits with anti-windup; HOLD brakes to a stop. */
    const uint8_t braking = (st == GUARD_RUN) ? 0u : 1u;
    const float pan_goal  = Guard_AxisStep(&ax_pan,  &lim_pan,  v_pan,  braking);
    const float tilt_goal = Guard_AxisStep(&ax_tilt, &lim_tilt, v_tilt, braking);

    g_pan_goal_u16  = (uint16_t)(pan_goal + 0.5f);
    g_tilt_goal_u16 = (uint16_t)(tilt_goal + 0.5f);

    /* 6. 22-byte SYNC WRITE via DMA (non-blocking). Slow servo speed while ramping back and right
          after boot (if the boot read failed, the first goal is home wherever the gimbal was left). */
    const uint16_t speed = (st == GUARD_RAMP || boot_ticks < BOOT_SLOW_TICKS) ? STS_RAMP_SPEED : STS_DEFAULT_SPEED;
    STS_SyncWrite_PosSpeed((uint16_t)(pan_goal + 0.5f), speed, (uint16_t)(tilt_goal + 0.5f), speed);
}

/* ========================================================================= */
/* Emergency stop: B1 pressed while running (falling edge after a release)   */
/* ========================================================================= */
static void Estop_Task(void)
{
    static uint8_t  armed = 0u;        /* B1 seen released once (it may still be held after commissioning) */
    static uint32_t t_low = 0u;
    static uint8_t  reported = 0u;
    const uint32_t now = HAL_GetTick();
    const uint8_t pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) ? 1u : 0u;

    if (!pressed) {
        armed = 1u;
        t_low = 0u;
    } else if (armed && !g_estop) {
        if (t_low == 0u) {
            t_low = now ? now : 1u;
        } else if ((now - t_low) >= ESTOP_DEBOUNCE_MS) {
            g_estop = 1u;              /* latched until reset */
        }
    }
    if (g_estop && !reported) {
        static const char msg[] = "ESTOP: B1 pressed, servo torque off. Press RESET to resume.\r\n";
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1u), 20u);
        reported = 1u;
    }
}

/* tiny formatting helpers for the boot report (no printf in the image) */
static size_t put_str(char *b, size_t n, const char *s)
{
    while (*s && n < 120u) { b[n++] = *s++; }
    return n;
}

static size_t put_u16(char *b, size_t n, uint16_t v)
{
    char t[6];
    int k = 0;
    do { t[k++] = (char)('0' + (v % 10u)); v = (uint16_t)(v / 10u); } while (v && k < 5);
    while (k > 0 && n < 120u) { b[n++] = t[--k]; }
    return n;
}

/* ========================================================================= */
/* Uplink STATUS to the Pi, 50 Hz (thread mode, DMA, never blocks)            */
/* ========================================================================= */
static void Status_Task(void)
{
    static uint32_t last = 0u;
    const uint32_t now = HAL_GetTick();
    if ((now - last) < STATUS_PERIOD_MS) {
        return;
    }
    last = now;
    LinkStatus_t s;
    s.seq_echo  = g_vision_pkt.seq;
    s.guard     = (uint8_t)g_servo_state;
    s.flags     = (uint8_t)((g_estop ? LINK_S_ESTOP_B1 : 0u) | (g_comm_lost ? LINK_S_COMM_LOST : 0u) |
                            (g_estop_sw ? LINK_S_ESTOP_SW : 0u) | (g_operator_hold ? LINK_S_OPERATOR_HOLD : 0u) |
                            g_ping_flags | (g_home_set ? LINK_S_HOME_SET : 0u));
    s.pan_goal  = g_pan_goal_u16;
    s.tilt_goal = g_tilt_goal_u16;
    s.rx_ok     = (uint16_t)g_proto_ok_count;
    s.rx_err    = (uint16_t)g_proto_err_count;
    Protocol_SendStatus(&s);
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

    static const char msg[] = "Commissioning: servo ID 1 -> 2 = Pan (connect ONLY that servo)\r\n";
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

/* LD2: solid = RUN, 1 Hz = HOLD (target lost), 2.5 Hz = HOLD (no packets), 5 Hz = RAMP,
        short blink every 2 s = TORQUE_OFF (Pi gone > 10 s), 10 Hz = ESTOP. */
static void Led_Task(void)
{
    static uint32_t last = 0u;
    const uint32_t now = HAL_GetTick();
    const GuardState_t st = g_servo_state;

    if (st == GUARD_RUN) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        return;
    }
    if (st == GUARD_TORQUE_OFF) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, ((now % 2000u) < 60u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        return;
    }
    uint32_t half_period = 500u;
    if (st == GUARD_ESTOP)        { half_period = 50u; }
    else if (st == GUARD_RAMP)    { half_period = 100u; }
    else if (g_comm_lost)         { half_period = 200u; }
    if ((now - last) >= half_period) {
        last = now;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}
#endif /* !RCWS_SYSID */

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

    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 1, 0);   /* USART1 TX: servo bus */
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 3, 0);   /* USART6 RX: Pi link */
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 3, 0);   /* USART6 TX: Pi link uplink */
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 1000000;   /* 84 MHz / (16 * 5.25) = exactly 1 Mbps */
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
    huart2.Init.BaudRate     = RCWS_VCP_BAUD;
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
    huart6.Init.BaudRate     = LINK_BAUD_DEFAULT;   /* 115200 (team), 0.08 % error at 84 MHz */
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
        /* servo bus: PA9 TX / PA10 RX */
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        g.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
        g.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &g);

        /* USART1_TX: DMA2_Stream7 / Channel 4, normal */
        hdma_usart1_tx.Instance                 = DMA2_Stream7;
        hdma_usart1_tx.Init.Channel             = DMA_CHANNEL_4;
        hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart1_tx.Init.Mode                = DMA_NORMAL;
        hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
        hdma_usart1_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

        /* Required so HAL_UART_TxCpltCallback fires on UART_FLAG_TC (PA4 release, sysid RX). */
        HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    } else if (huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
        g.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &g);
    } else if (huart->Instance == USART6) {
        /* Pi link: PC6 TX -> Pi GPIO15 (RXD), PC7 RX <- Pi GPIO14 (TXD) */
        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        g.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &g);

        /* USART6_RX: DMA2_Stream1 / Channel 5, circular (never DMA1 on F401) */
        hdma_usart6_rx.Instance                 = DMA2_Stream1;
        hdma_usart6_rx.Init.Channel             = DMA_CHANNEL_5;
        hdma_usart6_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_usart6_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart6_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart6_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart6_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_usart6_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_usart6_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(huart, hdmarx, hdma_usart6_rx);

        /* USART6_TX: DMA2_Stream6 / Channel 5, normal (uplink STATUS) */
        hdma_usart6_tx.Instance                 = DMA2_Stream6;
        hdma_usart6_tx.Init.Channel             = DMA_CHANNEL_5;
        hdma_usart6_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_usart6_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart6_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart6_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart6_tx.Init.Mode                = DMA_NORMAL;
        hdma_usart6_tx.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_usart6_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(huart, hdmatx, hdma_usart6_tx);

        HAL_NVIC_SetPriority(USART6_IRQn, 3, 0);
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

/* UART error: Pi link (USART6) -> deferred DMA restart; servo bus (USART1) -> release the bus. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
#ifndef RCWS_SYSID
        Protocol_RequestRxRestart();
#endif
    } else if (huart->Instance == STS_USART) {
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
#ifdef RCWS_SYSID
        SysId_Tick();
#else
        Servo_ControlStep();
#endif
    }
}

void DMA2_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void DMA2_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart6_rx);
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
