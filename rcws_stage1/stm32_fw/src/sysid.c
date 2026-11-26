/**
 * sysid.c — STS3215 system-identification test mode. See sysid.h for the protocol.
 *
 * Bus budget per 2 ms tick at 1 Mbps: SYNC WRITE 22 B + READ 8 B = 300 us TX, servo return
 * delay (register 7) + status packet 12 B = 120 us RX  ->  ~0.5 ms, well inside the tick.
 */
#include "sysid.h"
#include "feetech_sts.h"
#include <math.h>
#include <string.h>

#define STS_INST_READ       0x02u
#define STS_ADDR_TORQUE_EN  0x28u    /* 40 */
#define STS_ADDR_PRESENT    0x38u    /* 56: position(2) speed(2) load(2) */
#define SYSID_READ_N        6u

#define TICK_HZ             500u
#define PREROLL_TICKS       250u      /* 0.5 s at zero before every profile */

/* step series: +A, 0, -A, 0 for each amplitude */
static const int16_t k_step_amp[] = {25, 50, 100, 200, 400};
#define STEP_PHASE_TICKS    300u      /* 0.6 s */
#define STEP_N_AMP          (sizeof(k_step_amp) / sizeof(k_step_amp[0]))

/* PRBS: x^9 + x^5 + 1, period 511 bits */
#define PRBS_AMP            60
#define PRBS_BIT_TICKS      10u       /* 20 ms -> excitation up to ~25 Hz */
#define PRBS_PERIODS        2u

/* log chirp */
#define CHIRP_AMP           60.0f
#define CHIRP_F0            0.2f
#define CHIRP_F1            12.0f
#define CHIRP_TICKS         15000u    /* 30 s */

/* ------------------------------------------------------------------------- */
/* Pure helpers                                                               */
/* ------------------------------------------------------------------------- */
int16_t SysId_DecodeSigned15(uint16_t raw)
{
    const int16_t mag = (int16_t)(raw & 0x7FFFu);
    return (raw & 0x8000u) ? (int16_t)-mag : mag;
}

uint8_t SysId_BuildRead(uint8_t *buf, uint8_t id, uint8_t addr, uint8_t len)
{
    buf[0] = 0xFFu;
    buf[1] = 0xFFu;
    buf[2] = id;
    buf[3] = 0x04u;
    buf[4] = STS_INST_READ;
    buf[5] = addr;
    buf[6] = len;
    buf[7] = (uint8_t)(~(uint32_t)(buf[2] + buf[3] + buf[4] + buf[5] + buf[6]) & 0xFFu);
    return 8u;
}

int SysId_ParseStatus(const uint8_t *buf, uint16_t size, uint8_t id, uint8_t *out, uint8_t n)
{
    /* status packet: FF FF id len err data[n] chk, len = n + 2 */
    const uint16_t pkt = (uint16_t)(n + 6u);
    for (uint16_t i = 0u; i + pkt <= size; i++) {
        if (buf[i] != 0xFFu || buf[i + 1u] != 0xFFu || buf[i + 2u] != id ||
            buf[i + 3u] != (uint8_t)(n + 2u)) {
            continue;
        }
        uint32_t sum = 0u;
        for (uint16_t j = 2u; j < pkt - 1u; j++) {
            sum += buf[i + j];
        }
        if ((uint8_t)(~sum & 0xFFu) != buf[i + pkt - 1u]) {
            continue;
        }
        memcpy(out, &buf[i + 5u], n);
        return 1;
    }
    return 0;
}

static int prbs_bit(uint32_t b)
{
    uint16_t s = 0x1FFu;
    for (uint32_t i = 0u; i < (b % 511u); i++) {
        const uint16_t fb = (uint16_t)(((s >> 8) ^ (s >> 4)) & 1u);   /* taps 9 and 5 */
        s = (uint16_t)(((s << 1) | fb) & 0x1FFu);
    }
    return (int)(s & 1u);
}

int16_t SysId_Profile(uint8_t prof, uint32_t k, uint8_t *done)
{
    *done = 0u;
    if (k < PREROLL_TICKS) {
        return 0;
    }
    const uint32_t t = k - PREROLL_TICKS;

    switch (prof) {
    case SYSID_PROF_STEP: {
        const uint32_t per_amp = 4u * STEP_PHASE_TICKS;
        const uint32_t a = t / per_amp;
        if (a >= STEP_N_AMP) {
            *done = 1u;
            return 0;
        }
        const uint32_t phase = (t % per_amp) / STEP_PHASE_TICKS;
        const int16_t amp = k_step_amp[a];
        return (phase == 0u) ? amp : (phase == 2u) ? (int16_t)-amp : 0;
    }
    case SYSID_PROF_PRBS: {
        const uint32_t b = t / PRBS_BIT_TICKS;
        if (b >= 511u * PRBS_PERIODS) {
            *done = 1u;
            return 0;
        }
        return prbs_bit(b) ? (int16_t)PRBS_AMP : (int16_t)-PRBS_AMP;
    }
    case SYSID_PROF_CHIRP: {
        if (t >= CHIRP_TICKS) {
            *done = 1u;
            return 0;
        }
        const float T = (float)CHIRP_TICKS / (float)TICK_HZ;
        const float tt = (float)t / (float)TICK_HZ;
        const float r = logf(CHIRP_F1 / CHIRP_F0);
        const float ph = 2.0f * 3.14159265f * CHIRP_F0 * T / r * (expf(tt * r / T) - 1.0f);
        return (int16_t)lrintf(CHIRP_AMP * sinf(ph));
    }
    default:
        *done = 1u;
        return 0;
    }
}

#if defined(RCWS_SYSID) && !defined(RCWS_HOST_TEST)
/* ------------------------------------------------------------------------- */
/* Target (only in the sysid build: the tracking firmware keeps the weak no-op hook)          */
/* ------------------------------------------------------------------------- */
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;

#define CENTER          2048
#define PAN_LO          (CENTER - (int16_t)STS_PAN_TRAVEL)     /* 626  : +-125 deg, same window as tracking */
#define PAN_HI          (CENTER + (int16_t)STS_PAN_TRAVEL)     /* 3470 */
#define TILT_LO         (CENTER - (int16_t)STS_TILT_TRAVEL)    /* 1536 : +-45 deg */
#define TILT_HI         (CENTER + (int16_t)STS_TILT_TRAVEL)    /* 2560 */
#define MOVE_SPEED      300u      /* steps/s for the move to centre (~26 deg/s, as the tracking RAMP) */
#define LOAD_ABORT      800       /* |load| > 80 % ...                                         */
#define LOAD_ABORT_TICKS 50u      /* ... for 0.1 s of fresh samples -> stall: stop the profile  */
#define B1_DEBOUNCE_MS  20u
#define RING_N          128u      /* records; power of two */

typedef struct {
    int16_t  y, v, load;
    uint32_t t_us;
    uint8_t  fresh;
} Meas_t;

static volatile uint8_t  s_axis = STS_PAN_ID;
static volatile uint8_t  s_prof = SYSID_PROF_IDLE;
static volatile uint8_t  s_start_req = SYSID_PROF_IDLE;
static volatile uint8_t  s_stop_req = 0u;
static volatile uint8_t  s_max_speed = 0u;
static volatile uint8_t  s_armed = 0u;      /* both servos answered and sit inside the travel window */
static volatile uint8_t  s_estop = 0u;      /* B1 latch (thread -> ISR), cleared only by RESET */
static volatile uint8_t  s_abort_note = 0u; /* ISR -> thread: print why the profile was stopped */
static uint8_t           s_torque_off_sent = 0u;
static uint32_t          s_load_cnt = 0u;
static uint32_t          s_k = 0u;
static uint16_t          s_seq = 0u;
static uint8_t           s_sticky_flags = 0u;

static volatile Meas_t   s_meas;
static volatile uint8_t  s_rx_pending = 0u;
static volatile uint8_t  s_rx_wanted = 0u;
static uint8_t           s_tx[STS_SYNC_WRITE_LEN + 8u];
static uint8_t           s_rx[32];

static uint8_t           s_ring[RING_N][SYSID_RECORD_LEN];
static volatile uint32_t s_head = 0u;   /* written by ISR */
static volatile uint32_t s_tail = 0u;   /* written by thread */
static volatile uint32_t s_dropped = 0u;

static inline uint32_t now_us(void) { return TIM5->CNT; }

static void tim5_us_init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->CR1 = 0u;
    TIM5->PSC = 84u - 1u;            /* 84 MHz timer clock -> 1 MHz */
    TIM5->ARR = 0xFFFFFFFFu;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CR1 = TIM_CR1_CEN;
}

static void vcp_text(const char *s)
{
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 20u);
}

static size_t put_str(char *b, size_t n, const char *s)
{
    while (*s != '\0' && n < 126u) { b[n++] = *s++; }
    return n;
}

static size_t put_u16(char *b, size_t n, uint16_t v)
{
    char t[6];
    int i = 0;
    do { t[i++] = (char)('0' + (v % 10u)); v = (uint16_t)(v / 10u); } while (v != 0u && i < 5);
    while (i > 0 && n < 126u) { b[n++] = t[--i]; }
    return n;
}

static void bus_blocking(const uint8_t *pkt, uint16_t len)
{
    HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_SET);
    (void)HAL_UART_Transmit(&STS_HUART, (uint8_t *)pkt, len, 20u);
    HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
}

static void torque_off_blocking(void)
{
    uint8_t pkt[STS_SYNC_TORQUE_LEN];
    (void)STS_BuildSyncTorque(pkt, 0u);
    bus_blocking(pkt, (uint16_t)sizeof(pkt));
}

static int in_window(uint16_t p, int16_t lo, int16_t hi)
{
    return ((int32_t)p >= lo) && ((int32_t)p <= hi);
}

static uint32_t absdiff(uint16_t a, uint16_t b)
{
    return (a > b) ? (uint32_t)(a - b) : (uint32_t)(b - a);
}

/*
 * No blind move at power-up: the servos are read first. Only when both answer and both sit inside
 * the tracking travel window (= the front pose was taught with "Set home") is the gimbal moved
 * slowly to 2048 and the profiles enabled. Otherwise torque goes off and every profile is refused
 * (a not-taught tilt can be near the 0/4095 seam, where a position goal swings it into the frame).
 */
void SysId_Init(void)
{
    tim5_us_init();

    uint16_t p0 = 0u, t0 = 0u;
    const uint8_t ok = STS_BootPing(&p0, &t0);
    char m[128];
    size_t n = put_str(m, 0u, "\r\n# servo read: Pan(ID2) ");
    n = (ok & 1u) ? put_u16(m, n, p0) : put_str(m, n, "NO ANSWER");
    n = put_str(m, n, ", Tilt(ID1) ");
    n = (ok & 2u) ? put_u16(m, n, t0) : put_str(m, n, "NO ANSWER");
    n = put_str(m, n, "\r\n");
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)m, (uint16_t)n, 20u);

    if (ok != 3u) {
        torque_off_blocking();
        vcp_text("# LOCKED: a servo does not answer (bus wiring / power / ID). Torque off.\r\n");
        return;
    }
    if (!in_window(p0, PAN_LO, PAN_HI) || !in_window(t0, TILT_LO, TILT_HI)) {
        torque_off_blocking();
        vcp_text("# LOCKED: outside the travel window, front pose not taught. Torque off.\r\n"
                 "#   flash env:nucleo_f401re -> web E-STOP -> hand to front -> Set home -> flash env:sysid\r\n");
        return;
    }

    /* hold where it is, then a slow move to centre (inside the window, so 2048 = front is safe) */
    uint8_t pkt[STS_SYNC_WRITE_LEN];
    (void)STS_BuildSyncWritePosSpeed(pkt, p0, MOVE_SPEED, t0, MOVE_SPEED);
    bus_blocking(pkt, (uint16_t)sizeof(pkt));
    HAL_Delay(5);
    (void)STS_BuildSyncWritePosSpeed(pkt, CENTER, MOVE_SPEED, CENTER, MOVE_SPEED);
    bus_blocking(pkt, (uint16_t)sizeof(pkt));
    const uint32_t d = (absdiff(p0, CENTER) > absdiff(t0, CENTER)) ? absdiff(p0, CENTER) : absdiff(t0, CENTER);
    HAL_Delay(d * 1000u / MOVE_SPEED + 500u);

    s_armed = 1u;
    vcp_text("# RCWS sysid mode. p/t axis, S step, R prbs, C chirp, M speed, X stop. B1 = ESTOP\r\n");
}

static void push_record(uint32_t t_tick, int16_t u, Meas_t m, uint8_t flags)
{
    const uint32_t head = s_head;
    if (head - s_tail >= RING_N) {
        s_dropped++;
        return;
    }
    uint8_t *r = s_ring[head & (RING_N - 1u)];
    r[0] = SYSID_SYNC0;
    r[1] = SYSID_SYNC1;
    memcpy(&r[2], &s_seq, 2);
    memcpy(&r[4], &t_tick, 4);
    memcpy(&r[8], &m.t_us, 4);
    memcpy(&r[12], &u, 2);
    memcpy(&r[14], &m.y, 2);
    memcpy(&r[16], &m.v, 2);
    memcpy(&r[18], &m.load, 2);
    r[20] = flags;
    r[21] = (uint8_t)((s_prof << 4) | (s_axis & 0x0Fu));
    r[22] = 0u;
    uint32_t sum = 0u;
    for (uint32_t i = 2u; i < 23u; i++) {
        sum += r[i];
    }
    r[23] = (uint8_t)sum;
    s_seq++;
    __DMB();
    s_head = head + 1u;
}

void SysId_Tick(void)
{
    const uint32_t t_tick = now_us();

    /* 0. locked or B1 ESTOP: no position writes. ESTOP sends one torque-off frame and stays silent. */
    if (s_estop) {
        if (s_prof != SYSID_PROF_IDLE) {
            /* close the capture: last record carries done + abort */
            Meas_t z = {0};
            push_record(t_tick, 0, z, (uint8_t)(0x04u | 0x20u));
            s_prof = SYSID_PROF_IDLE;
        }
        if (!s_torque_off_sent && STS_HUART.gState == HAL_UART_STATE_READY) {
            (void)STS_BuildSyncTorque(s_tx, 0u);
            s_rx_wanted = 0u;
            HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_SET);
            if (HAL_UART_Transmit_DMA(&STS_HUART, s_tx, STS_SYNC_TORQUE_LEN) == HAL_OK) {
                s_torque_off_sent = 1u;
            } else {
                HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
            }
        }
        return;
    }
    if (!s_armed) {
        return;
    }

    uint8_t flags = s_sticky_flags;
    s_sticky_flags = 0u;

    /* 1. reply of the previous tick never arrived -> abort and flag */
    if (s_rx_pending || s_rx_wanted) {
        s_rx_wanted = 0u;
        if (s_rx_pending) {
            (void)HAL_UART_AbortReceive(&STS_HUART);
            s_rx_pending = 0u;
        }
        flags |= 0x02u;
    }

    /* 2. latest measurement (reply to the previous tick's READ) */
    Meas_t m;
    __disable_irq();
    m.y = s_meas.y;
    m.v = s_meas.v;
    m.load = s_meas.load;
    m.t_us = s_meas.t_us;
    m.fresh = s_meas.fresh;
    s_meas.fresh = 0u;
    __enable_irq();
    if (m.fresh) {
        flags |= 0x01u;
    }

    /* 3. profile state machine (+ stall watch on the test axis) */
    if (s_stop_req) {
        s_stop_req = 0u;
        s_prof = SYSID_PROF_IDLE;
    }
    if (s_prof == SYSID_PROF_IDLE && s_start_req != SYSID_PROF_IDLE) {
        s_prof = s_start_req;
        s_start_req = SYSID_PROF_IDLE;
        s_k = 0u;
        s_load_cnt = 0u;
    }
    int16_t off = 0;
    uint8_t logging = 0u;
    if (s_prof != SYSID_PROF_IDLE) {
        uint8_t done;
        off = SysId_Profile(s_prof, s_k++, &done);
        logging = 1u;
        if (m.fresh) {
            const int16_t a = (m.load < 0) ? (int16_t)-m.load : m.load;
            s_load_cnt = (a > LOAD_ABORT) ? (s_load_cnt + 1u) : 0u;
        }
        if (s_load_cnt >= LOAD_ABORT_TICKS) {
            done = 1u;
            off = 0;                 /* back to centre */
            flags |= 0x20u;
            s_abort_note = 1u;
        }
        if (done) {
            flags |= 0x04u;
            s_prof = SYSID_PROF_IDLE;
        }
    }

    /* 4. goal, clamped to the absolute travel window (never beyond the tracking limits) */
    int16_t goal = (int16_t)(CENTER + off);
    const int16_t lo = (s_axis == STS_TILT_ID) ? TILT_LO : PAN_LO;
    const int16_t hi = (s_axis == STS_TILT_ID) ? TILT_HI : PAN_HI;
    if (goal > hi) { goal = hi; }
    if (goal < lo) { goal = lo; }
    const uint16_t pan  = (s_axis == STS_PAN_ID)  ? (uint16_t)goal : (uint16_t)CENTER;
    const uint16_t tilt = (s_axis == STS_TILT_ID) ? (uint16_t)goal : (uint16_t)CENTER;
    const uint16_t spd  = s_max_speed ? 0u : STS_DEFAULT_SPEED;
    if (s_max_speed) {
        flags |= 0x10u;
    }

    /* 5. one DMA frame: SYNC WRITE + READ(present pos/speed/load) */
    if (STS_HUART.gState == HAL_UART_STATE_READY) {
        (void)STS_BuildSyncWritePosSpeed(s_tx, pan, spd, tilt, spd);
        (void)SysId_BuildRead(&s_tx[STS_SYNC_WRITE_LEN], s_axis, STS_ADDR_PRESENT, SYSID_READ_N);
        s_rx_wanted = 1u;
        HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_SET);
        if (HAL_UART_Transmit_DMA(&STS_HUART, s_tx, (uint16_t)sizeof(s_tx)) != HAL_OK) {
            HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
            s_rx_wanted = 0u;
            flags |= 0x08u;
        }
    } else {
        flags |= 0x08u;
    }

    /* 6. record: goal of this tick + latest measurement */
    if (logging) {
        push_record(t_tick, goal, m, flags);
    }
}

/* Called from HAL_UART_TxCpltCallback (bus UART) after PA4 went low. */
void STS_TxDoneHook(void)
{
    if (!s_rx_wanted) {
        return;
    }
    s_rx_wanted = 0u;
    /* The FE-URT-1 echoes the 30-byte request on RX. Nobody read it, so ORE (and RXNE/IDLE) are
       set now; arming the IT receive on top of that would end in an ORE error callback every tick.
       SR then DR read clears ORE, RXNE and IDLE (F4 sequence). The echo is complete before TC. */
    (void)STS_USART->SR;
    (void)STS_USART->DR;
    if (HAL_UARTEx_ReceiveToIdle_IT(&STS_HUART, s_rx, (uint16_t)sizeof(s_rx)) == HAL_OK) {
        s_rx_pending = 1u;
    } else {
        s_sticky_flags |= 0x02u;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance != STS_USART) {
        return;
    }
    const uint32_t t = now_us();
    uint8_t d[SYSID_READ_N];
    if (SysId_ParseStatus(s_rx, size, s_axis, d, SYSID_READ_N)) {
        s_meas.y    = SysId_DecodeSigned15((uint16_t)(d[0] | (d[1] << 8)));
        s_meas.v    = SysId_DecodeSigned15((uint16_t)(d[2] | (d[3] << 8)));
        /* load: bits 0-9 magnitude (0.1 %), bit 10 direction */
        const uint16_t lr = (uint16_t)(d[4] | (d[5] << 8));
        s_meas.load = (int16_t)((lr & 0x0400u) ? -(int16_t)(lr & 0x03FFu) : (int16_t)(lr & 0x03FFu));
        s_meas.t_us = t;
        s_meas.fresh = 1u;
    } else {
        s_sticky_flags |= 0x02u;
    }
    s_rx_pending = 0u;
}

static void handle_cmd(uint8_t c)
{
    const uint8_t idle = (s_prof == SYSID_PROF_IDLE && s_start_req == SYSID_PROF_IDLE);
    if (c == 'S' || c == 'R' || c == 'C') {
        if (s_estop) {
            vcp_text("# refused: ESTOP latched (press RESET)\r\n");
            return;
        }
        if (!s_armed) {
            vcp_text("# refused: LOCKED (servo not answering or front pose not taught)\r\n");
            return;
        }
    }
    switch (c) {
    case 'p': if (idle) { s_axis = STS_PAN_ID;  vcp_text("# axis pan (ID 2)\r\n"); }  break;
    case 't': if (idle) { s_axis = STS_TILT_ID; vcp_text("# axis tilt (ID 1)\r\n"); } break;
    case 'M':
        if (idle) {
            s_max_speed = (uint8_t)!s_max_speed;
            vcp_text(s_max_speed ? "# goal speed 0 (max)\r\n" : "# goal speed default\r\n");
        }
        break;
    case 'S': if (idle) { s_start_req = SYSID_PROF_STEP;  vcp_text("# start step\r\n"); }  break;
    case 'R': if (idle) { s_start_req = SYSID_PROF_PRBS;  vcp_text("# start prbs\r\n"); }  break;
    case 'C': if (idle) { s_start_req = SYSID_PROF_CHIRP; vcp_text("# start chirp\r\n"); } break;
    case 'X': s_stop_req = 1u; vcp_text("# stop\r\n"); break;
    default: break;
    }
}

static void estop_poll(void)
{
    static uint8_t  armed = 0u;      /* B1 seen released once */
    static uint32_t t_low = 0u;
    if (s_estop) {
        return;
    }
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
        armed = 1u;
        t_low = 0u;
        return;
    }
    if (!armed) {
        return;
    }
    if (t_low == 0u) {
        t_low = HAL_GetTick() | 1u;
    } else if (HAL_GetTick() - t_low >= B1_DEBOUNCE_MS) {
        s_estop = 1u;
        vcp_text("# ESTOP: B1 pressed, torque off. Press RESET to resume.\r\n");
    }
}

void SysId_Poll(void)
{
    estop_poll();
    if (s_abort_note) {
        s_abort_note = 0u;
        vcp_text("# ABORT: load > 80 % for 0.1 s (stall / hit the frame). Back to centre.\r\n");
    }

    /* commands: polled RXNE, no interrupt needed at this rate */
    USART_TypeDef *u = huart2.Instance;
    const uint32_t sr = u->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        const uint8_t c = (uint8_t)u->DR;      /* reading DR also clears ORE */
        if (sr & USART_SR_RXNE) {
            handle_cmd(c);
        }
    }

    /* records: up to 8 per call, contiguous in the ring */
    const uint32_t head = s_head;
    uint32_t tail = s_tail;
    if (head != tail) {
        uint32_t n = head - tail;
        const uint32_t idx = tail & (RING_N - 1u);
        if (n > RING_N - idx) { n = RING_N - idx; }
        if (n > 8u) { n = 8u; }
        (void)HAL_UART_Transmit(&huart2, s_ring[idx], (uint16_t)(n * SYSID_RECORD_LEN), 50u);
        __DMB();
        s_tail = tail + n;
    }
}
#endif /* RCWS_SYSID && !RCWS_HOST_TEST */
