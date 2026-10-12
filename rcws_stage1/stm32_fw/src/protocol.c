/**
 * protocol.c — 64-byte DMA ring buffer + Byte-Slip / Sticky-0xFF immune FSM parser
 */
#include "protocol.h"

#ifdef RCWS_HOST_TEST
#define __disable_irq() ((void)0)
#define __enable_irq()  ((void)0)
#endif

/* ------------------------------------------------------------------------- */
/* Shared state                                                              */
/* ------------------------------------------------------------------------- */
uint8_t dma_rx_buf[PROTO_RX_BUF_SIZE] __attribute__((aligned(4)));

volatile VisionPacket_t g_vision_pkt     = { VISION_STATUS_LOST, 0, 0 };
volatile uint8_t        g_new_pkt_flag   = 0u;
/* Start in "timed-out" state: no motion until the first valid packet. */
volatile uint32_t       g_comm_watchdog  = COMM_WATCHDOG_LIMIT + 1u;

volatile uint32_t g_proto_ok_count  = 0u;
volatile uint32_t g_proto_err_count = 0u;

/* ------------------------------------------------------------------------- */
/* FSM state (main-loop context only)                                        */
/* ------------------------------------------------------------------------- */
static ProtoState_t s_state = STATE_WAIT_FF;
static uint8_t      s_payload[PROTO_PAYLOAD_SIZE];
static uint8_t      s_payload_idx = 0u;

void Protocol_Reset(void)
{
    s_state = STATE_WAIT_FF;
    s_payload_idx = 0u;
}

void Protocol_ParseByte(uint8_t b)
{
    switch (s_state) {
    case STATE_WAIT_FF:
        if (b == PROTO_HEADER1) {
            s_state = STATE_WAIT_FE;
        }
        break;

    case STATE_WAIT_FE:
        if (b == PROTO_HEADER2) {
            s_state = STATE_PAYLOAD;
            s_payload_idx = 0u;
        } else if (b == PROTO_HEADER1) {
            /* Sticky 0xFF: "FF FF FE ..." -> the latest 0xFF may be the real header. */
            s_state = STATE_WAIT_FE;
        } else {
            s_state = STATE_WAIT_FF;
        }
        break;

    case STATE_PAYLOAD:
        s_payload[s_payload_idx++] = b;
        if (s_payload_idx >= PROTO_PAYLOAD_SIZE) {
            const uint8_t sum = (uint8_t)((s_payload[0] + s_payload[1] + s_payload[2] +
                                           s_payload[3] + s_payload[4]) & 0xFFu);
            if (sum == s_payload[5]) {
                const uint8_t status = s_payload[0];
                const int16_t ex = (int16_t)((uint16_t)s_payload[1] | ((uint16_t)s_payload[2] << 8));
                const int16_t ey = (int16_t)((uint16_t)s_payload[3] | ((uint16_t)s_payload[4] << 8));

                /* Extra sanity gate against checksum false-positives on slipped bytes. */
                if (status <= VISION_STATUS_COASTING &&
                    ex >= -VISION_ERR_X_LIMIT && ex <= VISION_ERR_X_LIMIT &&
                    ey >= -VISION_ERR_Y_LIMIT && ey <= VISION_ERR_Y_LIMIT) {
                    __disable_irq();
                    g_vision_pkt.status = status;
                    g_vision_pkt.err_x  = ex;
                    g_vision_pkt.err_y  = ey;
                    g_new_pkt_flag      = 1u;
                    g_comm_watchdog     = 0u;
                    __enable_irq();
                    g_proto_ok_count++;
                } else {
                    g_proto_err_count++;
                }
            } else {
                g_proto_err_count++;
            }
            s_state = STATE_WAIT_FF;
            s_payload_idx = 0u;
        }
        break;

    default:
        Protocol_Reset();
        break;
    }
}

#ifndef RCWS_HOST_TEST
/* ------------------------------------------------------------------------- */
/* DMA ring-buffer front end (target only)                                   */
/* ------------------------------------------------------------------------- */
static UART_HandleTypeDef *s_huart = 0;
static uint32_t            s_tail = 0u;
static volatile uint8_t    s_restart_req = 0u;

HAL_StatusTypeDef Protocol_StartRx(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_tail = 0u;
    s_restart_req = 0u;
    Protocol_Reset();
    return HAL_UART_Receive_DMA(huart, dma_rx_buf, (uint16_t)PROTO_RX_BUF_SIZE);
}

void Protocol_RequestRxRestart(void)
{
    s_restart_req = 1u;
}

void Protocol_Poll(void)
{
    if (s_huart == 0) {
        return;
    }

    /* HAL aborts circular DMA on UART errors (ORE/FE/NE) — restart from thread mode. */
    if (s_restart_req || s_huart->RxState != HAL_UART_STATE_BUSY_RX) {
        (void)HAL_UART_AbortReceive(s_huart);
        (void)Protocol_StartRx(s_huart);
        return;
    }

    const uint32_t dma_head =
        (PROTO_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(s_huart->hdmarx)) & PROTO_RX_BUF_MASK;

    while (s_tail != dma_head) {
        Protocol_ParseByte(dma_rx_buf[s_tail]);
        s_tail = (s_tail + 1u) & PROTO_RX_BUF_MASK;
    }
}
#else
/* ------------------------------------------------------------------------- */
/* Host test helper                                                          */
/* ------------------------------------------------------------------------- */
uint8_t Protocol_HostTakePacket(uint8_t *status, int16_t *err_x, int16_t *err_y)
{
    if (!g_new_pkt_flag) {
        return 0u;
    }
    *status = g_vision_pkt.status;
    *err_x  = g_vision_pkt.err_x;
    *err_y  = g_vision_pkt.err_y;
    g_new_pkt_flag = 0u;
    return 1u;
}
#endif
