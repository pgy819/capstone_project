/**
 * protocol.c — 128-byte DMA ring + packet v2 frame finder (link_proto.c) + uplink STATUS
 */
#include "protocol.h"

#include <stdlib.h>

#ifdef RCWS_HOST_TEST
#define __disable_irq() ((void)0)
#define __enable_irq()  ((void)0)
#endif

/* ------------------------------------------------------------------------- */
/* Shared state                                                              */
/* ------------------------------------------------------------------------- */
uint8_t dma_rx_buf[PROTO_RX_BUF_SIZE] __attribute__((aligned(4)));

volatile VisionPacket_t g_vision_pkt     = { 0u, LINK_MODE_HOLD, 0u, 0, 0, 0, 0 };
volatile uint8_t        g_new_pkt_flag   = 0u;
/* Start in "timed-out" state: no motion until the first valid packet. */
volatile uint32_t       g_comm_watchdog  = COMM_WATCHDOG_LIMIT + 1u;

volatile uint32_t g_proto_ok_count  = 0u;
volatile uint32_t g_proto_err_count = 0u;

/* ------------------------------------------------------------------------- */
/* Frame finder (main-loop context only)                                     */
/* ------------------------------------------------------------------------- */
static LinkParser_t s_parser;
static uint32_t     s_crc_err_seen = 0u;

void Protocol_Reset(void)
{
    Link_ParserInit(&s_parser, LINK_SYNC1_DOWN);
    s_crc_err_seen = 0u;
}

void Protocol_ParseByte(uint8_t b)
{
    uint8_t f[LINK_FRAME_LEN];
    if (s_parser.sync1 != LINK_SYNC1_DOWN) {
        Protocol_Reset();     /* first call before StartRx (host tests) */
    }
    const int got = Link_ParseByte(&s_parser, b, f);
    if (s_parser.crc_err != s_crc_err_seen) {
        g_proto_err_count += s_parser.crc_err - s_crc_err_seen;
        s_crc_err_seen = s_parser.crc_err;
    }
    if (!got) {
        return;
    }
    LinkAim_t a;
    if (!Link_DecodeAim(f, &a) ||
        abs(a.err_pan) > VISION_ERR_LIMIT || abs(a.err_tilt) > VISION_ERR_LIMIT ||
        abs(a.rate_pan) > VISION_RATE_LIMIT || abs(a.rate_tilt) > VISION_RATE_LIMIT) {
        g_proto_err_count++;
        return;
    }
    __disable_irq();
    g_vision_pkt.seq       = a.seq;
    g_vision_pkt.mode      = a.mode;
    g_vision_pkt.flags     = a.flags;
    g_vision_pkt.err_pan   = a.err_pan;
    g_vision_pkt.err_tilt  = a.err_tilt;
    g_vision_pkt.rate_pan  = a.rate_pan;
    g_vision_pkt.rate_tilt = a.rate_tilt;
    g_new_pkt_flag         = 1u;
    g_comm_watchdog        = 0u;
    __enable_irq();
    g_proto_ok_count++;
}

#ifndef RCWS_HOST_TEST
/* ------------------------------------------------------------------------- */
/* DMA ring-buffer front end (target only)                                   */
/* ------------------------------------------------------------------------- */
static UART_HandleTypeDef *s_huart = 0;
static uint32_t            s_tail = 0u;
static volatile uint8_t    s_restart_req = 0u;
static uint8_t             s_tx[LINK_FRAME_LEN];   /* static: valid during the DMA transfer */

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

    /* HAL aborts circular DMA on UART errors (ORE/FE/NE) — restart from thread mode.
       AbortReceive leaves an uplink transmission in flight untouched. */
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

void Protocol_SendStatus(const LinkStatus_t *s)
{
    if (s_huart == 0 || s_huart->gState != HAL_UART_STATE_READY) {
        return;
    }
    Link_EncodeStatus(s_tx, s);
    (void)HAL_UART_Transmit_DMA(s_huart, s_tx, (uint16_t)LINK_FRAME_LEN);
}
#else
/* ------------------------------------------------------------------------- */
/* Host test helper                                                          */
/* ------------------------------------------------------------------------- */
uint8_t Protocol_HostTakePacket(VisionPacket_t *out)
{
    if (!g_new_pkt_flag) {
        return 0u;
    }
    out->seq       = g_vision_pkt.seq;
    out->mode      = g_vision_pkt.mode;
    out->flags     = g_vision_pkt.flags;
    out->err_pan   = g_vision_pkt.err_pan;
    out->err_tilt  = g_vision_pkt.err_tilt;
    out->rate_pan  = g_vision_pkt.rate_pan;
    out->rate_tilt = g_vision_pkt.rate_tilt;
    g_new_pkt_flag = 0u;
    return 1u;
}
#endif
