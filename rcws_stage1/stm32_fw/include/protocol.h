/**
 * protocol.h — Pi 5 <-> STM32 vision link front end (packet v2, see link_proto.h)
 *
 * Reception : USART6 RX (PC7) -> DMA2_Stream1 / DMA_CHANNEL_5 (circular) -> dma_rx_buf[128]
 *             drained from the main loop into the resynchronising frame finder (Link_ParseByte).
 * Uplink    : STATUS frame every 20 ms from the main loop, USART6 TX (PC6) -> DMA2_Stream6 / CH5.
 * Pins/baud : team wiring (hyunjinscode): USART6 PC6/PC7 115200 8N1.
 *
 * The decoded AIM frame is handed to TIM2_IRQHandler through g_vision_pkt + g_new_pkt_flag
 * (written in a critical section, latest frame wins).
 *
 * Define RCWS_HOST_TEST to build the pure parser on a PC.
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#include "link_proto.h"

#ifndef RCWS_HOST_TEST
#include "stm32f4xx_hal.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_RX_BUF_SIZE      128u
#define PROTO_RX_BUF_MASK      (PROTO_RX_BUF_SIZE - 1u)

/* Sanity gate behind the CRC: the target cannot be further than +-62 deg off the camera axis
   (Camera Module 3 Wide HFOV 102 deg) and cannot move faster than 1500 deg/s. */
#define VISION_ERR_LIMIT       (LINK_ERR_SCALE * 700)
#define VISION_RATE_LIMIT      17000

/** 500 Hz ticks without a valid packet before safety hold (250 * 2 ms = 500 ms). */
#define COMM_WATCHDOG_LIMIT     250u

/** Decoded AIM frame shared with TIM2_IRQHandler. */
typedef struct {
    uint8_t seq;
    uint8_t mode;        /* LINK_MODE_* */
    uint8_t flags;       /* LINK_F_* */
    int16_t err_pan;     /* 1/16 count, +right */
    int16_t err_tilt;    /* 1/16 count, +up */
    int16_t rate_pan;    /* count/s */
    int16_t rate_tilt;
} VisionPacket_t;

/* Shared with TIM2_IRQHandler -> volatile; written inside a critical section. */
extern volatile VisionPacket_t g_vision_pkt;
extern volatile uint8_t        g_new_pkt_flag;
extern volatile uint32_t       g_comm_watchdog;

/* Diagnostics (sent in the uplink) */
extern volatile uint32_t g_proto_ok_count;
extern volatile uint32_t g_proto_err_count;

extern uint8_t dma_rx_buf[PROTO_RX_BUF_SIZE];

/** Reset the frame finder (does not touch the shared packet). */
void Protocol_Reset(void);

/** Feed one received byte. */
void Protocol_ParseByte(uint8_t b);

#ifndef RCWS_HOST_TEST
/** Start circular DMA reception into dma_rx_buf (huart = Pi link, USART6). */
HAL_StatusTypeDef Protocol_StartRx(UART_HandleTypeDef *huart);

/** Call continuously from main while(1): drains the DMA ring. */
void Protocol_Poll(void);

/** Called from HAL_UART_ErrorCallback (ISR) — DMA restart is deferred to Protocol_Poll. */
void Protocol_RequestRxRestart(void);

/** Non-blocking uplink STATUS frame (thread mode). Skipped while the previous one is in flight. */
void Protocol_SendStatus(const LinkStatus_t *s);
#else
/** Host test helper: returns 1 and pops the packet if g_new_pkt_flag was set. */
uint8_t Protocol_HostTakePacket(VisionPacket_t *out);
#endif

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
