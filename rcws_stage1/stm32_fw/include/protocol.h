/**
 * protocol.h — Pi 5 -> STM32 8-byte vision packet (Immutable Data Contract)
 *
 *   Byte 0   : 0xFF            Header 1
 *   Byte 1   : 0xFE            Header 2
 *   Byte 2   : uint8_t status  0x00 lost / 0x01 detected / 0x02 coasting
 *   Byte 3-4 : int16_t err_x   little-endian, -960..+960
 *   Byte 5-6 : int16_t err_y   little-endian, -540..+540
 *   Byte 7   : uint8_t checksum = (status + exL + exH + eyL + eyH) & 0xFF
 *
 * Reception: USART1 RX -> DMA2_Stream2 / DMA_CHANNEL_4 (circular) -> dma_rx_buf[64]
 * Parsing  : byte-by-byte FSM (WAIT_FF -> WAIT_FE -> PAYLOAD), sticky-0xFF safe.
 *
 * Define RCWS_HOST_TEST to build the pure parser on a PC (ctypes cross-check).
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#ifndef RCWS_HOST_TEST
#include "stm32f4xx_hal.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_HEADER1          0xFFu
#define PROTO_HEADER2          0xFEu
#define PROTO_PACKET_SIZE      8u
#define PROTO_PAYLOAD_SIZE     6u      /* status, exL, exH, eyL, eyH, checksum */
#define PROTO_RX_BUF_SIZE      64u
#define PROTO_RX_BUF_MASK      (PROTO_RX_BUF_SIZE - 1u)

#define VISION_STATUS_LOST      0x00u
#define VISION_STATUS_DETECTED  0x01u
#define VISION_STATUS_COASTING  0x02u

#define VISION_ERR_X_LIMIT      960
#define VISION_ERR_Y_LIMIT      540

/** 500 Hz ticks without a valid packet before safety hold (250 * 2 ms = 500 ms). */
#define COMM_WATCHDOG_LIMIT     250u

/** Wire layout — documentation/size check ONLY. Never cast the DMA buffer to it. */
typedef struct __attribute__((packed)) {
    uint8_t header1;
    uint8_t header2;
    uint8_t status;
    int16_t err_x;
    int16_t err_y;
    uint8_t checksum;
} VisionWirePacket_t;

_Static_assert(sizeof(VisionWirePacket_t) == PROTO_PACKET_SIZE, "Vision packet must be 8 bytes");

/** Decoded packet shared with TIM2_IRQHandler. */
typedef struct {
    uint8_t status;
    int16_t err_x;
    int16_t err_y;
} VisionPacket_t;

typedef enum {
    STATE_WAIT_FF = 0,
    STATE_WAIT_FE,
    STATE_PAYLOAD
} ProtoState_t;

/* Shared with TIM2_IRQHandler -> volatile; written inside a critical section. */
extern volatile VisionPacket_t g_vision_pkt;
extern volatile uint8_t        g_new_pkt_flag;
extern volatile uint32_t       g_comm_watchdog;

/* Diagnostics */
extern volatile uint32_t g_proto_ok_count;
extern volatile uint32_t g_proto_err_count;

extern uint8_t dma_rx_buf[PROTO_RX_BUF_SIZE];

/** Reset FSM state (does not touch shared packet). */
void Protocol_Reset(void);

/** Feed one received byte through the FSM. */
void Protocol_ParseByte(uint8_t b);

#ifndef RCWS_HOST_TEST
/** Start USART1 circular DMA reception into dma_rx_buf. */
HAL_StatusTypeDef Protocol_StartRx(UART_HandleTypeDef *huart);

/** Call continuously from main while(1): drains DMA ring up to dma_head. */
void Protocol_Poll(void);

/** Called from HAL_UART_ErrorCallback (ISR) — DMA restart is deferred to Protocol_Poll. */
void Protocol_RequestRxRestart(void);
#else
/** Host test helper: returns 1 and pops the packet if g_new_pkt_flag was set. */
uint8_t Protocol_HostTakePacket(uint8_t *status, int16_t *err_x, int16_t *err_y);
#endif

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
