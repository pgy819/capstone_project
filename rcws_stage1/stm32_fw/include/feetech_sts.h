/**
 * feetech_sts.h — Feetech STS3215 bus driver (USART6, 1 Mbps, half-duplex via PA4)
 *
 * USART6: PC6 (TX) / PC7 (RX), TX DMA = DMA2_Stream6 / DMA_CHANNEL_5 (normal mode)
 * PA4   : half-duplex direction (HIGH = transmit, LOW = receive / idle)
 *
 * Define RCWS_HOST_TEST to build only the pure packet builders on a PC.
 */
#ifndef FEETECH_STS_H
#define FEETECH_STS_H

#include <stdint.h>

#ifndef RCWS_HOST_TEST
#include "stm32f4xx_hal.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Instructions */
#define STS_INST_WRITE          0x03u
#define STS_INST_SYNC_WRITE     0x83u
#define STS_BROADCAST_ID        0xFEu

/* Control table */
#define STS_ADDR_ID             0x05u
#define STS_ADDR_GOAL_POSITION  0x2Au   /* pos(2) + time(2) + speed(2) */
#define STS_ADDR_LOCK           0x37u   /* EEPROM lock: 0 = unlocked, 1 = locked */

/* Bus layout */
#define STS_PAN_ID              1u
#define STS_TILT_ID             2u
#define STS_DEFAULT_SPEED       3400u
#define STS_POS_MAX             4095u

#define STS_SYNC_WRITE_LEN      22u     /* 2 servos x (pos,time,speed) */
#define STS_SYNC_WRITE_LENGTH   0x12u   /* (6 + 1) * 2 + 4 = 18 */
#define STS_WRITE_BYTE_LEN      8u

#define STS_DIR_PORT            GPIOA
#define STS_DIR_PIN             GPIO_PIN_4

/** Pure builder: fills buf[22]. Returns number of bytes (22). */
uint8_t STS_BuildSyncWritePosSpeed(uint8_t *buf,
                                   uint16_t pan_pos, uint16_t pan_speed,
                                   uint16_t tilt_pos, uint16_t tilt_speed);

/** Pure builder: single-byte WRITE (0x03) packet into buf[8]. Returns 8. */
uint8_t STS_BuildWriteByte(uint8_t *buf, uint8_t id, uint8_t addr, uint8_t value);

#ifndef RCWS_HOST_TEST
extern UART_HandleTypeDef huart6;

/** Non-blocking DMA SYNC WRITE (ISR-safe). Skips if the previous frame is still in flight. */
void STS_SyncWrite_PosSpeed(uint16_t pan_pos, uint16_t pan_speed,
                            uint16_t tilt_pos, uint16_t tilt_speed);

/**
 * Blocking one-time ID commissioning (thread mode only, before TIM2 starts).
 * Connect ONLY the servo to be renamed to the bus while running this.
 */
HAL_StatusTypeDef STS_Assign_ID(uint8_t old_id, uint8_t new_id);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FEETECH_STS_H */
