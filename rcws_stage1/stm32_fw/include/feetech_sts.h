/**
 * feetech_sts.h — Feetech STS3215 bus driver (USART1, 1 Mbps)
 *
 * Team wiring (hyunjinscode ball5_2_best.ino): USART1 PA9 (TX) / PA10 (RX) to the bus adapter
 * (FE-URT-1, automatic direction). TX DMA = DMA2_Stream7 / DMA_CHANNEL_4 (normal mode).
 * PA4   : half-duplex direction for a manual-direction adapter (HIGH = transmit, LOW = receive /
 *         idle); still driven, harmless when not connected.
 * IDs   : team assignment, Pan = 2, Tilt = 1.
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
#define STS_INST_READ           0x02u
#define STS_INST_WRITE          0x03u
#define STS_INST_SYNC_WRITE     0x83u
#define STS_BROADCAST_ID        0xFEu

/* Control table */
#define STS_ADDR_ID             0x05u
#define STS_ADDR_GOAL_POSITION  0x2Au   /* pos(2) + time(2) + speed(2) */
#define STS_ADDR_LOCK           0x37u   /* EEPROM lock: 0 = unlocked, 1 = locked */
#define STS_ADDR_TORQUE_ENABLE  0x28u   /* 40: 0 = limp, 1 = holding */
#define STS_ADDR_PRESENT_POS    0x38u   /* 56: present position (2 bytes) */
#define STS_ADDR_MODE           0x21u   /* 33: 0 = position, 1 = wheel (team firmware used 1) */
#define STS_TORQUE_CALIB_MID    128u    /* written to address 40: current position becomes 2048 */

/* Bus layout */
#define STS_PAN_ID              2u      /* team assignment: ID 2 = Pan (left-right) */
#define STS_TILT_ID             1u      /* ID 1 = Tilt (up-down) */
#define STS_DEFAULT_SPEED       3400u
#define STS_POS_MAX             4095u

#define STS_SYNC_WRITE_LEN      22u     /* 2 servos x (pos,time,speed) */
#define STS_SYNC_WRITE_LENGTH   0x12u   /* (6 + 1) * 2 + 4 = 18 */
#define STS_WRITE_BYTE_LEN      8u
#define STS_SYNC_TORQUE_LEN     12u     /* 2 servos x 1 byte */

#define STS_DIR_PORT            GPIOA
#define STS_DIR_PIN             GPIO_PIN_4

/** Pure builder: fills buf[22]. Returns number of bytes (22). */
uint8_t STS_BuildSyncWritePosSpeed(uint8_t *buf,
                                   uint16_t pan_pos, uint16_t pan_speed,
                                   uint16_t tilt_pos, uint16_t tilt_speed);

/** Pure builder: SYNC WRITE torque enable (both servos) into buf[12]. Returns 12. */
uint8_t STS_BuildSyncTorque(uint8_t *buf, uint8_t enable);

/** Pure builder: single-byte WRITE (0x03) packet into buf[8]. Returns 8. */
uint8_t STS_BuildWriteByte(uint8_t *buf, uint8_t id, uint8_t addr, uint8_t value);

#define STS_READ_POS_LEN        8u
/** Pure builder: READ present position (2 bytes) of one servo into buf[8]. Returns 8. */
uint8_t STS_BuildReadPos(uint8_t *buf, uint8_t id);

/**
 * Pure parser: find the status reply of `id` to STS_BuildReadPos in bytes received from the bus.
 * The FE-URT-1 adapter may echo the request on RX; a frame identical to the request is skipped.
 * Returns 1 and the position when a checksum-valid reply is found.
 */
int STS_ParseReadPosReply(const uint8_t *rx, uint32_t n, uint8_t id, uint16_t *pos);

#ifndef RCWS_HOST_TEST
extern UART_HandleTypeDef huart1;
#define STS_HUART               huart1
#define STS_USART               USART1

/** Non-blocking DMA SYNC WRITE (ISR-safe). Skips if the previous frame is still in flight. */
void STS_SyncWrite_PosSpeed(uint16_t pan_pos, uint16_t pan_speed,
                            uint16_t tilt_pos, uint16_t tilt_speed);

/** Non-blocking DMA SYNC WRITE of torque enable for both servos (ISR-safe, skipped when busy). */
void STS_SyncWrite_Torque(uint8_t enable);

/** Non-blocking single-byte WRITE via DMA (TIM2 context). Returns 1 if sent, 0 if the bus was busy. */
uint8_t STS_WriteByteDMA(uint8_t id, uint8_t addr, uint8_t value);

/**
 * Blocking one-time ID commissioning (thread mode only, before TIM2 starts).
 * Connect ONLY the servo to be renamed to the bus while running this.
 */
HAL_StatusTypeDef STS_Assign_ID(uint8_t old_id, uint8_t new_id);

/**
 * Blocking boot step (thread mode, before TIM2 starts): torque off, operating mode 0 (position)
 * on both servos, torque on. The team firmware left the servos in wheel mode (mode 1); the write
 * goes to RAM only (EEPROM stays locked), so it is repeated at every boot.
 */
HAL_StatusTypeDef STS_InitPositionMode(void);

/**
 * Blocking boot self-check (thread mode, before TIM2 starts): reads the present position of each
 * servo. Returns a bit mask (bit0 Pan answered, bit1 Tilt answered); positions in *pan / *tilt.
 * Not used in the sysid build (its RX callback owns the bus receiver).
 */
uint8_t STS_BootPing(uint16_t *pan, uint16_t *tilt);

/** Called after PA4 returns low at the end of every bus transmission (weak no-op). */
void STS_TxDoneHook(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FEETECH_STS_H */
