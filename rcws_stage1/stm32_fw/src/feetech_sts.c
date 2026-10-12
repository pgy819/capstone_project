/**
 * feetech_sts.c — STS3215 22-byte SYNC WRITE (DMA) + ID commissioning
 */
#include "feetech_sts.h"

/* ------------------------------------------------------------------------- */
/* Pure packet builders (host-testable)                                      */
/* ------------------------------------------------------------------------- */
uint8_t STS_BuildSyncWritePosSpeed(uint8_t *buf,
                                   uint16_t pan_pos, uint16_t pan_speed,
                                   uint16_t tilt_pos, uint16_t tilt_speed)
{
    if (pan_pos > STS_POS_MAX)  { pan_pos = STS_POS_MAX; }
    if (tilt_pos > STS_POS_MAX) { tilt_pos = STS_POS_MAX; }

    buf[0]  = 0xFFu;
    buf[1]  = 0xFFu;
    buf[2]  = STS_BROADCAST_ID;           /* 0xFE */
    buf[3]  = STS_SYNC_WRITE_LENGTH;      /* 0x12 */
    buf[4]  = STS_INST_SYNC_WRITE;        /* 0x83 */
    buf[5]  = STS_ADDR_GOAL_POSITION;     /* 0x2A */
    buf[6]  = 0x06u;                      /* data length per servo */
    /* Servo 1 — Pan */
    buf[7]  = STS_PAN_ID;
    buf[8]  = (uint8_t)(pan_pos & 0xFFu);
    buf[9]  = (uint8_t)(pan_pos >> 8);
    buf[10] = 0x00u;                      /* goal time L */
    buf[11] = 0x00u;                      /* goal time H */
    buf[12] = (uint8_t)(pan_speed & 0xFFu);
    buf[13] = (uint8_t)(pan_speed >> 8);
    /* Servo 2 — Tilt */
    buf[14] = STS_TILT_ID;
    buf[15] = (uint8_t)(tilt_pos & 0xFFu);
    buf[16] = (uint8_t)(tilt_pos >> 8);
    buf[17] = 0x00u;
    buf[18] = 0x00u;
    buf[19] = (uint8_t)(tilt_speed & 0xFFu);
    buf[20] = (uint8_t)(tilt_speed >> 8);

    uint32_t sum = 0u;
    for (uint8_t i = 2u; i <= 20u; i++) {
        sum += buf[i];
    }
    buf[21] = (uint8_t)(~sum & 0xFFu);
    return (uint8_t)STS_SYNC_WRITE_LEN;
}

uint8_t STS_BuildWriteByte(uint8_t *buf, uint8_t id, uint8_t addr, uint8_t value)
{
    buf[0] = 0xFFu;
    buf[1] = 0xFFu;
    buf[2] = id;
    buf[3] = 0x04u;               /* params(2) + 2 */
    buf[4] = STS_INST_WRITE;
    buf[5] = addr;
    buf[6] = value;
    buf[7] = (uint8_t)(~(uint32_t)(buf[2] + buf[3] + buf[4] + buf[5] + buf[6]) & 0xFFu);
    return (uint8_t)STS_WRITE_BYTE_LEN;
}

#ifndef RCWS_HOST_TEST
/* ------------------------------------------------------------------------- */
/* Target-only transport                                                     */
/* ------------------------------------------------------------------------- */

/* static: must stay valid in memory during the asynchronous DMA transfer. */
static uint8_t tx_buf[STS_SYNC_WRITE_LEN];

void STS_SyncWrite_PosSpeed(uint16_t pan_pos, uint16_t pan_speed,
                            uint16_t tilt_pos, uint16_t tilt_speed)
{
    /* Previous DMA frame still in flight -> never touch tx_buf. */
    if (huart6.gState != HAL_UART_STATE_READY) {
        return;
    }

    (void)STS_BuildSyncWritePosSpeed(tx_buf, pan_pos, pan_speed, tilt_pos, tilt_speed);

    HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_SET);      /* TX enable */
    if (HAL_UART_Transmit_DMA(&huart6, tx_buf, (uint16_t)STS_SYNC_WRITE_LEN) != HAL_OK) {
        HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
    }
}

static HAL_StatusTypeDef sts_write_byte_blocking(uint8_t id, uint8_t addr, uint8_t value)
{
    uint8_t pkt[STS_WRITE_BYTE_LEN];
    (void)STS_BuildWriteByte(pkt, id, addr, value);

    HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_SET);
    /* Blocking transmit returns only after UART_FLAG_TC (last stop bit shifted out). */
    HAL_StatusTypeDef st = HAL_UART_Transmit(&huart6, pkt, (uint16_t)sizeof(pkt), 20u);
    HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
    return st;
}

HAL_StatusTypeDef STS_Assign_ID(uint8_t old_id, uint8_t new_id)
{
    HAL_StatusTypeDef st;

    st = sts_write_byte_blocking(old_id, STS_ADDR_LOCK, 0u);     /* EEPROM unlock */
    if (st != HAL_OK) { return st; }
    HAL_Delay(15);

    st = sts_write_byte_blocking(old_id, STS_ADDR_ID, new_id);   /* write new ID */
    if (st != HAL_OK) { return st; }
    HAL_Delay(15);

    st = sts_write_byte_blocking(new_id, STS_ADDR_LOCK, 1u);     /* EEPROM lock (new ID) */
    HAL_Delay(15);
    return st;
}

/* PA4 back to LOW once the last byte has physically left the shift register. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
    }
}
#endif /* RCWS_HOST_TEST */
