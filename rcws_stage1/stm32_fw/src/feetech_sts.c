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

uint8_t STS_BuildSyncTorque(uint8_t *buf, uint8_t enable)
{
    buf[0]  = 0xFFu;
    buf[1]  = 0xFFu;
    buf[2]  = STS_BROADCAST_ID;
    buf[3]  = 0x08u;                      /* (1 + 1) * 2 + 4 */
    buf[4]  = STS_INST_SYNC_WRITE;
    buf[5]  = STS_ADDR_TORQUE_ENABLE;
    buf[6]  = 0x01u;                      /* data length per servo */
    buf[7]  = STS_PAN_ID;
    buf[8]  = enable ? 1u : 0u;
    buf[9]  = STS_TILT_ID;
    buf[10] = enable ? 1u : 0u;
    uint32_t sum = 0u;
    for (uint8_t i = 2u; i <= 10u; i++) {
        sum += buf[i];
    }
    buf[11] = (uint8_t)(~sum & 0xFFu);
    return (uint8_t)STS_SYNC_TORQUE_LEN;
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

uint8_t STS_BuildReadPos(uint8_t *buf, uint8_t id)
{
    buf[0] = 0xFFu;
    buf[1] = 0xFFu;
    buf[2] = id;
    buf[3] = 0x04u;               /* inst + addr + count + checksum */
    buf[4] = STS_INST_READ;
    buf[5] = STS_ADDR_PRESENT_POS;
    buf[6] = 0x02u;
    buf[7] = (uint8_t)(~(uint32_t)(buf[2] + buf[3] + buf[4] + buf[5] + buf[6]) & 0xFFu);
    return (uint8_t)STS_READ_POS_LEN;
}

int STS_ParseReadPosReply(const uint8_t *rx, uint32_t n, uint8_t id, uint16_t *pos)
{
    uint8_t req[STS_READ_POS_LEN];
    (void)STS_BuildReadPos(req, id);
    for (uint32_t i = 0u; i + 8u <= n; i++) {
        const uint8_t *f = &rx[i];
        if (f[0] != 0xFFu || f[1] != 0xFFu || f[2] != id || f[3] != 0x04u) {
            continue;
        }
        const uint8_t ck = (uint8_t)(~(uint32_t)(f[2] + f[3] + f[4] + f[5] + f[6]) & 0xFFu);
        if (ck != f[7]) {
            continue;
        }
        uint32_t same = 1u;
        for (uint32_t k = 0u; k < 8u; k++) {
            if (f[k] != req[k]) { same = 0u; break; }
        }
        if (same) {
            continue;                 /* our own request echoed by the adapter */
        }
        *pos = (uint16_t)(f[5] | ((uint16_t)f[6] << 8));
        return 1;
    }
    return 0;
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
    if (STS_HUART.gState != HAL_UART_STATE_READY) {
        return;
    }

    (void)STS_BuildSyncWritePosSpeed(tx_buf, pan_pos, pan_speed, tilt_pos, tilt_speed);

    HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_SET);      /* TX enable */
    if (HAL_UART_Transmit_DMA(&STS_HUART, tx_buf, (uint16_t)STS_SYNC_WRITE_LEN) != HAL_OK) {
        HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
    }
}

void STS_SyncWrite_Torque(uint8_t enable)
{
    if (STS_HUART.gState != HAL_UART_STATE_READY) {
        return;
    }
    (void)STS_BuildSyncTorque(tx_buf, enable);   /* tx_buf (22 B) is large enough for 12 B */
    HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_SET);
    if (HAL_UART_Transmit_DMA(&STS_HUART, tx_buf, (uint16_t)STS_SYNC_TORQUE_LEN) != HAL_OK) {
        HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
    }
}

uint8_t STS_WriteByteDMA(uint8_t id, uint8_t addr, uint8_t value)
{
    if (STS_HUART.gState != HAL_UART_STATE_READY) {
        return 0u;
    }
    (void)STS_BuildWriteByte(tx_buf, id, addr, value);
    HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_SET);
    if (HAL_UART_Transmit_DMA(&STS_HUART, tx_buf, (uint16_t)STS_WRITE_BYTE_LEN) != HAL_OK) {
        HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
        return 0u;
    }
    return 1u;
}

static HAL_StatusTypeDef sts_write_byte_blocking(uint8_t id, uint8_t addr, uint8_t value)
{
    uint8_t pkt[STS_WRITE_BYTE_LEN];
    (void)STS_BuildWriteByte(pkt, id, addr, value);

    HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_SET);
    /* Blocking transmit returns only after UART_FLAG_TC (last stop bit shifted out). */
    HAL_StatusTypeDef st = HAL_UART_Transmit(&STS_HUART, pkt, (uint16_t)sizeof(pkt), 20u);
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

HAL_StatusTypeDef STS_InitPositionMode(void)
{
    static const uint8_t ids[2] = { STS_PAN_ID, STS_TILT_ID };
    HAL_StatusTypeDef st = HAL_OK;
    for (uint32_t i = 0u; i < 2u; i++) {
        if (sts_write_byte_blocking(ids[i], STS_ADDR_TORQUE_ENABLE, 0u) != HAL_OK) { st = HAL_ERROR; }
        HAL_Delay(2);
        if (sts_write_byte_blocking(ids[i], STS_ADDR_MODE, 0u) != HAL_OK) { st = HAL_ERROR; }
        HAL_Delay(2);
        if (sts_write_byte_blocking(ids[i], STS_ADDR_TORQUE_ENABLE, 1u) != HAL_OK) { st = HAL_ERROR; }
        HAL_Delay(2);
    }
    return st;
}

uint8_t STS_BootPing(uint16_t *pan, uint16_t *tilt)
{
    static const uint8_t ids[2] = { STS_PAN_ID, STS_TILT_ID };
    uint16_t *out[2] = { pan, tilt };
    uint8_t ok = 0u;
    for (uint32_t i = 0u; i < 2u; i++) {
        uint8_t req[STS_READ_POS_LEN];
        static uint8_t rx[32];
        (void)STS_BuildReadPos(req, ids[i]);
        (void)HAL_UART_AbortReceive(&STS_HUART);
        /* armed before transmitting: catches an adapter echo and the reply (~16 B at 1 Mbps) */
        if (HAL_UART_Receive_IT(&STS_HUART, rx, (uint16_t)sizeof(rx)) != HAL_OK) {
            continue;
        }
        HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_SET);
        (void)HAL_UART_Transmit(&STS_HUART, req, (uint16_t)sizeof(req), 5u);
        HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
        HAL_Delay(4);                 /* reply: return delay (<= 0.5 ms) + 8 bytes at 1 Mbps */
        const uint32_t n = (uint32_t)sizeof(rx) - (uint32_t)STS_HUART.RxXferCount;
        (void)HAL_UART_AbortReceive(&STS_HUART);
        uint16_t pos = 0u;
        if (STS_ParseReadPosReply(rx, n, ids[i], &pos)) {
            ok |= (uint8_t)(1u << i);
            *out[i] = pos;
        }
    }
    return ok;
}

/* Overridden by sysid.c to arm the status-packet reception (weak: no-op in tracking mode). */
__attribute__((weak)) void STS_TxDoneHook(void)
{
}

/* PA4 back to LOW once the last byte has physically left the shift register. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == STS_USART) {
        HAL_GPIO_WritePin(STS_DIR_PORT, STS_DIR_PIN, GPIO_PIN_RESET);
        STS_TxDoneHook();
    }
}
#endif /* RCWS_HOST_TEST */
