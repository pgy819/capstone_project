/**
 * link_proto.h — Raspberry Pi 5 <-> STM32 link, packet v2 (single source of truth)
 *
 * Compiled by both sides: the STM32 firmware (src/link_proto.c) and the RPi runtime
 * (rpi/CMakeLists.txt adds ../stm32_fw/src/link_proto.c). Pure C99, no HAL, no allocation.
 *
 * Physical: STM32 USART6 (PC6 TX -> Pi RXD GPIO15, PC7 RX <- Pi TXD GPIO14), 115200 8N1, GND common.
 *           Same pins and baud as the team firmware (hyunjinscode ball5_2_best.ino); the ASCII
 *           "err_x,err_y\n" payload is replaced by the fixed binary frames below.
 *
 * Every frame is 16 bytes, little-endian, CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over
 * bytes 2..13 (type .. last payload byte), stored low byte first in bytes 14..15.
 *
 *  Downlink AIM (Pi -> STM32), sent once per camera frame (~60 Hz, 1.39 ms on the wire):
 *    0  0xAA  sync
 *    1  0x55  sync (downlink)
 *    2  0x21  type AIM
 *    3  seq        u8   frame counter (wraps); echoed in the uplink
 *    4  mode       u8   0 HOLD / 1 FOLLOW / 2 PREDICT   (perception aim gate)
 *                       3 JOG  (operator: rate_* is the commanded gimbal rate, err_* ignored)
 *                       4 HOME (operator: slow return to LINK_PAN_HOME / LINK_TILT_HOME)
 *                       5 SET_HOME (operator, only while the servos are limp in a software ESTOP:
 *                         store the current pose as each servo's centre 2048 = Feetech mid-position
 *                         calibration, write 128 to address 40; kept by the servo across power-off)
 *    5  flags      u8   bit0 OPERATOR_HOLD (web "manual": brake and hold, overrides FOLLOW/PREDICT)
 *                       bit1 ESTOP_REQ   (software emergency stop: torque off, latched)
 *                       bit2 ESTOP_CLEAR (release a software ESTOP; B1 ESTOP still needs RESET)
 *    6  err_pan    i16  target bearing relative to the camera axis, +right, [1/16 servo count]
 *    8  err_tilt   i16  target elevation relative to the camera axis, +up,  [1/16 servo count]
 *   10  rate_pan   i16  target angular rate, +right [servo count/s]  (feed-forward, U7)
 *   12  rate_tilt  i16  target angular rate, +up    [servo count/s]
 *   14  crc16
 *
 *  Uplink STATUS (STM32 -> Pi), 50 Hz:
 *    0  0xAA, 1 0x56, 2 0x81 type STATUS
 *    3  seq_echo   u8   seq of the last valid AIM frame
 *    4  guard      u8   GuardState_t: 0 RUN 1 HOLD 2 TORQUE_OFF 3 RAMP 4 ESTOP
 *    5  flags      u8   bit0 ESTOP_B1, bit1 COMM_LOST (>0.5 s without AIM), bit2 ESTOP_SW,
 *                       bit3 OPERATOR_HOLD active, bit4 PAN servo answered the boot PING,
 *                       bit5 TILT servo answered, bit6 boot PING done,
 *                       bit7 SET_HOME written since boot
 *    6  pan_goal   u16  current goal [servo count]
 *    8  tilt_goal  u16
 *   10  rx_ok      u16  valid AIM frames (wraps)
 *   12  rx_err     u16  CRC / range failures (wraps)
 *   14  crc16
 *
 * Units: 1 servo count = 2*pi/4096 rad = 0.08789 deg. err_* covers +-2048 counts (+-180 deg) at
 * 1/16 count resolution (0.096 mrad), well below one camera pixel (~0.8 mrad).
 */
#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LINK_SYNC0            0xAAu
#define LINK_SYNC1_DOWN       0x55u
#define LINK_SYNC1_UP         0x56u
#define LINK_FRAME_LEN        16u
#define LINK_TYPE_AIM         0x21u
#define LINK_TYPE_STATUS      0x81u
#define LINK_BAUD_DEFAULT     115200u

#define LINK_ERR_SCALE        16      /* err_* LSB = 1/16 servo count */
#define LINK_COUNTS_PER_REV   4096.0
#define LINK_SERVO_CENTER     2048u
/* Home = the gimbal facing front. The front pose is taught once (web "Set home", mode SET_HOME):
   each servo then reports 2048 there, so the travel limits are symmetric around it and never
   cross the 0 / 4095 seam of the single-turn position range. */
#define LINK_PAN_HOME         LINK_SERVO_CENTER
#define LINK_TILT_HOME        LINK_SERVO_CENTER

#define LINK_MODE_HOLD        0u
#define LINK_MODE_FOLLOW      1u
#define LINK_MODE_PREDICT     2u
#define LINK_MODE_JOG         3u
#define LINK_MODE_HOME        4u
#define LINK_MODE_SET_HOME    5u
#define LINK_MODE_MAX         LINK_MODE_SET_HOME

/* downlink flags */
#define LINK_F_OPERATOR_HOLD  0x01u
#define LINK_F_ESTOP_REQ      0x02u
#define LINK_F_ESTOP_CLEAR    0x04u

/* uplink flags */
#define LINK_S_ESTOP_B1       0x01u
#define LINK_S_COMM_LOST      0x02u
#define LINK_S_ESTOP_SW       0x04u
#define LINK_S_OPERATOR_HOLD  0x08u
#define LINK_S_PAN_PING_OK    0x10u
#define LINK_S_TILT_PING_OK   0x20u
#define LINK_S_PING_DONE      0x40u
#define LINK_S_HOME_SET       0x80u

typedef struct {
    uint8_t seq;
    uint8_t mode;
    uint8_t flags;
    int16_t err_pan;     /* 1/16 count, +right */
    int16_t err_tilt;    /* 1/16 count, +up */
    int16_t rate_pan;    /* count/s */
    int16_t rate_tilt;
} LinkAim_t;

typedef struct {
    uint8_t  seq_echo;
    uint8_t  guard;
    uint8_t  flags;
    uint16_t pan_goal;
    uint16_t tilt_goal;
    uint16_t rx_ok;
    uint16_t rx_err;
} LinkStatus_t;

/** Sliding-window frame finder: any byte slip, garbage or truncated frame resynchronises at the
 *  next valid header; a valid frame is reported exactly once. */
typedef struct {
    uint8_t  win[LINK_FRAME_LEN];
    uint8_t  n;           /* bytes in the window */
    uint8_t  sync1;       /* LINK_SYNC1_DOWN or LINK_SYNC1_UP */
    uint32_t frames;      /* frames with a valid CRC */
    uint32_t crc_err;     /* header seen in place but CRC wrong */
} LinkParser_t;

uint16_t Link_Crc16(const uint8_t *p, uint32_t n);

void Link_EncodeAim(uint8_t out[LINK_FRAME_LEN], const LinkAim_t *a);
void Link_EncodeStatus(uint8_t out[LINK_FRAME_LEN], const LinkStatus_t *s);

/** 1 when frame[] is a valid AIM frame (type and mode checked); fills *a. */
int Link_DecodeAim(const uint8_t frame[LINK_FRAME_LEN], LinkAim_t *a);
/** 1 when frame[] is a valid STATUS frame; fills *s. */
int Link_DecodeStatus(const uint8_t frame[LINK_FRAME_LEN], LinkStatus_t *s);

void Link_ParserInit(LinkParser_t *p, uint8_t sync1);
/** Feed one byte. Returns 1 and copies the frame to out[] when the window holds a CRC-valid
 *  frame with this parser's sync bytes; the window is then emptied. */
int Link_ParseByte(LinkParser_t *p, uint8_t b, uint8_t out[LINK_FRAME_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* LINK_PROTO_H */
