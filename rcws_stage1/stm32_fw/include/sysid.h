/**
 * sysid.h — STS3215 system-identification test mode (build env: sysid)
 *
 * Replaces the vision servo loop with a 500 Hz excitation + measurement loop:
 *   TIM2 tick (2 ms): goal position for the test axis (step / PRBS / chirp), the other axis held
 *                     at centre, one DMA frame on USART6 = SYNC WRITE(22 B) + READ(8 B) of
 *                     Present Position/Speed/Load of the test servo.
 *   USART6 RX       : status packet received with ReceiveToIdle after PA4 goes low, timestamped
 *                     with TIM5 (1 us, 32-bit free-running).
 *   USART2 (VCP)    : 24-byte binary records at 921600 bps, sent from thread mode out of a ring
 *                     buffer (never from an ISR). '#'-prefixed text lines for acknowledgements.
 *
 * PC commands on the VCP (one ASCII byte):
 *   'p' / 't'  select pan (ID 1) / tilt (ID 2) as the test axis
 *   'S'        step series   (+-25/50/100/200/400 counts, 0.6 s each phase)       ~12 s
 *   'R'        PRBS          (9-bit LFSR, 20 ms bit, +-60 counts, 2 periods)       ~21 s
 *   'C'        log chirp     (+-60 counts, 0.2 -> 12 Hz)                            30 s
 *   'M'        toggle goal speed: STS_DEFAULT_SPEED (as tracking firmware) <-> 0 (servo max)
 *   'X'        stop, return to centre
 *
 * Record layout (little-endian, 24 bytes) — keep in sync with tools/sysid_capture.py:
 *   0  u8  0xA5          1  u8  0x5A
 *   2  u16 seq           4  u32 t_tick_us   (TIM5, start of the 2 ms tick)
 *   8  u32 t_meas_us     (TIM5, when the status packet holding y/v/load was received)
 *   12 i16 u_goal        (counts, absolute goal position of the test axis written in this tick)
 *   14 i16 y_pos         16 i16 v_speed (counts/s)   18 i16 load (0.1 %, signed)
 *   20 u8  flags         bit0 new measurement since last record, bit1 RX timeout/error,
 *                        bit2 profile finished, bit3 TX overrun (bus busy), bit4 max-speed mode
 *   21 u8  profile<<4 | axis_id
 *   22 u8  reserved      23 u8  sum of bytes 2..22 (mod 256)
 */
#ifndef SYSID_H
#define SYSID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SYSID_RECORD_LEN   24u
#define SYSID_SYNC0        0xA5u
#define SYSID_SYNC1        0x5Au

enum {
    SYSID_PROF_IDLE  = 0,
    SYSID_PROF_STEP  = 1,
    SYSID_PROF_PRBS  = 2,
    SYSID_PROF_CHIRP = 3,
};

/* Pure helpers (host-testable with RCWS_HOST_TEST) */
int16_t  SysId_DecodeSigned15(uint16_t raw);            /* STS sign-magnitude (bit 15) */
uint8_t  SysId_BuildRead(uint8_t *buf, uint8_t id, uint8_t addr, uint8_t len);   /* 8 bytes */
/** Find a valid status packet from `id` in buf; copies its data bytes into out (len n). */
int      SysId_ParseStatus(const uint8_t *buf, uint16_t size, uint8_t id, uint8_t *out, uint8_t n);
/** Excitation in counts relative to centre at tick k (500 Hz). *done set when finished. */
int16_t  SysId_Profile(uint8_t prof, uint32_t k, uint8_t *done);

#if defined(RCWS_SYSID) && !defined(RCWS_HOST_TEST)
void SysId_Init(void);     /* thread mode, after peripherals are initialised, before TIM2 starts */
void SysId_Tick(void);     /* TIM2 ISR, 500 Hz */
void SysId_Poll(void);     /* main loop: VCP commands in, records out */
#endif

#ifdef __cplusplus
}
#endif

#endif /* SYSID_H */
