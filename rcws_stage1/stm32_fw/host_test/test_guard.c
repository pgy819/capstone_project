/*
 * Host tests for servo_guard.c (motion shaping, safety supervisor) and the STS packet builders.
 *   cc -std=c99 -Wall -Wextra -DRCWS_HOST_TEST -I../include test_guard.c ../src/servo_guard.c ../src/feetech_sts.c -lm -o test_guard && ./test_guard
 */
#include <math.h>
#include <stdio.h>

#include "feetech_sts.h"
#include "servo_guard.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static void test_limits_and_deadband(void)
{
    const AxisLimits_t l = Guard_Limits(626, 3470, 300, 3000, 6000);
    CHECK(fabsf(l.v_max - 300.0f * 4096.0f / 360.0f * 0.002f) < 1e-3f);   /* 6.83 steps/tick */
    CHECK(fabsf(l.a_max - 0.1365f) < 1e-3f);
    CHECK(Guard_Deadband(2.0f, 3.0f) == 0.0f && Guard_Deadband(-3.0f, 3.0f) == 0.0f);
    CHECK(Guard_Deadband(10.0f, 3.0f) == 7.0f && Guard_Deadband(-10.0f, 3.0f) == -7.0f);
}

static void test_velocity_accel_limits(void)
{
    const AxisLimits_t l = Guard_Limits(0, 4095, 300, 3000, 6000);
    AxisState_t s = { 2048, 0 };
    int t_vmax = -1;
    for (int k = 0; k < 200; k++) {
        const float v0 = s.v;
        Guard_AxisStep(&s, &l, 100.0f, 0);                 /* absurd request: limits must hold */
        CHECK(s.v - v0 <= l.a_max + 1e-5f);
        CHECK(s.v <= l.v_max + 1e-5f);
        if (t_vmax < 0 && s.v >= l.v_max - 1e-4f) { t_vmax = k + 1; }
    }
    printf("  reached 300 deg/s after %d ticks (%.0f ms, expected 100 ms)\n", t_vmax, t_vmax * 2.0);
    CHECK(t_vmax >= 49 && t_vmax <= 52);
    /* HOLD: brake to zero at a_brake, never coast on */
    const float g0 = s.goal;
    int ticks = 0;
    while (s.v > 0.0f && ticks < 1000) { Guard_AxisStep(&s, &l, 100.0f, 1); ticks++; }
    const float deg = (s.goal - g0) / (4096.0f / 360.0f);
    printf("  braking from 300 deg/s: %d ticks, %.1f deg travelled (expected 50 ms, 7.5 deg)\n", ticks, deg);
    CHECK(ticks <= 26 && deg < 8.0f);
    const float g1 = s.goal;
    for (int k = 0; k < 100; k++) { Guard_AxisStep(&s, &l, 100.0f, 1); }
    CHECK(s.goal == g1 && s.v == 0.0f);                    /* stays put while braking */
}

static void test_soft_limit_anti_windup(void)
{
    const AxisLimits_t l = Guard_Limits(626, 3470, 300, 3000, 6000);
    AxisState_t s = { 3400, 0 };
    for (int k = 0; k < 500; k++) { Guard_AxisStep(&s, &l, 5.0f, 0); }   /* keeps pushing beyond the limit */
    CHECK(s.goal == 3470.0f && s.v == 0.0f);
    /* reverse: it must leave the limit at once (no wound-up goal to unwind first) */
    for (int k = 0; k < 40; k++) { Guard_AxisStep(&s, &l, -5.0f, 0); }
    CHECK(s.goal < 3470.0f - 10.0f);
}

static void test_supervisor(void)
{
    const GuardCfg_t c = { 250u, 5000u, 500u };
    Guard_t g;
    Guard_Init(&g);
    CHECK(g.state == GUARD_HOLD);
    CHECK(Guard_Update(&g, &c, 0, 1, 0) == GUARD_RUN);                 /* fresh packet, target valid */
    CHECK(Guard_Update(&g, &c, 3, 0, 0) == GUARD_HOLD);                /* Pi says lost / HOLD */
    CHECK(Guard_Update(&g, &c, 3, 1, 0) == GUARD_RUN);
    CHECK(Guard_Update(&g, &c, 251, 1, 0) == GUARD_HOLD && g.torque_on);  /* 0.5 s silence */
    CHECK(Guard_Update(&g, &c, 5001, 1, 0) == GUARD_TORQUE_OFF && !g.torque_on);
    CHECK(Guard_Update(&g, &c, 9000, 1, 0) == GUARD_TORQUE_OFF);
    CHECK(Guard_Update(&g, &c, 0, 1, 0) == GUARD_RAMP && g.torque_on);   /* Pi back: gentle */
    int ramp = 1;
    while (Guard_Update(&g, &c, 0, 1, 0) == GUARD_RAMP) { ramp++; }
    CHECK(ramp >= 499 && ramp <= 501 && g.state == GUARD_RUN);
    CHECK(Guard_Update(&g, &c, 0, 1, 1) == GUARD_ESTOP && !g.torque_on);
    CHECK(Guard_Update(&g, &c, 0, 1, 0) == GUARD_ESTOP);               /* latched */
    CHECK(Guard_Update(&g, &c, 9000, 1, 0) == GUARD_ESTOP);
    /* software ESTOP released from the web: limp, then the slow RAMP, never straight to RUN */
    Guard_ClearEstop(&g);
    CHECK(g.state == GUARD_TORQUE_OFF && !g.torque_on);
    CHECK(Guard_Update(&g, &c, 0, 1, 0) == GUARD_RAMP && g.torque_on);
    Guard_t h;
    Guard_Init(&h);
    Guard_ClearEstop(&h);                                              /* no effect outside ESTOP */
    CHECK(h.state == GUARD_HOLD);
}

static void test_packets(void)
{
    uint8_t b[22];
    CHECK(STS_BuildSyncTorque(b, 0) == 12);
    uint32_t sum = 0;
    for (int i = 2; i <= 10; i++) { sum += b[i]; }
    CHECK(b[0] == 0xFF && b[1] == 0xFF && b[2] == 0xFE && b[3] == 0x08 && b[4] == 0x83 && b[5] == 0x28);
    CHECK(b[7] == STS_PAN_ID && b[8] == 0 && b[9] == STS_TILT_ID && b[10] == 0 && b[11] == (uint8_t)~sum);
    STS_BuildSyncTorque(b, 1);
    CHECK(b[8] == 1 && b[10] == 1);
    CHECK(STS_BuildSyncWritePosSpeed(b, 626, 300, 3470, 300) == 22);
    CHECK(STS_PAN_ID == 2 && STS_TILT_ID == 1);              /* team assignment */
    CHECK(b[7] == 2 && b[8] == (626 & 0xFF) && b[14] == 1 && b[15] == (3470 & 0xFF));
}

static void test_read_pos_reply(void)
{
    uint8_t req[8], rx[24];
    uint16_t pos = 0;
    CHECK(STS_BuildReadPos(req, STS_PAN_ID) == 8);
    CHECK(req[2] == 2 && req[3] == 4 && req[4] == 0x02 && req[5] == 0x38 && req[6] == 2);
    /* echo only: no reply */
    CHECK(!STS_ParseReadPosReply(req, 8, STS_PAN_ID, &pos));
    /* echo, then the reply FF FF 02 04 00 L H ck (position 2100) */
    for (int i = 0; i < 8; i++) { rx[i] = req[i]; }
    const uint8_t r[8] = { 0xFF, 0xFF, 2, 4, 0, (uint8_t)(2100 & 0xFF), (uint8_t)(2100 >> 8),
                           (uint8_t)~(2 + 4 + 0 + (2100 & 0xFF) + (2100 >> 8)) };
    for (int i = 0; i < 8; i++) { rx[8 + i] = r[i]; }
    CHECK(STS_ParseReadPosReply(rx, 16, STS_PAN_ID, &pos) && pos == 2100);
    /* reply of the other servo is not ours */
    CHECK(!STS_ParseReadPosReply(rx, 16, STS_TILT_ID, &pos));
    /* corrupted checksum */
    rx[15] ^= 1;
    CHECK(!STS_ParseReadPosReply(rx, 16, STS_PAN_ID, &pos));
}

int main(void)
{
    test_read_pos_reply();
    test_limits_and_deadband();
    test_velocity_accel_limits();
    test_soft_limit_anti_windup();
    test_supervisor();
    test_packets();
    printf("%s (%d failed checks)\n", fails ? "FAILED" : "all servo_guard tests passed", fails);
    return fails ? 1 : 0;
}
