/**
 * servo_guard.h — motion shaping and safety supervisor for the 500 Hz servo loop (scenarios I, J)
 *
 *  Motion shaping (per axis, all in servo steps, 4096 steps = 360 deg, 1 tick = 2 ms):
 *    - deadband on the vision pixel error (no hunting around the centre)
 *    - velocity limit and acceleration limit on the goal; a harder deceleration when braking
 *    - goal clamped to the EEPROM soft limits; at a limit the velocity is zeroed (anti-windup:
 *      the goal no longer runs away beyond the point where the servo has stopped)
 *
 *  Safety supervisor (one state for both axes):
 *    RUN        vision packets fresh and target valid             -> follow
 *    HOLD       target lost / coasting too long / comm gap > 0.5 s  -> brake to a stop, hold the goal
 *    TORQUE_OFF comm gap > 10 s (Pi gone)                          -> torque off, servos go limp
 *    RAMP       packets back after TORQUE_OFF                      -> torque on at a low speed for 1 s
 *    ESTOP      B1 pressed (latched until reset)                   -> torque off, nothing else
 *
 * Pure C, no HAL: build with -DRCWS_HOST_TEST for the host tests (host_test/test_guard.c).
 */
#ifndef SERVO_GUARD_H
#define SERVO_GUARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GUARD_STEPS_PER_DEG   (4096.0f / 360.0f)
#define GUARD_TICK_S          0.002f

typedef struct {
    float pos_min, pos_max;   /* goal clamp [steps] = EEPROM angle limits */
    float v_max;              /* [steps/tick] */
    float a_max;              /* [steps/tick^2] while following */
    float a_brake;            /* [steps/tick^2] while braking to a stop */
} AxisLimits_t;

typedef struct {
    float goal;               /* [steps] */
    float v;                  /* [steps/tick] */
} AxisState_t;

/** Limits from physical units (deg, deg/s, deg/s^2). */
AxisLimits_t Guard_Limits(float pos_min, float pos_max, float v_max_dps, float a_max_dps2, float a_brake_dps2);

/** Pixel error with a deadband: 0 inside +-db, shifted outside (no step at the edge). */
float Guard_Deadband(float e, float db);

/**
 * One tick of motion shaping. v_cmd is the requested goal velocity [steps/tick] (the PD increment);
 * braking = 1 ignores v_cmd and decelerates to zero at a_brake. Returns the new goal.
 */
float Guard_AxisStep(AxisState_t *s, const AxisLimits_t *l, float v_cmd, uint8_t braking);

typedef enum {
    GUARD_RUN = 0,
    GUARD_HOLD,
    GUARD_TORQUE_OFF,
    GUARD_RAMP,
    GUARD_ESTOP
} GuardState_t;

typedef struct {
    uint32_t hold_ticks;      /* comm gap -> HOLD           (250  = 0.5 s) */
    uint32_t off_ticks;       /* comm gap -> TORQUE_OFF     (5000 = 10 s)  */
    uint32_t ramp_ticks;      /* RAMP duration              (500  = 1 s)   */
} GuardCfg_t;

typedef struct {
    GuardState_t state;
    uint32_t ramp_left;
    uint8_t  torque_on;       /* torque wanted in this state */
    uint32_t ticks_in_state;  /* for periodic resends of the torque packet */
} Guard_t;

void Guard_Init(Guard_t *g);

/**
 * Advance the supervisor by one tick.
 *   comm_gap_ticks : ticks since the last valid packet
 *   target_valid   : last packet says DETECTED / COASTING (aim output FOLLOW / PREDICT)
 *   estop          : emergency stop latched (B1)
 * Returns the new state; g->torque_on tells whether the servos must be held or limp. The caller
 * sends the torque packet itself (periodically while limp, at the start of RAMP), because a DMA
 * frame can be skipped when the bus is busy.
 */
GuardState_t Guard_Update(Guard_t *g, const GuardCfg_t *c, uint32_t comm_gap_ticks, uint8_t target_valid, uint8_t estop);

const char *Guard_Name(GuardState_t s);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_GUARD_H */
