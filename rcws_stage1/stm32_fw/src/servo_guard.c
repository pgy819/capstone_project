/**
 * servo_guard.c — motion shaping + safety supervisor (see servo_guard.h). Runs inside TIM2_IRQHandler:
 * no HAL calls, no blocking, no dynamic memory; a few float operations per axis.
 */
#include "servo_guard.h"

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

AxisLimits_t Guard_Limits(float pos_min, float pos_max, float v_max_dps, float a_max_dps2, float a_brake_dps2)
{
    AxisLimits_t l;
    const float k = GUARD_STEPS_PER_DEG;
    l.pos_min = pos_min;
    l.pos_max = pos_max;
    l.v_max   = v_max_dps * k * GUARD_TICK_S;
    l.a_max   = a_max_dps2 * k * GUARD_TICK_S * GUARD_TICK_S;
    l.a_brake = a_brake_dps2 * k * GUARD_TICK_S * GUARD_TICK_S;
    return l;
}

float Guard_Deadband(float e, float db)
{
    if (e > db)  { return e - db; }
    if (e < -db) { return e + db; }
    return 0.0f;
}

float Guard_AxisStep(AxisState_t *s, const AxisLimits_t *l, float v_cmd, uint8_t braking)
{
    const float target = braking ? 0.0f : clampf(v_cmd, -l->v_max, l->v_max);
    const float a = braking ? l->a_brake : l->a_max;
    s->v += clampf(target - s->v, -a, a);
    s->goal += s->v;
    /* anti-windup: at a limit the goal stops there and so does the velocity */
    if (s->goal <= l->pos_min) { s->goal = l->pos_min; if (s->v < 0.0f) { s->v = 0.0f; } }
    if (s->goal >= l->pos_max) { s->goal = l->pos_max; if (s->v > 0.0f) { s->v = 0.0f; } }
    return s->goal;
}

void Guard_Init(Guard_t *g)
{
    g->state = GUARD_HOLD;     /* nothing moves until the first valid packet */
    g->ramp_left = 0u;
    g->torque_on = 1u;         /* servos power up with torque on */
    g->ticks_in_state = 0u;
}

static uint8_t want_torque(GuardState_t s)
{
    return (s == GUARD_TORQUE_OFF || s == GUARD_ESTOP) ? 0u : 1u;
}

GuardState_t Guard_Update(Guard_t *g, const GuardCfg_t *c, uint32_t comm_gap_ticks, uint8_t target_valid, uint8_t estop)
{
    GuardState_t next = g->state;
    if (estop || g->state == GUARD_ESTOP) {
        next = GUARD_ESTOP;                                   /* latched until reset */
    } else if (comm_gap_ticks > c->off_ticks) {
        next = GUARD_TORQUE_OFF;
    } else if (g->state == GUARD_TORQUE_OFF) {
        if (comm_gap_ticks <= c->hold_ticks) {                /* Pi is back: torque on, gently */
            next = GUARD_RAMP;
            g->ramp_left = c->ramp_ticks;
        }
    } else if (g->state == GUARD_RAMP) {
        if (g->ramp_left > 0u) { g->ramp_left--; }
        if (g->ramp_left == 0u) {
            next = (comm_gap_ticks > c->hold_ticks || !target_valid) ? GUARD_HOLD : GUARD_RUN;
        }
    } else {
        next = (comm_gap_ticks > c->hold_ticks || !target_valid) ? GUARD_HOLD : GUARD_RUN;
    }
    if (next != g->state) { g->ticks_in_state = 0u; } else if (g->ticks_in_state < 0xFFFFFFFFu) { g->ticks_in_state++; }
    g->state = next;
    g->torque_on = want_torque(next);
    return next;
}

const char *Guard_Name(GuardState_t s)
{
    switch (s) {
    case GUARD_RUN:        return "RUN";
    case GUARD_HOLD:       return "HOLD";
    case GUARD_TORQUE_OFF: return "TORQUE_OFF";
    case GUARD_RAMP:       return "RAMP";
    case GUARD_ESTOP:      return "ESTOP";
    }
    return "?";
}
