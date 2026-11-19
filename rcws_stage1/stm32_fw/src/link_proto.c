/**
 * link_proto.c — packet v2 encode / decode / resynchronising parser (see link_proto.h)
 * Shared by the STM32 firmware and the RPi runtime; keep it pure C99.
 */
#include "link_proto.h"

#include <string.h>

uint16_t Link_Crc16(const uint8_t *p, uint32_t n)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0u; i < n; i++) {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (int k = 0; k < 8; k++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void seal(uint8_t *f)
{
    put16(&f[14], Link_Crc16(&f[2], 12u));
}

static int crc_ok(const uint8_t *f)
{
    return Link_Crc16(&f[2], 12u) == get16(&f[14]);
}

void Link_EncodeAim(uint8_t out[LINK_FRAME_LEN], const LinkAim_t *a)
{
    out[0] = LINK_SYNC0;
    out[1] = LINK_SYNC1_DOWN;
    out[2] = LINK_TYPE_AIM;
    out[3] = a->seq;
    out[4] = a->mode;
    out[5] = a->flags;
    put16(&out[6], (uint16_t)a->err_pan);
    put16(&out[8], (uint16_t)a->err_tilt);
    put16(&out[10], (uint16_t)a->rate_pan);
    put16(&out[12], (uint16_t)a->rate_tilt);
    seal(out);
}

void Link_EncodeStatus(uint8_t out[LINK_FRAME_LEN], const LinkStatus_t *s)
{
    out[0] = LINK_SYNC0;
    out[1] = LINK_SYNC1_UP;
    out[2] = LINK_TYPE_STATUS;
    out[3] = s->seq_echo;
    out[4] = s->guard;
    out[5] = s->flags;
    put16(&out[6], s->pan_goal);
    put16(&out[8], s->tilt_goal);
    put16(&out[10], s->rx_ok);
    put16(&out[12], s->rx_err);
    seal(out);
}

int Link_DecodeAim(const uint8_t f[LINK_FRAME_LEN], LinkAim_t *a)
{
    if (f[0] != LINK_SYNC0 || f[1] != LINK_SYNC1_DOWN || f[2] != LINK_TYPE_AIM || !crc_ok(f)) {
        return 0;
    }
    if (f[4] > LINK_MODE_PREDICT) {
        return 0;
    }
    a->seq       = f[3];
    a->mode      = f[4];
    a->flags     = f[5];
    a->err_pan   = (int16_t)get16(&f[6]);
    a->err_tilt  = (int16_t)get16(&f[8]);
    a->rate_pan  = (int16_t)get16(&f[10]);
    a->rate_tilt = (int16_t)get16(&f[12]);
    return 1;
}

int Link_DecodeStatus(const uint8_t f[LINK_FRAME_LEN], LinkStatus_t *s)
{
    if (f[0] != LINK_SYNC0 || f[1] != LINK_SYNC1_UP || f[2] != LINK_TYPE_STATUS || !crc_ok(f)) {
        return 0;
    }
    s->seq_echo  = f[3];
    s->guard     = f[4];
    s->flags     = f[5];
    s->pan_goal  = get16(&f[6]);
    s->tilt_goal = get16(&f[8]);
    s->rx_ok     = get16(&f[10]);
    s->rx_err    = get16(&f[12]);
    return 1;
}

void Link_ParserInit(LinkParser_t *p, uint8_t sync1)
{
    memset(p, 0, sizeof(*p));
    p->sync1 = sync1;
}

static void drop_first(LinkParser_t *p)
{
    memmove(p->win, p->win + 1, (size_t)(p->n - 1u));
    p->n--;
}

/* keep the window starting at a possible header: "AA <sync1>" (handles "AA AA 55" and garbage) */
static void trim(LinkParser_t *p)
{
    for (;;) {
        if (p->n >= 1u && p->win[0] != LINK_SYNC0) { drop_first(p); continue; }
        if (p->n >= 2u && p->win[1] != p->sync1)   { drop_first(p); continue; }
        break;
    }
}

int Link_ParseByte(LinkParser_t *p, uint8_t b, uint8_t out[LINK_FRAME_LEN])
{
    p->win[p->n++] = b;            /* n < LINK_FRAME_LEN here: a full window is always consumed below */
    trim(p);
    if (p->n < LINK_FRAME_LEN) {
        return 0;
    }
    if (crc_ok(p->win)) {
        memcpy(out, p->win, LINK_FRAME_LEN);
        p->frames++;
        p->n = 0u;
        return 1;
    }
    p->crc_err++;
    drop_first(p);                 /* resynchronise on the next header inside the window */
    trim(p);
    return 0;
}
