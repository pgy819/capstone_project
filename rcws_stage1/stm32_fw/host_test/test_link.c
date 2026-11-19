/*
 * Host tests for link_proto.c (packet v2): round trip, CRC reference value, resynchronisation after
 * byte slips / garbage / truncated frames, header bytes inside the payload.
 * and the STM32 front end protocol.c (sanity gate, shared packet, error counter).
 *   cc -std=c99 -Wall -Wextra -DRCWS_HOST_TEST -I../include test_link.c ../src/link_proto.c ../src/protocol.c -o test_link && ./test_link
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "link_proto.h"
#include "protocol.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static int feed(LinkParser_t *p, const uint8_t *b, int n, LinkAim_t *got, int max)
{
    int k = 0;
    uint8_t f[LINK_FRAME_LEN];
    for (int i = 0; i < n; i++) {
        if (Link_ParseByte(p, b[i], f)) {
            if (k < max && Link_DecodeAim(f, &got[k])) { k++; }
        }
    }
    return k;
}

static LinkAim_t aim(int i)
{
    LinkAim_t a;
    a.seq = (uint8_t)i;
    a.mode = (uint8_t)(i % 3);
    a.flags = (uint8_t)(i & 3);
    a.err_pan = (int16_t)(i * 37 - 2000);
    a.err_tilt = (int16_t)(-i * 11);
    a.rate_pan = (int16_t)(i * 3);
    a.rate_tilt = (int16_t)-i;
    return a;
}

static void test_crc_reference(void)
{
    const uint8_t s[] = "123456789";
    CHECK(Link_Crc16(s, 9) == 0x29B1u);      /* CRC-16/CCITT-FALSE check value */
}

static void test_roundtrip(void)
{
    uint8_t f[LINK_FRAME_LEN];
    LinkAim_t a = aim(77), b;
    a.err_pan = -32768; a.err_tilt = 32767;
    Link_EncodeAim(f, &a);
    CHECK(f[0] == 0xAA && f[1] == 0x55 && f[2] == 0x21);
    CHECK(Link_DecodeAim(f, &b) && b.seq == a.seq && b.mode == a.mode && b.flags == a.flags);
    CHECK(b.err_pan == -32768 && b.err_tilt == 32767 && b.rate_pan == a.rate_pan && b.rate_tilt == a.rate_tilt);
    f[7] ^= 1;
    CHECK(!Link_DecodeAim(f, &b));
    LinkStatus_t s = { 9, 4, 0x05, 626, 3470, 65535, 3 }, t;
    Link_EncodeStatus(f, &s);
    CHECK(Link_DecodeStatus(f, &t) && t.guard == 4 && t.flags == 5 && t.pan_goal == 626 && t.tilt_goal == 3470 &&
          t.rx_ok == 65535 && t.rx_err == 3);
    CHECK(!Link_DecodeAim(f, &b));            /* uplink frame is not an AIM frame */
    LinkParser_t p;
    Link_ParserInit(&p, LINK_SYNC1_DOWN);
    int k = 0;
    uint8_t o[LINK_FRAME_LEN];
    for (unsigned i = 0; i < LINK_FRAME_LEN; i++) { k += Link_ParseByte(&p, f[i], o); }
    CHECK(k == 0);                             /* downlink parser ignores uplink frames */
}

static void test_stream_resync(void)
{
    /* 300 frames with random garbage, single dropped bytes and truncated frames between them */
    enum { N = 300 };
    static uint8_t buf[N * 40];
    int len = 0, expect = 0;
    int sent_seq[N];
    srand(7);
    for (int i = 0; i < N; i++) {
        uint8_t f[LINK_FRAME_LEN];
        LinkAim_t a = aim(i);
        a.err_pan = (int16_t)(0xAA55 & 0x7FFF);   /* header bytes inside the payload */
        Link_EncodeAim(f, &a);
        const int r = rand() % 6;
        if (r == 0) {                              /* garbage, may contain AA / AA 55 */
            for (int g = 0; g < 1 + rand() % 7; g++) { buf[len++] = (uint8_t)((rand() & 1) ? 0xAA : rand()); }
        }
        if (r == 1) {                              /* truncated frame first */
            memcpy(&buf[len], f, 9);
            len += 9;
        }
        if (r == 2) {                              /* one byte lost from this frame: must be rejected */
            const int d = 2 + rand() % 13;
            memcpy(&buf[len], f, (size_t)d);
            memcpy(&buf[len + d], f + d + 1, (size_t)(LINK_FRAME_LEN - d - 1));
            len += LINK_FRAME_LEN - 1;
            continue;
        }
        memcpy(&buf[len], f, LINK_FRAME_LEN);
        len += LINK_FRAME_LEN;
        sent_seq[expect++] = i;
    }
    LinkParser_t p;
    Link_ParserInit(&p, LINK_SYNC1_DOWN);
    static LinkAim_t got[N];
    const int k = feed(&p, buf, len, got, N);
    int in_order = 1, matched = 0;
    for (int i = 0, j = 0; i < k; i++) {
        while (j < expect && (uint8_t)sent_seq[j] != got[i].seq) { j++; }
        if (j == expect) { in_order = 0; break; }
        matched++;
    }
    printf("  stream: %d bytes, %d intact frames sent, %d decoded, crc errors %u\n", len, expect, k, (unsigned)p.crc_err);
    CHECK(in_order && matched == k);
    CHECK(k >= expect - 2);                    /* a truncated tail may swallow at most a frame or two */
    CHECK(k <= expect);                        /* no corrupted frame ever accepted */
}

static void test_split_bytes(void)
{
    /* frame delivered one byte at a time across reads, then back-to-back frames */
    uint8_t f[3 * LINK_FRAME_LEN];
    for (int i = 0; i < 3; i++) { LinkAim_t a = aim(i + 1); Link_EncodeAim(&f[i * LINK_FRAME_LEN], &a); }
    LinkParser_t p;
    Link_ParserInit(&p, LINK_SYNC1_DOWN);
    LinkAim_t got[3];
    CHECK(feed(&p, f, (int)sizeof f, got, 3) == 3 && got[2].seq == 3 && p.crc_err == 0);
}

static void test_stm32_front_end(void)
{
    uint8_t f[LINK_FRAME_LEN];
    VisionPacket_t v;
    LinkAim_t a = aim(5);
    a.mode = LINK_MODE_FOLLOW;
    a.err_pan = 16 * 100;                      /* 100 counts right */
    a.err_tilt = -16 * 40;
    Link_EncodeAim(f, &a);
    g_comm_watchdog = 999u;
    for (unsigned i = 0; i < LINK_FRAME_LEN; i++) { Protocol_ParseByte(f[i]); }
    CHECK(Protocol_HostTakePacket(&v) && v.mode == LINK_MODE_FOLLOW && v.err_pan == 1600 && v.err_tilt == -640);
    CHECK(g_comm_watchdog == 0u && g_proto_ok_count == 1u);
    CHECK(!Protocol_HostTakePacket(&v));
    a.err_pan = (int16_t)(VISION_ERR_LIMIT + 1);   /* CRC-valid but impossible angle: rejected */
    Link_EncodeAim(f, &a);
    const uint32_t e0 = g_proto_err_count;
    for (unsigned i = 0; i < LINK_FRAME_LEN; i++) { Protocol_ParseByte(f[i]); }
    CHECK(!Protocol_HostTakePacket(&v) && g_proto_err_count == e0 + 1u);
    f[0] = 0xAA; f[9] ^= 0x40;                 /* corrupted frame: counted, not used */
    for (unsigned i = 0; i < LINK_FRAME_LEN; i++) { Protocol_ParseByte(f[i]); }
    CHECK(!Protocol_HostTakePacket(&v) && g_proto_err_count >= e0 + 2u);
}

int main(void)
{
    test_stm32_front_end();
    test_crc_reference();
    test_roundtrip();
    test_split_bytes();
    test_stream_resync();
    printf(fails ? "link tests: %d FAILED\n" : "link tests: all passed\n", fails);
    return fails ? 1 : 0;
}
