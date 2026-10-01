/*
 * rs_rx.h — complete receiver: colour calibration, unmixing, decoding of one
 * (luma) or three (RGB) channels, message assembly, message queue. The
 * platform only has to provide per-row profiles.
 */
#ifndef RS_RX_H
#define RS_RX_H

#include "rs_decoder.h"
#include "rs_assembler.h"
#include "rs_rgb.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS_RX_MAX_PKTS  96
#define RS_RX_QUEUE     16
#define RS_RX_CAL_TTL   3.0f   /* seconds without a pilot before falling back to luma */

typedef struct {
    rs_packet_t pkt;
    uint8_t channel;         /* 0..2, or 0 in luma mode */
} rs_rx_packet_t;

/* Platform hook to decode the three channels of a frame at once: call job(ctx, i) for
 * i = 0..count-1 on separate threads and return when all are done (the core itself has no
 * threads; the decoder must be built with RS_DEC_THREADS so that its scratch is per thread). */
typedef void (*rs_parallel_fn)(void *user, int count, void (*job)(void *ctx, int i), void *ctx);

typedef struct {
    rs_dec_cfg_t  cfg;
    rs_asm_t      assembler;
    rs_rgb_cal_t  cal;
    float         last_pilot_t;
    int           mode;      /* 0 = luma, 1 = rgb (calibrated) */
    float         rows_per_chip;
    int           rpc_n;            /* packets that agreed with rows_per_chip; the decoder gets the hint from 3 on */
    int           empty_frames;     /* consecutive frames without a packet; the hint is dropped after 30 */
    uint32_t      frames, packets_total;
    uint32_t      grid_ok;          /* packets decoded at a predicted grid position (no sync) */
    int           defer_assembly;   /* 1: rs_rx_process only decodes; rs_rx_assemble() feeds the assembler */
    rs_dec_stats_t last_stats;   /* of the last decoded channel */
    rs_parallel_fn parallel;     /* optional: decodes the channels of a frame in parallel (see rs_parallel_fn) */
    void          *parallel_user;
    int           npkts;
    rs_rx_packet_t pkts[RS_RX_MAX_PKTS];
    rs_message_t  queue[RS_RX_QUEUE];
    int           qhead, qlen;
} rs_rx_t;

void rs_rx_init(rs_rx_t *rx);

/* Process one frame. r, g, b: per-row profiles (n samples); pass b == NULL to
 * treat r as luma (single-channel). t: time in seconds (for pilot expiry).
 * Returns the number of packets decoded; complete messages are queued. */
/* 1 when all three camera channels are modulated (an RGB LED), 0 for a single-colour light. */
int rs_rx_three_coloured(const float *r, const float *g, const float *b, int n);
/* With defer_assembly set: feed this frame's packets (pkts[i] with keep[i] != 0, or all when
 * keep is NULL) to the assembler. Lets a multi-source receiver drop cross-talk first. */
/* Install the platform's parallel hook (see rs_parallel_fn); NULL decodes the channels in turn. */
void rs_rx_set_parallel(rs_rx_t *rx, rs_parallel_fn fn, void *user);
void rs_rx_assemble(rs_rx_t *rx, const uint8_t *keep);
int rs_rx_process(rs_rx_t *rx, const float *r, const float *g, const float *b, int n, float t);

/* Pop the next complete message; returns 0 when the queue is empty. */
int rs_rx_pop_message(rs_rx_t *rx, rs_message_t *out);

#ifdef __cplusplus
}
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
/* Small accessors for bindings (Python ctypes, JNI). */
size_t rs_rx_sizeof(void);
int    rs_rx_mode(const rs_rx_t *rx);
float  rs_rx_rows_per_chip(const rs_rx_t *rx);
int    rs_rx_pilots(const rs_rx_t *rx);
float  rs_rx_cal_cond(const rs_rx_t *rx);
uint32_t rs_rx_packets(const rs_rx_t *rx);
uint32_t rs_rx_messages(const rs_rx_t *rx);
int    rs_rx_packet_at(const rs_rx_t *rx, int i, rs_packet_t *pkt, uint8_t *channel);
const rs_dec_stats_t *rs_rx_stats(const rs_rx_t *rx);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
uint32_t rs_rx_resets(const rs_rx_t *rx);       /* assembler slot resets (poisoned or replaced messages) */
#ifdef __cplusplus
}
#endif
