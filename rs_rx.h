/*
 * rs_rx.h — complete receiver of one light: colour calibration, unmixing, decoding of one
 * (luma) or three (RGB) channels, message assembly, message queue. The platform only has to
 * provide per-row profiles.
 *
 * One frame at a time per process: the receiver and the profile extraction keep their scratch in
 * static buffers (the decoder's alone is per thread, for the parallel hook), so two receivers
 * must not process frames on two threads at once. Time is in seconds from any origin of the
 * caller's choice, as a float: keep it small (start from the first frame, not from the uptime),
 * the receiver needs it to a fraction of a millisecond.
 */
#ifndef RS_RX_H
#define RS_RX_H

#include "rs_decoder.h"
#include "rs_stitch.h"
#include "rs_assembler.h"
#include "rs_rgb.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS_RX_MAX_PKTS  96     /* distinct packets kept per frame */
#define RS_RX_CH_PKTS   32     /* packets the detector may return per channel and frame */
#define RS_RX_QUEUE     16
#define RS_RX_CAL_TTL   10.0f  /* seconds without a pilot before falling back to luma/direct. The colour
                                * matrix changes only when the light or the camera does; a 30 fps phone
                                * whose blob covers a third of the readout sees a whole block in 15-20 %
                                * of its frames, and 3 s left it dropping to direct mode on unlucky runs. */

/* What rs_rx_process decoded in the last frame (rs_rx_mode). */
#define RS_RX_MODE_LUMA    0   /* one stream: (r+g+b)/3, or the single profile given */
#define RS_RX_MODE_RGB     1   /* three streams unmixed with the matrix measured on the pilots */
#define RS_RX_MODE_DIRECT  2   /* three streams, the camera's own R, G, B: a three-coloured light without a pilot lock */

typedef struct {
    rs_packet_t pkt;
    uint8_t channel;         /* 0..2, or 0 in luma mode */
} rs_rx_packet_t;

typedef struct {
    rs_dec_cfg_t  cfg;                /* first field: bindings reach the decoder configuration through the receiver's address */
    rs_asm_t      assembler;
    rs_rgb_cal_t  cal;
    float         last_pilot_t;
    int           mode;               /* RS_RX_MODE_* */
    float         rows_per_chip;
    int           rpc_n;              /* packets that agreed with rows_per_chip; the decoder gets the hint from 3 on */
    int           empty_frames;       /* consecutive frames without a packet; the hint is dropped after 30 */
    uint32_t      frames, packets_total;   /* packets_total counts distinct packets per frame */
    int           defer_assembly;     /* 1: rs_rx_process only decodes; rs_rx_assemble() feeds the assembler */
    rs_dec_stats_t last_stats;        /* of the last decoded channel */
    rs_parallel_fn parallel;          /* optional: decodes the channels of a frame in parallel; also handed to the decoder via cfg */
    void          *parallel_user;
    float         row_seconds;        /* sensor row time (s), 0 = unknown; enables phase prediction in the stitcher */
    float         frame_t;            /* time of the frame being processed */
    int           stitch_enabled;     /* default 0: experimental, see rs_stitch.h */
    rs_stitch_t   stitch[3];          /* one per channel */
    uint32_t      stitched_total;     /* packets obtained by stitching */
    int           npkts;
    rs_rx_packet_t pkts[RS_RX_MAX_PKTS];   /* this frame's distinct packets */
    rs_message_t  queue[RS_RX_QUEUE];
    int           qhead, qlen;
} rs_rx_t;

void rs_rx_init(rs_rx_t *rx);

/* The camera's exposure (in rows of the profiles given) and row time (seconds), see rs_camera_t. */
void rs_rx_set_camera(rs_rx_t *rx, rs_camera_t cam);
/* Install the platform's parallel hook (see rs_parallel_fn); NULL decodes the channels in turn. */
void rs_rx_set_parallel(rs_rx_t *rx, rs_parallel_fn fn, void *user);

/* Process one frame. r, g, b: per-row profiles (n samples, at most RS_DEC_MAX_ROWS are used);
 * pass b == NULL to treat r as luma (single-channel). t: time in seconds (pilot expiry,
 * stitching). Returns the number of distinct packets decoded; complete messages are queued. */
int rs_rx_process(rs_rx_t *rx, const float *r, const float *g, const float *b, int n, float t);
/* With defer_assembly set: feed this frame's packets (pkts[i] with keep[i] != 0, or all when
 * keep is NULL) to the assembler. Lets a multi-source receiver drop cross-talk first. */
void rs_rx_assemble(rs_rx_t *rx, const uint8_t *keep);
/* Pop the next complete message; returns 0 when the queue is empty. */
int rs_rx_pop_message(rs_rx_t *rx, rs_message_t *out);

/* 1 when all three camera channels are modulated (an RGB LED), 0 for a single-colour light.
 * Used by rs_multi for the profile hypothesis choice. */
int rs_rx_three_coloured(const float *r, const float *g, const float *b, int n);

/* Accessors for bindings (Python ctypes, JNI). */
size_t rs_rx_sizeof(void);
int    rs_rx_mode(const rs_rx_t *rx);                 /* RS_RX_MODE_* of the last frame */
float  rs_rx_rows_per_chip(const rs_rx_t *rx);
int    rs_rx_pilots(const rs_rx_t *rx);
float  rs_rx_cal_cond(const rs_rx_t *rx);
uint32_t rs_rx_packets(const rs_rx_t *rx);            /* distinct packets since init */
uint32_t rs_rx_messages(const rs_rx_t *rx);
uint32_t rs_rx_stitched(const rs_rx_t *rx);           /* packets obtained by stitching pieces across frames */
uint32_t rs_rx_resets(const rs_rx_t *rx);             /* assembler slot resets (poisoned or replaced messages) */
int    rs_rx_packet_at(const rs_rx_t *rx, int i, rs_packet_t *pkt, uint8_t *channel);   /* i-th packet of the last frame; 0 past the end */
const rs_dec_stats_t *rs_rx_stats(const rs_rx_t *rx);

#ifdef __cplusplus
}
#endif
#endif
