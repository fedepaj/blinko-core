/*
 * rs_stitch.h — stroboscopic stitching: a packet read in pieces over several frames.
 * Experimental: it works in simulation from about 40 % of a packet per frame and has not
 * decoded a packet on a phone yet, so a receiver leaves it off unless rs_rx_t.stitch_enabled
 * is set.
 *
 * When the lit blob is shorter than a packet (a far or small light, a 30 fps phone whose
 * readout shows a few milliseconds of each frame), no frame holds a whole packet. If the
 * transmitter repeats each packet many times (rs_tx_set_repeat, tens of copies), successive frames read
 * successive pieces of the same 82-chip cycle at phases that advance by (frame period mod
 * packet period). The stitcher places each piece on the cycle, accumulates the normalized
 * brightness per chip, and hands the completed cycle to the ordinary detector.
 *
 * Phase of a piece: exact when the frame shows a sync (its gap is chip 0); otherwise predicted
 * from the last anchored frame with the chip period in seconds. That period is fitted to the
 * recent anchored frames: their phases must all equal phase0 + t/period modulo the cycle, and a
 * search of +-12 % around the row-time guess (periodogram over the anchors) finds the period
 * to ~0.1 %, because a frame is a thousand chips and a wrong period scatters the phases. A
 * predicted piece is then refined by correlating it against what the cycle already holds. A piece that
 * contradicts the accumulated cycle (the transmitter moved on to the next packet) restarts it.
 *
 * Freestanding C99, no allocation. One instance per channel of a receiver.
 */
#ifndef RS_STITCH_H
#define RS_STITCH_H
#include "rs_decoder.h"
#ifdef __cplusplus
extern "C" {
#endif

#define RS_STITCH_RES 4                               /* accumulator cells per chip */
#define RS_STITCH_N   (RS_PKT_CHIPS * RS_STITCH_RES)

#define RS_STITCH_PIECES 12
typedef struct { float t, phase_cells; float val[RS_STITCH_N]; uint8_t has[RS_STITCH_N]; } rs_piece_t;

typedef struct {
    rs_piece_t piece[RS_STITCH_PIECES]; int npieces, phead;   /* recent pieces, each a frame's stretch of the cycle at its phase */
    float acc[RS_STITCH_N], wgt[RS_STITCH_N];         /* composite of the newest pieces that cover the cycle (built per frame) */
    int   have_anchor;                                /* a frame with a sync has been seen */
    float t_anchor, phase_anchor;                     /* time (s) and phase (chips at row 0, 0..82) of the last anchored frame */
    float anc_t[8], anc_phase[8]; int anc_n, anc_head;  /* recent anchors: the chip period is fitted to them */
    float chip_seconds;                               /* chip period in seconds; 0 = unknown (then only anchored frames are placed) */
    int   chip_seconds_locked;                        /* fitted to the anchors (vs the row-time guess) */
    float rpc;                                        /* rows per chip of the last placed piece */
    float rpc_est;                                    /* clock agreed by the syncs seen (median of the last few); a sync off by > 4 % is not an anchor */
    float rpc_seen[8]; int rpc_seen_n, rpc_seen_head;
    int   misses;                                     /* anchors rejected against the locked prediction in a row; 3 unlock */
    float chain_chips;                                /* chips travelled by the chain of unanchored pieces since its first piece */
    int   chain_mode;                                 /* the stored pieces are chained (arbitrary phase origin), not anchored */
    float rpc_chain;                                  /* clock used by the chain (run-length estimate), kept apart from the syncs' */
    int   chain_fixes;                                /* clock corrections made at the wrap of a chain */
    int   pieces;                                     /* pieces placed since the last decode */
    uint32_t placed_total, decoded_total, restarts;   /* statistics */
} rs_stitch_t;

void rs_stitch_init(rs_stitch_t *s);

typedef struct {
    const float *p, *norm, *amp;    /* raw profile, the detector's normalized profile and local amplitude (rs_decode_normalized) */
    int n;                          /* rows */
    const rs_sync_t *syncs; int ns; /* validated syncs of this frame (rs_decode_syncs / rs_decode_last_syncs) */
    float rpc_hint;                 /* the receiver's clock (0 = unknown) */
    float row_seconds;              /* sensor row time (0 = unknown: no phase prediction between frames) */
    float t;                        /* frame time, seconds */
    const rs_dec_cfg_t *cfg;        /* decoder configuration (min_contrast, exposure_rows; the composite is decoded with it) */
} rs_stitch_in_t;

/* Feed one frame of one channel. When the cycle is complete the packet is decoded into *out;
 * returns 1 then, else 0. */
int rs_stitch_feed(rs_stitch_t *s, const rs_stitch_in_t *in, rs_packet_t *out);

#ifdef __cplusplus
}
#endif
#endif
