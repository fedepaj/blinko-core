/*
 * rs_decoder.h — Blinko rolling-shutter decoder.
 * Input: a per-row brightness profile p[0..n-1] (row 0 = first exposed row).
 * Output: decoded packets with row positions and estimated rows-per-chip.
 * Self-calibrating: the chip length in rows is measured from each packet's
 * sync pattern, so no camera-specific configuration is required; the exposure
 * (rows) improves the detection when known. Protocol v3: RLL(2,7) chips, ML detection.
 */
#ifndef RS_DECODER_H
#define RS_DECODER_H

#include "rs_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS_DEC_MAX_ROWS 4096

/* Platform hook for parallel work: call job(ctx, i) for i = 0..count-1 on separate threads and
 * return when all are done. The core has no threads of its own; the decoder's scratch must be
 * thread-local (build with RS_DEC_THREADS) for the jobs to run concurrently. Used by the receiver
 * for a light's three channels and by the detector for its sync candidates. */
typedef void (*rs_parallel_fn)(void *user, int count, void (*job)(void *ctx, int i), void *ctx);

typedef struct {
    float min_rows_per_chip;  /* chip clocks searched, in rows per chip; default 1.2 */
    float max_rows_per_chip;  /* default 30 */
    float sync_tol;           /* tolerance of the sync's OFF runs against the clock, default 0.30 */
    float min_contrast;       /* min local (max-min) to trust a row, profile units, default 6 */
    int   track_timing;       /* per-codeword +-1 row timing slips driving the survivor's clock PLL (default 1) */
    float min_quality;        /* quality = 1/(1 + 30 mse per row of the normalized profile); default 0.4 */
    float rows_per_chip_hint; /* receiver's confirmed clock (0 = unknown): narrows the scales, is the first hypothesis, budget 6 instead of 24 */
    int   timing_retries;     /* clock hypotheses around the sync's: 1 = +-2.5 % (default), 2 = also +-5 %; 0 = the sync's only */
    int   grid_decode;        /* receiver: also decode at predicted grid positions next to decoded packets (default 1) */
    float exposure_rows;      /* camera exposure in rows (exposure_us / row_us); 0 = unknown, half a chip is assumed */
    rs_parallel_fn parallel;  /* optional: the validated sync candidates of a profile are detected in parallel (map), then merged (reduce) */
    void *parallel_user;
} rs_dec_cfg_t;

typedef struct {
    uint8_t  id;              /* slot 0..7 */
    uint16_t seed;            /* < len systematic, len..123 coded, 124/125 message CRCs, 126/127 META */
    uint8_t  payload;
    float   row_start;        /* first row of the packet (gap chip), template coordinates */
    float   row_end;          /* row after the last decoded cell */
    float   rows_per_chip;    /* the detector's clock after its PLL */
    float   quality;          /* template fit, 0..1 (see min_quality) */
    float   amplitude;        /* local envelope amplitude over the sync (profile units): how bright the light was */
} rs_packet_t;

typedef struct {
    int   syncs;              /* sync candidates run through the detector */
    int   crc_ok;             /* packets returned */
    int   crc_fail;           /* detector runs that completed and failed the CRC (or the quality / rigid-grid check) */
    int   truncated;          /* candidates whose packet did not fit the blob and could not be read cyclically or backward */
    float rows_per_chip;      /* mean over valid packets, 0 if none */
    float contrast;           /* global max-min of the profile */
} rs_dec_stats_t;

void rs_dec_cfg_default(rs_dec_cfg_t *cfg);

/* Decode packets from a profile. Returns number of packets written to out
 * (deduplicated, sorted by row). n is clamped to RS_DEC_MAX_ROWS. */
int rs_decode_profile(const float *p, int n, const rs_dec_cfg_t *cfg,
                      rs_packet_t *out, int max_out, rs_dec_stats_t *st);

/* Decode a packet at a known position (row of its gap chip, chip length in rows): no sync
 * search. Returns 1 and fills out when the start bit and CRC pass. */
int rs_decode_at(const float *p, int n, const rs_dec_cfg_t *cfg, float row_start, float rpc, rs_packet_t *out);
/* Same, reusing the cumulative sums of the last rs_decode_profile / rs_decode_at call on this
 * very profile (valid right after them; do not use after decoding another profile). */
int rs_decode_at_prepared(const float *p, int n, const rs_dec_cfg_t *cfg, float row_start, float rpc, rs_packet_t *out);

/* Debug: binarized profile of the last scale that produced a packet
 * (or the last scale tried). Values 0/1, or 2 for low-contrast rows. */
const uint8_t *rs_decode_debug_binary(int *n);

#ifdef __cplusplus
}
#endif
#endif
