/*
 * rs_decoder.h — RSLog rolling-shutter decoder.
 * Input: a per-row brightness profile p[0..n-1] (row 0 = first exposed row).
 * Output: decoded packets with row positions and estimated rows-per-chip.
 * Self-calibrating: the chip length in rows is measured from each packet's
 * sync pattern, so no camera-specific configuration is required.
 */
#ifndef RS_DECODER_H
#define RS_DECODER_H

#include "rs_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS_DEC_MAX_ROWS 4096

typedef struct {
    float min_rows_per_chip;  /* default 2.5 */
    float max_rows_per_chip;  /* default 48  */
    float sync_tol;           /* max relative mismatch between sync runs, default 0.30 */
    float min_contrast;       /* min local (max-min) to trust a row, profile units, default 6 */
    float pll_gain;           /* mid-bit edge phase correction, default 0.35 */
    float min_quality;        /* reject packets whose weakest bit confidence < this, default 0.05 */
    float rows_per_chip_hint; /* receiver's current estimate (0 = unknown); used by the edge path */
    int   use_edges;          /* also run the rising-edge path for saturated signals (default 1) */
} rs_dec_cfg_t;

typedef struct {
    uint8_t  id;              /* slot 0..7 */
    uint16_t seed;            /* 0..509 data/coded, 510/511 META */
    uint8_t  payload;
    float   row_start;        /* first row of the packet (gap chip) */
    float   row_end;          /* row after the last chip */
    float   rows_per_chip;
    float   quality;          /* min bit confidence, 0..1 */
} rs_packet_t;

typedef struct {
    int   syncs;              /* sync candidates found */
    int   crc_ok;
    int   crc_fail;
    int   start_fail;
    int   truncated;          /* sync found but packet ran past the last row */
    float rows_per_chip;      /* mean over valid packets, 0 if none */
    float contrast;           /* global max-min of the profile */
} rs_dec_stats_t;

void rs_dec_cfg_default(rs_dec_cfg_t *cfg);

/* Decode packets from a profile. Returns number of packets written to out
 * (deduplicated, sorted by row). n is clamped to RS_DEC_MAX_ROWS. */
int rs_decode_profile(const float *p, int n, const rs_dec_cfg_t *cfg,
                      rs_packet_t *out, int max_out, rs_dec_stats_t *st);

/* Debug: binarized profile of the last scale that produced a packet
 * (or the last scale tried). Values 0/1, or 2 for low-contrast rows. */
const uint8_t *rs_decode_debug_binary(int *n);

#ifdef __cplusplus
}
#endif
#endif
