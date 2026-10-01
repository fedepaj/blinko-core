/*
 * rs_decoder.c — Blinko rolling-shutter decoder, protocol v3 (RLL(2,7) line code, CRC-12).
 *
 * Input: a per-row brightness profile of one light (one colour channel or luma). Output: the
 * packets it contains, each with its row span, chip clock (rows per chip) and a fit quality.
 *
 * Pipeline (rs_decode_profile):
 *   1. Candidates, at every plausible scale (2..28 rows per chip): the profile is envelope-
 *      normalized for that scale (sliding min/max over 15 chips: the blob changes brightness
 *      several-fold along a packet), then two sync finders run — the binarized runs (a run of
 *      RS_SYNC_ON_CHIPS is a run-length violation of the code, data runs are 3..8 chips) and a
 *      correlation with the exposure-smeared sync template at 0.7x..1.3x of the scale (survives
 *      the chatter of half-height pulses that breaks the runs). Candidates are ranked globally.
 *   2. Validation, per candidate (no detector run spent): refine_clock() places the template;
 *      sync_on_rows() measures the ON run between its two 0.5-crossings — it must be 10 chips
 *      +-15 % (this rejects the blob's own bright edge posing as a sync) and it *is* the clock
 *      estimate (two edges 10 chips apart beat any correlation grid); the 5 chips before the gap
 *      must contain a bright chip (a real gap follows modulated signal: the filler runs are 4
 *      chips); the exposure must be under 3 chips (flatter templates fit anything).
 *   3. Detection (decode_candidate -> viterbi): a Viterbi search over the RLL(2,7) encoder
 *      states (tree node, level) whose chip templates include the camera exposure smear (a box
 *      of cfg->exposure_rows rows; 1.4x and 2x of it are also tried, phones under-report).
 *      Each codeword slips by +-1 row to track timing, and the slip drives a per-survivor clock
 *      PLL (0.3 * slip / cells, clamped +-6 %), so a clock hypothesis a few percent off converges.
 *      Clock hypotheses: the receiver's confirmed clock, the sync's, +-2.5 % (+-5 % with long
 *      exposure). Early aborts at codewords 5/9/17; the best survivor is checked once against
 *      the CRC-12 and, when it passes, re-fitted on a rigid grid (rigid_mse) to reject aliases.
 *   4. Framing beyond the sync: when the packet after the sync does not fit the blob, its tail
 *      is read one period earlier (cyclic decode: the transmitter may repeat packets, the seam
 *      sits half a normalization window before the blob's fade); and the packet *before* the
 *      sync is decoded too (backward: its data field ends at this gap and takes clock and phase
 *      from this sync). Every complete data field in the blob is read from the nearest sync.
 *
 * The budget is 24 detector runs per profile without a confirmed clock, 6 with one. Quality is
 * 1 / (1 + 30 * mean squared error per row) of the normalized profile; cfg->min_quality gates.
 * History: the v2 decoder thresholded the envelope-normalized profile, found the 4+4-chip
 * Manchester sync by run lengths and decided bits from the two half-bit integrals with a
 * mid-bit PLL, retrying a failed CRC with the receiver's clock and +-3 %. It stopped working
 * at an exposure of about one chip (the Samsung S21 FE's 57 us against 30 us chips) and never
 * saw the long-exposure, few-rows-per-chip regime this detector was written for; the reasons
 * for the v3 line code are in rs_proto.h. Steps 2 and 4 above and the clock PLL came from a
 * second pass on real Samsung RAW profiles (October 2026), where a brute-force decode at the
 * true clock succeeded while the detector produced no candidate within 7 % of it.
 * Freestanding C99: no libc, static scratch buffers, no allocation. -DRS_DEC_DEBUG traces the
 * candidates and hypotheses on stderr and keeps timing counters (rs_decode_debug_counters).
 */
#include "rs_decoder.h"

/* The decoder keeps its scratch (about 150 KB) in file-scope buffers: no allocation, and one
 * decode at a time per thread. With RS_DEC_THREADS defined they become thread-local, so the
 * receiver's channels can be decoded on several threads at once (rs_rx_t.parallel); each thread
 * that decodes then owns a copy. Platforms without threads leave it undefined. */
#ifdef RS_DEC_THREADS
#define RS_TLS __thread
#else
#define RS_TLS
#endif

typedef struct { uint8_t level; int start; int len; } rs_run_t;

static RS_TLS float    s_norm[RS_DEC_MAX_ROWS];      /* envelope-normalized profile, 0..1 */
static RS_TLS float    s_amp[RS_DEC_MAX_ROWS];       /* local amplitude (envelope max - min) */
static RS_TLS float    s_emin[RS_DEC_MAX_ROWS], s_emax[RS_DEC_MAX_ROWS];
static RS_TLS uint8_t  s_bin[RS_DEC_MAX_ROWS];       /* 0/1, 2 = too little contrast */
static RS_TLS uint8_t  s_dbg[RS_DEC_MAX_ROWS];
static RS_TLS int      s_dbg_n = 0;
static RS_TLS int      s_deque[RS_DEC_MAX_ROWS];
static RS_TLS rs_run_t s_runs[RS_DEC_MAX_ROWS + 1];
static RS_TLS float    s_prep_rpc = -1.0f;
static RS_TLS const float *s_prep_p = 0;
static RS_TLS int      s_prep_n = 0;

#ifdef RS_DEC_DEBUG
#include <stdio.h>
#define RS_DBG(...) fprintf(stderr, __VA_ARGS__)
#else
#define RS_DBG(...) ((void)0)
#endif
static float fabsf_(float x) { return x < 0 ? -x : x; }
static float e_rows_of(const rs_dec_cfg_t *cfg, float rpc) { return cfg->exposure_rows > 0 ? cfg->exposure_rows : 0.5f * rpc; }
static int   iroundf_(float x) { return (int)(x + (x >= 0 ? 0.5f : -0.5f)); }
static float sqrtf_(float x) { if (x <= 0) return 0; float r = x > 1 ? x : 1; for (int i = 0; i < 12; i++) r = 0.5f * (r + x / r); return r; }

void rs_dec_cfg_default(rs_dec_cfg_t *cfg)
{
    cfg->min_rows_per_chip = 1.2f;
    cfg->max_rows_per_chip = 30.0f;
    cfg->sync_tol = 0.30f;
    cfg->min_contrast = 6.0f;
    cfg->track_timing = 1;
    cfg->min_quality = 0.4f;          /* 1/(1+30 mse per row): true packets ~0.6-0.9, false accepts below ~0.3 */
    cfg->rows_per_chip_hint = 0.0f;
    cfg->timing_retries = 1;
    cfg->grid_decode = 1;
    cfg->exposure_rows = 0.0f;
}

/* ---- envelope: sliding min/max with monotonic deques, window w rows centred on each row */
static void sliding_minmax(const float *p, int n, int w)
{
    int half = w / 2;
    int head = 0, tail = 0;                              /* deque of indices with increasing values: min */
    for (int r = 0; r < n + half; r++) {
        if (r < n) {
            while (tail > head && p[s_deque[tail - 1]] >= p[r]) tail--;
            s_deque[tail++] = r;
        }
        int out = r - half;                                /* row whose window [out-half, out+half] is complete */
        if (out >= 0) {
            while (head < tail && s_deque[head] < out - half) head++;
            s_emin[out] = p[s_deque[head]];
        }
    }
    head = tail = 0;
    for (int r = 0; r < n + half; r++) {
        if (r < n) {
            while (tail > head && p[s_deque[tail - 1]] <= p[r]) tail--;
            s_deque[tail++] = r;
        }
        int out = r - half;
        if (out >= 0) {
            while (head < tail && s_deque[head] < out - half) head++;
            s_emax[out] = p[s_deque[head]];
        }
    }
}

static void prepare(const float *p, int n, float rpc, float min_contrast)
{
    int w = iroundf_(15.0f * rpc);
    if (w < 6) w = 6;
    if (w > n) w = n;
    sliding_minmax(p, n, w);
    for (int r = 0; r < n; r++) {
        float a = s_emax[r] - s_emin[r];
        s_amp[r] = a;
        if (a >= min_contrast) { s_norm[r] = (p[r] - s_emin[r]) / a; s_bin[r] = s_norm[r] >= 0.5f ? 1 : 0; }
        else { s_norm[r] = 0.5f; s_bin[r] = 2; }
    }
    s_prep_rpc = rpc; s_prep_p = p; s_prep_n = n;
}

/* Sub-row position where the normalized profile crosses 0.5 between rows a and a+1
 * (row r is sampled at r + 0.5). */
static float crossing(int a, int n)
{
    if (a < 0) a = 0;
    if (a + 1 >= n) return (float)a + 0.5f;
    float y0 = s_norm[a], y1 = s_norm[a + 1];
    float d = y1 - y0;
    float f = fabsf_(d) > 1e-6f ? (0.5f - y0) / d : 0.5f;
    if (f < 0) f = 0; if (f > 1) f = 1;
    return (float)a + 0.5f + f;
}

/* ---- exposure-smeared chip templates
 * Row r reads out at time (r + 0.5) (rows) and integrates the LED over the E rows before it.
 * With chips of rpc rows starting at `pos` (rows), the mean LED level seen by row r is the
 * integral of the chip levels over [x - E, x] with x = (r + 0.5 - pos) / rpc in chips (E in
 * chips), divided by E. Chips before the codeword come from the survivor's history. */
#define RS_HIST 8
#define RS_Q_SCALE 30.0f            /* quality = 1 / (1 + 30 * mean squared error per row of the normalized profile) */
static float tmpl_level(float x, float e, const uint8_t *hist, const uint8_t *cw, int k)
{
    if (e < 0.02f) {                                    /* no smear: the chip under x */
        int c = (int)(x < 0 ? x - 1 : x);
        if (c < 0) return c >= -RS_HIST ? hist[RS_HIST + c] : hist[0];
        return c < k ? cw[c] : cw[k - 1];
    }
    float lo = x - e, sum = 0;
    int c0 = (int)(lo < 0 ? lo - 1 : lo), c1 = (int)(x < 0 ? x - 1 : x);
    for (int c = c0; c <= c1; c++) {
        float a = (float)c < lo ? lo : (float)c, b = (float)(c + 1) > x ? x : (float)(c + 1);
        if (b <= a) continue;
        int lvl;
        if (c < 0) lvl = c >= -RS_HIST ? hist[RS_HIST + c] : hist[0];
        else lvl = c < k ? cw[c] : cw[k - 1];
        if (lvl) sum += b - a;
    }
    return sum / e;
}

/* ---- Viterbi over the RLL(2,7) encoder states */
typedef struct {
    uint8_t  valid, node, level, nbits, nhist_unused;
    int      ncells;
    float    metric, pos, rpc;                    /* rpc: the survivor's own clock estimate (slip-driven PLL) */
    uint64_t bits;
    uint8_t  hist[RS_HIST];
} surv_t;

static RS_TLS float s_rpc_cur = 1.0f;                 /* rows per chip of the running detector, for the per-row metric */
static RS_TLS int   s_wrap_at = 1 << 30;               /* cyclic decode: rows >= s_wrap_at read one packet period earlier (repeated packets) */
static RS_TLS int   s_period_rows = 0;
static inline float prof_at(int r) { if (r >= s_wrap_at) r -= s_period_rows; return s_norm[r]; }
static float per_cell(const surv_t *s) { return s->ncells > 0 ? s->metric / ((float)s->ncells * s_rpc_cur) : 0.0f; }   /* mean squared error per row */

static void merge(surv_t *dst, const surv_t *c)
{
    if (!dst->valid || per_cell(c) < per_cell(dst)) *dst = *c;
}

/* Detect RS_DATA_BITS bits starting at row `start` (true start of the first data chip) with
 * chip length rpc (rows) and exposure e (chips). Returns 1 and the bits / mean squared error /
 * end row when a full packet fits, 0 otherwise. */
static int   s_cnt[12];                          /* debug counters: 0 viterbi calls, 1 candidates run, 2 cyclic calls, 3 backward calls, 4 exposure retries, 5 early aborts, 6 viterbi us, 7 total us, 8 candidate-collection us, 9 refine us */
#ifdef RS_DEC_DEBUG
#include <time.h>
static long now_us_(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return (long)ts.tv_sec * 1000000L + ts.tv_nsec / 1000L; }
#define RS_TIMER(var) long var = now_us_()
#define RS_TIMED(idx, var) s_cnt[idx] += (int)(now_us_() - var)
#else
#define RS_TIMER(var) ((void)0)
#define RS_TIMED(idx, var) ((void)0)
#endif
int *rs_decode_debug_counters(void) { return s_cnt; }
static RS_TLS float s_vit_rpc = 0;                      /* clock of the last successful viterbi's best survivor (PLL output) */
static RS_TLS float s_vit_e = 0;                        /* exposure (rows) the winning hypothesis of decode_candidate used */
static RS_TLS int   s_vit_ncells = 0;                   /* cells the best survivor covered (data + flush, without the filler) */
static int viterbi(int n, float start, float rpc, float e, float slip_chips, float abort_mse,
                   uint32_t *bits_out, float *mse_out, float *end_out)
{
    surv_t cur[12], nxt[12];
    RS_TIMER(tv0);
    s_cnt[0]++; if (s_period_rows) s_cnt[2]++;
    s_rpc_cur = rpc;
    for (int i = 0; i < 12; i++) cur[i].valid = 0;
    surv_t s0; s0.valid = 1; s0.node = 0; s0.level = 0; s0.nbits = 0; s0.ncells = 0; s0.metric = 0; s0.pos = start; s0.bits = 0; s0.rpc = rpc;
    for (int i = 0; i < RS_HIST; i++) s0.hist[i] = 0;    /* the sync ends with OFF chips */
    cur[0] = s0;
    int slips = slip_chips > 0;

    for (int step = 0; step < RS_DATA_BITS + 3; step++) {
        for (int i = 0; i < 12; i++) nxt[i].valid = 0;
        int any = 0;
        for (int i = 0; i < 12; i++) {
            const surv_t *s = &cur[i];
            if (!s->valid) continue;
            if (s->nbits >= RS_DATA_BITS && s->node == 0) { merge(&nxt[i], s); any = 1; continue; }   /* done */
            int nb = s->nbits < RS_DATA_BITS ? 2 : 1;                                       /* flush with zeros */
            for (int bit = 0; bit < nb; bit++) {
                uint8_t cw[8], level = s->level; int k;
                uint8_t node = rs_rll27_step(s->node, (uint8_t)bit, &level, cw, &k);
                surv_t c = *s;
                c.node = node; c.level = level; c.nbits = (uint8_t)(s->nbits + 1); c.bits = (s->bits << 1) | (uint64_t)bit;
                if (k == 0) { merge(&nxt[node * 2 + level], &c); any = 1; continue; }
                /* the template is computed once at the survivor's position and clock; the timing
                 * slip is a shift of +-1 row of the profile under it (an integer-row PLL). +-1 row
                 * per codeword absorbs a 4 % clock error at 4 rows/cell but only 1 % at 15, so the
                 * slip also corrects the survivor's clock (a first-order PLL): a hypothesis that is
                 * off by a few percent converges instead of drifting out of the slip range. */
                float srpc = s->rpc;
                float best = -1, best_pos = 0; int best_sh = 0;
                int a0 = iroundf_(s->pos), rows = iroundf_((float)k * srpc);
                int S = slips ? 1 : 0;   /* wider slips let the detector fit the stream at 2/3 of its clock (an alias with a valid CRC); the PLL below takes the drift instead */
                int limit = s_wrap_at < n ? s_wrap_at + s_period_rows : n;        /* with a wrap, rows up to one period past it are readable */
                if (a0 - S < 0 || a0 + rows + S > limit) continue;
                float tmpl[256]; if (rows > 256) rows = 256;
                /* at many rows per chip the template is evaluated once per ds rows against the
                 * profile's mean over those rows (the exposure smear is wider than that anyway) */
                int ds = srpc >= 6.0f ? (int)(srpc / 4.0f + 0.5f) : 1; if (ds < 1) ds = 1; if (ds > 8) ds = 8;
                int nb = rows / ds;
                for (int j = 0; j < nb; j++) tmpl[j] = tmpl_level(((float)(a0 + j * ds) + 0.5f * (float)ds - s->pos) / srpc, e, s->hist, cw, k);
                for (int sh = -S; sh <= S; sh++) {
                    float err = 0;
                    if (ds == 1) { for (int j = 0; j < nb; j++) { float d = prof_at(a0 + sh + j) - tmpl[j]; err += d * d; } }
                    else {
                        float inv = 1.0f / (float)ds;
                        for (int j = 0; j < nb; j++) { float m = 0; int r0 = a0 + sh + j * ds; for (int q = 0; q < ds; q++) m += prof_at(r0 + q); float d = m * inv - tmpl[j]; err += d * d; }
                        err *= (float)ds;                                                          /* per-row scale, as with ds == 1 */
                    }
                    err += 0.02f * (float)(sh < 0 ? -sh : sh);
                    if (best < 0 || err < best) { best = err; best_pos = s->pos + (float)sh; best_sh = sh; }
                }
                if (best < 0) continue;                                                    /* runs past the profile */
                c.metric = s->metric + best; c.pos = best_pos + (float)k * srpc; c.ncells = s->ncells + k;
                c.rpc = srpc + 0.3f * (float)best_sh / (float)k;                           /* PLL: a slip of sh rows over k cells means the clock is off by sh/k rows per cell */
                if (c.rpc < 0.94f * rpc) c.rpc = 0.94f * rpc; if (c.rpc > 1.06f * rpc) c.rpc = 1.06f * rpc;
                { uint8_t tmp[RS_HIST + 8]; int m = 0;
                  for (int h = 0; h < RS_HIST; h++) tmp[m++] = s->hist[h];
                  for (int h = 0; h < k; h++) tmp[m++] = cw[h];
                  for (int h = 0; h < RS_HIST; h++) c.hist[h] = tmp[m - RS_HIST + h]; }
                merge(&nxt[node * 2 + level], &c); any = 1;
            }
        }
        if (!any) { RS_TIMED(6, tv0); return 0; }
        for (int i = 0; i < 12; i++) cur[i] = nxt[i];
        if (step == 5 || step == 9 || step == 17) {        /* early abort: a false sync or a wrong clock shows in the first codewords */
            float best = -1;
            for (int i = 0; i < 12; i++) if (cur[i].valid && cur[i].ncells >= 12 && (best < 0 || per_cell(&cur[i]) < best)) best = per_cell(&cur[i]);
            if (best > abort_mse) { s_cnt[5]++; RS_TIMED(6, tv0); return 0; }
        }
    }
    const surv_t *best = 0;
    for (int i = 0; i < 12; i++) {
        const surv_t *s = &cur[i];
        if (!s->valid || s->node != 0 || s->nbits < RS_DATA_BITS) continue;
        if (!best || per_cell(s) < per_cell(best)) best = s;
    }
    RS_TIMED(6, tv0);
    if (!best) return 0;
    *bits_out = (uint32_t)(best->bits >> (best->nbits - RS_DATA_BITS)) & ((1u << RS_DATA_BITS) - 1u);
    *mse_out = per_cell(best);
    *end_out = best->pos;
    s_vit_rpc = best->rpc; s_vit_ncells = best->ncells;
    return 1;
}



/* Length of the sync's ON run measured on the normalized profile: the 0.5-crossings nearest to
 * where the candidate puts the rising and the falling edge (searched within +-1.5 chips). A
 * candidate whose ON run is not ~RS_SYNC_ON_CHIPS long is not a sync: typically the blob's own
 * bright edge, which the correlation likes because the gap before it is dark too. Returns the
 * run length in rows, or -1 when an edge is not found. */
static float sync_on_rows(int n, float t_rise, float rpc, float *rise_out)
{
    float t_fall = t_rise + (float)RS_SYNC_ON_CHIPS * rpc;
    int w = (int)(1.5f * rpc) + 1;
    float r1 = -1, f1 = -1, bd = 1e9f;
    for (int a = iroundf_(t_rise) - w; a <= iroundf_(t_rise) + w; a++) {
        if (a < 0 || a + 1 >= n) continue;
        if (s_norm[a] < 0.5f && s_norm[a + 1] >= 0.5f) { float c = crossing(a, n), d = fabsf_(c - t_rise); if (d < bd) { bd = d; r1 = c; } }
    }
    if (r1 < 0) return -1;
    bd = 1e9f;
    for (int a = iroundf_(t_fall) - w; a <= iroundf_(t_fall) + w; a++) {
        if (a < 0 || a + 1 >= n) continue;
        if (s_norm[a] >= 0.5f && s_norm[a + 1] < 0.5f) { float c = crossing(a, n), d = fabsf_(c - t_fall); if (d < bd) { bd = d; f1 = c; } }
    }
    if (f1 < 0) return -1;
    if (rise_out) *rise_out = r1;
    return f1 - r1;
}

/* Rigid-grid check of a CRC-valid decode. The detector's per-codeword slips let it fit the
 * stream at 2/3 of its clock: runs of 3 and 4 cells become 4.5 and 6, and alternating 4/5-cell
 * codewords with a slip each time still fit soft edges well; once in a while that alias passes
 * the CRC-12 (always the same packet, so it recurs). A real packet lies on a rigid grid from
 * the sync to its end (the slips only track a smooth clock error), so the whole packet is
 * re-fitted with one clock = (end - start) / data cells and a global phase of +-2 rows: the
 * alias needs the alternating slips and fits much worse. Returns the rigid mse per row. */
static float rigid_mse(int n, uint32_t bits, float start, float end, int ncells, float e_rows)
{
    uint8_t chips[RS_PKT_CHIPS];
    uint32_t f = bits >> RS_CRC_BITS;
    rs_encode_packet((uint8_t)(f >> (RS_SEED_BITS + RS_PAYLOAD_BITS)), (uint16_t)((f >> RS_PAYLOAD_BITS) & RS_SEED_MASK), (uint8_t)(f & 0xFFu), chips);
    const uint8_t *data = chips + RS_SYNC_CHIPS;             /* the data field, preceded by the sync's OFF chips */
    static const uint8_t off_hist[RS_HIST] = { 0 };
    if (ncells < 20 || ncells > RS_DATA_CHIPS) return 1.0f;
    float rpc = (end - start) / (float)ncells;
    if (rpc <= 0) return 1.0f;
    float e = e_rows / rpc, best = -1;
    for (int ph = -2; ph <= 2; ph++) {
        float s0 = start + (float)ph, err = 0; int cnt = 0;
        int a = iroundf_(s0), b = iroundf_(s0 + (float)ncells * rpc);
        if (a < 0) a = 0;
        int limit = s_wrap_at < n ? s_wrap_at + s_period_rows : n;
        if (b > limit) b = limit;
        for (int r = a; r < b; r++) { float d = prof_at(r) - tmpl_level(((float)r + 0.5f - s0) / rpc, e, off_hist, data, RS_DATA_CHIPS); err += d * d; cnt++; }
        if (cnt < ncells) continue;
        err /= (float)cnt;
        if (best < 0 || err < best) best = err;
    }
    return best < 0 ? 1.0f : best;
}

/* One sync candidate: rising edge of the ON run at t0 (rows, 0.5-crossing), chip length rpc.
 * Chip-clock hypotheses, ML detection, one CRC check. */

static int decode_candidate(int n, const rs_dec_cfg_t *cfg, float t0, float rpc_ref, float amp,
                            int backward, rs_packet_t *pkt, rs_dec_stats_t *st)
{
    /* backward: decode the packet *before* this sync instead (its data field ends where this
     * sync's gap begins; this sync gives it clock and phase). Sync-free framing: every complete
     * data field in the blob is read from the nearest sync, its own or the next one, so a blob
     * holding one packet length yields a packet wherever the sync fell. */
    float e_rows = cfg->exposure_rows > 0 ? cfg->exposure_rows : 0.5f * rpc_ref;   /* measuring the exposure on the sync's edge was tried and was worse than the configured value */
    float abort_mse = 0.9f / RS_Q_SCALE / (cfg->min_quality > 0.05f ? cfg->min_quality : 0.05f);   /* q would end below ~min_quality */
    /* Clock hypotheses, tried until one passes the CRC: the receiver's confirmed clock first (the
     * most accurate estimate there is), then the sync's own (two 0.5-crossings 10 chips apart:
     * ~1 % with sharp edges, 5-8 % when a long exposure smears them into ramps, so the set widens
     * with the exposure). The Viterbi's PLL closes the remaining gap. */
    float cands[9]; int nc = 0;
    if (cfg->rows_per_chip_hint > 0 && fabsf_(cfg->rows_per_chip_hint - rpc_ref) < 0.15f * rpc_ref) cands[nc++] = cfg->rows_per_chip_hint;
    cands[nc++] = rpc_ref;
    if (cfg->timing_retries) { cands[nc++] = rpc_ref * 0.975f; cands[nc++] = rpc_ref * 1.025f; }
    if (cfg->timing_retries > 1 || e_rows > 2.0f * rpc_ref) { cands[nc++] = rpc_ref * 0.95f; cands[nc++] = rpc_ref * 1.05f; }
    uint32_t best_bits = 0; float best_mse = -1, best_end = 0, best_rpc = rpc_ref, best_pll = rpc_ref;
    int crc_ok = 0, ran = 0;                       /* ran: at least one detector run (else the packet did not fit) */
    /* usable end of the signal after the sync: where the local amplitude has fallen well below
     * the sync's own (the blob's soft edge: the envelope normalization, a 15-chip window, does
     * not follow a fade that short, so the normalized rows there are garbage even though they
     * still have some contrast) or the first stretch of two chips without contrast */
    int usable_end = n;
    {
        int run = 0, a = iroundf_(t0); if (a < 0) a = 0;
        float low = 0.6f * amp > cfg->min_contrast ? 0.6f * amp : cfg->min_contrast;
        for (int r = a; r < n; r++) {
            if (s_amp[r] < low) { if (++run >= iroundf_(0.5f * rpc_ref)) { usable_end = r - run + 1; break; } } else run = 0;
        }
    }
    int usable_start = 0;
    if (backward) {
        int run = 0, a = iroundf_(t0); if (a >= n) a = n - 1;
        float low = 0.6f * amp > cfg->min_contrast ? 0.6f * amp : cfg->min_contrast;
        for (int r = a; r >= 0; r--) {
            if (s_amp[r] < low) { if (++run >= iroundf_(0.5f * rpc_ref)) { usable_start = r + run; break; } } else run = 0;
        }
    }
    for (int c = 0; c < nc; c++) {
        float rpc = cands[c];
        float start = (t0 - 0.5f * e_rows) + (float)(RS_SYNC_ON_CHIPS + RS_SYNC_OFF_CHIPS) * rpc;
        float need = start + (float)RS_DATA_CHIPS * rpc;
        int period = iroundf_((float)RS_PKT_CHIPS * rpc);
        s_wrap_at = 1 << 30; s_period_rows = 0;
        if (backward) {
            need = t0 - 0.5f * e_rows - (float)RS_SYNC_GAP_CHIPS * rpc;       /* this sync's gap start */
            start = need - (float)RS_DATA_CHIPS * rpc;
            if (start - 0.5f * rpc < (float)usable_start) { RS_DBG("    hyp rpc %.3f backward: previous data field starts at %.0f, usable from %d\n", rpc, start, usable_start); if (c == 0) st->truncated++; continue; }
        } else
        if (need > (float)usable_end + 0.5f) {
            /* cyclic: primary clock hypothesis only (the PLL takes the rest); the +-2.5 % variants
             * would cost a detector run each, and the packet before the sync is tried instead */
            if (c > 0 && cands[c] != cfg->rows_per_chip_hint && cands[0] != cfg->rows_per_chip_hint) continue;
            if (c > 1) continue;
            /* cyclic decode: the transmitter may repeat packets, so the missing tail is one period
             * earlier, before this sync. The seam is put half a normalization window before the
             * usable end: the envelope normalization (a 15-chip window) lags the blob's fade, so
             * the last rows before usable_end are distorted while the rows one period earlier,
             * well inside the blob, are clean. The wrapped source rows must be modulated signal. */
            float gap_start = t0 - 0.5f * e_rows - (float)RS_SYNC_GAP_CHIPS * rpc;
            int wrap = usable_end - iroundf_(7.5f * rpc);
            if ((float)wrap < start + 2.0f * rpc) wrap = iroundf_(start + 2.0f * rpc);
            if (wrap > usable_end) wrap = usable_end;
            int src0 = wrap - period, src1 = iroundf_(need) - period;      /* wrapped source rows */
            float low = 0.6f * amp > cfg->min_contrast ? 0.6f * amp : cfg->min_contrast;
            if (src0 < 0 || src1 > iroundf_(gap_start) + 1 || s_amp[src0] < low || s_amp[(src0 + src1) / 2] < low || s_amp[src1 > 0 ? src1 - 1 : 0] < low) {
                RS_DBG("    hyp rpc %.3f: truncated (need %.0f usable_end %d src %d..%d)\n", rpc, need, usable_end, src0, src1); if (c == 0) st->truncated++; continue;
            }
            s_wrap_at = wrap; s_period_rows = period;
        }
        uint32_t bits; float mse, end;
        if (!s_period_rows) ran = 1;                   /* a cyclic run does not count: the packet did not fit, the caller then tries the one before the sync */
        if (s_period_rows) {
            /* cyclic: the wrapped tail is one period earlier, but the period in rows is only known
             * to the clock's precision (2 % of 82 chips is several rows), so a few period offsets
             * are tried and the best fit kept */
            int ok = 0; float bm = -1; uint32_t bb = 0; float be = 0, bp = rpc; int bn = 0;
            static const int offs[5] = { 0, -2, 2, -4, 4 };
            for (int d = 0; d < 5; d++) {
                s_period_rows = period + offs[d];
                uint32_t b2; float m2, e2;
                int r2 = viterbi(n, start, rpc, e_rows / rpc, (float)cfg->track_timing, abort_mse, &b2, &m2, &e2);
                if (d == 0 && !r2) break;                  /* aborted at the nominal period: nothing packet-like here, the seam cannot fix that */
                if (!r2) continue;
                if (!ok || m2 < bm) { ok = 1; bm = m2; bb = b2; be = e2; bp = s_vit_rpc; bn = s_vit_ncells; }
                uint32_t f2 = b2 >> RS_CRC_BITS;           /* a valid CRC ends the search: the other offsets cannot do better than right */
                if (rs_crc_fields((uint8_t)(f2 >> (RS_SEED_BITS + RS_PAYLOAD_BITS)), (uint16_t)((f2 >> RS_PAYLOAD_BITS) & RS_SEED_MASK), (uint8_t)(f2 & 0xFFu)) == (uint16_t)(b2 & ((1u << RS_CRC_BITS) - 1u))) { bm = m2; bb = b2; be = e2; bp = s_vit_rpc; bn = s_vit_ncells; break; }
            }
            s_period_rows = period; s_vit_rpc = bp; s_vit_ncells = bn; s_vit_e = e_rows;
            if (!ok) { if (c == 0) { s_wrap_at = 1 << 30; s_period_rows = 0; return 0; } continue; }
            bits = bb; mse = bm; end = be;
        } else {
            /* phones report less exposure than their edges show (S21 FE: 57 us set, ~76 us measured):
             * the first clock hypothesis is also tried with 1.4x the exposure */
            int ok1 = viterbi(n, start, rpc, e_rows / rpc, (float)cfg->track_timing, abort_mse, &bits, &mse, &end);
            float pll = s_vit_rpc; int pn = s_vit_ncells; s_vit_e = e_rows;
            if (c == 0 && cfg->exposure_rows > 0) {
                /* phones under-report their exposure (S21 FE RAW: 57 us set, ~100 us on the edges):
                 * the first clock hypothesis is also tried at 1.4x and 2x the configured exposure */
                static const float ks[2] = { 1.4f, 2.0f };
                for (int q = 0; q < 2; q++) {
                    s_cnt[4]++;
                    uint32_t b2; float m2, e2;
                    float start2 = start - 0.5f * (ks[q] - 1.0f) * e_rows;   /* a longer exposure moves the template's origin back by half the extra smear (forward and backward alike) */
                    if (viterbi(n, start2, rpc, ks[q] * e_rows / rpc, (float)cfg->track_timing, abort_mse, &b2, &m2, &e2) && (!ok1 || m2 < mse)) { ok1 = 1; bits = b2; mse = m2; end = e2; pll = s_vit_rpc; pn = s_vit_ncells; s_vit_e = ks[q] * e_rows; start = start2; }
                }
            }
            s_vit_rpc = pll; s_vit_ncells = pn;
            if (!ok1) { if (c == 0) return 0; continue; }
        }
        uint32_t f = bits >> RS_CRC_BITS;
        RS_DBG("    hyp rpc %.3f start %.1f%s%s: mse %.4f q %.2f  fields id %u seed %u payload %u (usable_end %d, wrap %d, period %d, ncells %d, end %.0f)\n", rpc, start, s_period_rows ? " cyclic" : "", backward ? " backward" : "", mse, 1.0f / (1.0f + RS_Q_SCALE * mse),
               (unsigned)(f >> (RS_SEED_BITS + RS_PAYLOAD_BITS)), (unsigned)((f >> RS_PAYLOAD_BITS) & RS_SEED_MASK), (unsigned)(f & 0xFFu), usable_end, s_wrap_at, s_period_rows, s_vit_ncells, end);
        int ok = rs_crc_fields((uint8_t)(f >> (RS_SEED_BITS + RS_PAYLOAD_BITS)), (uint16_t)((f >> RS_PAYLOAD_BITS) & RS_SEED_MASK), (uint8_t)(f & 0xFFu))
                 == (uint16_t)(bits & ((1u << RS_CRC_BITS) - 1u));
        if (ok) {
            float rm = rigid_mse(n, bits, start, end, s_vit_ncells, s_vit_e > 0 ? s_vit_e : e_rows);
            RS_DBG("    crc ok: rigid mse %.4f vs %.4f\n", rm, mse);
            if (rm > 3.0f * mse + 0.01f) { st->crc_fail++; ok = 0; }   /* an alias of the stream at another clock */
        }
        if (ok && (best_mse < 0 || !crc_ok || mse < best_mse)) { best_mse = mse; best_bits = bits; best_end = end; best_rpc = rpc; best_pll = s_vit_rpc; crc_ok = 1; }
        else if (!crc_ok && (best_mse < 0 || mse < best_mse)) { best_mse = mse; }
        if (crc_ok) break;                                 /* first clock hypothesis that passes: done */
        if (c == 0 && mse > abort_mse * 0.8f) break;       /* not a packet at any nearby clock */
    }
    s_wrap_at = 1 << 30; s_period_rows = 0;
    if (!crc_ok) { if (best_mse >= 0) st->crc_fail++; return ran ? 0 : -1; }
    float q = 1.0f / (1.0f + RS_Q_SCALE * best_mse);
    if (q < cfg->min_quality) { st->crc_fail++; return 0; }
    uint32_t fields = best_bits >> RS_CRC_BITS;
    pkt->id = (uint8_t)(fields >> (RS_SEED_BITS + RS_PAYLOAD_BITS));
    pkt->seed = (uint16_t)((fields >> RS_PAYLOAD_BITS) & RS_SEED_MASK);
    pkt->payload = (uint8_t)(fields & 0xFFu);
    pkt->row_start = t0 - 0.5f * e_rows - (float)RS_SYNC_GAP_CHIPS * best_rpc - (backward ? (float)RS_PKT_CHIPS * best_rpc : 0.0f);
    pkt->row_end = best_end;
    pkt->rows_per_chip = best_pll;                 /* the PLL's clock: hypothesis + what the slips corrected */
    pkt->quality = q;
    pkt->amplitude = amp;
    st->crc_ok++;
    return 1;
}

/* ---- sync search by correlation with the exposure-smeared sync template.
 * Run-length syncs break down once the exposure smears the 3-chip gaps (E/T > ~1): the
 * normalized profile chatters around 0.5. Correlating the whole [gap][on][off] shape against
 * the profile is robust to that, and trying a few chip lengths around the scale gives the
 * clock to ~3 %, which the detector's hypotheses then refine. */
#define RS_MAX_CAND 6
#define RS_CAND_BUDGET 16          /* detector runs per profile, across scales */
typedef struct { float x, rpc, c; } cand_t;
static RS_TLS float s_tmpl[512];

static void add_cand(cand_t *cands, int *nc, float x, float rpc, float c)
{
    for (int i = 0; i < *nc; i++) {
        if (fabsf_(cands[i].x - x) < 3.0f * rpc) { if (c > cands[i].c) { cands[i].x = x; cands[i].rpc = rpc; cands[i].c = c; } return; }
    }
    if (*nc < RS_MAX_CAND) { cands[*nc].x = x; cands[*nc].rpc = rpc; cands[*nc].c = c; (*nc)++; return; }
    int w = 0; for (int i = 1; i < *nc; i++) if (cands[i].c < cands[w].c) w = i;
    if (c > cands[w].c) { cands[w].x = x; cands[w].rpc = rpc; cands[w].c = c; }
}

static int sync_correlate(int n, const rs_dec_cfg_t *cfg, float rpc, float e_rows, cand_t *cands, int *nc)
{
    static const uint8_t sync_chips[RS_SYNC_CHIPS] = { 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0 };
    static const uint8_t dark[RS_HIST] = { 0 };
    int d = rpc >= 4.0f ? (int)(rpc / 2.0f) : 1;                        /* decimation: the peak is refined by the detector's slips anyway */
    int L = iroundf_((float)RS_SYNC_CHIPS * rpc / (float)d);
    if (L < 8 || L > 512 || L * d >= n) return 0;
    float e = e_rows / rpc, mean = 0;
    for (int r = 0; r < L; r++) { s_tmpl[r] = tmpl_level(((float)(r * d) + 0.5f * d) / rpc, e, dark, sync_chips, RS_SYNC_CHIPS); mean += s_tmpl[r]; }
    mean /= (float)L;
    float tn = 0; for (int r = 0; r < L; r++) { s_tmpl[r] -= mean; tn += s_tmpl[r] * s_tmpl[r]; }
    if (tn < 1e-6f) return 0;
    int m = n / d;                                                       /* decimated profile length */
    float sum = 0, sq = 0;
    for (int r = 0; r < L; r++) { float v = s_norm[r * d]; sum += v; sq += v * v; }
    float prev = -1, cprev = -2; int found = 0;
    for (int x = 0; x + L <= m; x++) {
        if (x > 0) { float o = s_norm[(x - 1) * d], i = s_norm[(x + L - 1) * d]; sum += i - o; sq += i * i - o * o; }
        float wm = sum / (float)L, var = sq - (float)L * wm * wm;
        float c = 0;
        if (var > 1e-4f) {
            float dot = 0; const float *w = s_norm + x * d;
            for (int r = 0; r < L; r++) dot += (w[r * d] - wm) * s_tmpl[r];
            float dd = var * tn; c = dot / (dd > 0 ? sqrtf_(dd) : 1.0f);
        }
        if (prev >= 0.6f && prev >= cprev && prev >= c) { add_cand(cands, nc, (float)((x - 1) * d), rpc, prev); found++; }
        cprev = prev; prev = c;
    }
    return found;
}

/* Sync candidates from the binarized profile's runs: [OFF >= 2 chips][ON ~10][OFF >= 2], the ON
 * run's two 0.5-crossings giving the chip length. Cheap and precise while the exposure keeps
 * the edges sharp (E below ~1.2 chips); with longer exposures the correlation path is used. */
static int sync_runs(int n, const rs_dec_cfg_t *cfg, float rpc_hint, cand_t *cands, int *nc)
{
    int nr = 0;
    for (int r = 0; r < n; r++) {
        if (nr > 0 && s_runs[nr - 1].level == s_bin[r]) { s_runs[nr - 1].len++; continue; }
        s_runs[nr].level = s_bin[r]; s_runs[nr].start = r; s_runs[nr].len = 1; nr++;
    }
    int found = 0;
    for (int i = 1; i + 1 < nr; i++) {
        if (s_runs[i - 1].level != 0 || s_runs[i].level != 1 || s_runs[i + 1].level != 0) continue;
        int a0 = s_runs[i].start, a1 = s_runs[i + 1].start;
        float t_rise = crossing(a0 - 1, n), t_fall = crossing(a1 - 1, n);
        float rpc = (t_fall - t_rise) / (float)RS_SYNC_ON_CHIPS;
        if (rpc < cfg->min_rows_per_chip || rpc > cfg->max_rows_per_chip) continue;
        if (rpc < rpc_hint * 0.7f || rpc > rpc_hint * 1.3f) continue;
        if (cfg->rows_per_chip_hint > 0 && fabsf_(rpc - cfg->rows_per_chip_hint) > 0.12f * cfg->rows_per_chip_hint) continue;
        float e_rows = e_rows_of(cfg, rpc);
        float min_off = (float)(RS_SYNC_GAP_CHIPS - 1) * rpc * (1.0f - cfg->sync_tol);
        if ((float)s_runs[i - 1].len < min_off || (float)s_runs[i + 1].len < min_off) continue;
        /* run-length validation inside the packet: every run 3..8 chips of this clock */
        float lo = (float)RS_RLL_MIN_RUN * rpc * 0.75f, hi = (float)RS_RLL_MAX_RUN * rpc * 1.15f + e_rows;
        float end = t_rise + (float)(RS_SYNC_ON_CHIPS + RS_SYNC_OFF_CHIPS + RS_DATA_CHIPS) * rpc;
        int bad = 0;
        for (int j = i + 2; j < nr && (float)s_runs[j].start < end - 2.0f * rpc; j++) {
            if (s_runs[j].level == 2) break;
            float L = (float)s_runs[j].len;
            if (L < lo || L > hi) { bad = 1; break; }
        }
        if (bad) continue;
        float x = t_rise - 0.5f * e_rows - (float)RS_SYNC_GAP_CHIPS * rpc;   /* true start of the gap, template coordinates */
        /* rank: closest to the scale's own chip length first (a data run seen at a wrong scale
         * lands off-centre), so the budget goes to the plausible syncs */
        float dev = fabsf_(rpc / rpc_hint - 1.0f);
        add_cand(cands, nc, x, rpc, 0.95f - 0.5f * dev);
        found++;
    }
    return found;
}

/* candidates from one scale: normalization at that scale, then runs (short exposure) or
 * correlation at a few chip lengths (long exposure) */
static void collect_scale(const float *p, int n, const rs_dec_cfg_t *cfg, float rpc_hint, cand_t *cands, int *nc)
{
    prepare(p, n, rpc_hint, cfg->min_contrast);
    float e_rows = e_rows_of(cfg, rpc_hint);
    /* both sync finders at every scale: the runs give a precise clock when the binarized profile
     * is clean, the correlation survives chatter (half-height pulses from a long exposure or
     * from the blob's brightness gradient); add_cand merges duplicates */
    sync_runs(n, cfg, rpc_hint, cands, nc);
    if (cfg->rows_per_chip_hint > 0) {
        float h = cfg->rows_per_chip_hint;
        sync_correlate(n, cfg, h, e_rows, cands, nc); sync_correlate(n, cfg, h * 0.97f, e_rows, cands, nc); sync_correlate(n, cfg, h * 1.03f, e_rows, cands, nc);
    } else {
        for (int k = -2; k <= 2; k++) {
            float rpc = rpc_hint * (1.0f + 0.15f * (float)k);            /* 0.7x .. 1.3x of the scale */
            if (rpc < cfg->min_rows_per_chip || rpc > cfg->max_rows_per_chip) continue;
            if ((float)RS_PKT_CHIPS * rpc * 0.7f > (float)n) continue;   /* a packet would not fit the profile */
            sync_correlate(n, cfg, rpc, e_rows_of(cfg, rpc), cands, nc);
        }
    }
}

/* Local clock refinement: the correlation finds syncs on a 15 % clock grid, the detector's
 * hypotheses only span +-2.5 %, so the gap is closed here by correlating the smeared sync
 * template at fine clock steps (+-8 % in 1 % steps) and a few row offsets around the
 * candidate. Returns the best (x, rpc); undecimated, the template is short. */
static void refine_clock(int n, const rs_dec_cfg_t *cfg, float *x_io, float *rpc_io)
{
    static const uint8_t sync_chips[RS_SYNC_CHIPS] = { 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0 };
    static const uint8_t dark[RS_HIST] = { 0 };
    float best_c = -2, best_x = *x_io, best_rpc = *rpc_io;
    for (int k = -8; k <= 8; k += 2) {                 /* 2 % steps: the sync's ON run then gives the clock to ~1 %, this only has to place the edges */
        float rpc = *rpc_io * (1.0f + 0.01f * (float)k);
        if (rpc < cfg->min_rows_per_chip || rpc > cfg->max_rows_per_chip) continue;
        float e = e_rows_of(cfg, rpc) / rpc;
        int L = iroundf_((float)RS_SYNC_CHIPS * rpc);
        if (L < 8 || L > 512) continue;
        float mean = 0;
        for (int r = 0; r < L; r++) { s_tmpl[r] = tmpl_level(((float)r + 0.5f) / rpc, e, dark, sync_chips, RS_SYNC_CHIPS); mean += s_tmpl[r]; }
        mean /= (float)L;
        float tn = 0; for (int r = 0; r < L; r++) { s_tmpl[r] -= mean; tn += s_tmpl[r] * s_tmpl[r]; }
        if (tn < 1e-6f) continue;
        /* the gap start moves with the clock so that the ON run's centre stays put */
        float centre = *x_io + ((float)RS_SYNC_GAP_CHIPS + 0.5f * (float)RS_SYNC_ON_CHIPS) * *rpc_io;
        int x0 = iroundf_(centre - ((float)RS_SYNC_GAP_CHIPS + 0.5f * (float)RS_SYNC_ON_CHIPS) * rpc);
        int span = (int)(0.3f * rpc) + 2;
        for (int x = x0 - span; x <= x0 + span; x++) {
            if (x < 0 || x + L > n) continue;
            float sum = 0, sq = 0, dot = 0;
            for (int r = 0; r < L; r++) { float v = s_norm[x + r]; sum += v; sq += v * v; dot += v * s_tmpl[r]; }
            float wm = sum / (float)L, var = sq - (float)L * wm * wm;
            if (var <= 1e-4f) continue;
            float c = (dot - wm * 0.0f) / sqrtf_(var * tn);   /* template is zero-mean: dot already centred */
            if (c > best_c) { best_c = c; best_x = (float)x; best_rpc = rpc; }
        }
    }
    if (best_c > -1) { *x_io = best_x; *rpc_io = best_rpc; }
}

/* One validated sync candidate as a unit of parallel work (see rs_decode_profile, step 4). */
typedef struct {
    const float *p; int n; const rs_dec_cfg_t *cfg;
    float scale, x, t0, rpc, amp;
    rs_packet_t pk[2]; int npk;                   /* the packet after the sync and the one before it */
    rs_dec_stats_t st;
} rs_cjob_t;

static void cand_job(void *ctx, int i)
{
    rs_cjob_t *j = &((rs_cjob_t *)ctx)[i];
    if (s_prep_p != j->p || s_prep_n != j->n || fabsf_(s_prep_rpc - j->scale) > 1e-3f) prepare(j->p, j->n, j->scale, j->cfg->min_contrast);
    s_cnt[1]++;
    /* the packet after this sync; then the one before it, but only when this sync is trusted
     * (its packet decoded) or its own packet did not fit the blob: a sync whose packet fit and
     * failed the CRC is most likely not a sync, and every detector run on random data is a
     * 1/4096 chance of a false CRC pass */
    int fwd = decode_candidate(j->n, j->cfg, j->t0, j->rpc, j->amp, 0, &j->pk[j->npk], &j->st);
    if (fwd == 1) j->npk++;
    if (fwd != 0) { s_cnt[3]++; if (decode_candidate(j->n, j->cfg, j->t0, j->rpc, j->amp, 1, &j->pk[j->npk], &j->st) == 1) j->npk++; }
}

int rs_decode_profile(const float *p, int n, const rs_dec_cfg_t *cfg,
                      rs_packet_t *out, int max_out, rs_dec_stats_t *st)
{
    rs_dec_stats_t local; if (!st) st = &local;
    st->syncs = st->crc_ok = st->crc_fail = st->truncated = 0;
    st->rows_per_chip = 0; st->contrast = 0;
    if (n > RS_DEC_MAX_ROWS) n = RS_DEC_MAX_ROWS;
    if (n < 16 || max_out <= 0) return 0;

    RS_TIMER(tt0);
    float gmin = p[0], gmax = p[0];
    for (int r = 0; r < n; r++) { if (p[r] < gmin) gmin = p[r]; if (p[r] > gmax) gmax = p[r]; }
    st->contrast = gmax - gmin;
    if (st->contrast < cfg->min_contrast) return 0;

    /* 1. candidates from every plausible scale (each scale normalizes the profile for its own
     *    chip length and correlates the sync template at 0.7x..1.3x of it) */
    static const float scales[] = { 2.0f, 3.5f, 6.0f, 10.0f, 17.0f, 28.0f };
    static RS_TLS cand_t cands[6 * RS_MAX_CAND]; static RS_TLS float cand_scale[6 * RS_MAX_CAND];
    int nc = 0;
    float hint = cfg->rows_per_chip_hint;
    for (unsigned s = 0; s < sizeof(scales) / sizeof(scales[0]); s++) {
        if (scales[s] * 0.7f > cfg->max_rows_per_chip) break;
        if (scales[s] * 1.3f < cfg->min_rows_per_chip) continue;
        if (hint > 0 && (hint < scales[s] * 0.64f || hint > scales[s] * 1.36f)) continue;
        if ((float)RS_PKT_CHIPS * scales[s] * 0.7f > (float)n) break;    /* larger scales cannot hold a packet in this profile */
        cand_t local_c[RS_MAX_CAND]; int lc = 0;
        collect_scale(p, n, cfg, scales[s], local_c, &lc);
        for (int i = 0; i < lc && nc < 6 * RS_MAX_CAND; i++) { cands[nc] = local_c[i]; cand_scale[nc] = scales[s]; nc++; }
    }
    RS_TIMED(8, tt0);
    /* 2. best correlations first, across scales, within the detector budget */
    for (int i = 1; i < nc; i++) { cand_t k = cands[i]; float ks = cand_scale[i]; int j = i - 1; while (j >= 0 && cands[j].c < k.c) { cands[j + 1] = cands[j]; cand_scale[j + 1] = cand_scale[j]; j--; } cands[j + 1] = k; cand_scale[j + 1] = ks; }
    /* 3. validation (cheap, sequential): the candidates that survive, best first, up to the
     *    budget. The checks cost no detector run; they also fix the clock from the ON run. */
    int budget = hint > 0 ? 6 : 24;
    static RS_TLS rs_cjob_t jobs[24]; int nj = 0;       /* per calling thread: a channel's detector may itself run on a channel thread */
    for (int i = 0; i < nc && nj < budget; i++) {
        float rpc = cands[i].rpc, x = cands[i].x;
        if (s_prep_p != p || s_prep_n != n || fabsf_(s_prep_rpc - cand_scale[i]) > 1e-3f) prepare(p, n, cand_scale[i], cfg->min_contrast);
        { RS_TIMER(tr0); refine_clock(n, cfg, &x, &rpc); RS_TIMED(9, tr0); }
        float er = e_rows_of(cfg, rpc);
        float t0 = x + (float)RS_SYNC_GAP_CHIPS * rpc + 0.5f * er;      /* where the ON run's 0.5-crossing sits */
        int dup = 0;
        for (int j = 0; j < nj; j++) if (fabsf_(jobs[j].x - x) < 2.0f * rpc) { dup = 1; break; }   /* the same sync seen from another scale */
        if (dup) continue;
        st->syncs++;
        int a = iroundf_(x + RS_SYNC_GAP_CHIPS * rpc), b2 = iroundf_(x + (RS_SYNC_GAP_CHIPS + RS_SYNC_ON_CHIPS) * rpc);
        float amp = 0; int cnt = 0;
        for (int r = a; r < b2 && r < n; r++) if (r >= 0) { amp += s_amp[r]; cnt++; }
        amp = cnt ? amp / (float)cnt : 0;
        if (amp < cfg->min_contrast) continue;
        {
            /* the ON run must be RS_SYNC_ON_CHIPS long at this clock (+-15 %): rejects the blob's
             * bright edge posing as a sync (which the detector could otherwise fit at an aliased
             * clock with a valid CRC), and costs no detector run. Its length is also the best
             * clock estimate there is (two edges 10 chips apart, ~1 row each): the correlation's
             * clock is a 15 % grid refined by a weak template fit, 5 % off at times, more than
             * the detector's hypotheses and its PLL can take back at 15 rows/chip. */
            float rise = t0, on = sync_on_rows(n, t0, rpc, &rise);
            RS_DBG("cand scale %.1f x %.1f rpc %.3f: ON run %.1f rows (expect %.1f)\n", cand_scale[i], x, rpc, on, (float)RS_SYNC_ON_CHIPS * rpc);
            if (on < 0 || fabsf_(on - (float)RS_SYNC_ON_CHIPS * rpc) > 0.15f * (float)RS_SYNC_ON_CHIPS * rpc) continue;   /* no edge, or the wrong length */
            rpc = on / (float)RS_SYNC_ON_CHIPS; t0 = rise;
            er = e_rows_of(cfg, rpc);
            /* an exposure over 3 chips smears every run below full amplitude and the templates
             * go flat: the detector then fits anything (the stream at half its clock, say) */
            if (er > 3.0f * rpc) continue;
            /* a real gap follows modulated signal (the filler runs are 4 chips, so a bright chip
             * lies within 5 chips before the gap); the blob's dark surroundings do not, and a
             * sync "found" at the blob's edge is the other source of aliased decodes */
            {
                int a = iroundf_(x - 5.0f * rpc), b = iroundf_(x);
                if (a < 0 || b >= n) continue;
                float mx = p[a]; for (int r = a + 1; r < b; r++) if (p[r] > mx) mx = p[r];
                if (mx < s_emin[b] + 0.5f * amp) continue;
            }
        }
        RS_DBG("cand scale %.1f x %.1f rpc %.3f (from %.1f/%.3f) corr %.2f amp %.1f\n", cand_scale[i], x, rpc, cands[i].x, cands[i].rpc, cands[i].c, amp);
        rs_cjob_t *j = &jobs[nj++];
        j->p = p; j->n = n; j->cfg = cfg; j->scale = cand_scale[i]; j->x = x; j->t0 = t0; j->rpc = rpc; j->amp = amp; j->npk = 0;
        j->st.syncs = j->st.crc_ok = j->st.crc_fail = j->st.truncated = 0;
    }
    /* 4. detection (map): every validated candidate is independent — forward decode, then the
     *    packet before the sync — so they run through the platform's parallel hook when there is
     *    one (each job normalizes the profile for its own scale in its thread's scratch), else
     *    in turn on this thread. */
    if (cfg->parallel && nj > 1) cfg->parallel(cfg->parallel_user, nj, cand_job, jobs);
    else for (int i = 0; i < nj; i++) cand_job(jobs, i);
    /* 5. reduce: statistics, then the packets deduplicated by row (best quality kept) */
    int nout = 0;
    for (int i = 0; i < nj; i++) {
        st->crc_fail += jobs[i].st.crc_fail; st->truncated += jobs[i].st.truncated;
        for (int q = 0; q < jobs[i].npk; q++) {
            const rs_packet_t *pkt = &jobs[i].pk[q]; int d2 = 0;
            for (int j = 0; j < nout; j++) if (fabsf_(out[j].row_start - pkt->row_start) < 2.0f * jobs[i].rpc) { d2 = 1; if (pkt->quality > out[j].quality) out[j] = *pkt; break; }
            if (!d2 && nout < max_out) out[nout++] = *pkt;
        }
    }
    for (int i = 1; i < nout; i++) {                 /* sort by row */
        rs_packet_t k = out[i]; int j = i - 1;
        while (j >= 0 && out[j].row_start > k.row_start) { out[j + 1] = out[j]; j--; }
        out[j + 1] = k;
    }
    RS_TIMED(7, tt0);
    if (nout > 0) {
        float sum = 0; for (int i = 0; i < nout; i++) sum += out[i].rows_per_chip;
        st->rows_per_chip = sum / (float)nout;
        st->crc_ok = nout;
    }
    return nout;
}

const uint8_t *rs_decode_debug_binary(int *n) { if (n) *n = s_dbg_n; return s_dbg; }

/* Decode one packet at a known position (grid prediction): row_start is the first gap chip,
 * rpc the chip length in rows. No sync search: the ML detection and the CRC only. */
int rs_decode_at_prepared(const float *p, int n, const rs_dec_cfg_t *cfg, float row_start, float rpc, rs_packet_t *out)
{
    rs_dec_stats_t st = { 0 };
    if (n > RS_DEC_MAX_ROWS) n = RS_DEC_MAX_ROWS;
    if (n < 16 || rpc <= 0) return 0;
    if (s_prep_p != p || s_prep_n != n || fabsf_(s_prep_rpc - rpc) > 0.5f * rpc) prepare(p, n, rpc, cfg->min_contrast);
    float e_rows = cfg->exposure_rows > 0 ? cfg->exposure_rows : 0.5f * rpc;
    float t0 = row_start + (float)RS_SYNC_GAP_CHIPS * rpc + 0.5f * e_rows;   /* where the sync's 0.5-crossing would be */
    if (t0 < 0) return 0;
    rs_dec_cfg_t c1 = *cfg; c1.timing_retries = 0; c1.rows_per_chip_hint = 0; cfg = &c1;   /* the clock is given: one hypothesis */
    float amp = 0; int cnt = 0, a = iroundf_(t0);
    for (int r = a; r < a + iroundf_(RS_SYNC_ON_CHIPS * rpc) && r < n; r++) { if (r >= 0) { amp += s_amp[r]; cnt++; } }
    return decode_candidate(n, cfg, t0, rpc, cnt ? amp / (float)cnt : 0, 0, out, &st) == 1;
}

int rs_decode_at(const float *p, int n, const rs_dec_cfg_t *cfg, float row_start, float rpc, rs_packet_t *out)
{
    if (n > RS_DEC_MAX_ROWS) n = RS_DEC_MAX_ROWS;
    if (n < 16 || rpc <= 0) return 0;
    prepare(p, n, rpc, cfg->min_contrast);
    return rs_decode_at_prepared(p, n, cfg, row_start, rpc, out);
}
