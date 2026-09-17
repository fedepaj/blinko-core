#include "rs_decoder.h"
#ifdef RS_DEC_DEBUG
#include <stdio.h>
#define DBG(...) fprintf(stderr, __VA_ARGS__)
#else
#define DBG(...) do {} while (0)
#endif

/* ---- static workspace (decoder runs on the phone, not on the MCU) ---- */
static float   s_cum[RS_DEC_MAX_ROWS + 1];
static float   s_emin[RS_DEC_MAX_ROWS];
static float   s_emax[RS_DEC_MAX_ROWS];
static uint8_t s_bin[RS_DEC_MAX_ROWS];
static uint8_t s_dbg[RS_DEC_MAX_ROWS];
static int     s_dbg_n = 0;
static int     s_deque[RS_DEC_MAX_ROWS];

typedef struct { uint8_t level; int start; int len; } rs_run_t;
static rs_run_t s_runs[RS_DEC_MAX_ROWS + 1];

void rs_dec_cfg_default(rs_dec_cfg_t *cfg)
{
    cfg->min_rows_per_chip = 2.5f;
    cfg->max_rows_per_chip = 48.0f;
    cfg->sync_tol = 0.30f;
    cfg->min_contrast = 6.0f;
    cfg->pll_gain = 0.35f;
    cfg->min_quality = 0.05f;
    cfg->rows_per_chip_hint = 0.0f;
    cfg->use_edges = 0;   /* experimental: measured worse than the classic path on the corpus and in simulation */
    cfg->timing_retries = 1;
}

static float fabsf_(float x) { return x < 0 ? -x : x; }

/* Integral of p over [0, x) with p piecewise constant on [r, r+1). */
static float integ(const float *p, int n, float x)
{
    if (x <= 0) return 0;
    if (x >= n) return s_cum[n];
    int i = (int)x;
    return s_cum[i] + (x - (float)i) * p[i];
}

/* Centered sliding window min and max (window w) via monotonic deques. */
static void sliding_minmax(const float *p, int n, int w)
{
    if (w < 3) w = 3;
    int half = w / 2;
    /* max */
    int head = 0, tail = 0;
    for (int i = 0; i < n + half; i++) {
        if (i < n) {
            while (tail > head && p[s_deque[tail - 1]] <= p[i]) tail--;
            s_deque[tail++] = i;
        }
        int out = i - half;              /* row whose window [out-half, out+half] is complete */
        if (out >= 0) {
            while (s_deque[head] < out - half) head++;
            s_emax[out] = p[s_deque[head]];
        }
    }
    head = 0; tail = 0;
    for (int i = 0; i < n + half; i++) {
        if (i < n) {
            while (tail > head && p[s_deque[tail - 1]] >= p[i]) tail--;
            s_deque[tail++] = i;
        }
        int out = i - half;
        if (out >= 0) {
            while (s_deque[head] < out - half) head++;
            s_emin[out] = p[s_deque[head]];
        }
    }
}

/* Sub-row threshold crossing between rows a and a+1 (row centres at r+0.5). */
static float crossing(const float *p, int a, float thr)
{
    float pa = p[a], pb = p[a + 1];
    float d = pb - pa;
    float t = (d == 0) ? 0.5f : (thr - pa) / d;
    if (t < 0) t = 0; if (t > 1) t = 1;
    return (float)a + 0.5f + t;
}

/* Find a threshold crossing of the given direction (+1 rising, -1 falling)
 * closest to x in [x-r, x+r]. Returns -1 if none. */
static float find_edge(const float *p, int n, float x, float r, float thr, int dir)
{
    int lo = (int)(x - r); if (lo < 0) lo = 0;
    int hi = (int)(x + r); if (hi > n - 2) hi = n - 2;
    float best = -1, best_d = 1e30f;
    for (int a = lo; a <= hi; a++) {
        float da = p[a] - thr, db = p[a + 1] - thr;
        int ok = (dir > 0) ? (da < 0 && db >= 0) : (da >= 0 && db < 0);
        if (!ok) continue;
        float c = crossing(p, a, thr);
        float d = fabsf_(c - x);
        if (d < best_d) { best_d = d; best = c; }
    }
    return best;
}

/* Decode the 29 bits following a sync ending at t2 (start of start bit).
 *
 * Bit decision from the rising edges: Manchester 1 = "01" has an OFF->ON
 * transition at mid-bit, 0 = "10" has ON->OFF. With a bright, saturating LED
 * the sensor's exposure window makes every ON run grow into the following
 * OFF chip (falling edges arrive late, OFF runs shrink), but OFF->ON edges
 * stay exact. So: rising edge near mid-bit -> 1, otherwise 0. The half-bit
 * integrals only provide the confidence and a sanity check for zeros.
 * Returns 1 on CRC ok, 0 otherwise. Fills pkt on success. */
static int decode_bits(const float *p, int n, const rs_dec_cfg_t *cfg,
                       float t0, float t2, float rpc, float smear, rs_packet_t *pkt, rs_dec_stats_t *st)
{
    float pos = t2;
    float quality = 1.0f;
    uint32_t bits = 0; /* 29 bits, MSB first: start + id(3) seed(9) payload(8) crc(8) */
    (void)smear;
    int edge_mode = 0;   /* the classic path only sees symmetric syncs; asymmetry is handled by decode_edges */
    for (int k = 0; k < RS_PKT_BITS; k++) {
        float a = pos, m = pos + rpc, b = pos + 2 * rpc;
        if (b > (float)n + 0.01f) { st->truncated++; return 0; }
        float first = integ(p, n, m) - integ(p, n, a);
        float second = integ(p, n, b) - integ(p, n, m);
        int bit; float conf;
        if (!edge_mode) {
            bit = second > first;
            conf = fabsf_(second - first) / (first + second + 1e-6f);
            float thr = (first + second) / (2.0f * rpc);
            float edge = find_edge(p, n, m, 0.5f * rpc, thr, bit ? +1 : -1);
            if (edge >= 0) pos += cfg->pll_gain * (edge - m);
        } else {
            int im = (int)m; if (im < 0) im = 0; if (im >= n) im = n - 1;
            float thr = 0.5f * (s_emax[im] + s_emin[im]);          /* local envelope midpoint */
            float edge = find_edge(p, n, m, 0.4f * rpc, thr, +1);
            if (edge >= 0) {
                bit = 1;
                conf = (second - first) / (first + second + 1e-6f);
                if (conf < 0.1f) conf = 0.1f;
                pos += cfg->pll_gain * (edge - m);                /* rising edges are exact */
            } else {
                bit = 0;
                /* the delayed falling edge should sit between mid-bit and mid-bit + smear */
                float fe = find_edge(p, n, m + 0.5f * smear, 0.5f * smear + 0.4f * rpc, thr, -1);
                conf = (fe >= 0) ? 0.3f : 0.05f;
                float c2 = (first - second) / (first + second + 1e-6f);
                if (c2 > conf) conf = c2;
            }
        }
        if (conf < quality) quality = conf;
        bits = (bits << 1) | (uint32_t)bit;
        pos += 2 * rpc;
    }
    /* start bit must be 0 */
    if (bits & (1u << (RS_PKT_BITS - 1))) { st->start_fail++; return 0; }
    uint8_t  id      = (uint8_t)((bits >> 25) & 7u);
    uint16_t seed    = (uint16_t)((bits >> 16) & 511u);
    uint8_t  payload = (uint8_t)((bits >> 8) & 255u);
    uint8_t  crc     = (uint8_t)(bits & 255u);
    if (rs_crc_fields(id, seed, payload) != crc) { st->crc_fail++; return 0; }
    if (quality < cfg->min_quality) { st->crc_fail++; return 0; }
    pkt->id = id; pkt->seed = seed; pkt->payload = payload;
    pkt->row_start = t0 - rpc;
    pkt->row_end = pos;
    pkt->rows_per_chip = rpc;
    pkt->quality = quality;
    st->crc_ok++;
    return 1;
}

static int decode_scale(const float *p, int n, const rs_dec_cfg_t *cfg, float rpc_hint,
                        rs_packet_t *out, int max_out, int nout, rs_dec_stats_t *st)
{
    int w = (int)(rpc_hint * 9.0f);   /* window must span > 8 chips (sync = 4+4) */
    if (w > n) w = n;
    sliding_minmax(p, n, w);

    /* binarize */
    for (int r = 0; r < n; r++) {
        float c = s_emax[r] - s_emin[r];
        if (c < cfg->min_contrast) { s_bin[r] = 2; continue; }
        s_bin[r] = p[r] >= 0.5f * (s_emax[r] + s_emin[r]) ? 1 : 0;
    }
    /* run-length encode */
    int nr = 0;
    for (int r = 0; r < n; r++) {
        if (nr > 0 && s_runs[nr - 1].level == s_bin[r]) { s_runs[nr - 1].len++; continue; }
        s_runs[nr].level = s_bin[r]; s_runs[nr].start = r; s_runs[nr].len = 1; nr++;
    }
    /* scan for sync: low(gap), high L1, low L2, high L3(~1 chip) */
    for (int i = 1; i + 2 < nr; i++) {
        if (s_runs[i - 1].level != 0 || s_runs[i].level != 1 || s_runs[i + 1].level != 0 || s_runs[i + 2].level != 1) continue;
        float L1 = (float)s_runs[i].len, L2 = (float)s_runs[i + 1].len, L3 = (float)s_runs[i + 2].len;
        float rpc = (L1 + L2) / 8.0f;                 /* smear cancels: (4rpc + s) + (4rpc - s) */
        if (rpc < cfg->min_rows_per_chip || rpc > cfg->max_rows_per_chip) continue;
        if (rpc < rpc_hint * 0.45f || rpc > rpc_hint * 2.2f) continue;  /* belongs to another scale */
        /* the ON run may exceed the OFF run by the exposure smear (up to ~1 chip), never the reverse */
        if (fabsf_(L1 - L2) > cfg->sync_tol * 0.5f * (L1 + L2)) continue;   /* asymmetric syncs: see decode_edges */
        float smear = 0.0f;
        if (L3 < 0.35f * rpc || L3 > 1.9f * rpc) continue;
        if ((float)s_runs[i - 1].len < 0.35f * rpc) continue;
        st->syncs++;
        /* sub-row edge positions */
        int a0 = s_runs[i].start - 1, a1 = s_runs[i + 1].start - 1, a2 = s_runs[i + 2].start - 1;
        float thr0 = 0.5f * (s_emax[a0 + 1] + s_emin[a0 + 1]);
        float thr1 = 0.5f * (s_emax[a1 + 1] + s_emin[a1 + 1]);
        float thr2 = 0.5f * (s_emax[a2 + 1] + s_emin[a2 + 1]);
        float t0 = crossing(p, a0, thr0);                 /* rising: exact */
        float t1 = crossing(p, a1, thr1) - smear;         /* falling: delayed by the smear */
        float t2 = crossing(p, a2, thr2);                 /* rising: exact */
        float rpc_ref = (t2 - t0) / 8.0f;
        if (fabsf_((t1 - t0) / 4.0f - rpc_ref) > cfg->sync_tol * rpc_ref) continue;
        rs_packet_t pkt;
        int ok = decode_bits(p, n, cfg, t0, t2, rpc_ref, smear, &pkt, st);
        if (!ok && cfg->timing_retries) {
            /* Timing hypotheses: the 8-chip sync alone fixes the chip length to ~2 %, which is
             * 1.4 chips of drift at the end of the packet when the PLL loses the edges (saturated
             * or noisy rows). Retry with the receiver's chip clock (a mean over many packets and
             * frames) and with the sync estimate stretched by +-3 %. A retried packet must be
             * decoded with higher confidence than a first-try one (CRC-8 alone would let
             * ~1/256 of the corrupted syncs through per hypothesis). */
            float cand[3]; int nc = 0;
            if (cfg->rows_per_chip_hint > 0 && fabsf_(cfg->rows_per_chip_hint - rpc_ref) < 0.2f * rpc_ref &&
                fabsf_(cfg->rows_per_chip_hint - rpc_ref) > 0.002f * rpc_ref) cand[nc++] = cfg->rows_per_chip_hint;
            cand[nc++] = rpc_ref * 1.03f; cand[nc++] = rpc_ref * 0.97f;
            rs_dec_stats_t scratch = *st;
            for (int c = 0; c < nc && !ok; c++) {
                rs_dec_stats_t tmp = scratch;
                if (decode_bits(p, n, cfg, t0, t2, cand[c], smear, &pkt, &tmp) && pkt.quality >= 2.0f * cfg->min_quality) {
                    ok = 1; st->retry_ok++;
                }
            }
        }
        if (ok) {
            if (nout < max_out) out[nout++] = pkt;
            /* skip runs inside this packet */
            while (i + 1 < nr && (float)s_runs[i + 1].start < pkt.row_end - 1.0f) i++;
        }
    }
    /* keep the binary map for debugging */
    for (int r = 0; r < n; r++) s_dbg[r] = s_bin[r];
    s_dbg_n = n;
    return nout;
}


/* ------------------------------------------------------------------------
 * Saturated / long-exposure regime ("edge path").
 *
 * With a bright LED the ON state spreads into the following chip: single-chip
 * OFF gaps become shallow notches while 2- and 4-chip gaps still reach the
 * dark level. Level thresholds miss the notches and merge the sync's ON run
 * with the preceding data. What survives exactly is the position of every
 * OFF->ON transition. So: rising edges = positive gradient peaks (relative
 * to the local brightness), the sync = a deep gap of 4 chips whose end is a
 * rising edge and whose start-of-sync rising edge sits exactly 8 chips
 * earlier, bits = rising edge within +-0.4 chip of mid-bit.
 * ---------------------------------------------------------------------- */
#define RS_MAX_EVENTS 2048
static float s_ev[RS_MAX_EVENTS];        /* sub-row positions of rising edges */
static float s_evw[RS_MAX_EVENTS];       /* their relative strength 0..1 */
static int   s_nev;
static float s_locmax[RS_DEC_MAX_ROWS];  /* local maximum level (window ~ 3 chips) */

static void rising_events(const float *p, int n, int win)
{
    /* local max over a centred window (reuse the deque helper: fills s_emax/s_emin) */
    sliding_minmax(p, n, win);
    for (int r = 0; r < n; r++) s_locmax[r] = s_emax[r];
    s_nev = 0;
    for (int r = 1; r < n - 2; r++) {
        float d = p[r + 1] - p[r];
        if (d <= 0) continue;
        float dl = p[r] - p[r - 1], dr = p[r + 2] - p[r + 1];
        if (d < dl || d < dr) continue;                       /* gradient peak */
        float level = s_locmax[r + 1] - s_emin[r + 1];
        if (level < 6.0f) continue;
        float rel = d / level;
        if (rel < 0.10f) continue;                            /* too weak vs local range */
        /* merge with a peak 1 row earlier (same edge spread over 2 rows) */
        if (s_nev > 0 && (float)r + 0.5f - s_ev[s_nev - 1] < 1.6f) {
            if (rel > s_evw[s_nev - 1]) { s_ev[s_nev - 1] = (float)r + 0.5f; s_evw[s_nev - 1] = rel > 1 ? 1 : rel; }
            continue;
        }
        if (s_nev >= RS_MAX_EVENTS) break;
        s_ev[s_nev] = (float)r + 0.5f; s_evw[s_nev] = rel > 1 ? 1 : rel; s_nev++;
    }
}

/* index of the strongest rising event within [x - r, x + r], or -1 */
static int event_near(float x, float r)
{
    int best = -1; float bw = 0;
    for (int i = 0; i < s_nev; i++) {
        float d = s_ev[i] - x;
        if (d < -r) continue;
        if (d > r) break;
        if (best < 0 || s_evw[i] > bw) { best = i; bw = s_evw[i]; }
    }
    return best;
}

static int decode_bits_edges(int n, const rs_dec_cfg_t *cfg, float t0, float t2, float rpc,
                             rs_packet_t *pkt, rs_dec_stats_t *st)
{
    float pos = t2, quality = 1.0f;
    uint32_t bits = 0;
    int ones = 0, prev = -1, pairs = 0, pairs_ok = 0;
    for (int k = 0; k < RS_PKT_BITS; k++) {
        float m = pos + rpc, b = pos + 2 * rpc;
        if (b > (float)n + 0.01f) { st->truncated++; return 0; }
        int e = event_near(m, 0.4f * rpc);
        int bit; float conf;
        if (e >= 0) {
            bit = 1; ones++; conf = s_evw[e];
            pos += cfg->pll_gain * (s_ev[e] - m);
            if (k >= 2) {                                        /* refine the chip period from this exact edge */
                float est = (s_ev[e] - t2) / (2.0f * k + 1.0f);
                if (est > 0.8f * rpc && est < 1.25f * rpc) rpc = 0.75f * rpc + 0.25f * est;
            }
        }
        else {
            bit = 0;
            int e2 = event_near(m, 0.75f * rpc);
            conf = (e2 < 0) ? 0.6f : 0.15f;
            /* Manchester structure: "0,0" = 10|10 has an OFF->ON edge at the bit boundary */
            if (prev == 0) {
                pairs++;
                int eb = event_near(pos, 0.4f * rpc);
                if (eb >= 0) { pairs_ok++; pos += cfg->pll_gain * (s_ev[eb] - pos); }
            }
        }
        if (conf < quality) quality = conf;
        bits = (bits << 1) | (uint32_t)bit;
        prev = bit;
        pos += 2 * rpc;
    }
    /* reject silence decoded as zeros and garbage: need real ones and the "0,0" boundary edges */
    DBG(" [bits ones=%d pairs=%d/%d", ones, pairs_ok, pairs);
    if (ones < 3) { st->crc_fail++; return 0; }
    if (pairs >= 2 && pairs_ok * 10 < pairs * 7) { st->crc_fail++; return 0; }
    /* every rising edge inside the packet must be explained by a 1 or a "0,0" boundary */
    int seen = 0;
    for (int i = 0; i < s_nev; i++) { if (s_ev[i] < t2 + 0.5f * rpc) continue; if (s_ev[i] > pos - 0.5f * rpc) break; seen++; }
    DBG(" seen=%d]", seen);
    if (seen > ones + pairs_ok + 2) { st->crc_fail++; return 0; }
    if (bits & (1u << (RS_PKT_BITS - 1))) { st->start_fail++; return 0; }
    uint8_t  id      = (uint8_t)((bits >> 25) & 7u);
    uint16_t seed    = (uint16_t)((bits >> 16) & 511u);
    uint8_t  payload = (uint8_t)((bits >> 8) & 255u);
    uint8_t  crc     = (uint8_t)(bits & 255u);
    if (rs_crc_fields(id, seed, payload) != crc) { st->crc_fail++; return 0; }
    pkt->id = id; pkt->seed = seed; pkt->payload = payload;
    pkt->row_start = t0 - rpc; pkt->row_end = pos; pkt->rows_per_chip = rpc; pkt->quality = quality;
    st->crc_ok++;
    return 1;
}

/* Deep gaps: rows below min + 0.25 * range in a window, runs >= min_len rows. */
static int decode_edges(const float *p, int n, const rs_dec_cfg_t *cfg,
                        rs_packet_t *out, int max_out, int nout, rs_dec_stats_t *st)
{
    int win = (int)(cfg->max_rows_per_chip * 4.0f); if (win > n) win = n; if (win < 24) win = 24;
    rising_events(p, n, win);                              /* also fills s_emin/s_emax with win */
    DBG("[edges] n=%d win=%d events=%d\n", n, win, s_nev);
    if (s_nev < 8) return nout;
    /* deep-gap runs */
    int nr = 0;
    for (int r = 0; r < n; r++) {
        float range = s_emax[r] - s_emin[r];
        uint8_t deep = (range >= cfg->min_contrast) && (p[r] < s_emin[r] + 0.25f * range);
        if (nr > 0 && s_runs[nr - 1].level == deep) { s_runs[nr - 1].len++; continue; }
        s_runs[nr].level = deep; s_runs[nr].start = r; s_runs[nr].len = 1; nr++;
    }
    for (int i = 0; i < nr; i++) {
        if (s_runs[i].level != 1) continue;
        float L = (float)s_runs[i].len;
        if (L < 2.4f * cfg->min_rows_per_chip) continue;
        int e_end = s_runs[i].start + s_runs[i].len;             /* first bright row after the gap */
        int ie = event_near((float)e_end, 2.5f);
        DBG("[edges] deep gap rows %d-%d (len %d) end-event %s\n", s_runs[i].start, e_end, s_runs[i].len, ie >= 0 ? "yes" : "NO");
        if (ie < 0) continue;
        float t2 = s_ev[ie];
        /* Chip-period hypotheses. The sync-start rising edge (8 chips before t2) often does
         * not exist: the single gap chip before the sync is erased when the previous data
         * chip was ON. So besides exact edges in the t0 window we try the receiver's own
         * estimate and the gap length (4 chips minus a smear of ~0.7..1.3 chips). */
        float cand[8]; int ncand = 0;
        float lo = t2 - 8.0f * L / 2.7f, hi = t2 - 8.0f * L / 4.0f;
        for (int j = 0; j < s_nev && ncand < 4; j++) {
            if (s_ev[j] < lo) continue;
            if (s_ev[j] > hi) break;
            cand[ncand++] = (t2 - s_ev[j]) / 8.0f;
        }
        if (cfg->rows_per_chip_hint > 0) cand[ncand++] = cfg->rows_per_chip_hint;
        cand[ncand++] = L / 3.0f;
        cand[ncand++] = L / 3.4f;
        int decoded = 0;
        for (int c = 0; c < ncand && !decoded; c++) {
            float rpc = cand[c];
            if (rpc < cfg->min_rows_per_chip || rpc > cfg->max_rows_per_chip) continue;
            int dup = 0; for (int q = 0; q < c; q++) if (fabsf_(cand[q] - rpc) < 0.04f * rpc) dup = 1;
            if (dup) continue;
            float t0 = t2 - 8.0f * rpc;
            float smear = (float)s_runs[i].start - (t0 + 4.0f * rpc);
            DBG("[edges]   t2=%.1f rpc=%.2f (cand %d) smear=%.2f rpc", t2, rpc, c, smear / rpc);
            if (smear < -0.4f * rpc || smear > 1.4f * rpc) { DBG(" -> smear out\n"); continue; }
            /* start bit "10" is followed by a rising edge at t2 + 2 chips (next bit 0) or
             * t2 + 3 chips (next bit 1): validates the chip period before decoding */
            if (event_near(t2 + 2.0f * rpc, 0.35f * rpc) < 0 && event_near(t2 + 3.0f * rpc, 0.35f * rpc) < 0) { DBG(" -> no start-bit edge\n"); continue; }
            st->syncs++;
            rs_packet_t pkt;
            int ok = decode_bits_edges(n, cfg, t0, t2, rpc, &pkt, st);
            DBG(" -> decode %s\n", ok ? "OK" : "fail");
            if (ok) {
                if (nout < max_out) out[nout++] = pkt;
                decoded = 1;
            }
        }
    }
    return nout;
}

int rs_decode_profile(const float *p, int n, const rs_dec_cfg_t *cfg,
                      rs_packet_t *out, int max_out, rs_dec_stats_t *st)
{
    rs_dec_stats_t local; if (!st) st = &local;
    st->syncs = st->crc_ok = st->crc_fail = st->start_fail = st->truncated = 0; st->retry_ok = 0;
    st->rows_per_chip = 0; st->contrast = 0;
    if (n > RS_DEC_MAX_ROWS) n = RS_DEC_MAX_ROWS;
    if (n < 16 || max_out <= 0) return 0;

    s_cum[0] = 0;
    float gmin = p[0], gmax = p[0];
    for (int r = 0; r < n; r++) {
        s_cum[r + 1] = s_cum[r] + p[r];
        if (p[r] < gmin) gmin = p[r];
        if (p[r] > gmax) gmax = p[r];
    }
    st->contrast = gmax - gmin;
    if (st->contrast < cfg->min_contrast) return 0;

    /* multi-scale: candidate rows-per-chip, each scale accepts [0.45x, 2.2x] */
    static const float scales[] = { 3.0f, 6.0f, 12.0f, 24.0f, 44.0f };
    int nout = 0;
    for (unsigned s = 0; s < sizeof(scales) / sizeof(scales[0]); s++) {
        if (scales[s] * 0.45f > cfg->max_rows_per_chip) break;
        if (scales[s] * 2.2f < cfg->min_rows_per_chip) continue;
        nout = decode_scale(p, n, cfg, scales[s], out, max_out, nout, st);
    }
    if (cfg->use_edges) nout = decode_edges(p, n, cfg, out, max_out, nout, st);
    /* dedupe (same packet found at two scales / two paths): keep higher quality */
    for (int i = 0; i < nout; i++) {
        for (int j = i + 1; j < nout; j++) {
            if (fabsf_(out[i].row_start - out[j].row_start) < 2.0f * out[i].rows_per_chip) {
                if (out[j].quality > out[i].quality) out[i] = out[j];
                out[j] = out[--nout]; j--;
            }
        }
    }
    /* sort by row */
    for (int i = 1; i < nout; i++) {
        rs_packet_t k = out[i]; int j = i - 1;
        while (j >= 0 && out[j].row_start > k.row_start) { out[j + 1] = out[j]; j--; }
        out[j + 1] = k;
    }
    if (nout > 0) {
        float s = 0; for (int i = 0; i < nout; i++) s += out[i].rows_per_chip;
        st->rows_per_chip = s / (float)nout;
        st->crc_ok = nout;
    }
    return nout;
}

const uint8_t *rs_decode_debug_binary(int *n) { if (n) *n = s_dbg_n; return s_dbg; }
