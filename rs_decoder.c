#include "rs_decoder.h"

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

/* Decode the 25 bits following a sync ending at t2 (start of start bit).
 * Returns 1 on CRC ok, 0 otherwise. Fills pkt on success. */
static int decode_bits(const float *p, int n, const rs_dec_cfg_t *cfg,
                       float t0, float t2, float rpc, rs_packet_t *pkt, rs_dec_stats_t *st)
{
    float pos = t2;
    float quality = 1.0f;
    uint32_t bits = 0; /* 29 bits, MSB first: start + id(3) seed(9) payload(8) crc(8) */
    for (int k = 0; k < RS_PKT_BITS; k++) {
        float a = pos, m = pos + rpc, b = pos + 2 * rpc;
        if (b > (float)n + 0.01f) { st->truncated++; return 0; }
        float first = integ(p, n, m) - integ(p, n, a);
        float second = integ(p, n, b) - integ(p, n, m);
        int bit = second > first;
        float conf = fabsf_(second - first) / (first + second + 1e-6f);
        if (conf < quality) quality = conf;
        bits = (bits << 1) | (uint32_t)bit;
        /* PLL: re-lock on the mid-bit transition */
        float thr = (first + second) / (2.0f * rpc);
        float edge = find_edge(p, n, m, 0.5f * rpc, thr, bit ? +1 : -1);
        if (edge >= 0) pos += cfg->pll_gain * (edge - m);
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
        float rpc = (L1 + L2) / 8.0f;
        if (rpc < cfg->min_rows_per_chip || rpc > cfg->max_rows_per_chip) continue;
        if (rpc < rpc_hint * 0.45f || rpc > rpc_hint * 2.2f) continue;  /* belongs to another scale */
        if (fabsf_(L1 - L2) > cfg->sync_tol * 0.5f * (L1 + L2)) continue;
        if (L3 < 0.35f * rpc || L3 > 1.9f * rpc) continue;
        if ((float)s_runs[i - 1].len < 0.35f * rpc) continue;
        st->syncs++;
        /* sub-row edge positions */
        int a0 = s_runs[i].start - 1, a1 = s_runs[i + 1].start - 1, a2 = s_runs[i + 2].start - 1;
        float thr0 = 0.5f * (s_emax[a0 + 1] + s_emin[a0 + 1]);
        float thr1 = 0.5f * (s_emax[a1 + 1] + s_emin[a1 + 1]);
        float thr2 = 0.5f * (s_emax[a2 + 1] + s_emin[a2 + 1]);
        float t0 = crossing(p, a0, thr0);
        float t1 = crossing(p, a1, thr1);
        float t2 = crossing(p, a2, thr2);
        float rpc_ref = (t2 - t0) / 8.0f;
        if (fabsf_((t1 - t0) / 4.0f - rpc_ref) > cfg->sync_tol * rpc_ref) continue;
        rs_packet_t pkt;
        if (decode_bits(p, n, cfg, t0, t2, rpc_ref, &pkt, st)) {
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

int rs_decode_profile(const float *p, int n, const rs_dec_cfg_t *cfg,
                      rs_packet_t *out, int max_out, rs_dec_stats_t *st)
{
    rs_dec_stats_t local; if (!st) st = &local;
    st->syncs = st->crc_ok = st->crc_fail = st->start_fail = st->truncated = 0;
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
    /* dedupe (same packet found at two scales): keep higher quality */
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
