/* rs_stitch.c — see rs_stitch.h. */
#include "rs_stitch.h"
#ifdef RS_DEC_DEBUG
#include <stdio.h>
#define ST_DBG(...) fprintf(stderr, "stitch: " __VA_ARGS__)
#else
#define ST_DBG(...) ((void)0)
#endif

static float fabsf_(float x) { return x < 0 ? -x : x; }
static float fmodp_(float x, float m) { x -= m * (float)(int)(x / m); if (x < 0) x += m; if (x >= m) x -= m; return x; }
static int   iroundf_(float x) { return (int)(x + (x >= 0 ? 0.5f : -0.5f)); }

void rs_stitch_init(rs_stitch_t *s)
{
    uint8_t *p = (uint8_t *)s; for (size_t i = 0; i < sizeof(*s); i++) p[i] = 0;
}

static void restart(rs_stitch_t *s) { s->npieces = 0; s->phead = 0; s->pieces = 0; for (int i = 0; i < RS_STITCH_N; i++) { s->acc[i] = 0; s->wgt[i] = 0; } }

/* Composite of the newest pieces: newest first until every cell of the cycle has a value (or
 * the pieces run out). Returns the number of covered chips. */
/* Agreement of a piece with the composite so far over their common cells: normalized
 * correlation, or 2 when fewer than 6 chips are common (nothing to disagree about). */
static float agree(const rs_stitch_t *s, const rs_piece_t *pc)
{
    float sxy = 0, sxx = 0, syy = 0, sx = 0, sy = 0; int cnt = 0;
    for (int i = 0; i < RS_STITCH_N; i++) {
        if (!pc->has[i] || s->wgt[i] <= 0) continue;
        float x = pc->val[i], y = s->acc[i] / s->wgt[i];
        sxy += x * y; sxx += x * x; syy += y * y; sx += x; sy += y; cnt++;
    }
    if (cnt < 6 * RS_STITCH_RES) return 2;
    float n = (float)cnt, cov = sxy - sx * sy / n, vx = sxx - sx * sx / n, vy = syy - sy * sy / n;
    if (vx <= 1e-6f || vy <= 1e-6f) return 2;
    float d = vx * vy, r = 1; for (int i = 0; i < 12; i++) r = 0.5f * (r + d / r);
    return cov / r;
}

/* Composite of the newest pieces: newest first until every cell of the cycle has a value (or
 * the pieces run out); a piece that disagrees with the composite built so far where they
 * overlap belongs to another packet and is left out. `skip` newest pieces are ignored (a
 * second attempt without the very newest one, which may already be the next packet).
 * Returns the number of covered chips. */
static int compose(rs_stitch_t *s, float now, int skip)
{
    for (int i = 0; i < RS_STITCH_N; i++) { s->acc[i] = 0; s->wgt[i] = 0; }
    int covered_cells = 0;
    for (int k = skip; k < s->npieces && covered_cells < RS_STITCH_N; k++) {
        const rs_piece_t *pc = &s->piece[(s->phead + RS_STITCH_PIECES - 1 - k) % RS_STITCH_PIECES];
        if (now - pc->t > 1.5f) break;                                  /* too old to be the same packet */
        if (agree(s, pc) < 0.5f) continue;
        for (int i = 0; i < RS_STITCH_N; i++) if (pc->has[i]) { if (s->wgt[i] == 0) covered_cells++; s->acc[i] += pc->val[i]; s->wgt[i] += 1.0f; }
    }
    int covered = 0;
    for (int c = 0; c < RS_PKT_CHIPS; c++) { int ok = 1; for (int q = 0; q < RS_STITCH_RES; q++) if (s->wgt[c * RS_STITCH_RES + q] <= 0) { ok = 0; break; } covered += ok; }
    return covered;
}

/* Correlation of a piece against the composite at a phase offset (in cells): the piece is
 * sampled per cell; returns the normalized correlation over the overlapping cells, or -2 when
 * fewer than 8 chips overlap. */
static float match(const rs_stitch_t *s, const float *norm, int r0, int r1, float phase_cells, float rpc)
{
    float sxy = 0, sxx = 0, syy = 0, sx = 0, sy = 0; int cnt = 0;
    float cells_per_row = (float)RS_STITCH_RES / rpc;
    for (int r = r0; r < r1; r++) {
        int c = (int)fmodp_(phase_cells + ((float)r + 0.5f) * cells_per_row, (float)RS_STITCH_N);
        if (s->wgt[c] <= 0) continue;
        float x = norm[r], y = s->acc[c] / s->wgt[c];
        sxy += x * y; sxx += x * x; syy += y * y; sx += x; sy += y; cnt++;
    }
    if (cnt < 8 * (int)(rpc + 0.5f)) return -2;
    float n = (float)cnt, cov = sxy - sx * sy / n, vx = sxx - sx * sx / n, vy = syy - sy * sy / n;
    if (vx <= 1e-6f || vy <= 1e-6f) return -2;
    float d = vx * vy, r = 1; for (int i = 0; i < 12; i++) r = 0.5f * (r + d / r);
    return cov / r;
}

/* Store a piece: the mean normalized brightness per cell of the cycle it covers. */
static void place(rs_stitch_t *s, const float *norm, int r0, int r1, float phase_cells, float rpc, float t)
{
    rs_piece_t *pc = &s->piece[s->phead]; s->phead = (s->phead + 1) % RS_STITCH_PIECES; if (s->npieces < RS_STITCH_PIECES) s->npieces++;
    pc->t = t; pc->phase_cells = phase_cells;
    static float sum[RS_STITCH_N]; static int cnt[RS_STITCH_N];
    for (int i = 0; i < RS_STITCH_N; i++) { sum[i] = 0; cnt[i] = 0; }
    float cells_per_row = (float)RS_STITCH_RES / rpc;
    for (int r = r0; r < r1; r++) { int c = (int)fmodp_(phase_cells + ((float)r + 0.5f) * cells_per_row, (float)RS_STITCH_N); sum[c] += norm[r]; cnt[c]++; }
    for (int i = 0; i < RS_STITCH_N; i++) { pc->has[i] = cnt[i] > 0; pc->val[i] = cnt[i] ? sum[i] / (float)cnt[i] : 0; }
    s->pieces++; s->placed_total++;
}

int rs_stitch_feed(rs_stitch_t *s, const float *p, const float *norm, const float *amp, int n, float min_contrast,
                   const rs_sync_t *syncs, int ns, float rpc_hint, float row_seconds, float t,
                   const rs_dec_cfg_t *cfg, rs_packet_t *out)
{
    /* the usable piece: the lit blob, read from the raw profile as the longest stretch above
     * a quarter of its range (the detector's local amplitude lags the blob's edges by half its
     * normalization window, which would add chips of noise at both ends), trimmed by a chip
     * for the soft edge */
    (void)amp;
    float pmin = p[0], pmax = p[0];
    for (int r = 0; r < n; r++) { if (p[r] < pmin) pmin = p[r]; if (p[r] > pmax) pmax = p[r]; }
    if (pmax - pmin < min_contrast) { ST_DBG("t=%.3f no contrast\n", t); return 0; }
    float low = pmin + 0.25f * (pmax - pmin);
    int r0 = -1, r1 = -1, best_len = 0, cur0 = -1, dark = 0;
    for (int r = 0; r <= n; r++) {
        int on = r < n && p[r] >= low;
        if (on) { dark = 0; if (cur0 < 0) cur0 = r; }
        else if (cur0 >= 0 && ++dark > 60) { if (r - dark - cur0 > best_len) { best_len = r - dark - cur0; r0 = cur0; r1 = r - dark; } cur0 = -1; dark = 0; }   /* the code's dark runs are at most 8 chips: a longer dark stretch ends the blob */
        if (r == n && cur0 >= 0 && r - dark - cur0 > best_len) { best_len = r - dark - cur0; r0 = cur0; r1 = r - dark; }
    }
    if (r0 < 0) { ST_DBG("t=%.3f no blob\n", t); return 0; }
    /* the anchor: a sync inside the piece, strong (its amplitude near the frame's), and at a
     * clock consistent with what is known (the receiver's hint or the last piece's); the
     * detector's validated candidates still include a few false ones at other scales */
    const rs_sync_t *sy = 0;
    float ref = rpc_hint > 0 ? rpc_hint : s->rpc;
    if (ref <= 0) {
        /* no clock known yet: the shortest runs of the piece are 3 chips (the code's minimum run
         * is frequent), so a low percentile of the binarized run lengths gives the clock to ~10 % */
        int lens[256], nl = 0, cur = 0, lvl = norm[r0] >= 0.5f;
        for (int r = r0; r < r1 && nl < 256; r++) { int l = norm[r] >= 0.5f; if (l == lvl) cur++; else { lens[nl++] = cur; cur = 1; lvl = l; } }
        if (nl >= 6) {
            for (int i = 1; i < nl; i++) { int k = lens[i], j = i - 1; while (j >= 0 && lens[j] > k) { lens[j + 1] = lens[j]; j--; } lens[j + 1] = k; }
            ref = (float)lens[nl / 6] / (float)RS_RLL_MIN_RUN;
        }
    }
    for (int i = 0; i < ns; i++) {
        if (syncs[i].x < (float)r0 + 1.0f * syncs[i].rpc || syncs[i].x + (float)RS_SYNC_CHIPS * syncs[i].rpc > (float)r1 - 1.0f * syncs[i].rpc) continue;   /* a sync cut by the blob's edge measures a wrong clock */
        if (syncs[i].amp < 0.3f * (pmax - pmin)) continue;             /* a strong sync: its local amplitude near the blob's */
        if (ref > 0 && fabsf_(syncs[i].rpc - ref) > 0.12f * ref) continue;
        if (!sy || syncs[i].amp > sy->amp) sy = &syncs[i];
    }
    ns = sy ? 1 : 0;
    float rpc = sy ? sy->rpc : (s->rpc_est > 0 ? s->rpc_est : (rpc_hint > 0 ? rpc_hint : s->rpc));
    if (rpc <= 0) { ST_DBG("t=%.3f no clock (hint %.2f)\n", t, rpc_hint); return 0; }
    if ((float)(r1 - r0) > (float)RS_PKT_CHIPS * rpc) return 0;          /* the blob holds a whole packet: the detector's job, nothing to stitch */
    int trim = iroundf_(1.0f * rpc); r0 += trim; r1 -= trim;
    if (r1 - r0 < iroundf_(RS_SYNC_CHIPS * rpc)) { ST_DBG("t=%.3f piece too short (%d rows)\n", t, r1 - r0); return 0; }
    ST_DBG("t=%.3f rows %d-%d sync %s rpc %.2f%s\n", t, r0, r1, sy ? "yes" : "no", rpc, sy ? "" : (s->have_anchor && s->chip_seconds > 0 ? " predicted" : " no anchor"));
    float phase = 0; int anchored = 0;                                 /* chips at row 0 */
    if (sy) {
        /* the stream's clock is the median of the sync clocks seen: a sync cut by the blob's
         * edge, or a data run posing as one, measures another clock and is not an anchor */
        s->rpc_seen[s->rpc_seen_head] = sy->rpc; s->rpc_seen_head = (s->rpc_seen_head + 1) % 8; if (s->rpc_seen_n < 8) s->rpc_seen_n++;
        if (s->rpc_seen_n >= 3) {
            float v[8]; for (int i = 0; i < s->rpc_seen_n; i++) v[i] = s->rpc_seen[i];
            for (int i = 1; i < s->rpc_seen_n; i++) { float k = v[i]; int j = i - 1; while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; } v[j + 1] = k; }
            s->rpc_est = v[s->rpc_seen_n / 2];
            if (fabsf_(sy->rpc - s->rpc_est) > 0.04f * s->rpc_est) { ST_DBG("  sync clock %.2f off the stream's %.2f: not an anchor\n", sy->rpc, s->rpc_est); sy = 0; }
        } else sy = 0;                                                  /* the stream's clock is not known yet: no anchors */
    }
    if (sy && s->chip_seconds_locked && s->have_anchor) {
        /* against a lock, a sync must agree with the prediction (false syncs at the right clock
         * exist: data runs that look like one); three disagreements in a row drop the lock */
        float dt = t - s->t_anchor, pred = fmodp_(s->phase_anchor + dt / s->chip_seconds, (float)RS_PKT_CHIPS);
        float got = fmodp_(-sy->x / sy->rpc, (float)RS_PKT_CHIPS), d = fabsf_(fmodp_(got - pred + 41.0f, (float)RS_PKT_CHIPS) - 41.0f);
        if (d > 3.0f) { ST_DBG("  sync disagrees with the lock by %.1f chips: ignored\n", d); sy = 0; if (++s->misses >= 3) { s->chip_seconds_locked = 0; s->anc_n = 0; s->misses = 0; ST_DBG("  lock dropped\n"); } }
        else s->misses = 0;
    }
    if (sy) {
        phase = fmodp_(-sy->x / rpc, (float)RS_PKT_CHIPS); anchored = 1;
        /* remember the anchor, drop stale ones, fit the chip period to the recent anchors */
        if (s->anc_n && t - s->anc_t[(s->anc_head + 8 - 1) % 8] > 2.0f) { s->anc_n = 0; s->chip_seconds_locked = 0; }
        s->anc_t[s->anc_head] = t; s->anc_phase[s->anc_head] = phase; s->anc_head = (s->anc_head + 1) % 8; if (s->anc_n < 8) s->anc_n++;
        if (s->anc_n >= 2 && row_seconds > 0) {
            float guess = s->chip_seconds_locked ? s->chip_seconds : rpc * row_seconds;
            float best_cs = guess; int best_in = 0; float best_sq = 1e30f;
            int i0 = (s->anc_head + 8 - s->anc_n) % 8;
            /* frames come at multiples of the frame period, so any period that changes the chips
             * per frame by a whole cycle fits the anchors just as well: the row-time guess picks
             * the right one, hence the search goes outward from the guess and keeps the first
             * best. A period fits when the anchors' phases, corrected by the elapsed chips, agree
             * within 2.5 chips (circular); the agreement is tested from every anchor as reference
             * so that a false anchor is outvoted instead of spoiling the fit. */
            int span = s->chip_seconds_locked ? 100 : 600;                /* a lock is refined within +-2 %, never re-chosen: the +-1 cycle-per-frame aliases fit the anchors just as well */
            for (int q = 0; q <= 2 * span; q++) {                        /* +-12 % (or 2 %) in 0.02 % steps, from the guess outward */
                int step = (q & 1) ? -((q + 1) / 2) : (q / 2);
                float cs = guess * (1.0f + 0.0002f * (float)step);
                float res[8];
                for (int k = 0; k < s->anc_n; k++) { int i = (i0 + k) % 8; res[k] = fmodp_(s->anc_phase[i] - (s->anc_t[i] - s->anc_t[i0]) / cs, (float)RS_PKT_CHIPS); }
                int in = 0; float sq = 0;
                for (int j = 0; j < s->anc_n; j++) {
                    int cnt = 0; float sq_j = 0;
                    for (int k = 0; k < s->anc_n; k++) { float d = fmodp_(res[k] - res[j] + 41.0f, (float)RS_PKT_CHIPS) - 41.0f; if (fabsf_(d) < 2.5f) { cnt++; sq_j += d * d; } }
                    if (cnt > in || (cnt == in && sq_j < sq)) { in = cnt; sq = sq_j; }
                }
                if (in > best_in || (in == best_in && sq < best_sq - 1e-3f)) { best_in = in; best_sq = sq; best_cs = cs; }
            }
            int need = s->anc_n >= 4 ? s->anc_n - 1 : s->anc_n;        /* one outlier allowed from 4 anchors on */
            if (s->anc_n >= 3 && best_in >= need) { s->chip_seconds = best_cs; s->chip_seconds_locked = 1; ST_DBG("  chip_seconds fitted %.4g us: %d of %d anchors agree\n", best_cs * 1e6, best_in, s->anc_n); }
            else {
                ST_DBG("  period fit: %d of %d anchors agree, no lock\n", best_in, s->anc_n);
                if (s->anc_n >= 4) {
                    /* drop the anchor that agrees least under the best period */
                    float res[8]; for (int k = 0; k < s->anc_n; k++) { int i = (i0 + k) % 8; res[k] = fmodp_(s->anc_phase[i] - (s->anc_t[i] - s->anc_t[i0]) / best_cs, (float)RS_PKT_CHIPS); }
                    int worst = -1, wcnt = 99;
                    for (int j = 0; j < s->anc_n; j++) { int cnt = 0; for (int k = 0; k < s->anc_n; k++) { float d = fmodp_(res[k] - res[j] + 41.0f, (float)RS_PKT_CHIPS) - 41.0f; if (fabsf_(d) < 2.5f) cnt++; } if (cnt < wcnt) { wcnt = cnt; worst = (i0 + j) % 8; } }
                    int k = 0; float tt[8], pp[8];
                    for (int q = 0; q < s->anc_n; q++) { int i = (i0 + q) % 8; if (i != worst) { tt[k] = s->anc_t[i]; pp[k] = s->anc_phase[i]; k++; } }
                    for (int q = 0; q < k; q++) { s->anc_t[q] = tt[q]; s->anc_phase[q] = pp[q]; }
                    s->anc_n = k; s->anc_head = k % 8;
                }
            }
        }
        s->have_anchor = 1; s->t_anchor = t; s->phase_anchor = phase;
    } else {
        if (!s->have_anchor || s->chip_seconds <= 0) return 0;
        float dt = t - s->t_anchor;
        if (dt < 0 || dt > 2.0f) return 0;                              /* prediction too stale */
        phase = fmodp_(s->phase_anchor + dt / s->chip_seconds, (float)RS_PKT_CHIPS);
    }
    s->rpc = rpc;
    float pc = phase * (float)RS_STITCH_RES;                            /* in cells */
    int covered = compose(s, t, 0);
    if (s->npieces > 0 && covered > 0) {
        /* refine against the composite (+-4 chips for a predicted phase, +-1 for an anchored one) */
        int span = anchored ? RS_STITCH_RES : 4 * RS_STITCH_RES;
        float best = -2, bestpc = pc;
        for (int d = -span; d <= span; d++) { float c = match(s, norm, r0, r1, pc + (float)d, rpc); if (c > best) { best = c; bestpc = pc + (float)d; } }
        ST_DBG("  match best %.2f at %+d cells (pieces %d)\n", best, (int)(bestpc - pc), s->npieces);
        if (best > -2 && best >= 0.3f) pc = bestpc;
        else if (best > -2 && best < 0.0f && !anchored) { ST_DBG("  predicted piece contradicts the composite: skipped\n"); return 0; }
    }
    place(s, norm, r0, r1, pc, rpc, t);
    covered = compose(s, t, 0);
    ST_DBG("  placed at phase %.1f chips; composite covers %d/%d chips from %d pieces\n", pc / RS_STITCH_RES, covered, RS_PKT_CHIPS, s->npieces);
    if (covered < RS_PKT_CHIPS) return 0;
    /* lay the cycle out twice as a profile at RS_STITCH_RES rows per chip and decode it; if
     * that fails, once more without the newest piece (it may already belong to the next packet) */
    static float prof[2 * RS_STITCH_N];
    rs_dec_cfg_t c1 = *cfg; c1.exposure_rows = 1.0f; c1.rows_per_chip_hint = (float)RS_STITCH_RES; c1.parallel = 0; c1.min_contrast = 10.0f;
    rs_packet_t pk[4]; rs_dec_stats_t st; int k = 0;
    for (int attempt = 0; attempt < 2 && k <= 0; attempt++) {
        if (attempt == 1 && compose(s, t, 1) < RS_PKT_CHIPS) break;
        int m = 0;
        for (int i = 0; i < 2 * RS_STITCH_N; i++) { int c = i % RS_STITCH_N; prof[m++] = 100.0f * s->acc[c] / s->wgt[c]; }
        k = rs_decode_profile(prof, m, &c1, pk, 4, &st);
        ST_DBG("  cycle complete (attempt %d): decode -> %d packets (syncs %d crc_fail %d)\n", attempt, k, st.syncs, st.crc_fail);
#ifdef RS_DEC_DEBUG
        { char line[RS_PKT_CHIPS + 1]; for (int c = 0; c < RS_PKT_CHIPS; c++) { float v = 0, w = 0; for (int q = 0; q < RS_STITCH_RES; q++) { v += s->acc[c * RS_STITCH_RES + q]; w += s->wgt[c * RS_STITCH_RES + q]; } v /= w; line[c] = v > 0.65f ? '1' : (v < 0.35f ? '0' : '?'); } line[RS_PKT_CHIPS] = 0; ST_DBG("  cycle: %s\n", line); }
#endif
    }
    if (k <= 0) return 0;                                               /* keep the pieces: the newest frames will push the stale ones out */
    restart(s);
    *out = pk[0];
    out->rows_per_chip = rpc; out->row_start = -1; out->row_end = -1;  /* not a position in this frame */
    s->decoded_total++;
    return 1;
}
