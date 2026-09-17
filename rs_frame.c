/*
 * rs_frame.c — frame -> per-row profiles.
 *
 * One region of interest per frame on the cross axis: the bright region of the
 * defocused LED blob, found on a thumbnail. Each scan line is the mean of that
 * window. peak/sat_frac report how close the sensor is to clipping (a clipped
 * channel breaks the colour separation).
 */
#include "rs_frame.h"

#define RS_FRAME_DS        8
#ifndef RS_SAT_LEVEL
#define RS_SAT_LEVEL 250      /* pixel value treated as clipped */
#endif
#ifndef RS_DROP_CLIPPED
#define RS_DROP_CLIPPED 0     /* blob profile: 1 = drop clipped columns outright, 0 = only ignore their steps */
#endif
#ifndef RS_SAT_MIN_COLS
#define RS_SAT_MIN_COLS 8     /* unclipped columns needed to drop the clipped ones */
#endif

#define RS_FRAME_MAX_CROSS 512

typedef struct {
    const uint8_t *px;
    int n_scan, n_cross;      /* lengths along the scan axis and the cross axis */
    int st_scan, st_cross;    /* byte strides */
    int nch;                  /* channels summed for luma (1 or 3) */
    int off[3];
} rs_view_t;

static void make_view(rs_view_t *v, const uint8_t *px, int w, int h, int row_stride, int pixel_stride, int axis,
                      int nch, int o0, int o1, int o2)
{
    v->px = px; v->nch = nch; v->off[0] = o0; v->off[1] = o1; v->off[2] = o2;
    if (axis == 0) { v->n_scan = h; v->n_cross = w; v->st_scan = row_stride; v->st_cross = pixel_stride; }
    else           { v->n_scan = w; v->n_cross = h; v->st_scan = pixel_stride; v->st_cross = row_stride; }
}

static void roi_of(const float *m, int n, int *lo_out, int *hi_out)
{
    float mn = m[0], mx = m[0]; int peak = 0;
    for (int i = 1; i < n; i++) { if (m[i] > mx) { mx = m[i]; peak = i; } if (m[i] < mn) mn = m[i]; }
    if (mx - mn < 8.0f) { *lo_out = 0; *hi_out = n; return; }
    float thr = mn + 0.4f * (mx - mn);
    int lo = peak, hi = peak;
    while (lo > 0 && m[lo - 1] > thr) lo--;
    while (hi < n - 1 && m[hi + 1] > thr) hi++;
    if (hi - lo < 2) { if (lo > 0) lo--; if (hi < n - 1) hi++; }
    *lo_out = lo; *hi_out = hi + 1;
}

/* Bright window on the cross axis, in full-resolution coordinates. */
static void cross_window(const rs_view_t *v, int *c0, int *c1)
{
    static float cross[RS_FRAME_MAX_CROSS];
    int tc = v->n_cross / RS_FRAME_DS; if (tc > RS_FRAME_MAX_CROSS) tc = RS_FRAME_MAX_CROSS;
    int ts = v->n_scan / RS_FRAME_DS;
    for (int c = 0; c < tc; c++) {
        uint32_t sum = 0;
        for (int s = 0; s < ts; s++) {
            const uint8_t *p = v->px + (s * RS_FRAME_DS) * v->st_scan + (c * RS_FRAME_DS) * v->st_cross;
            for (int k = 0; k < v->nch; k++) sum += p[v->off[k]];
        }
        cross[c] = (float)sum / (float)(ts * v->nch);
    }
    int lo, hi;
    roi_of(cross, tc, &lo, &hi);
    *c0 = lo * RS_FRAME_DS; *c1 = hi * RS_FRAME_DS;
    if (*c1 > v->n_cross) *c1 = v->n_cross;
    if (*c1 - *c0 < 1) { *c0 = 0; *c1 = v->n_cross; }
}

void rs_frame_profile(const uint8_t *y, int w, int h, int row_stride, int pixel_stride,
                      int axis, float *out, rs_frame_info_t *info)
{
    rs_view_t v; make_view(&v, y, w, h, row_stride, pixel_stride, axis, 1, 0, 0, 0);
    int c0, c1; cross_window(&v, &c0, &c1);
    int len = c1 - c0, peak = 0, sat = 0;
    for (int s = 0; s < v.n_scan; s++) {
        const uint8_t *p = v.px + s * v.st_scan + c0 * v.st_cross;
        uint32_t sum = 0; int mx = 0;
        for (int c = 0; c < len; c++, p += v.st_cross) { sum += *p; if (*p > mx) mx = *p; }
        out[s] = (float)sum / (float)len;
        if (mx > peak) peak = mx;
        if (mx >= 250) sat++;
    }
    info->roi_start = c0; info->roi_end = c1; info->count = v.n_scan;
    info->peak = peak; info->sat_frac = (float)sat / (float)v.n_scan;
    info->kept_cols = 1.0f;
}

void rs_frame_profile_rgb(const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                          int r_off, int g_off, int b_off, int axis,
                          float *r, float *g, float *b, rs_frame_info_t *info)
{
    static uint16_t sat_col[4096];
    rs_view_t v; make_view(&v, px, w, h, row_stride, pixel_stride, axis, 3, r_off, g_off, b_off);
    int c0, c1; cross_window(&v, &c0, &c1);
    int len = c1 - c0, peak = 0, sat = 0;
    if (len > 4096) len = 4096;
    /* same clipped-column exclusion as rs_frame_profile_rgb_blob (the halo keeps the gaps) */
    for (int c = 0; c < len; c++) sat_col[c] = 0;
    for (int s = 0; s < v.n_scan; s++) {
        const uint8_t *p = v.px + s * v.st_scan + c0 * v.st_cross;
        int mx = 0;
        for (int c = 0; c < len; c++, p += v.st_cross) {
            int pr = p[r_off], pg = p[g_off], pb = p[b_off];
            int m = pr > pg ? pr : pg; if (pb > m) m = pb;
            if (m >= RS_SAT_LEVEL) sat_col[c]++;
            if (m > mx) mx = m;
        }
        if (mx > peak) peak = mx;
        if (mx >= 250) sat++;
    }
    int kept = 0;
    for (int c = 0; c < len; c++) if (sat_col[c] < 2) kept++;
    int use_mask = kept >= RS_SAT_MIN_COLS && kept < len;
    float inv = 1.0f / (float)(use_mask ? kept : len);
    for (int s = 0; s < v.n_scan; s++) {
        const uint8_t *p = v.px + s * v.st_scan + c0 * v.st_cross;
        uint32_t sr = 0, sg = 0, sb = 0;
        for (int c = 0; c < len; c++, p += v.st_cross) {
            if (use_mask && sat_col[c] >= 2) continue;
            sr += p[r_off]; sg += p[g_off]; sb += p[b_off];
        }
        r[s] = sr * inv; g[s] = sg * inv; b[s] = sb * inv;
    }
    info->roi_start = c0; info->roi_end = c1; info->count = v.n_scan;
    info->peak = peak; info->sat_frac = (float)sat / (float)v.n_scan;
    info->kept_cols = use_mask ? (float)kept / (float)len : 1.0f;
}

void rs_frame_profile_yuv420(const uint8_t *y, int y_rs, int y_ps,
                             const uint8_t *u, int u_rs, int u_ps,
                             const uint8_t *v_, int v_rs, int v_ps,
                             int w, int h, int axis,
                             float *r, float *g, float *b, rs_frame_info_t *info)
{
    static float yp[4096], up[4096], vp[4096];
    rs_view_t v; make_view(&v, y, w, h, y_rs, y_ps, axis, 1, 0, 0, 0);
    int c0, c1; cross_window(&v, &c0, &c1);
    int len = c1 - c0, peak = 0, sat = 0;
    int u_ss = axis == 0 ? u_rs : u_ps, u_sc = axis == 0 ? u_ps : u_rs;
    int v_ss = axis == 0 ? v_rs : v_ps, v_sc = axis == 0 ? v_ps : v_rs;
    int hc0 = c0 / 2, hc1 = c1 / 2; if (hc1 <= hc0) hc1 = hc0 + 1;
    for (int s = 0; s < v.n_scan; s++) {
        const uint8_t *p = v.px + s * v.st_scan + c0 * v.st_cross;
        uint32_t sy = 0; int mx = 0;
        for (int c = 0; c < len; c++, p += v.st_cross) { sy += *p; if (*p > mx) mx = *p; }
        yp[s] = (float)sy / (float)len;
        if (mx > peak) peak = mx;
        if (mx >= 235) sat++;                 /* video-range luma clips earlier */
        const uint8_t *pu = u + (s / 2) * u_ss + hc0 * u_sc, *pv = v_ + (s / 2) * v_ss + hc0 * v_sc;
        uint32_t su = 0, sv = 0;
        for (int c = hc0; c < hc1; c++, pu += u_sc, pv += v_sc) { su += *pu; sv += *pv; }
        up[s] = (float)su / (float)(hc1 - hc0); vp[s] = (float)sv / (float)(hc1 - hc0);
    }
    for (int i = 0; i < v.n_scan; i++) {
        float Y = yp[i], U = up[i] - 128.0f, V = vp[i] - 128.0f;
        float R = Y + 1.402f * V, G = Y - 0.344f * U - 0.714f * V, B = Y + 1.772f * U;
        r[i] = R < 0 ? 0 : R; g[i] = G < 0 ? 0 : G; b[i] = B < 0 ? 0 : B;
    }
    info->roi_start = c0; info->roi_end = c1; info->count = v.n_scan;
    info->peak = peak; info->sat_frac = (float)sat / (float)v.n_scan;
    info->kept_cols = 1.0f;
}

/* ------------------------------------------------------------ segmentation */
#define RS_SEG_TW 256
#define RS_SEG_TH 144

int rs_frame_segment_rgb(const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                         int r_off, int g_off, int b_off, rs_blob_t *out, int max_out)
{
    static float th_img[RS_SEG_TW * RS_SEG_TH];
    static int16_t label[RS_SEG_TW * RS_SEG_TH];
    static int stack[RS_SEG_TW * RS_SEG_TH];
    int ds = RS_FRAME_DS;
    int tw = w / ds, th = h / ds;
    if (tw > RS_SEG_TW) tw = RS_SEG_TW;
    if (th > RS_SEG_TH) th = RS_SEG_TH;
    float mn = 1e30f, mx = -1e30f;
    for (int y = 0; y < th; y++) {
        const uint8_t *row = px + (y * ds) * row_stride;
        for (int x = 0; x < tw; x++) {
            const uint8_t *p = row + (x * ds) * pixel_stride;
            float v = (float)(p[r_off] + p[g_off] + p[b_off]) * (1.0f / 3.0f);
            th_img[y * tw + x] = v;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
    }
    if (mx - mn < 20.0f) return 0;                    /* blank frame (burst pause): nothing to segment */
    /* the modulation stripes the blob with dark rows: sync gaps (24 rows) and the RGB
     * pilot block (~216 rows with single dim pulses). Fill them with a vertical running
     * maximum of +-14 thumbnail rows (+-112 px) so a source stays one component. */
    static float filled[RS_SEG_TW * RS_SEG_TH];
    for (int y = 0; y < th; y++) for (int x = 0; x < tw; x++) {
        float v = 0;
        for (int k = -14; k <= 14; k++) { int yy = y + k; if (yy < 0 || yy >= th) continue; float u = th_img[yy * tw + x]; if (u > v) v = u; }
        filled[y * tw + x] = v;
    }
    for (int i = 0; i < tw * th; i++) th_img[i] = filled[i];
    float thr = mn + 0.4f * (mx - mn);
    for (int i = 0; i < tw * th; i++) label[i] = -1;
    int nb = 0;
    rs_blob_t blobs[RS_MAX_BLOBS * 2];
    for (int y = 0; y < th; y++) for (int x = 0; x < tw; x++) {
        int i = y * tw + x;
        if (label[i] >= 0 || th_img[i] < thr) continue;
        if (nb >= RS_MAX_BLOBS * 2) break;
        /* flood fill */
        int sp = 0; stack[sp++] = i; label[i] = (int16_t)nb;
        int minx = x, maxx = x, miny = y, maxy = y, area = 0; float sx = 0, sy = 0, sw = 0, peak = 0;
        while (sp) {
            int j = stack[--sp]; int jy = j / tw, jx = j % tw;
            float v = th_img[j] - thr;
            area++; sx += jx * v; sy += jy * v; sw += v;
            if (th_img[j] > peak) peak = th_img[j];
            if (jx < minx) minx = jx; if (jx > maxx) maxx = jx; if (jy < miny) miny = jy; if (jy > maxy) maxy = jy;
            static const int dx[4] = { 1, -1, 0, 0 }, dy[4] = { 0, 0, 1, -1 };
            for (int k = 0; k < 4; k++) {
                int nx = jx + dx[k], ny = jy + dy[k];
                if (nx < 0 || ny < 0 || nx >= tw || ny >= th) continue;
                int n = ny * tw + nx;
                if (label[n] >= 0 || th_img[n] < thr) continue;
                label[n] = (int16_t)nb; stack[sp++] = n;
            }
        }
        /* not a LED: speck, whole frame, one-cell-wide line (sensor column noise stretched by the
         * vertical max filter) or too dim in absolute terms (noise when the real light is off) */
        if (area < 4 || area > (tw * th) / 2 || maxx - minx < 2 || maxy - miny < 2 || peak < 48.0f || peak < mn + 40.0f) {
            blobs[nb].area = 0; blobs[nb].peak = 0; nb++; continue;
        }
        rs_blob_t b;
        b.r0 = miny * ds; b.r1 = (maxy + 1) * ds; if (b.r1 > h) b.r1 = h;
        b.c0 = minx * ds; b.c1 = (maxx + 1) * ds; if (b.c1 > w) b.c1 = w;
        b.cx = (sw > 0 ? sx / sw : (float)x) * ds + ds * 0.5f;
        b.cy = (sw > 0 ? sy / sw : (float)y) * ds + ds * 0.5f;
        b.area = area; b.peak = peak;
        blobs[nb++] = b;
    }
    /* report the brightest max_out blobs (insertion sort by peak, ignoring specks with area 0) */
    int n = 0;
    for (int i = 0; i < nb; i++) {
        if (blobs[i].area < 4) continue;
        int k = n < max_out ? n : max_out - 1;
        if (n >= max_out && blobs[i].peak <= out[k].peak) continue;
        if (n < max_out) n++;
        int pos = n - 1;
        while (pos > 0 && out[pos - 1].peak < blobs[i].peak) { out[pos] = out[pos - 1]; pos--; }
        out[pos] = blobs[i];
    }
    return n;
}

void rs_frame_profile_rgb_blob(const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                               int r_off, int g_off, int b_off, const rs_blob_t *blob,
                               float *r, float *g, float *b, rs_frame_info_t *info)
{
    rs_frame_profile_rgb_blob2(px, w, h, row_stride, pixel_stride, r_off, g_off, b_off, blob, RS_DROP_CLIPPED, r, g, b, info);
}

void rs_frame_profile_rgb_blob2(const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                                int r_off, int g_off, int b_off, const rs_blob_t *blob, int drop_clipped_cols,
                                float *r, float *g, float *b, rs_frame_info_t *info)
{
    static float wcol[RS_SEG_TW * RS_FRAME_DS + 64];
    static float csum[RS_SEG_TW * RS_FRAME_DS + 64], csq[RS_SEG_TW * RS_FRAME_DS + 64];
    static uint16_t sat_col[RS_SEG_TW * RS_FRAME_DS + 64];
    int maxlen = (int)(sizeof(wcol) / sizeof(wcol[0]));
    int c0 = blob->c0, c1 = blob->c1; if (c0 < 0) c0 = 0; if (c1 > w) c1 = w;
    int len = c1 - c0; if (len < 1) len = 1; if (len > maxlen) len = maxlen;
    int r0 = blob->r0 < 0 ? 0 : blob->r0, r1 = blob->r1 > h ? h : blob->r1;
    /* Matched column weights: each column is weighted by the variance of its luma along the
     * rows of the box. Stripes (the signal) are what varies along rows, so columns carrying
     * the modulation dominate; a clipped core (flat at 255), the dark surround and noise-only
     * columns get little weight. This replaces both the clipped-column exclusion and any
     * guess about how far the halo extends. */
    /* The weight is the mean squared row-to-row difference (a high-pass), not the raw variance:
     * a burst envelope or a pulsed fault LED makes every column vary a lot at low frequency,
     * and the clipped core would win; only the stripes vary from row to row. */
    int peak = 0, sat = 0, cnt = 0;
    for (int c = 0; c < len; c++) { csum[c] = 0; csq[c] = 0; sat_col[c] = 0; }
    int rstep = (r1 - r0) > 400 ? 4 : 2;                 /* weights need ~100+ sampled rows, not all */
    int wstep = len > 256 ? 2 : 1;                       /* and ~128+ columns: neighbours are alike */
    for (int s = r0; s < r1; s += rstep) {
        const uint8_t *p = px + s * row_stride + c0 * pixel_stride;
        int mx = 0;
        for (int c = 0; c < len; c += wstep, p += wstep * pixel_stride) {
            int pr = p[r_off], pg = p[g_off], pb = p[b_off];
            int m = pr > pg ? pr : pg; if (pb > m) m = pb;
            if (m > mx) mx = m;
            int clipped = m >= RS_SAT_LEVEL;
            if (clipped) sat_col[c]++;
            float y = (float)(pr + pg + pb);
            /* a step into or out of clipping is not a stripe: skip diffs touching a clipped sample */
            if (cnt && !clipped && csum[c] >= 0) { float d = y - csum[c]; csq[c] += d * d; }
            csum[c] = clipped ? -1.0f : y;                  /* previous sampled row, -1 = clipped */
        }
        if (mx > peak) peak = mx;
        if (mx >= 250) sat++;
        cnt++;
    }
    /* a clipped column (a burst or fault-pulse edge gives it one huge step) is dropped outright
     * when enough unclipped columns remain, as in rs_frame_profile_rgb */
    int unclipped = 0;
    for (int c = 0; c < len; c++) if (sat_col[c] < 2) unclipped++;
    int drop_clipped = drop_clipped_cols && unclipped >= RS_SAT_MIN_COLS && unclipped < len;
    float wsum = 0, wmax = 0;
    if (cnt > 1) {
        for (int c = 0; c < len; c += wstep) {
            wcol[c] = (drop_clipped && sat_col[c] >= 2) ? 0 : csq[c] / (float)(cnt - 1);
            if (wcol[c] > wmax) wmax = wcol[c];
        }
        if (wstep > 1) for (int c = 0; c < len; c++) if (c % wstep) wcol[c] = 0;   /* unsampled columns unused */
    }
    int kept = 0;
    for (int c = 0; c < len; c++) {
        if (wmax > 0 && wcol[c] < 0.05f * wmax) wcol[c] = 0;   /* noise-only columns */
        if (wcol[c] > 0) { wsum += wcol[c]; kept++; }
    }
    if (wsum <= 0) { for (int c = 0; c < len; c++) wcol[c] = 1; wsum = (float)len; kept = len; }
    /* the weighted mean visits only the weighted columns, every other one when there are many
     * (the profile is an average: half the columns cost a little noise, not information) */
    static int   use_col[RS_SEG_TW * RS_FRAME_DS + 64];
    static float use_w[RS_SEG_TW * RS_FRAME_DS + 64];
    int cstep = kept > 128 ? (kept + 127) / 128 : 1, nu = 0; wsum = 0;   /* ~128 columns in the mean */
    for (int c = 0, k = 0; c < len; c++) {
        if (wcol[c] == 0) continue;
        if ((k++ % cstep) == 0) { use_col[nu] = c * pixel_stride; use_w[nu] = wcol[c]; wsum += wcol[c]; nu++; }
    }
    float inv = 1.0f / wsum;
    for (int s = 0; s < h; s++) {
        if (s < r0 || s >= r1) { r[s] = g[s] = b[s] = 0; continue; }
        const uint8_t *p = px + s * row_stride + c0 * pixel_stride;
        float sr = 0, sg = 0, sb = 0;
        for (int k = 0; k < nu; k++) {
            const uint8_t *q = p + use_col[k]; float wc = use_w[k];
            sr += wc * q[r_off]; sg += wc * q[g_off]; sb += wc * q[b_off];
        }
        r[s] = sr * inv; g[s] = sg * inv; b[s] = sb * inv;
    }
    info->roi_start = c0; info->roi_end = c1; info->count = h;
    info->peak = peak; info->sat_frac = cnt ? (float)sat / (float)cnt : 0;
    info->kept_cols = (float)kept / (float)len;
}
