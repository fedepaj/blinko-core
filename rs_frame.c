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
}

void rs_frame_profile_rgb(const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                          int r_off, int g_off, int b_off, int axis,
                          float *r, float *g, float *b, rs_frame_info_t *info)
{
    rs_view_t v; make_view(&v, px, w, h, row_stride, pixel_stride, axis, 3, r_off, g_off, b_off);
    int c0, c1; cross_window(&v, &c0, &c1);
    int len = c1 - c0, peak = 0, sat = 0;
    float inv = 1.0f / (float)len;
    for (int s = 0; s < v.n_scan; s++) {
        const uint8_t *p = v.px + s * v.st_scan + c0 * v.st_cross;
        uint32_t sr = 0, sg = 0, sb = 0; int mx = 0;
        for (int c = 0; c < len; c++, p += v.st_cross) {
            int pr = p[r_off], pg = p[g_off], pb = p[b_off];
            sr += pr; sg += pg; sb += pb;
            if (pr > mx) mx = pr; if (pg > mx) mx = pg; if (pb > mx) mx = pb;
        }
        r[s] = sr * inv; g[s] = sg * inv; b[s] = sb * inv;
        if (mx > peak) peak = mx;
        if (mx >= 250) sat++;
    }
    info->roi_start = c0; info->roi_end = c1; info->count = v.n_scan;
    info->peak = peak; info->sat_frac = (float)sat / (float)v.n_scan;
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
}
