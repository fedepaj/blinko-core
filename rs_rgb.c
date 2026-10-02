#include "rs_rgb.h"
#include "rs_proto.h"

#define RS_RGB_MAX_RUNS 4096

static float fabsf_(float x) { return x < 0 ? -x : x; }

void rs_rgb_cal_init(rs_rgb_cal_t *cal)
{
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { cal->m[i][j] = (i == j); cal->inv[i][j] = (i == j); }
    cal->valid = 0; cal->cond = 0; cal->rows_per_chip = 0; cal->pilot_row = -1; cal->pilots_seen = 0;
}

static int invert3(const float m[3][3], float inv[3][3], float *cond)
{
    float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
              - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
              + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    float norm = 1.0f;
    for (int j = 0; j < 3; j++) { float c = 0; for (int i = 0; i < 3; i++) c += m[i][j] * m[i][j]; norm *= (c > 0 ? c : 1e-6f); }
    *cond = fabsf_(det) / (norm > 0 ? (float)__builtin_sqrtf(norm) : 1e-6f);   /* 1 for orthogonal columns */
    if (fabsf_(det) < 1e-6f || *cond < 0.05f) return 0;
    float d = 1.0f / det;
    inv[0][0] =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * d;
    inv[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * d;
    inv[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * d;
    inv[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * d;
    inv[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * d;
    inv[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * d;
    inv[2][0] =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * d;
    inv[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * d;
    inv[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * d;
    return 1;
}

static void mean3(const float *r, const float *g, const float *b, int a, int e, float out[3])
{
    float sr = 0, sg = 0, sb = 0; int n = 0;
    for (int i = a; i < e; i++) { sr += r[i]; sg += g[i]; sb += b[i]; n++; }
    if (n == 0) n = 1;
    out[0] = sr / n; out[1] = sg / n; out[2] = sb / n;
}

int rs_rgb_pilot_detect(rs_rgb_cal_t *cal, const float *r, const float *g, const float *b, int n)
{
    if (n < 64 || n > 4096) return 0;
    static float y[4096];
    static int run_start[RS_RGB_MAX_RUNS], run_len[RS_RGB_MAX_RUNS];
    static uint8_t run_lvl[RS_RGB_MAX_RUNS];
    float mn = 1e30f, mx = -1e30f;
    for (int i = 0; i < n; i++) { y[i] = r[i] + g[i] + b[i]; if (y[i] < mn) mn = y[i]; if (y[i] > mx) mx = y[i]; }
    if (mx - mn < 30.0f) return 0;
    float thr = mn + 0.15f * (mx - mn);   /* a pilot pulse lights one LED only: well below the data peak */
    int nr = 0;
    for (int i = 0; i < n; i++) {
        uint8_t l = y[i] > thr;
        if (nr && run_lvl[nr - 1] == l) { run_len[nr - 1]++; continue; }
        if (nr >= RS_RGB_MAX_RUNS) break;
        run_lvl[nr] = l; run_start[nr] = i; run_len[nr] = 1; nr++;
    }
    /* Glitch removal: a RAW Bayer profile toggles for 1-3 rows at every threshold crossing
     * (R, Gr/Gb and B rows alternate in sensitivity), which chops the H L H L H pattern into
     * dozens of runs. Runs shorter than 4 rows are absorbed by their neighbours. */
    for (int pass = 0; pass < 2; pass++) {
        int w = 0;
        for (int i = 0; i < nr; i++) {
            if (run_len[i] < 4 && w > 0 && i + 1 < nr) {          /* glitch between two runs of the other level: join them */
                run_len[w - 1] += run_len[i] + run_len[i + 1]; i++; continue;
            }
            if (w > 0 && run_lvl[w - 1] == run_lvl[i]) { run_len[w - 1] += run_len[i]; continue; }
            run_lvl[w] = run_lvl[i]; run_start[w] = run_start[i]; run_len[w] = run_len[i]; w++;
        }
        nr = w;
    }
    /* pattern: dark(>=1.5P) H L H L H dark(>=1.5P), the five inner runs equal within 35 % */
    for (int i = 1; i + 6 < nr; i++) {
        if (run_lvl[i] != 1) continue;
        float P = (float)run_len[i];
        if (P < 3) continue;
        /* three pulses of equal width; the two gaps equal to each other and not shorter than
         * 12 % of a pulse. The exposure smear and a bright LED lengthen the ON runs and shorten
         * the OFF runs alike: on a 57 us-exposure phone at T = 105 us the 4-chip pulses read
         * ~100 rows and the 4-chip gaps ~25 at this threshold (a 0.25 ratio), still a pilot. */
        int ok = 1;
        for (int k = 2; k < 5; k += 2) if (fabsf_((float)run_len[i + k] - P) > 0.3f * P) { ok = 0; break; }
        float G = (float)run_len[i + 1], gtol = 0.35f * G > 0.1f * P ? 0.35f * G : 0.1f * P;
        if (ok && (fabsf_((float)run_len[i + 3] - G) > gtol || G < 0.12f * P || G > 1.6f * P)) ok = 0;
        if (!ok) continue;
        if (run_lvl[i - 1] != 0 || (float)run_len[i - 1] < 1.5f * G) continue;
        if (run_lvl[i + 5] != 0 || (float)run_len[i + 5] < 1.5f * G) continue;
        /* found: measure RGB in the middle half of each pulse minus the dark baseline */
        float base[3], m[3][3], v[3];
        mean3(r, g, b, run_start[i + 1] + run_len[i + 1] / 4, run_start[i + 1] + 3 * run_len[i + 1] / 4, base);
        for (int k = 0; k < 3; k++) {
            int s = run_start[i + 2 * k], l = run_len[i + 2 * k];
            mean3(r, g, b, s + l / 4, s + 3 * l / 4, v);
            for (int c = 0; c < 3; c++) m[c][k] = v[c] - base[c];
        }
        /* normalise columns so unmixed channels have comparable amplitude */
        for (int k = 0; k < 3; k++) {
            float s = 0; for (int c = 0; c < 3; c++) s += m[c][k];
            if (s < 3.0f) { ok = 0; break; }
            for (int c = 0; c < 3; c++) m[c][k] /= s;
        }
        if (!ok) continue;
        float inv[3][3], cond;
        if (!invert3(m, inv, &cond) || cond < 0.35f) continue;   /* degenerate colour response: not a usable pilot */
        float a = cal->valid ? 0.5f : 1.0f;
        for (int c = 0; c < 3; c++) for (int k = 0; k < 3; k++) cal->m[c][k] = a * m[c][k] + (1 - a) * cal->m[c][k];
        if (!invert3(cal->m, cal->inv, &cal->cond)) { for (int c = 0; c < 3; c++) for (int k = 0; k < 3; k++) cal->m[c][k] = m[c][k]; invert3(cal->m, cal->inv, &cal->cond); }
        cal->valid = 1;
        cal->rows_per_chip = P / (float)RS_PILOT_P;
        cal->pilot_row = (float)run_start[i];
        cal->pilots_seen++;
        return 1;
    }
    return 0;
}

void rs_rgb_unmix(const rs_rgb_cal_t *cal, const float *r, const float *g, const float *b, int n,
                  float *o0, float *o1, float *o2)
{
    for (int i = 0; i < n; i++) {
        float x = r[i], y = g[i], z = b[i];
        o0[i] = cal->inv[0][0] * x + cal->inv[0][1] * y + cal->inv[0][2] * z;
        o1[i] = cal->inv[1][0] * x + cal->inv[1][1] * y + cal->inv[1][2] * z;
        o2[i] = cal->inv[2][0] * x + cal->inv[2][1] * y + cal->inv[2][2] * z;
    }
}

#include <stddef.h>
size_t rs_rgb_cal_sizeof(void) { return sizeof(rs_rgb_cal_t); }
