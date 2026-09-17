/*
 * rs_rgb.h — colour calibration for 3-channel (RGB) reception.
 * The transmitter emits pilot blocks (see rs_proto.h); from the camera's
 * R, G, B row profiles the receiver measures the 3x3 response matrix M
 * (column k = RGB response to LED k) and unmixes the three streams.
 */
#ifndef RS_RGB_H
#define RS_RGB_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float m[3][3];       /* observed = m * channels */
    float inv[3][3];
    int   valid;
    float cond;          /* rough condition estimate (|det| normalised), higher is better */
    float rows_per_chip; /* measured from the pilot pulse width */
    float pilot_row;     /* last pilot position */
    int   pilots_seen;
} rs_rgb_cal_t;

void rs_rgb_cal_init(rs_rgb_cal_t *cal);

/* Look for a pilot block in the luma profile (r+g+b); when found update the
 * calibration from the r, g, b profiles. Returns 1 if a pilot was found. */
int rs_rgb_pilot_detect(rs_rgb_cal_t *cal, const float *r, const float *g, const float *b, int n);

/* Unmix r,g,b (n samples) into channel profiles o0,o1,o2 using cal->inv. */
void rs_rgb_unmix(const rs_rgb_cal_t *cal, const float *r, const float *g, const float *b, int n,
                  float *o0, float *o1, float *o2);

#ifdef __cplusplus
}
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
size_t rs_rgb_cal_sizeof(void);
#ifdef __cplusplus
}
#endif
