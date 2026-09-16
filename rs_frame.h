/*
 * rs_frame.h — luma plane -> 1-D brightness profile along the rolling-shutter
 * scan axis, averaged over the bright region (defocused LED) on the other
 * axis. Shared by the Android app (JNI) and tools; the iOS app has an
 * Accelerate implementation of the same algorithm (FrameProcessor.swift).
 */
#ifndef RS_FRAME_H
#define RS_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int roi_start, roi_end;   /* bright region on the cross axis */
    int count;                /* samples written to out */
    int peak;                 /* brightest pixel value inside the ROI (any channel) */
    float sat_frac;           /* fraction of scan lines with a saturated (>= 250) pixel in the ROI */
} rs_frame_info_t;

/* axis 0: profile along rows (out[r] = mean over ROI columns of row r), count = h.
 * axis 1: profile along columns, count = w. out must hold max(w, h) floats. */
void rs_frame_profile(const uint8_t *y, int w, int h, int row_stride, int pixel_stride,
                      int axis, float *out, rs_frame_info_t *info);

#ifdef __cplusplus
}
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
/* Three per-row profiles (r, g, b) from a BGRA/RGBA interleaved frame
 * (offsets of the R, G, B bytes within a pixel given by r_off, g_off, b_off;
 * pixel_stride = 4). ROI from the luma thumbnail. axis as rs_frame_profile. */
void rs_frame_profile_rgb(const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                          int r_off, int g_off, int b_off, int axis,
                          float *r, float *g, float *b, rs_frame_info_t *info);

/* Three per-row profiles from YUV 4:2:0 planes (Android YUV_420_888). Chroma is
 * half resolution on both axes; it is upsampled by repetition. */
void rs_frame_profile_yuv420(const uint8_t *y, int y_rs, int y_ps,
                             const uint8_t *u, int u_rs, int u_ps,
                             const uint8_t *v, int v_rs, int v_ps,
                             int w, int h, int axis,
                             float *r, float *g, float *b, rs_frame_info_t *info);
#ifdef __cplusplus
}
#endif
