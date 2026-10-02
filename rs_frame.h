/*
 * rs_frame.h — camera frame -> lights and their brightness profiles along the rolling-shutter
 * scan axis (one sample per scan line, averaged across the defocused LED).
 * Two families: the blob functions (rs_frame_segment_rgb, rs_frame_profile_rgb_blob2) used by
 * the multi-source receiver, and the whole-frame profiles (one bright region per frame) the
 * applications use for their charts and as a single-light path.
 * Profiles are at most 4096 samples long; static scratch, one call at a time per process.
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
    float sat_frac;           /* fraction of scan lines with a saturated pixel in the ROI (0 = nothing clips) */
    float kept_cols;          /* fraction of ROI columns that enter the profile */
} rs_frame_info_t;

/* axis 0: profile along rows (out[r] = mean over ROI columns of row r), count = h.
 * axis 1: profile along columns, count = w. out must hold max(w, h) floats. */
void rs_frame_profile(const uint8_t *y, int w, int h, int row_stride, int pixel_stride,
                      int axis, float *out, rs_frame_info_t *info);

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

/* Bright blobs (defocused LEDs) found on a thumbnail of the frame. */
typedef struct {
    int r0, r1, c0, c1;       /* bounding box in full-resolution pixels: rows [r0,r1), cols [c0,c1) */
    float cx, cy;             /* brightness-weighted centroid (pixels) */
    int area;                 /* thumbnail pixels above threshold */
    float peak;               /* max thumbnail luma */
} rs_blob_t;

#define RS_MAX_BLOBS 6

/* Segment an interleaved RGB(A) frame into up to max_out blobs (sorted by peak brightness).
 * A frame without a light at least 20 counts above the background, and a blob whose peak is
 * under 48 counts, give nothing. */
int rs_frame_segment_rgb(const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                         int r_off, int g_off, int b_off, rs_blob_t *out, int max_out);

/* R, G, B profiles restricted to one blob: weighted mean over its column window on the rows it
 * spans, zero elsewhere (h samples; h must not exceed the caller's buffers). Each column is
 * weighted by how much it varies from row to row, so the columns carrying the stripes dominate.
 * drop_clipped_cols = 1 removes columns that clip in >= 2 rows (a saturated core, where the
 * smear fills the short dark runs), 0 keeps them with their weights.
 * rs_frame_profile_rgb_blob is the same with the compile-time default RS_DROP_CLIPPED. */
void rs_frame_profile_rgb_blob2(const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                                int r_off, int g_off, int b_off, const rs_blob_t *blob, int drop_clipped_cols,
                                float *r, float *g, float *b, rs_frame_info_t *info);
void rs_frame_profile_rgb_blob(const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                               int r_off, int g_off, int b_off, const rs_blob_t *blob,
                               float *r, float *g, float *b, rs_frame_info_t *info);
#ifdef __cplusplus
}
#endif
#endif /* RS_FRAME_H */
