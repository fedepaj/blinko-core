/*
 * rs_multi.h — several light sources in one frame: segmentation, tracking by
 * centroid, one complete receiver (calibration, decoder, assembler) per track.
 */
#ifndef RS_MULTI_H
#define RS_MULTI_H

#include "rs_rx.h"
#include "rs_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS_MAX_TRACKS 4
#define RS_TRACK_TTL  1.0f     /* seconds without a detection before a track is dropped */

typedef struct {
    int      active;
    int      id;               /* stable identifier (1, 2, ...) */
    float    cx, cy, radius;   /* smoothed position and size (pixels) */
    int      r0, r1, c0, c1;   /* bounding box of the source (union of its blobs) */
    float    last_seen;        /* time of the last detection */
    int      seen_frames;
    rs_rx_t  rx;
    int      packets_frame;    /* packets decoded in the last frame */
    int      drop_clipped;     /* profile variant in use: 1 = clipped columns dropped, 0 = matched weights only */
    int      last_packets;     /* packets from the previous frame (0 triggers a variant re-evaluation) */
} rs_track_t;

typedef struct {
    rs_track_t tracks[RS_MAX_TRACKS];
    int next_id;
    int nblobs;                /* blobs found in the last frame */
    rs_blob_t blobs[RS_MAX_BLOBS];
} rs_multi_t;

void rs_multi_init(rs_multi_t *m);

/* Process one interleaved RGB(A) frame. Returns the total packets decoded. */
int rs_multi_process(rs_multi_t *m, const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                     int r_off, int g_off, int b_off, float t);

/* Pop the next message from any track; *track_id receives the source. */
int rs_multi_pop_message(rs_multi_t *m, rs_message_t *out, int *track_id);

size_t rs_multi_sizeof(void);
int rs_multi_track_count(const rs_multi_t *m);
/* Receiver of the i-th reported track (per-track packets/stats via the rs_rx accessors). */
const rs_rx_t *rs_multi_track_rx(const rs_multi_t *m, int i);
const rs_track_t *rs_multi_track(const rs_multi_t *m, int i);   /* i-th active track, or NULL */

#ifdef __cplusplus
}
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
/* Flat accessor for bindings: fills the i-th active track's summary. Returns 0 if none. */
int rs_multi_track_info(const rs_multi_t *m, int i, int *id, float *cx, float *cy, float *radius,
                        int *mode, uint32_t *packets, uint32_t *messages, int *pilots);
#ifdef __cplusplus
}
#endif
