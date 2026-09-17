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

#define RS_TRACK_RECENT 24

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
    int      frames_since_eval;
    int      group;            /* logical source: the smallest track id among the lights sending the same packets */
    /* recent packets for the same-board test (rows and frame times give the packet index) */
    struct { float t, row, rpc; uint16_t seed; uint8_t id, ch; } recent[RS_TRACK_RECENT];
    int      nrecent, recent_head;
} rs_track_t;

typedef struct {
    rs_track_t tracks[RS_MAX_TRACKS];
    float link[RS_MAX_TRACKS][RS_MAX_TRACKS];   /* pair evidence: identical data packets in the same frame, decayed */
    unsigned char linked[RS_MAX_TRACKS][RS_MAX_TRACKS];   /* current link state (hysteresis) */
    int next_id;
    int nblobs;                /* blobs found in the last frame */
    rs_blob_t blobs[RS_MAX_BLOBS];
} rs_multi_t;

void rs_multi_init(rs_multi_t *m);

/* Process one interleaved RGB(A) frame. Returns the total packets decoded. */
int rs_multi_process(rs_multi_t *m, const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                     int r_off, int g_off, int b_off, float t);

/* Logical source of the i-th reported track: its own id, or the id of the track it is linked
 * to because both lights transmit the same packets (two LEDs of one board). */
int rs_multi_track_group(const rs_multi_t *m, int i);
/* Same-board evidence between the i-th and j-th reported tracks (diagnostics). */
float rs_multi_link_score(const rs_multi_t *m, int i, int j);

/* Pop the next message from any track; *track_id receives the logical source (group id). */
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
