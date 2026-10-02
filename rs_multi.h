/*
 * rs_multi.h — several light sources in one frame: segmentation, tracking, one complete
 * receiver (calibration, decoder, assembler) per track, cross-talk filter, grouping of the lights
 * of one board. This is the entry point for an application that has camera frames.
 *
 * Frames are interleaved 8-bit RGB(A) with rows along the rolling-shutter scan (rows are time).
 * Limits: up to RS_DEC_MAX_ROWS rows are read; one frame at a time per process (see rs_rx.h);
 * RS_MAX_TRACKS lights at once. Time: seconds as a float, small numbers (see rs_rx.h).
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

/* A packet a track decoded recently: when, where and which, for the same-board test. */
typedef struct { float t, row, rpc; uint16_t seed; uint8_t id, ch; } rs_track_pkt_t;

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
    float    amp_typ;          /* typical packet amplitude of this light (EMA); leaks are far dimmer */
    int      amp_n;
    rs_track_pkt_t recent[RS_TRACK_RECENT];
    int      nrecent, recent_head;
} rs_track_t;

typedef struct {
    rs_track_t tracks[RS_MAX_TRACKS];
    float link[RS_MAX_TRACKS][RS_MAX_TRACKS];   /* pair evidence: packets of one carousel seen by both, decayed */
    unsigned char linked[RS_MAX_TRACKS][RS_MAX_TRACKS];   /* current link state (hysteresis) */
    unsigned char keep[RS_MAX_TRACKS][RS_RX_MAX_PKTS];    /* per-frame cross-talk mask */
    rs_camera_t camera;        /* pushed into every track's receiver (see rs_multi_set_camera) */
    rs_parallel_fn parallel; void *parallel_user;   /* see rs_multi_set_parallel */
    float min_contrast;        /* 0 = the decoder's default (see rs_multi_set_min_contrast) */
    int next_id;
    int nblobs;                /* blobs found in the last frame */
    rs_blob_t blobs[RS_MAX_BLOBS];
} rs_multi_t;

void rs_multi_init(rs_multi_t *m);
/* Forget every track and message but keep what the application configured (camera, parallel
 * hook, minimum contrast): what an application's "clear" should call. */
void rs_multi_reset(rs_multi_t *m);

/* The camera's exposure in rows of the frames given (exposure_us / row_us; 0 = unknown) and its
 * row time in seconds, for every track's receiver, present and future. */
void rs_multi_set_camera(rs_multi_t *m, rs_camera_t cam);
/* Platform hook for decoding a light's three channels on several threads (see rs_parallel_fn);
 * applies to every track, present and future. */
void rs_multi_set_parallel(rs_multi_t *m, rs_parallel_fn fn, void *user);
/* The decoder's minimum local contrast (profile units) for every track, present and future. */
void rs_multi_set_min_contrast(rs_multi_t *m, float min_contrast);

/* Process one frame: w x h pixels of pixel_stride bytes, rows row_stride bytes apart, the R, G
 * and B bytes at r_off, g_off, b_off within a pixel. t: time in seconds. Returns the distinct
 * packets decoded in this frame, all tracks together. */
int rs_multi_process(rs_multi_t *m, const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                     int r_off, int g_off, int b_off, float t);

/* Pop the next message from any track; *track_id receives the logical source (group id). */
int rs_multi_pop_message(rs_multi_t *m, rs_message_t *out, int *track_id);

/* The tracks reported: those that decoded something or have been seen for 12 frames. */
int rs_multi_track_count(const rs_multi_t *m);
const rs_track_t *rs_multi_track(const rs_multi_t *m, int i);   /* i-th reported track, or NULL */
/* Receiver of the i-th reported track (per-track packets/stats via the rs_rx accessors). */
const rs_rx_t *rs_multi_track_rx(const rs_multi_t *m, int i);
/* Logical source of the i-th reported track: its own id, or the id of the track it is linked
 * to because both lights transmit the same packets (two LEDs of one board). */
int rs_multi_track_group(const rs_multi_t *m, int i);

/* Accessors for bindings: the structure's size, and the i-th reported track's summary
 * (mode is RS_RX_MODE_*). Returns 0 if there is no such track. */
size_t rs_multi_sizeof(void);
int rs_multi_track_info(const rs_multi_t *m, int i, int *id, float *cx, float *cy, float *radius,
                        int *mode, uint32_t *packets, uint32_t *messages, int *pilots);

#ifdef __cplusplus
}
#endif
#endif
