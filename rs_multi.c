#include "rs_multi.h"

static float s_r[RS_DEC_MAX_ROWS], s_g[RS_DEC_MAX_ROWS], s_b[RS_DEC_MAX_ROWS];

void rs_multi_init(rs_multi_t *m)
{
    uint8_t *p = (uint8_t *)m;
    for (size_t i = 0; i < sizeof(*m); i++) p[i] = 0;
    m->next_id = 1;
}


static int overlap(const rs_track_t *tr, const rs_blob_t *b)
{
    /* boxes touch or overlap after growing the track box by a margin */
    int m = 24;
    return !(b->r1 < tr->r0 - m || b->r0 > tr->r1 + m || b->c1 < tr->c0 - m || b->c0 > tr->c1 + m);
}

static void track_feed(rs_track_t *tr, const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                       int r_off, int g_off, int b_off, const rs_blob_t *u, float t)
{
    float rad = 0.25f * (float)((u->r1 - u->r0) + (u->c1 - u->c0));
    if (tr->seen_frames == 0) { tr->cx = u->cx; tr->cy = u->cy; tr->radius = rad; tr->r0 = u->r0; tr->r1 = u->r1; tr->c0 = u->c0; tr->c1 = u->c1; }
    else {
        tr->cx = 0.6f * tr->cx + 0.4f * u->cx; tr->cy = 0.6f * tr->cy + 0.4f * u->cy; tr->radius = 0.7f * tr->radius + 0.3f * rad;
        tr->r0 = (int)(0.5f * tr->r0 + 0.5f * u->r0); tr->r1 = (int)(0.5f * tr->r1 + 0.5f * u->r1);
        tr->c0 = (int)(0.5f * tr->c0 + 0.5f * u->c0); tr->c1 = (int)(0.5f * tr->c1 + 0.5f * u->c1);
    }
    tr->last_seen = t; tr->seen_frames++;
    rs_frame_info_t info;
    rs_frame_profile_rgb_blob(px, w, h, row_stride, pixel_stride, r_off, g_off, b_off, u, s_r, s_g, s_b, &info);
    tr->packets_frame = rs_rx_process(&tr->rx, s_r, s_g, s_b, h, t);
}

int rs_multi_process(rs_multi_t *m, const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
                     int r_off, int g_off, int b_off, float t)
{
    m->nblobs = rs_frame_segment_rgb(px, w, h, row_stride, pixel_stride, r_off, g_off, b_off, m->blobs, RS_MAX_BLOBS);
    int assigned[RS_MAX_BLOBS]; for (int i = 0; i < RS_MAX_BLOBS; i++) assigned[i] = 0;
    for (int k = 0; k < RS_MAX_TRACKS; k++) m->tracks[k].packets_frame = 0;

    /* existing tracks take every blob overlapping their box (fragments of one source merge) */
    for (int k = 0; k < RS_MAX_TRACKS; k++) {
        rs_track_t *tr = &m->tracks[k];
        if (!tr->active) continue;
        rs_blob_t u; int have = 0; float sw = 0;
        for (int i = 0; i < m->nblobs; i++) {
            if (assigned[i] || !overlap(tr, &m->blobs[i])) continue;
            const rs_blob_t *b = &m->blobs[i];
            if (!have) { u = *b; have = 1; sw = (float)b->area; u.cx *= sw; u.cy *= sw; }
            else {
                if (b->r0 < u.r0) u.r0 = b->r0; if (b->r1 > u.r1) u.r1 = b->r1;
                if (b->c0 < u.c0) u.c0 = b->c0; if (b->c1 > u.c1) u.c1 = b->c1;
                u.cx += b->cx * (float)b->area; u.cy += b->cy * (float)b->area; sw += (float)b->area;
                u.area += b->area; if (b->peak > u.peak) u.peak = b->peak;
            }
            assigned[i] = 1;
        }
        if (have) { u.cx /= sw; u.cy /= sw; track_feed(tr, px, w, h, row_stride, pixel_stride, r_off, g_off, b_off, &u, t); }
        else if (t - tr->last_seen > RS_TRACK_TTL) tr->active = 0;
    }
    /* new tracks for the remaining blobs (brightest first) */
    for (int i = 0; i < m->nblobs; i++) {
        if (assigned[i]) continue;
        int k = -1;
        for (int j = 0; j < RS_MAX_TRACKS; j++) if (!m->tracks[j].active) { k = j; break; }
        if (k < 0) break;
        rs_track_t *tr = &m->tracks[k];
        tr->active = 1; tr->id = m->next_id++; tr->seen_frames = 0;
        rs_rx_init(&tr->rx);
        track_feed(tr, px, w, h, row_stride, pixel_stride, r_off, g_off, b_off, &m->blobs[i], t);
        assigned[i] = 1;
    }
    int total = 0;
    for (int k = 0; k < RS_MAX_TRACKS; k++) if (m->tracks[k].active) total += m->tracks[k].packets_frame;
    return total;
}

int rs_multi_pop_message(rs_multi_t *m, rs_message_t *out, int *track_id)
{
    for (int k = 0; k < RS_MAX_TRACKS; k++) {
        rs_track_t *tr = &m->tracks[k];
        if (!tr->active) continue;
        if (rs_rx_pop_message(&tr->rx, out)) { if (track_id) *track_id = tr->id; return 1; }
    }
    return 0;
}

size_t rs_multi_sizeof(void) { return sizeof(rs_multi_t); }

int rs_multi_track_count(const rs_multi_t *m)
{
    int n = 0;
    for (int k = 0; k < RS_MAX_TRACKS; k++) if (m->tracks[k].active) n++;
    return n;
}

const rs_track_t *rs_multi_track(const rs_multi_t *m, int i)
{
    for (int k = 0; k < RS_MAX_TRACKS; k++) {
        if (!m->tracks[k].active) continue;
        if (i-- == 0) return &m->tracks[k];
    }
    return NULL;
}

int rs_multi_track_info(const rs_multi_t *m, int i, int *id, float *cx, float *cy, float *radius,
                        int *mode, uint32_t *packets, uint32_t *messages, int *pilots)
{
    const rs_track_t *tr = rs_multi_track(m, i);
    if (!tr) return 0;
    *id = tr->id; *cx = tr->cx; *cy = tr->cy; *radius = tr->radius;
    *mode = tr->rx.mode; *packets = tr->rx.packets_total; *messages = tr->rx.assembler.messages_total; *pilots = tr->rx.cal.pilots_seen;
    return 1;
}
