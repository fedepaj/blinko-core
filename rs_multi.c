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

/* The profile box of a track: the blob plus its halo (the clipped core loses the 1-chip gaps,
 * the halo keeps them, and dim stripes continue well outside the 40 % threshold box). Rows are
 * extended by one blob height above and below, columns by half a width each side, clipped to
 * the frame and to the midpoint towards any other active track's box. */
#ifndef RS_TRACK_ROW_EXT
#define RS_TRACK_ROW_EXT 1.0f     /* halo rows added above and below, in blob heights */
#endif
#ifndef RS_TRACK_CONFIRM_FRAMES
#define RS_TRACK_CONFIRM_FRAMES 12 /* frames before a track that never decoded is reported */
#endif
#ifndef RS_TRACK_REEVAL
#define RS_TRACK_REEVAL 8         /* frames between profile-variant comparisons */
#endif
#ifndef RS_TRACK_WIDE_COLS
#define RS_TRACK_WIDE_COLS 1      /* include half a blob width of halo columns each side (matched weights sort them out) */
#endif

static void profile_box(const rs_multi_t *m, const rs_track_t *self, const rs_blob_t *u, int w, int h, int wide_cols, rs_blob_t *e)
{
    *e = *u;
    int bh = u->r1 - u->r0, bw = u->c1 - u->c0;
    e->r0 = u->r0 - (int)(RS_TRACK_ROW_EXT * bh); e->r1 = u->r1 + (int)(RS_TRACK_ROW_EXT * bh);
    if (wide_cols) { e->c0 = u->c0 - bw / 2; e->c1 = u->c1 + bw / 2; }
    for (int k = 0; k < RS_MAX_TRACKS; k++) {
        const rs_track_t *o = &m->tracks[k];
        if (!o->active || o == self || o->seen_frames == 0) continue;
        /* only neighbours sharing columns can mix into the row profile */
        if (o->c1 <= e->c0 || o->c0 >= e->c1) continue;
        if (o->r1 <= u->r0 && e->r0 < (o->r1 + u->r0) / 2) e->r0 = (o->r1 + u->r0) / 2;   /* neighbour above */
        if (o->r0 >= u->r1 && e->r1 > (u->r1 + o->r0) / 2) e->r1 = (u->r1 + o->r0) / 2;   /* neighbour below */
    }
    if (e->r0 < 0) e->r0 = 0; if (e->r1 > h) e->r1 = h;
    if (e->c0 < 0) e->c0 = 0; if (e->c1 > w) e->c1 = w;
}

/* Packets the decoder would get from a profile set, without touching the receiver state. */
static int profile_yield(const rs_rx_t *rx, const float *r, const float *g, const float *b, int n)
{
    static float y[RS_DEC_MAX_ROWS];
    static rs_packet_t pk[32];
    rs_dec_cfg_t cfg = rx->cfg;
    int total = 0;
    for (int i = 0; i < n; i++) y[i] = (r[i] + g[i] + b[i]) * (1.0f / 3.0f);
    total += rs_decode_profile(y, n, &cfg, pk, 32, NULL);
    if (rs_rx_three_coloured(r, g, b, n)) {
        total += rs_decode_profile(r, n, &cfg, pk, 32, NULL);
        total += rs_decode_profile(g, n, &cfg, pk, 32, NULL);
        total += rs_decode_profile(b, n, &cfg, pk, 32, NULL);
    }
    return total;
}

static void track_feed(rs_multi_t *m, rs_track_t *tr, const uint8_t *px, int w, int h, int row_stride, int pixel_stride,
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
    /* rows always include the halo; columns only when most of the blob's own columns clip
     * (a saturated core: the halo carries the gaps; an unsaturated LED: extra columns are noise) */
    rs_blob_t e; profile_box(m, tr, u, w, h, RS_TRACK_WIDE_COLS, &e);
    /* Two profile hypotheses: with the clipped core columns (matched weights only) and without
     * them. Which one carries the gaps depends on the light (a pulsed fault LED saturates its
     * core, an RGB LED's white rows clip only briefly), so the decoder decides per frame: the
     * variant that yields more packets in this frame (luma plus, for a three-coloured light,
     * each camera channel) feeds the receiver; ties keep the previous choice. */
    /* The comparison costs two profiles and up to eight decoder runs, so it is made every
     * RS_TRACK_REEVAL frames, or right after a frame in which the current variant decoded
     * nothing; in between the track keeps its choice. */
    static float a_r[RS_DEC_MAX_ROWS], a_g[RS_DEC_MAX_ROWS], a_b[RS_DEC_MAX_ROWS];
    /* an empty frame also triggers a comparison, but not more often than every 3 frames:
     * with several dim lights most frames of a track are empty and the comparison would
     * run every frame (measured: 3 tracks took the iPhone from 93 to 55 fps) */
    tr->frames_since_eval++;
    int reeval = tr->frames_since_eval >= RS_TRACK_REEVAL || (tr->last_packets == 0 && tr->frames_since_eval >= 3);
    if (reeval) {
        tr->frames_since_eval = 0;
        rs_frame_info_t ia;
        rs_frame_profile_rgb_blob2(px, w, h, row_stride, pixel_stride, r_off, g_off, b_off, &e, 0, a_r, a_g, a_b, &ia);
        rs_frame_profile_rgb_blob2(px, w, h, row_stride, pixel_stride, r_off, g_off, b_off, &e, 1, s_r, s_g, s_b, &info);
        if (info.kept_cols < 1.0f) {        /* the two variants differ only when something clips */
            int ya = profile_yield(&tr->rx, a_r, a_g, a_b, h), yb = profile_yield(&tr->rx, s_r, s_g, s_b, h);
            if (ya > yb || (ya == yb && !tr->drop_clipped)) {
                tr->drop_clipped = 0;
                for (int i = 0; i < h; i++) { s_r[i] = a_r[i]; s_g[i] = a_g[i]; s_b[i] = a_b[i]; }
            } else tr->drop_clipped = 1;
        } else if (!tr->drop_clipped) {
            for (int i = 0; i < h; i++) { s_r[i] = a_r[i]; s_g[i] = a_g[i]; s_b[i] = a_b[i]; }
        }
    } else {
        rs_frame_profile_rgb_blob2(px, w, h, row_stride, pixel_stride, r_off, g_off, b_off, &e, tr->drop_clipped, s_r, s_g, s_b, &info);
    }
    tr->packets_frame = rs_rx_process(&tr->rx, s_r, s_g, s_b, h, t);
    tr->last_packets = tr->packets_frame;
}

/* Two lights of one board (the RGB LED and the builtin LED mirroring a channel) show packets
 * of one carousel: the sequence number q of a packet is nchan * (its time in packets) + channel,
 * and within a slot the seed grows by 1 per carousel packet, minus 3 per control triplet. So for
 * two packets of the same slot seen in one frame by tracks A and B, same board means
 *   seed_b - seed_a  in  { n*dp + dch (+/- 3, +/- 6) }  for n in {1, 3},  dp = row distance in packets.
 * Rows are time, so this holds whether the two blobs overlap or sit far apart in the frame.
 * Matches add to a decayed pair score, mismatches subtract; above RS_LINK_ON the tracks share a
 * group (id = the smallest track id), below RS_LINK_OFF they part again. A random pair of
 * boards matches ~6 % of the time, so its score drifts negative. */
#ifndef RS_LINK_ON
#define RS_LINK_ON  2.0f
#endif
#ifndef RS_LINK_OFF
#define RS_LINK_OFF 0.5f
#endif
static int seed_relation_ok(int dseed, int dq)
{
    static const int shift[5] = { 0, -3, 3, -6, 6 };
    for (int k = 0; k < 5; k++) { int d = dseed - (dq + shift[k]); if (d < 0) d = -d; if (d <= 1) return 1; }
    return 0;
}
/* Packet index distance between two packets (rows within a frame plus the time between frames,
 * with the protocol's default chip length: the test only needs +-1 packet). */
#ifndef RS_LINK_CHIP_S
#define RS_LINK_CHIP_S 30e-6f
#endif
static float packet_distance(float ta, float rowa, float tb, float rowb, float rpc)
{
    return (rowb - rowa) / (rpc * (float)RS_PKT_CHIPS) + (tb - ta) / (RS_LINK_CHIP_S * (float)RS_PKT_CHIPS);
}
static float pair_evidence(const rs_track_t *ta, const rs_rx_packet_t *pa, float t, const rs_track_t *tb, int skip_same_frame)
{
    float e = 0;
    if (pa->pkt.seed >= RS_SEED_MSGCRC2) return 0;
    for (int j = 0; j < tb->nrecent; j++) {
        const __typeof__(tb->recent[0]) *rb = &tb->recent[j];
        if (rb->id != pa->pkt.id || rb->seed >= RS_SEED_MSGCRC2) continue;
        if (skip_same_frame && rb->t == t) continue;
        if (t - rb->t > 0.02f || rb->t - t > 0.02f) continue;         /* within ~2 frames: at most one pilot block (0.5 packet) in between */
        float rpc = 0.5f * (pa->pkt.rows_per_chip + rb->rpc);
        float dp_f = packet_distance(t, pa->pkt.row_start, rb->t, rb->row, rpc);
        int dp = (int)(dp_f + (dp_f >= 0 ? 0.5f : -0.5f));
        int dseed = (int)rb->seed - (int)pa->pkt.seed, dch = (int)rb->ch - (int)pa->channel;
        int ok = seed_relation_ok(dseed, dp + dch) || seed_relation_ok(dseed, 3 * dp + dch);
        e += ok ? 1.0f : -0.3f;
    }
    (void)ta;
    return e;
}
static void remember_packets(rs_track_t *tr, float t)
{
    for (int i = 0; i < tr->rx.npkts; i++) {
        __typeof__(tr->recent[0]) *r = &tr->recent[tr->recent_head];
        r->t = t; r->row = tr->rx.pkts[i].pkt.row_start; r->rpc = tr->rx.pkts[i].pkt.rows_per_chip;
        r->seed = tr->rx.pkts[i].pkt.seed; r->id = tr->rx.pkts[i].pkt.id; r->ch = tr->rx.pkts[i].channel;
        tr->recent_head = (tr->recent_head + 1) % RS_TRACK_RECENT;
        if (tr->nrecent < RS_TRACK_RECENT) tr->nrecent++;
    }
}
static void link_tracks(rs_multi_t *m, float t)
{
    /* every track remembers this frame's packets first; then, per pair, A-new vs B-all (same
     * frame included) and B-new vs A-past, so each packet pair is counted once */
    for (int k = 0; k < RS_MAX_TRACKS; k++) if (m->tracks[k].active) remember_packets(&m->tracks[k], t);
    for (int a = 0; a < RS_MAX_TRACKS; a++) {
        rs_track_t *ta = &m->tracks[a];
        if (!ta->active) continue;
        for (int b = a + 1; b < RS_MAX_TRACKS; b++) {
            rs_track_t *tb = &m->tracks[b];
            if (!tb->active) continue;
            float evidence = 0;
            for (int i = 0; i < ta->rx.npkts; i++) evidence += pair_evidence(ta, &ta->rx.pkts[i], t, tb, 0);
            for (int i = 0; i < tb->rx.npkts; i++) evidence += pair_evidence(tb, &tb->rx.pkts[i], t, ta, 1);
            float s = m->link[a][b] * 0.995f + evidence;
            if (s > 12.0f) s = 12.0f; if (s < -4.0f) s = -4.0f;
            m->link[a][b] = m->link[b][a] = s;
        }
    }
    /* groups: start from own id, then pull every linked pair to the smaller id */
    for (int k = 0; k < RS_MAX_TRACKS; k++) if (m->tracks[k].active) m->tracks[k].group = m->tracks[k].id;
    for (int pass = 0; pass < RS_MAX_TRACKS; pass++) {
        for (int a = 0; a < RS_MAX_TRACKS; a++) for (int b = a + 1; b < RS_MAX_TRACKS; b++) {
            if (!m->tracks[a].active || !m->tracks[b].active) continue;
            int linked = m->link[a][b] >= RS_LINK_ON || (m->link[a][b] >= RS_LINK_OFF && m->linked[a][b]);
            if (pass == 0) m->linked[a][b] = (unsigned char)linked;
            if (!linked) continue;
            int g = m->tracks[a].group < m->tracks[b].group ? m->tracks[a].group : m->tracks[b].group;
            m->tracks[a].group = g; m->tracks[b].group = g;
        }
    }
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
        if (have) { u.cx /= sw; u.cy /= sw; track_feed(m, tr, px, w, h, row_stride, pixel_stride, r_off, g_off, b_off, &u, t); }
        else if (t - tr->last_seen > RS_TRACK_TTL) tr->active = 0;
    }
    /* new tracks for the remaining blobs (brightest first) */
    for (int i = 0; i < m->nblobs; i++) {
        if (assigned[i]) continue;
        int k = -1;
        for (int j = 0; j < RS_MAX_TRACKS; j++) if (!m->tracks[j].active) { k = j; break; }
        if (k < 0) break;
        rs_track_t *tr = &m->tracks[k];
        tr->active = 1; tr->id = m->next_id++; tr->seen_frames = 0; tr->group = tr->id;
        tr->drop_clipped = 0; tr->last_packets = 0; tr->frames_since_eval = 0; tr->nrecent = 0; tr->recent_head = 0;
        for (int j = 0; j < RS_MAX_TRACKS; j++) { m->link[k][j] = m->link[j][k] = 0; m->linked[k][j] = m->linked[j][k] = 0; }
        rs_rx_init(&tr->rx);
        track_feed(m, tr, px, w, h, row_stride, pixel_stride, r_off, g_off, b_off, &m->blobs[i], t);
        assigned[i] = 1;
    }
    link_tracks(m, t);
    int total = 0;
    for (int k = 0; k < RS_MAX_TRACKS; k++) if (m->tracks[k].active) total += m->tracks[k].packets_frame;
    return total;
}

int rs_multi_pop_message(rs_multi_t *m, rs_message_t *out, int *track_id)
{
    for (int k = 0; k < RS_MAX_TRACKS; k++) {
        rs_track_t *tr = &m->tracks[k];
        if (!tr->active) continue;
        if (rs_rx_pop_message(&tr->rx, out)) { if (track_id) *track_id = tr->group; return 1; }
    }
    return 0;
}

size_t rs_multi_sizeof(void) { return sizeof(rs_multi_t); }

/* Reported tracks are the confirmed ones: a light that decoded something, or one seen for a
 * while (a reflection or a speck at the frame edge lives a few frames and never decodes). */
static int confirmed(const rs_track_t *tr)
{
    return tr->active && (tr->rx.packets_total > 0 || tr->seen_frames >= RS_TRACK_CONFIRM_FRAMES);
}
const rs_rx_t *rs_multi_track_rx(const rs_multi_t *m, int i)
{
    const rs_track_t *tr = rs_multi_track(m, i);
    return tr ? &tr->rx : NULL;
}
float rs_multi_link_score(const rs_multi_t *m, int i, int j)
{
    const rs_track_t *a = rs_multi_track(m, i), *b = rs_multi_track(m, j);
    if (!a || !b) return 0;
    return m->link[(int)(a - m->tracks)][(int)(b - m->tracks)];
}
int rs_multi_track_group(const rs_multi_t *m, int i)
{
    const rs_track_t *tr = rs_multi_track(m, i);
    return tr ? tr->group : 0;
}
int rs_multi_track_count(const rs_multi_t *m)
{
    int n = 0;
    for (int k = 0; k < RS_MAX_TRACKS; k++) if (confirmed(&m->tracks[k])) n++;
    return n;
}

const rs_track_t *rs_multi_track(const rs_multi_t *m, int i)
{
    for (int k = 0; k < RS_MAX_TRACKS; k++) {
        if (!confirmed(&m->tracks[k])) continue;
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
