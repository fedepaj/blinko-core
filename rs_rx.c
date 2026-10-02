#include "rs_rx.h"

static float s_ch[3][RS_DEC_MAX_ROWS];
static float s_luma[RS_DEC_MAX_ROWS];

void rs_rx_init(rs_rx_t *rx)
{
    uint8_t *p = (uint8_t *)rx;
    for (size_t i = 0; i < sizeof(*rx); i++) p[i] = 0;
    rs_dec_cfg_default(&rx->cfg);
    rs_asm_init(&rx->assembler);
    rs_rgb_cal_init(&rx->cal);
    rx->stitch_enabled = 1;
    for (int c = 0; c < 3; c++) rs_stitch_init(&rx->stitch[c]);
    rx->last_pilot_t = -1e9f;
}

static void push_msg(rs_rx_t *rx, const rs_message_t *m)
{
    if (rx->qlen >= RS_RX_QUEUE) return;
    rx->queue[(rx->qhead + rx->qlen) % RS_RX_QUEUE] = *m;
    rx->qlen++;
}

/* Packets sit on a grid: the transmitter sends them back to back, so from one decoded packet
 * the others in the frame are RS_PKT_CHIPS chips away (pilot blocks and burst pauses break the
 * grid; those positions simply fail). A packet whose sync was destroyed (clipped, smeared)
 * can still be decoded at its predicted position: the bits survive longer than the sync. */
static int grid_decode(rs_rx_t *rx, const float *p, int n, rs_packet_t *out, int k, int max_out)
{
    /* called right after rs_decode_profile on the same profile: its cumulative sums are reused */
    if (!rx->cfg.grid_decode || k == 0) return k;
    int n0 = k;
    float period = 0;
    for (int i = 0; i < n0; i++) period += out[i].rows_per_chip;
    period = period / (float)n0 * (float)RS_PKT_CHIPS;
    float rpc = period / (float)RS_PKT_CHIPS;
    for (int i = 0; i < n0 && k < max_out; i++) {
        for (int dir = -1; dir <= 1; dir += 2) {
            for (int m = 1; m <= 4 && k < max_out; m++) {
                float pos = out[i].row_start + (float)dir * (float)m * period;
                if (pos < 0 || pos + period > (float)n) break;
                int taken = 0;
                for (int j = 0; j < k; j++) if (out[j].row_start > pos - 2 * rpc && out[j].row_start < pos + 2 * rpc) { taken = 1; break; }
                if (taken) continue;
                static const float d[3] = { 0.0f, -0.3f, 0.3f };
                rs_packet_t pk; int ok = 0;
                for (int q = 0; q < 3 && !ok; q++)
                    ok = rs_decode_at_prepared(p, n, &rx->cfg, pos + d[q] * rpc, rpc, &pk) && pk.quality >= 2.0f * rx->cfg.min_quality;
                if (!ok) break;              /* the grid is broken here (pilot, pause, edge): stop this direction */
                out[k++] = pk; rx->grid_ok++;
            }
        }
    }
    return k;
}

/* Decoding of one channel's profile: the detector, then the grid decode on the same prepared
 * profile. A pure function of the profile and the configuration, so channels can run in parallel. */
static int stitch_channel(rs_rx_t *rx, const float *p, int n, uint8_t channel, int got_packets, float t, rs_packet_t *out);
static int decode_profile(rs_rx_t *rx, const float *p, int n, uint8_t channel, rs_packet_t *out, rs_dec_stats_t *st, int *stitched)
{
    int k = rs_decode_profile(p, n, &rx->cfg, out, 32, st);
    k = grid_decode(rx, p, n, out, k, 32);
    *stitched = 0;
    if (k == 0 && stitch_channel(rx, p, n, channel, k, rx->frame_t, &out[0])) { k = 1; *stitched = 1; }
    return k;
}

/* Bookkeeping of one channel's packets: clock hint, packet list, assembly. Sequential. */
static void take_packets(rs_rx_t *rx, const rs_packet_t *out, int k, uint8_t channel)
{
    if (k == 0) { if (++rx->empty_frames >= 30) { rx->rpc_n = 0; rx->empty_frames = 0; } } else rx->empty_frames = 0;   /* a stale clock (chip changed) is released after 30 empty frames, not on every one */
    for (int i = 0; i < k && rx->npkts < RS_RX_MAX_PKTS; i++) {
        rx->pkts[rx->npkts].pkt = out[i];
        rx->pkts[rx->npkts].channel = channel;
        rx->npkts++;
        rx->packets_total++;
        if (rx->rows_per_chip == 0) { rx->rows_per_chip = out[i].rows_per_chip; rx->rpc_n = 1; }
        else if (out[i].rows_per_chip > 0.85f * rx->rows_per_chip && out[i].rows_per_chip < 1.15f * rx->rows_per_chip) {
            rx->rows_per_chip = 0.9f * rx->rows_per_chip + 0.1f * out[i].rows_per_chip; if (rx->rpc_n < 1000) rx->rpc_n++;
        } else if (--rx->rpc_n <= 0) { rx->rows_per_chip = out[i].rows_per_chip; rx->rpc_n = 1; }   /* the clock changed (chip setting) or the first one was false */
        if (rx->defer_assembly) continue;
        rs_message_t m;
        if (rs_asm_feed(&rx->assembler, &out[i], &m)) push_msg(rx, &m);
    }
}

/* The clock hint narrows the sync search; it needs 3 concordant packets (a single false accept
 * must not lock the clock) and is kept only while packets keep coming (the board's chip length
 * can change), see take_packets. */
static void set_hint(rs_rx_t *rx) { rx->cfg.rows_per_chip_hint = rx->rpc_n >= 3 ? rx->rows_per_chip : 0; }

static void decode_channel(rs_rx_t *rx, const float *p, int n, uint8_t channel)
{
    rs_packet_t out[32]; int stitched;
    set_hint(rx);
    int k = decode_profile(rx, p, n, channel, out, &rx->last_stats, &stitched);
    if (stitched) rx->stitched_total++;
    take_packets(rx, out, k, channel);
}

/* Three channels at once through the platform's parallel hook, then the bookkeeping in order. */
typedef struct { rs_rx_t *rx; const float *p[3]; int n; rs_packet_t out[3][32]; int k[3]; int stitched[3]; rs_dec_stats_t st[3]; } rs_job3_t;
static void job3(void *ctx, int i) { rs_job3_t *j = (rs_job3_t *)ctx; j->k[i] = decode_profile(j->rx, j->p[i], j->n, (uint8_t)i, j->out[i], &j->st[i], &j->stitched[i]); }
static void decode_channels3(rs_rx_t *rx, const float *c0, const float *c1, const float *c2, int n)
{
    if (!rx->parallel) { decode_channel(rx, c0, n, 0); decode_channel(rx, c1, n, 1); decode_channel(rx, c2, n, 2); return; }
    static rs_job3_t j;                         /* one frame at a time per receiver; the hook returns when every job is done */
    j.rx = rx; j.p[0] = c0; j.p[1] = c1; j.p[2] = c2; j.n = n;
    set_hint(rx);
    rx->parallel(rx->parallel_user, 3, job3, &j);
    for (int c = 0; c < 3; c++) { if (j.stitched[c]) rx->stitched_total++; take_packets(rx, j.out[c], j.k[c], (uint8_t)c); }
    rx->last_stats = j.st[2];
}

void rs_rx_set_camera(rs_rx_t *rx, rs_camera_t cam) { rx->cfg.exposure_rows = cam.exposure_rows; rx->row_seconds = cam.row_seconds; }

/* Stitching: when a channel's frame yielded no packet, its piece joins the stitcher of that
 * channel. Runs on the thread that ran the detector on this profile (the decoder's validated
 * syncs and normalized profile are thread-local), right after it. Returns 1 with a packet. */
static int stitch_channel(rs_rx_t *rx, const float *p, int n, uint8_t channel, int got_packets, float t, rs_packet_t *out)
{
    if (!rx->stitch_enabled || got_packets) return 0;
    rs_sync_t syncs[4]; int ns = rs_decode_last_syncs(p, n, syncs, 4);
    if (ns < 0) ns = rs_decode_syncs(p, n, &rx->cfg, syncs, 4);
    const float *norm, *amp; int m; rs_decode_normalized(&norm, &amp, &m);
    if (m != n) return 0;
    rs_stitch_in_t in = { p, norm, amp, n, syncs, ns, rx->cfg.rows_per_chip_hint, rx->row_seconds, t, &rx->cfg };
    return rs_stitch_feed(&rx->stitch[channel], &in, out);
}

void rs_rx_set_parallel(rs_rx_t *rx, rs_parallel_fn fn, void *user) { rx->parallel = fn; rx->parallel_user = user; rx->cfg.parallel = fn; rx->cfg.parallel_user = user; }

void rs_rx_assemble(rs_rx_t *rx, const uint8_t *keep)
{
    int n = 0;
    for (int i = 0; i < rx->npkts; i++) {
        if (keep && !keep[i]) { rx->packets_total--; continue; }
        rs_message_t m;
        if (rs_asm_feed(&rx->assembler, &rx->pkts[i].pkt, &m)) push_msg(rx, &m);
        if (n != i) rx->pkts[n] = rx->pkts[i];
        n++;
    }
    rx->npkts = n;
}

int rs_rx_three_coloured(const float *r, const float *g, const float *b, int n)
{
    const float *ch[3] = { r, g, b }; float rng[3], maxrng = 0;
    for (int c = 0; c < 3; c++) {
        float a = 1e30f, z = -1e30f;
        for (int i = 0; i < n; i++) { float v = ch[c][i]; if (v < a) a = v; if (v > z) z = v; }
        rng[c] = z - a; if (rng[c] > maxrng) maxrng = rng[c];
    }
    if (maxrng < 30.0f) return 0;
    for (int c = 0; c < 3; c++) if (rng[c] < 0.3f * maxrng) return 0;
    return 1;
}

int rs_rx_process(rs_rx_t *rx, const float *r, const float *g, const float *b, int n, float t)
{
    if (n > RS_DEC_MAX_ROWS) n = RS_DEC_MAX_ROWS;
    rx->frames++;
    rx->npkts = 0;
    rx->frame_t = t;
    if (b == NULL || g == NULL) {
        rx->mode = 0;
        decode_channel(rx, r, n, 0);
        return rx->npkts;
    }
    /* Is the light genuinely three-coloured (all camera channels modulated)? A 3-die RGB LED
     * seen defocused is three colour discs offset by ~40 % of their diameter, so no pilot lock
     * may ever happen; in that case the camera channels are decoded directly. */
    int three = rs_rx_three_coloured(r, g, b, n);
    if (rs_rgb_pilot_detect(&rx->cal, r, g, b, n)) rx->last_pilot_t = t;
    int rgb = rx->cal.valid && (t - rx->last_pilot_t) < RS_RX_CAL_TTL;
#ifdef RS_RX_DIRECT_ALWAYS
    if (three) rgb = 0;
#endif
    rx->mode = rgb ? 1 : 0;
    if (rgb) {
        rs_rgb_unmix(&rx->cal, r, g, b, n, s_ch[0], s_ch[1], s_ch[2]);
        decode_channels3(rx, s_ch[0], s_ch[1], s_ch[2], n);
    } else if (three) {
        /* three-coloured light but no pilot lock yet: decode the camera channels directly
         * (crosstalk is moderate for LED primaries; the CRC rejects what it corrupts) */
        rx->mode = 2;
        decode_channels3(rx, r, g, b, n);
    } else {
        for (int i = 0; i < n; i++) s_luma[i] = (r[i] + g[i] + b[i]) * (1.0f / 3.0f);
        decode_channel(rx, s_luma, n, 0);
    }
    return rx->npkts;
}

int rs_rx_pop_message(rs_rx_t *rx, rs_message_t *out)
{
    if (rx->qlen == 0) return 0;
    *out = rx->queue[rx->qhead];
    rx->qhead = (rx->qhead + 1) % RS_RX_QUEUE; rx->qlen--;
    return 1;
}

size_t rs_rx_sizeof(void) { return sizeof(rs_rx_t); }
int rs_rx_mode(const rs_rx_t *rx) { return rx->mode; }
float rs_rx_rows_per_chip(const rs_rx_t *rx) { return rx->rows_per_chip; }
int rs_rx_pilots(const rs_rx_t *rx) { return rx->cal.pilots_seen; }
float rs_rx_cal_cond(const rs_rx_t *rx) { return rx->cal.cond; }
uint32_t rs_rx_packets(const rs_rx_t *rx) { return rx->packets_total; }
uint32_t rs_rx_stitched(const rs_rx_t *rx) { return rx->stitched_total; }
uint32_t rs_rx_messages(const rs_rx_t *rx) { return rx->assembler.messages_total; }
int rs_rx_packet_at(const rs_rx_t *rx, int i, rs_packet_t *pkt, uint8_t *channel)
{
    if (i < 0 || i >= rx->npkts) return 0;
    *pkt = rx->pkts[i].pkt; *channel = rx->pkts[i].channel; return 1;
}
const rs_dec_stats_t *rs_rx_stats(const rs_rx_t *rx) { return &rx->last_stats; }

uint32_t rs_rx_resets(const rs_rx_t *rx)
{
    uint32_t n = 0;
    for (int i = 0; i < RS_NUM_SLOTS; i++) n += rx->assembler.slots[i].resets;
    return n;
}
