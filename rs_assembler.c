#include "rs_assembler.h"
#include "rs_pack.h"

static void slot_reset(rs_asm_slot_t *s)
{
    uint32_t packets = s->packets, resets = s->resets;
    uint8_t *p = (uint8_t *)s;
    for (size_t i = 0; i < sizeof(*s); i++) p[i] = 0;
    s->packets = packets; s->resets = resets + 1;
}

void rs_asm_init(rs_asm_t *a)
{
    uint8_t *p = (uint8_t *)a;
    for (size_t i = 0; i < sizeof(*a); i++) p[i] = 0;
}

static int lowest_bit(uint32_t m) { int p = 0; while (!(m & 1u)) { m >>= 1; p++; } return p; }

/* Insert a row (mask, val). Returns 1 new info, 0 redundant, -1 inconsistent. */
static int insert_row(rs_asm_slot_t *s, uint32_t mask, uint8_t val)
{
    while (mask) {
        int p = lowest_bit(mask);
        if (s->pivot_have & (1u << p)) {
            mask ^= s->pivot_mask[p];
            val ^= s->pivot_val[p];
        } else {
            s->pivot_mask[p] = mask; s->pivot_val[p] = val;
            s->pivot_have |= 1u << p;
            s->rank++;
            return 1;
        }
    }
    return val == 0 ? 0 : -1;
}

static void solve(rs_asm_slot_t *s)
{
    for (int p = RS_MSG_MAX_LEN; p >= 0; p--) {
        if (!(s->pivot_have & (1u << p))) continue;
        uint8_t v = s->pivot_val[p];
        uint32_t rest = s->pivot_mask[p] & ~(1u << p);
        for (int q = p + 1; q <= RS_MSG_MAX_LEN; q++) if (rest & (1u << q)) v ^= s->data[q];
        s->data[p] = v;
    }
}

static int try_complete(rs_asm_t *a, rs_asm_slot_t *s, uint8_t id, rs_message_t *out)
{
    if (!s->have_meta || !s->have_crc || !s->have_crc2 || s->len == 0 || s->delivered || s->rank < s->len) return 0;
    solve(s);
    if (rs_crc8(s->data, s->len) != s->msg_crc || rs_crc8b(s->data, s->len) != s->msg_crc2) {
        /* a corrupted row slipped through the packet CRC: start over */
        slot_reset(s);
        return 0;
    }
    s->delivered = 1;
    a->messages_total++;
    if (out) {
        out->id = id; out->level = s->level;
        if (s->packed) {
            out->len = (uint8_t)rs_unpack6(s->data, s->len, out->text, RS_TEXT_MAX);
        } else {
            for (int i = 0; i < s->len; i++) out->text[i] = (char)s->data[i];
            out->text[s->len] = 0; out->len = s->len;
        }
    }
    return 1;
}

/* Feed a (seed, value) into the linear system; needs META. */
static int feed_row(rs_asm_slot_t *s, uint16_t seed, uint8_t val)
{
    uint32_t mask = (seed < s->len) ? (1u << seed) : rs_code_mask(seed, s->len);
    int r = insert_row(s, mask, val);
    if (r < 0) {
        /* contradiction: a new message replaced this slot -> start over with this row */
        slot_reset(s);
        return -1;
    }
    return r;
}

int rs_asm_feed(rs_asm_t *a, const rs_packet_t *pkt, rs_message_t *out)
{
    uint8_t id = pkt->id & 7u;
    uint16_t seed = pkt->seed;
    rs_asm_slot_t *s = &a->slots[id];
    a->packets_total++;
    s->packets++;

    if (seed == RS_SEED_MSGCRC) {
        if (s->have_crc && s->msg_crc != pkt->payload) slot_reset(s);   /* new message in this slot */
        s->have_crc = 1; s->msg_crc = pkt->payload;
        return try_complete(a, s, id, out);
    }
    if (seed == RS_SEED_MSGCRC2) {
        if (s->have_crc2 && s->msg_crc2 != pkt->payload) slot_reset(s);
        s->have_crc2 = 1; s->msg_crc2 = pkt->payload;
        return try_complete(a, s, id, out);
    }
    if (rs_seed_is_meta(seed)) {
        uint8_t len = rs_meta_len(pkt->payload), level = rs_meta_level(pkt->payload);
        uint8_t packed = (seed == RS_SEED_META_PACKED);
        if (len == 0) { slot_reset(s); return 0; }
        if (s->have_meta && (s->len != len || s->level != level || s->packed != packed)) slot_reset(s);
        if (!s->have_meta) {
            s->have_meta = 1; s->len = len; s->level = level; s->packed = packed;
            /* replay packets that arrived before the META */
            uint8_t n = s->npend; s->npend = 0;
            for (uint8_t i = 0; i < n; i++) {
                if (feed_row(s, s->pend_seed[i], s->pend_val[i]) < 0) {
                    s->have_meta = 1; s->len = len; s->level = level; s->packed = packed;
                }
            }
        }
        return try_complete(a, s, id, out);
    }
    if (!s->have_meta) {
        if (s->npend < RS_ASM_PENDING) { s->pend_seed[s->npend] = seed; s->pend_val[s->npend] = pkt->payload; s->npend++; }
        return 0;
    }
    if (feed_row(s, seed, pkt->payload) < 0) {
        /* keep the offending row as the first evidence of the new message */
        if (s->npend < RS_ASM_PENDING) { s->pend_seed[s->npend] = seed; s->pend_val[s->npend] = pkt->payload; s->npend++; }
        return 0;
    }
    return try_complete(a, s, id, out);
}

float rs_asm_progress(const rs_asm_t *a, uint8_t id)
{
    const rs_asm_slot_t *s = &a->slots[id & 7u];
    if (!s->have_meta || s->len == 0) return 0;
    return (float)s->rank / (float)s->len;
}
