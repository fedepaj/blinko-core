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

static void raw_push(rs_asm_slot_t *s, uint32_t mask, uint8_t val)
{
    uint8_t i = (uint8_t)((s->raw_head + s->nraw) % RS_ASM_RAW);
    if (s->nraw < RS_ASM_RAW) s->nraw++; else { s->raw_head = (uint8_t)((s->raw_head + 1) % RS_ASM_RAW); i = (uint8_t)((s->raw_head + s->nraw - 1) % RS_ASM_RAW); }
    s->raw_mask[i] = mask; s->raw_val[i] = val;
}

/* Solve the message from the raw rows, skipping row `skip` (or none if < 0).
 * Returns 1 if full rank and both message CRCs match; data[] is filled. */
static int solve_from_raw(const rs_asm_slot_t *s, int skip, uint8_t data[RS_MSG_MAX_LEN])
{
    uint32_t pm[RS_MSG_MAX_LEN + 1]; uint8_t pv[RS_MSG_MAX_LEN + 1]; uint32_t have = 0; int rank = 0;
    for (int k = 0; k < s->nraw; k++) {
        if (k == skip) continue;
        int idx = (s->raw_head + k) % RS_ASM_RAW;
        uint32_t mask = s->raw_mask[idx]; uint8_t val = s->raw_val[idx];
        while (mask) {
            int p = lowest_bit(mask);
            if (have & (1u << p)) { mask ^= pm[p]; val ^= pv[p]; }
            else { pm[p] = mask; pv[p] = val; have |= 1u << p; rank++; break; }
        }
    }
    if (rank < s->len) return 0;
    for (int p = RS_MSG_MAX_LEN; p >= 0; p--) {
        if (!(have & (1u << p))) continue;
        uint8_t v = pv[p]; uint32_t rest = pm[p] & ~(1u << p);
        for (int q = p + 1; q <= RS_MSG_MAX_LEN; q++) if (rest & (1u << q)) v ^= data[q];
        data[p] = v;
    }
    return rs_crc8(data, s->len) == s->msg_crc && rs_crc8b(data, s->len) == s->msg_crc2;
}

/* The message CRCs that have been received (one is enough to deliver a directly solved
 * system: the packet CRCs already filter rows; both are required for the leave-one-out
 * recovery, whose many trials would make a single CRC-8 unsafe). */
static int msg_crc_ok(const rs_asm_slot_t *s, const uint8_t *data)
{
    if (!s->have_crc && !s->have_crc2) return 0;
    if (s->have_crc && rs_crc8(data, s->len) != s->msg_crc) return 0;
    if (s->have_crc2 && rs_crc8b(data, s->len) != s->msg_crc2) return 0;
    return 1;
}

static int try_complete(rs_asm_t *a, rs_asm_slot_t *s, uint8_t id, rs_message_t *out)
{
    if (!s->have_meta || (!s->have_crc && !s->have_crc2) || s->len == 0 || s->delivered || s->rank < s->len) return 0;
    solve(s);
    if (!msg_crc_ok(s, s->data)) {
        if (!s->have_crc || !s->have_crc2) return 0;   /* wait for the second CRC before trying recoveries */
        /* A row that passed the packet CRC by chance poisoned the system. Find it:
         * re-solve from the raw rows leaving one out at a time (<= 56 small GF(2)
         * eliminations). If a leave-one-out solution satisfies both message CRCs,
         * that is the message; rebuild the pivots without the poisoned row. */
        uint8_t data[RS_MSG_MAX_LEN];
        int found = -1;
        for (int k = 0; k < s->nraw && found < 0; k++) if (solve_from_raw(s, k, data)) found = k;
        if (found < 0) { slot_reset(s); return 0; }
        s->recovered++;
        /* drop the poisoned row and rebuild the reduced system */
        int idx = (s->raw_head + found) % RS_ASM_RAW;
        s->raw_mask[idx] = 0;
        s->pivot_have = 0; s->rank = 0;
        for (int k = 0; k < s->nraw; k++) {
            int j = (s->raw_head + k) % RS_ASM_RAW;
            if (s->raw_mask[j]) insert_row(s, s->raw_mask[j], s->raw_val[j]);
        }
        for (int i = 0; i < s->len; i++) s->data[i] = data[i];
        s->contradictions = 0;
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
    raw_push(s, mask, val);
    int r = insert_row(s, mask, val);
    /* A contradiction means either a poisoned row already sits among the pivots
     * (this row may be the correct one) or a new message replaced the slot. The
     * row stays in the raw ring so the leave-one-out solve at completion can tell. */
    return r;
}

int rs_asm_feed(rs_asm_t *a, const rs_packet_t *pkt, rs_message_t *out)
{
    uint8_t id = pkt->id & 7u;
    uint16_t seed = pkt->seed;
    rs_asm_slot_t *s = &a->slots[id];
    a->packets_total++;
    s->packets++;

    if (seed == RS_SEED_MSGCRC || seed == RS_SEED_MSGCRC2) {
        uint8_t have = (seed == RS_SEED_MSGCRC) ? s->have_crc : s->have_crc2;
        uint8_t cur  = (seed == RS_SEED_MSGCRC) ? s->msg_crc  : s->msg_crc2;
        if (have && cur != pkt->payload) {
            /* one disagreeing CRC packet may itself be corrupted; two in a row mean a new message */
            if (++s->meta_strikes < 2) return 0;
            slot_reset(s);
        } else if (have) {
            s->meta_strikes = 0;
        }
        if (seed == RS_SEED_MSGCRC) { s->have_crc = 1; s->msg_crc = pkt->payload; }
        else                        { s->have_crc2 = 1; s->msg_crc2 = pkt->payload; }
        return try_complete(a, s, id, out);
    }
    if (rs_seed_is_meta(seed)) {
        uint8_t len = rs_meta_len(pkt->payload), level = rs_meta_level(pkt->payload);
        uint8_t packed = (seed == RS_SEED_META_PACKED);
        if (len == 0) { slot_reset(s); return 0; }
        if (s->have_meta && (s->len != len || s->level != level || s->packed != packed)) {
            if (++s->meta_strikes < 2) return 0;
            slot_reset(s);
        } else if (s->have_meta) {
            s->meta_strikes = 0;
        }
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
        if (++s->contradictions >= 6) {            /* really a new message: start over with this row */
            slot_reset(s);
            if (s->npend < RS_ASM_PENDING) { s->pend_seed[s->npend] = seed; s->pend_val[s->npend] = pkt->payload; s->npend++; }
        }
        return try_complete(a, s, id, out);
    }
    return try_complete(a, s, id, out);
}

float rs_asm_progress(const rs_asm_t *a, uint8_t id)
{
    const rs_asm_slot_t *s = &a->slots[id & 7u];
    if (!s->have_meta || s->len == 0) return 0;
    return (float)s->rank / (float)s->len;
}

size_t rs_asm_sizeof(void) { return sizeof(rs_asm_t); }
uint32_t rs_asm_recovered(const rs_asm_t *a)
{
    uint32_t n = 0;
    for (int i = 0; i < RS_NUM_SLOTS; i++) n += a->slots[i].recovered;
    return n;
}
