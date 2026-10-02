/*
 * rs_assembler.h — reassemble messages from decoded packets.
 * Systematic and fountain-coded packets are rows of a linear system over
 * GF(2); incremental Gaussian elimination solves the message as soon as the
 * rank reaches the length. Packets received before the META are buffered.
 */
#ifndef RS_ASSEMBLER_H
#define RS_ASSEMBLER_H

#include "rs_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS_ASM_PENDING 24
#define RS_ASM_RAW     56      /* raw rows kept for poisoned-row recovery */
#define RS_ASM_MAX_CONTRADICTIONS 6   /* rows inconsistent with the system before the slot starts over */
#define RS_TEXT_MAX    64

typedef struct {
    uint8_t  have_meta, len, level, packed, delivered;
    uint8_t  have_crc, msg_crc, have_crc2, msg_crc2;
    uint8_t  rank;
    uint32_t pivot_have;              /* bit p: pivot row with lowest bit p stored */
    uint32_t pivot_mask[RS_MSG_MAX_LEN + 1];
    uint8_t  pivot_val[RS_MSG_MAX_LEN + 1];
    uint16_t pend_seed[RS_ASM_PENDING];
    uint8_t  pend_val[RS_ASM_PENDING];
    uint8_t  npend;
    uint8_t  data[RS_MSG_MAX_LEN];    /* solved bytes */
    uint32_t raw_mask[RS_ASM_RAW];    /* raw rows as received (mask, value), ring */
    uint8_t  raw_val[RS_ASM_RAW];
    uint8_t  nraw, raw_head;
    uint32_t packets;
    uint32_t resets;
    uint32_t recovered;               /* messages saved by excluding a poisoned row */
    uint8_t  contradictions;          /* rows inconsistent with the system since the last reset */
    uint8_t  meta_strikes;            /* consecutive META/CRC packets disagreeing with the stored ones */
} rs_asm_slot_t;

typedef struct {
    rs_asm_slot_t slots[RS_NUM_SLOTS];
    uint32_t packets_total;
    uint32_t messages_total;
} rs_asm_t;

typedef struct {
    uint8_t id;
    uint8_t level;
    uint8_t len;                      /* text length */
    char    text[RS_TEXT_MAX];        /* NUL-terminated */
} rs_message_t;

void rs_asm_init(rs_asm_t *a);
size_t rs_asm_sizeof(void);
uint32_t rs_asm_recovered(const rs_asm_t *a);   /* messages saved by poisoned-row exclusion */

/* Feed one decoded packet. Returns 1 when a new complete message is in *out. */
int rs_asm_feed(rs_asm_t *a, const rs_packet_t *pkt, rs_message_t *out);

/* Fraction (0..1) of the slot's message recovered (rank/len). */
float rs_asm_progress(const rs_asm_t *a, uint8_t id);

#ifdef __cplusplus
}
#endif
#endif
