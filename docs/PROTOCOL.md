# Blinko — Optical protocol (v2)

## Physical layer

- **Modulation**: OOK (LED on/off). Time unit: the **chip**, of duration
  `T_chip` (default 30 µs). The chip is the only timing constant: the
  receiver recovers it, in rows, from the sync of every packet.
- **Data coding**: Manchester. `bit 1 = chip 0 then 1` (rising edge at
  mid-bit), `bit 0 = chip 1 then 0`. Data therefore contains only runs of 1
  or 2 chips.
- **Byte order**: MSB first.
- **Polarity**: chip `1` = LED on.

## Packet

Every packet has the same length: **67 chips** (2.0 ms @ 30 µs).

```
 gap  sync                 start   id (3 bit) seed (9 bit)  payload (8 bit)  CRC8 (8 bit)
 [0]  [1 1 1 1][0 0 0 0]   [1 0]   6 chip     18 chip       16 chip          16 chip     = 67
```

- **gap** `0` and **start** `10` delimit the sync (runs of exactly 4+4 chips,
  impossible in Manchester data): three edges at a known distance → an
  estimate of `T_chip` in rows for every packet.
- **id** 0..7: message slot. 0..5 rotating logs, 6 = STATUS, 7 = FAULT.
- **seed** 0..511:
  - 511 = META: payload `[len:5][level:3]`, message bytes stored raw;
  - 510 = META: same, but the bytes are 6-bit packed text;
  - 509, 508 = CRC-8/ATM and CRC-8/MAXIM of the whole message (16 guard bits);
  - `seed < len`: systematic packet, payload = message byte `seed`;
  - `len ≤ seed ≤ 507`: **coded** packet, payload = XOR of the bytes selected
    by `rs_code_mask(seed, len)` (xorshift32, identical on every platform).
- **CRC8** (poly 0x07, init 0x00) over the 20 id+seed+payload bits (3 bytes,
  last nibble zero).

Levels: 0 DEBUG, 1 INFO, 2 WARN, 3 ERROR, 4 FATAL, 5 STATUS, 6 FAULT.

## Fountain coding

Every packet is a row of a linear system over GF(2): the receiver runs
incremental Gaussian elimination and reconstructs the message as soon as the
rank reaches `len`, from any subset of packets (on average `len + 2` useful
packets, against `len·ln len` for an index-based protocol). The two message
CRCs reject solutions poisoned by a packet that slipped past its CRC-8
(residual probability 1/65536). Packets received before the META are queued
(24) and replayed.

## 6-bit text (`rs_pack.h`)

Symbols: space, a–z, 0–9, SHIFT (next letter uppercase), 25 punctuation marks,
ESC (raw 8-bit byte). Lossless; used only when it shortens the message
(typically −20/25 %). A 31-byte message carries up to ~41 characters.

## Carousel and visible blink

One visit of a message: `[META CRC CRC2] data… [META CRC CRC2] data…` for a
total of `len + 6` packets; the second control triplet sits after `len/2` data
packets, so any camera window of about half a visit contains one. The data
seeds advance at every visit (first pass systematic, then coded, wrapping from
507 back to `len`). Slot order per round: FAULT, STATUS, then the log slots
newest first. The transmitter alternates bursts of `burst_on` and pauses of
`burst_off` aligned to packet boundaries (default 150/50 ms): the LED blinks
visibly at 5 Hz and the receiver loses 25 % of the time.

### Weighted FAULT carousel

`rs_tx_set_fault_weight(tx, w)` (1..4) inserts `w−1` extra FAULT visits after
every other visit in the round: for `w = 3` the round is
`[F S F F L1 F F L2 F F …]`. The death loops use `w = 3`, which leaves the
FAULT slot about 85 % of the airtime, so the fault reason completes in one or
two seconds while the last log lines still follow it.

## RGB channels

With an RGB LED the transmitter sends **three independent streams** (R, G, B),
taking packets from the carousel in turn: three packets per packet interval,
synchronised (their syncs coincide). Every `pilot_ms` (default 30 ms, i.e.
1000 chips) a **pilot block** of 9·P chips is emitted at a packet boundary
(P = `RS_PILOT_P` = 4, so 36 chips ≈ 3.6 % overhead):
`[dark 2P][R P][dark P][G P][dark P][B P][dark 2P]`.

The receiver looks for that block in the luma profile (`r+g+b`): three equal,
equally spaced pulses (widths within 30 %, gaps within 35 % of each other and
between 0.3 and 1.6 pulse widths) between two dark zones. From the middle half
of each pulse, minus the dark baseline of the first gap, it measures the
camera's RGB response to each LED (a 3×3 matrix, columns normalised to sum 1),
checks that it is well conditioned (`cond ≥ 0.35`), smooths it into the stored
matrix (EMA, 0.5) and inverts it; the pulse width also gives an independent
rows-per-chip estimate. Without a recent pilot (3 s, `RS_RX_CAL_TTL`) or with
a degenerate matrix the receiver falls back to the other modes below, so a
board with a single LED works with the same app. No backwards compatibility
with luma-only receivers is planned.

## Reception

### Per-frame pipeline

For each camera frame (`rs_multi_process`):

1. **Segmentation** (`rs_frame_segment_rgb`): on a ⅛ thumbnail, a vertical
   running maximum of ±14 thumbnail rows bridges the dark stripes the
   modulation carves into the blob (sync gaps, pilot blocks) so one light stays
   one component; threshold at 40 % of the thumbnail range, flood fill, reject
   specks, frame-wide regions, one-cell lines and dim blobs. Up to 6 blobs,
   brightest first.
2. **Tracking** (`rs_multi.c`): up to 4 tracks, each holding a complete
   receiver. A track takes every blob overlapping its box (grown by 24 px), so
   fragments of one source merge; it is dropped after `RS_TRACK_TTL` = 1 s
   without a detection, and reported once it has decoded something or has been
   seen for 12 frames.
3. **Profile per light** (`rs_frame_profile_rgb_blob2`): three per-row
   profiles (R, G, B) over the track's box, extended by one blob height above
   and below and half a blob width each side (the halo keeps the 1-chip gaps a
   clipped core loses), clipped to the frame and to the midpoint towards any
   neighbouring track. Each column is weighted by its **mean squared row-to-row
   difference** of luma — a high-pass matched to the stripes, so columns
   carrying the modulation dominate while a clipped core (flat at 255), the
   dark surround and noise-only columns (< 5 % of the peak weight) get nothing.
   Steps into or out of clipping are excluded from the weight. The weighted
   mean uses ~128 columns.
   Two variants exist — clipped columns kept with matched weights, or dropped
   outright — and the track re-evaluates them every 8 frames, or after an empty
   frame (at most every 3), keeping the one that yields more packets on this
   frame.
4. **Decode** (`rs_rx_process`) on one or three profiles, see below.
5. **Assemble**: in multi-source mode assembly is deferred
   (`defer_assembly`), the cross-talk filter runs first, then
   `rs_rx_assemble` feeds the surviving packets to `rs_asm_feed`.

### The three decoding modes of `rs_rx`

| mode | when | what is decoded |
|---|---|---|
| 0 luma | single-channel input, or no pilot lock and the light is not three-coloured | `(r+g+b)/3` |
| 1 RGB | `cal.valid` and a pilot seen in the last 3 s | the three unmixed channels `inv · (r,g,b)` |
| 2 direct | all three camera channels modulated (`rs_rx_three_coloured`) but no pilot lock | the camera's own R, G and B as the three streams |

Mode 2 exists because a 3-die RGB LED seen defocused is three colour discs
offset by about 40 % of their diameter: the pilot pattern may never appear in
the combined profile, yet each camera channel already sees mostly one die.
Cross-talk between LED primaries is moderate and the packet CRC rejects what it
corrupts. `rs_rx_three_coloured` requires a luma range of at least 30 counts
and every channel range at least 30 % of the largest.

### Packet decoding (`rs_decode_profile`)

Per profile, at candidate scales 3, 6, 12, 24 and 44 rows/chip (each accepting
[0.45×, 2.2×]; once the receiver knows its chip clock only the covering scales
are searched — 1–2 passes instead of 5):

1. **Envelope**: sliding min/max over a window of 9 chips; a row whose local
   contrast is below `min_contrast` (6) is marked unknown.
2. **Binarize** at the local envelope midpoint, then run-length encode.
3. **Sync**: a low(gap) / high `L1` / low `L2` / high `L3` sequence with
   `L1 ≈ L2` (within `sync_tol` = 0.30), `L3` between 0.35 and 1.9 chips, and
   `rows_per_chip = (L1+L2)/8` inside the scale's range. The three edges are
   refined to sub-row precision by linear interpolation of the threshold
   crossing; `rpc_ref = (t2−t0)/8` must agree with `(t1−t0)/4`.
4. **Bits**: for each of the 29 bits (start + 28) the integrals of the first
   and second half of the bit are compared on the analogue profile; the
   confidence is their normalised difference, and the mid-bit edge re-locks the
   phase (PLL, gain 0.35).
5. **Checks**: start bit = 0, header CRC-8, and worst-bit confidence ≥
   `min_quality` (0.05). The packet also carries its mean |ON−OFF| amplitude,
   used later by the cross-talk filter.
6. **Timing hypotheses**: the 8-chip sync alone fixes the chip length to only
   ~2 %, which is 1.4 chips of drift by the end of a packet when the PLL loses
   its edges (saturated or noisy rows). On a failed CRC the packet is retried
   with the receiver's own chip clock (an EMA over many packets and frames) and
   with the sync estimate stretched by ±3 %. A rescued packet must decode at
   twice the minimum confidence, since CRC-8 alone would let ~1/256 of the
   corrupted syncs through per hypothesis.
7. Decoding continues after the packet (several packets per frame when the
   blob is tall); results from different scales are deduplicated (keeping the
   higher quality) and sorted by row.

An experimental **edge path** for heavily saturated signals (rising-edge
positions only) is in `rs_decoder.c` but is off by default: measured worse
than the classic path on the corpus and in simulation.

### Grid decode

The transmitter sends packets back to back, so from one decoded packet the
others in the frame sit exactly `RS_PKT_CHIPS` apart. `rs_rx` therefore calls
`rs_decode_at` (no sync search: bit decisions, PLL, start bit, CRC — the row
given is the gap chip) at up to 4 predicted positions each side of every
decoded packet, trying three sub-chip offsets (0, ±0.3 chip) and demanding
twice the minimum confidence. A packet whose sync was destroyed by clipping or
smear can still be read this way, because the bits survive longer than the
sync. The first failure in a direction stops that direction: a pilot block or a
burst pause has broken the grid there.

### Assembly (`rs_assembler.c`)

Per slot the assembler keeps the linear system (pivots and values), the two
message CRCs, the pre-META queue (24) and a ring of the last 56 **raw** rows as
received.

- **Delivery**: when the rank reaches `len` the system is solved and the
  message is delivered if the message CRCs that have arrived all match — **one
  of the two is enough** for a directly solved system, because the per-packet
  CRCs already filter the rows.
- **Leave-one-out recovery**: if a solved system fails its CRCs, a row that
  passed its packet CRC by chance poisoned it. The assembler waits until
  **both** message CRCs are known, then re-solves from the raw rows leaving one
  out at a time (≤ 56 small GF(2) eliminations). A leave-one-out solution that
  satisfies **both** CRCs is the message: the poisoned row is dropped, the
  pivots are rebuilt without it and the message is delivered (counted in
  `rs_asm_recovered`). Both CRCs are required here because the many trials
  would make a single CRC-8 unsafe. If no candidate works, the slot is reset.
- **Slot replacement**: a META or message-CRC packet disagreeing with the
  stored one is ignored once (it may itself be corrupt) and resets the slot at
  the second consecutive disagreement; six rows inconsistent with the system
  also reset it.

### Several lights in one frame (`rs_multi.c`)

**Cross-talk filter.** The stripes of a bright LED spread across the whole
frame width (flare, row gain), so a track whose own light is momentarily dark
decodes its neighbour's packets in its halo rows. A foreign CRC packet would
poison a slot for good, so before assembly each packet is checked three ways:

1. **Amplitude + timing**: a packet below `RS_LEAK_AMP` = 0.3 of another
   track's typical amplitude (EMA, after 5 packets) that also fits that track's
   carousel timing is that light's leak.
2. **Row ownership**: a packet whose centre row falls outside its own track's
   blob rows (±25 % of the box height) but inside those of another track from a
   different group belongs to that track.
3. **Duplicates**: the same (id, seed, payload) decoded by two tracks within
   2 chips of the same row is one packet — it is kept by the track whose own
   rows contain it. If both or neither contain it nothing is dropped; two
   linked lights of one board are exactly that case.

**Same-board link.** Two lights of one board (the RGB LED and the builtin LED
mirroring a channel) show packets of a single carousel. The sequence number of
a packet is `nchan · (its time in packets) + channel`, and within a slot the
seed grows by 1 per carousel packet, minus 3 per control triplet. So for two
packets of the same slot seen within ~2 frames by tracks A and B,

```
seed_b − seed_a ∈ { n·dp + dch + s }   n ∈ {1, 3}, s ∈ {0, ±3, ±6}, ±1 tolerance
```

where `dp` is their distance in packets (rows inside a frame plus the time
between frames) and `dch` their channel difference. Rows are time, so this
holds whether the two blobs overlap or sit far apart. Matches add 1 to a
decayed pair score, mismatches subtract 0.3; above 2.0 the two tracks share a
group (the smaller track id), below 0.5 they part again. Control packets
(seed ≥ 508) are excluded, and a random pair of boards matches only ~6 % of
the time, so its score drifts negative. Messages are reported under the group
id, so one board is one source in the UI.

## Fault record (payload of slot 7)

Text generated by the firmware, e.g.
`HF pc=0x0000a3f2 lr=0x0000a1c1 cfsr=0x00000400` (hard fault),
`WDT reset` (watchdog reset), `FATAL 12: sensor init` (from `Blinko.fatal`).
The reset cause (`RSTSR0/1/2`) is included in the STATUS at every boot.

## Recommended parameters (to be confirmed by calibration)

| Parameter | Value |
|---|---|
| `T_chip` | 30 µs (16.6 kbit/s gross per channel; measured on an iPhone 14, see CALIBRATION.md) |
| phone exposure | the shortest available (≤ `T_chip`/2); iPhone 14: 15 µs at 120 fps |
| rows per chip | ≥ 4 |
| packet length | 67 chips = 2.0 ms |
| channels | 3 (RGB), pilot every 30 ms (36 chips, 3.6 % overhead) |
| visible blink | 150 ms on / 50 ms off |
| FAULT weight | 3 in the death loops |
