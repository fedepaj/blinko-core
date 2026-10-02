# Blinko — Optical protocol (v3)

The wire format and the receiver pipeline, in enough detail to write another
implementation. Design rationale and the history of the protocol live in the
source comments (`rs_proto.h`, `rs_decoder.c`); measurements are in
`docs/FINDINGS.md` of the umbrella repository.

## Physical layer

- **Modulation**: OOK (LED on/off). Time unit: the **chip** (one code cell,
  the transmitter's timer period). The constant a board is configured with is
  **T = 3 chips** (`RS_CELLS_PER_T`), the shortest run the line code produces:
  T is what the camera exposure must stay below. Default T = 60 µs, so the
  timer runs at 20 µs. The receiver recovers the chip clock, in rows, from the
  sync of every packet.
- **Line code**: RLL(2,7) in NRZI. Transitions are between 3 and 8 chips
  apart; the code carries 1.5 bits per T. It is the variable-length table code
  of the hard-disk literature, driven bit by bit
  (`rs_rll27_step` in `rs_proto.h`):

  | bits | code (chips, transition = 1) |
  |---|---|
  | `10` | `0100` |
  | `11` | `1000` |
  | `000` | `000100` |
  | `010` | `100100` |
  | `011` | `001000` |
  | `0010` | `00100100` |
  | `0011` | `00001000` |

  A `1` in the code toggles the LED level; the encoder starts from OFF after
  the sync. Only runs of 3..8 chips exist in data, so a run of 10 chips cannot
  occur and is the sync.
- **Byte order**: MSB first.
- **Polarity**: LED on = high level.

## Packet

Every packet has the same length: **82 chips** (3.3 ms at T = 60 µs, 1.6 ms at
T = 30 µs).

```
 gap     sync ON    off     data field (66 chips)
 [0 0 0] [1 × 10]   [0 0 0] RLL(2,7) of 30 bits: id(3) seed(7) payload(8) crc12(12), flush, filler
```

- **sync** `[OFF 3][ON 10][OFF 3]`: the ON run is a run-length violation of
  the code, its two edges 10 chips apart give the chip clock, and the OFF runs
  on both sides are the shortest the code allows, so the sync is also a legal
  neighbour of the data around it.
- **data field**: the 30 bits encoded from level OFF; the code tree is flushed
  with zero bits (≤ 3) and the field is **filled to 66 chips with runs of 4**,
  so every packet has the same length, packets sit on a grid, and a bright
  chip always lies within 5 chips before the next gap (the receiver uses this
  to tell a gap from the dark surroundings of the blob).
- **id** 0..7: message slot. 0..5 rotating logs, 6 = STATUS, 7 = FAULT.
- **seed** 0..127:
  - 127 = META: payload `[len:5][level:3]`, message bytes stored raw;
  - 126 = META: same, but the bytes are 6-bit packed text;
  - 125, 124 = CRC-8/ATM and CRC-8/MAXIM of the whole message (16 guard bits);
  - `seed < len`: systematic packet, payload = message byte `seed`;
  - `len ≤ seed ≤ 123`: **coded** packet, payload = XOR of the bytes selected
    by `rs_code_mask(seed, len)` (xorshift32, identical on every platform).
- **CRC-12** (poly 0x80F, init 0xFFF) over the 18 id+seed+payload bits. The
  non-zero init means an all-dark or all-bright stretch never decodes to a
  valid packet.
- **Repetition** (`rs_tx_set_repeat`, 1..4): every packet can be sent n times
  back to back. A camera whose readout window is shorter than a packet (a 30 fps
  Android phone sees ~4 ms of each frame) reads one packet across two copies;
  see cyclic decoding below. Default 1.

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
1500 chips at T = 60 µs) a **pilot block** of 9·P chips is emitted at a packet boundary
(P = `RS_PILOT_P` = 4, so 36 chips ≈ 3.6 % overhead):
`[dark 2P][R P][dark P][G P][dark P][B P][dark 2P]`.

The receiver looks for that block in the luma profile (`r+g+b`), binarized at
15 % of its range with runs shorter than 4 rows absorbed by their neighbours
(a RAW Bayer profile toggles for a few rows at every threshold crossing): three
equal, equally spaced pulses (widths within 30 %, gaps equal within 35 % or
0.1 pulse width, and between 0.12 and 1.6 pulse widths: the exposure smear
widens the pulses and narrows the gaps alike) between two dark zones. From the middle half
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
4. **Decode** (`rs_rx_process`) on one or three profiles, see below. The
   three channels of a light are independent, so a platform can hand the
   receiver a parallel-for hook (`rs_rx_set_parallel`, `rs_multi_set_parallel`)
   and have them decoded on three threads; the core is built with
   `RS_DEC_THREADS` so that the decoder's scratch is thread-local. The clock
   hint is then the one at the start of the frame for every channel.
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

The decoder is an exposure-aware maximum-likelihood detector: it does not
threshold the profile into chips, it fits chip templates that include the
camera's exposure smear to the analogue profile. One row integrates the LED
over the exposure window E, so a step becomes a ramp E rows long and a 3-chip
run at E = T has only 64 % of its amplitude; the templates reproduce that, so
the detector works up to E ≈ 2–3 T where a threshold decoder stops at E ≈ T.
The app passes the exposure in rows (`exposure_rows = exposure_µs / row_µs`);
half a chip is assumed when unknown, and 1.4× and 2× the given value are also
tried because phones report less exposure than their edges show.

Per profile:

1. **Candidates, at every plausible scale** (2, 3.5, 6, 10, 17, 28 rows per
   chip; with a confirmed clock only the covering scale). The profile is
   envelope-normalized for the scale (sliding min/max over 15 chips: the blob
   changes brightness several-fold along a packet) and two sync finders run:
   the **binarized runs** (OFF/ON/OFF with the ON run near 10 chips, every data
   run after it 3..8 chips) and a **correlation** with the exposure-smeared sync
   template at 0.7×..1.3× of the scale, which survives the chatter of
   half-height pulses that breaks the runs. Candidates are ranked globally
   across scales, best correlation first.
2. **Validation**, without spending a detector run. The sync template is
   placed precisely (±8 % clock, ±0.3 chip); then the ON run is measured between
   its two 0.5-crossings: it must be 10 chips ± 15 % and both edges must exist
   (the blob's own bright edge, which the correlation likes because the gap
   before it is dark too, fails this), and **its length is the clock estimate**
   (two edges 10 chips apart give ~1 %; a correlation grid cannot). The 5 chips
   before the gap must contain a bright chip (a real gap follows modulated
   signal). Candidates whose exposure exceeds 3 chips are dropped: the
   templates are flat and fit anything.
3. **Detection**: a Viterbi search over the 12 states of the RLL(2,7) encoder
   (tree node × level). For each survivor and input bit the codeword's chip
   template is built at the survivor's position and clock, with the exposure
   smear and the survivor's last 8 chips as history, and compared to the
   profile at ±1 row of **timing slip**; the slip also corrects the survivor's
   own clock (a first-order PLL, gain 0.3·slip/cells, clamped ±6 %), so a clock
   hypothesis a few percent off converges instead of drifting out of the slip
   range. Metric: squared error per row. Early abort at codewords 5, 9 and 17
   when the best survivor's error is already above what the quality threshold
   allows. At ≥ 6 rows per chip the template is evaluated once per `rpc/4`
   rows against the profile's mean over those rows.
4. **Clock hypotheses**, tried until one passes: the receiver's confirmed
   clock, the sync's, ±2.5 % (±5 % with E > 2 chips).
5. **Checks**: CRC-12; **quality** = 1 / (1 + 30 · mse per row) ≥
   `min_quality` (0.4; true packets score 0.6–0.95); and a **rigid-grid
   re-fit** of the decoded chips with one clock = (end − start)/cells and a
   global phase of ±2 rows must not be more than 3× worse than the slipped fit
   (the slips can otherwise fit the stream at 2/3 of its clock, and such an
   alias can pass the CRC).
6. **Framing beyond the sync.** The usable end of the signal is where the
   local amplitude falls below 60 % of the sync's (the blob's soft edge: the
   envelope normalization lags a fade shorter than its window, so rows with
   some contrast left are still garbage). When the packet after the sync runs
   past it:
   - **cyclic decode**: the tail is read one packet period earlier, before this
     sync, where the previous copy of a repeated packet lies; the seam sits half
     a normalization window before the fade, the wrapped rows must carry signal,
     and period offsets 0, ±2, ±4 rows are tried (the period is only known to
     the clock's precision), stopping at a valid CRC;
   - **backward decode**: the packet *before* the sync is decoded — its data
     field ends where this gap begins and takes clock and phase from this sync.
     This is done for every trusted sync (its own packet decoded, or did not
     fit), so every complete data field in the blob is read from the nearest
     sync, its own or the next one, and a blob one packet long yields a packet
     wherever the sync fell.
7. Results from different candidates are deduplicated by row (keeping the
   higher quality) and sorted. The budget is 24 detector runs per profile while
   the receiver has no confirmed clock, 6 once it has one; a candidate costs one
   run forward and one backward.

The receiver (`rs_rx`) confirms its clock after 3 concordant packets (EMA
0.1), passes it as `rows_per_chip_hint`, and drops it after 30 consecutive
empty frames so that a chip-length change on the board is picked up.

Cost: about 2 ms per 1080-row profile at 4 rows per chip and 4–8 ms per
3000-row RAW profile on an M-series Mac, 2–4× that on a phone.

Debugging: build the core with `-DRS_DEC_DEBUG` (`RS_CFLAGS=-DRS_DEC_DEBUG` for
the Python tools) to trace candidates and hypotheses on stderr and read the
timing counters (`rs_decode_debug_counters`); `RS_LIB=<cached dylib>` runs the
Python tools against an older core build for A/B comparisons.

### Grid decode

The transmitter sends packets back to back, so from one decoded packet the
others in the frame sit exactly `RS_PKT_CHIPS` apart. `rs_rx` therefore calls
`rs_decode_at` (no sync search: the detector at the given clock, the row given
is the gap chip) at up to 4 predicted positions each side of every decoded
packet, trying three sub-chip offsets (0, ±0.3 chip). A packet whose sync was destroyed by clipping or
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

## Recommended parameters

| Parameter | Value |
|---|---|
| T (`chip_us`) | 60 µs default (25 kbit/s gross per channel). 45 µs on an iPhone 14 (15 µs exposure); 90–120 µs with `repeat` 2–3 for a 57 µs-exposure Android phone (see CALIBRATION.md) |
| phone exposure | the shortest available; it must stay ≤ T, the detector tolerates up to ~2 T with a penalty |
| rows per chip | ≥ 1.2 for the detector; ≥ 3 for comfort |
| packet length | 82 chips = 3.3 ms at T = 60 µs |
| repetition | 1 when the blob is taller than a packet, 2–3 when it is not |
| channels | 3 (RGB), pilot every 30 ms (36 chips) |
| visible blink | 150 ms on / 50 ms off |
| FAULT weight | 3 in the death loops |
