# Blinko — Optical protocol

The wire format and the receiver pipeline, in enough detail to write another
implementation. The protocol version is `RS_PROTO_VERSION` = 3 (`rs_proto.h`).
Design rationale lives in the source comments (`rs_proto.h`, `rs_decoder.c`);
measurements are in `docs/FINDINGS.md` of the umbrella repository.

## Units

| Name | Meaning |
|---|---|
| **chip** | one code cell: the transmitter's timer period, the smallest time step of the signal |
| **T** | 3 chips (`RS_CELLS_PER_T`), the shortest run the line code produces. A board is configured with T: `chip_us` in the firmware APIs **is T**, not the chip. Default T = 60 µs, so the chip is 20 µs |
| **row** | one sample of the profile the receiver is given; consecutive rows are `t_row` apart in time (the sensor's row time, or a multiple of it when the platform reduces the image) |
| **rows per chip** | chip / `t_row` = `T / (3 · t_row)`: the clock the decoder measures (`rows_per_chip`) |
| **exposure in rows** | camera exposure / `t_row` (`rs_camera_t.exposure_rows`): the length of the box filter every row applies to the signal |
| **packet height** | 82 chips = `82 · T / (3 · t_row)` rows |

Times given to the receiver are seconds as a `float`, counted from an origin
of the caller's choice that keeps them small (the first frame, not the uptime).

## Physical layer

- **Modulation**: OOK (LED on/off). T is what the camera exposure must stay
  below. The receiver recovers the chip clock, in rows, from the sync of every
  packet.
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

Every packet has the same length: **82 chips** (`RS_PKT_CHIPS`), i.e.
`82 · T / 3`: 1.64 ms at T = 60 µs, 1.23 ms at T = 45 µs, 3.28 ms at
T = 120 µs.

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
  - `seed < len`: systematic packet, payload = message byte `seed`;
  - `len ≤ seed ≤ 123`: **coded** packet, payload = XOR of the bytes selected
    by `rs_code_mask(seed, len)` (xorshift32, identical on every platform);
  - 124 = CRC-8/MAXIM and 125 = CRC-8/ATM of the whole message (the two
    message CRCs, 16 guard bits together);
  - 126 = META: payload `[len:5][level:3]`, the message bytes are 6-bit packed
    text;
  - 127 = META: same payload, the message bytes are stored raw.
- **CRC-12** (poly 0x80F, init 0xFFF) over the 18 id+seed+payload bits. The
  non-zero init means an all-dark or all-bright stretch never decodes to a
  valid packet.
- **Repetition** (`rs_tx_set_repeat`, 1..100, `RS_TX_MAX_REPEAT`): every
  packet can be sent n times back to back. 2–3 copies are for a camera whose
  window is about one packet long: it reads one packet across two copies (see
  cyclic decoding below). Default 1.

Levels: 0 DEBUG, 1 INFO, 2 WARN, 3 ERROR, 4 FATAL, 5 STATUS, 6 FAULT.

## Messages and fountain coding

A message is at most **31 bytes** (`RS_MSG_MAX_LEN`). Every data packet is a
row of a linear system over GF(2) whose unknowns are the message bytes: the
receiver runs incremental Gaussian elimination and can solve the message as
soon as the rank reaches `len`, from any subset of packets, a few more than
`len` on average. The two message CRCs guard the solution (see Assembly).
Packets received before the META are queued (24) and replayed.

### 6-bit text (`rs_pack.h`)

Symbols: space, a–z, 0–9, SHIFT (next letter uppercase), 25 punctuation marks,
ESC (raw 8-bit byte). Lossless. `rs_tx_slot_prepare` packs a text when that
carries more of it than the raw bytes would, or the same text in fewer bytes;
31 bytes hold up to **41 characters** of ordinary log text. A longer text is
split where the packing ends: the call returns how many characters it took and
is called again with the rest, each piece becoming a message of its own.

## Carousel and visible blink

One visit of a message: `[META CRC CRC2] data… [META CRC CRC2] data…` for a
total of `len + 6` packets; the second control triplet sits after `len/2` data
packets, so any camera window of about half a visit contains one. The data
seeds advance at every visit (first pass systematic, then coded, wrapping from
123 back to `len`). Slot order per round: FAULT, STATUS, then the log slots
newest first. A transmitter with no message sends an idle packet (slot 6,
META with `len` 0). The transmitter alternates bursts of `burst_on` and pauses
of `burst_off` aligned to packet boundaries (150/50 ms in the firmware
defaults): the LED blinks visibly at 5 Hz and the receiver loses 25 % of the
time.

### Weighted FAULT carousel

`rs_tx_set_fault_weight(tx, w)` (1..4) inserts `w−1` extra FAULT visits after
every other visit in the round: for `w = 3` the round is
`[F S F F L1 F F L2 F F …]`. The death loops use `w = 3`, which gives the
FAULT slot about three quarters of the packets (74–79 % with a STATUS and
0–6 log lines), so the fault reason completes in one or two seconds while the
last log lines still follow it.

## RGB channels

With an RGB LED the transmitter sends **three independent streams** (R, G, B),
taking packets from the carousel in turn: three packets per packet interval,
synchronised (their syncs coincide). In 1-channel mode every LED carries the
same stream. On average every `pilot_period` chips (30 ms in the firmware
defaults, 1500 chips at T = 60 µs) a **pilot block** of 9·P chips is emitted
at a packet boundary (P = `RS_PILOT_P` = 4, so 36 chips, 2.4 % of the time at
T = 60 µs): `[dark 2P][R P][dark P][G P][dark P][B P][dark 2P]`. The interval
between blocks is not constant: it is the period scaled by 0.5–1.5 along a
golden-ratio sequence, so the block's position within the camera frame changes
from one block to the next and a phone whose frame period is close to the
pilot cadence still sees a whole block inside the LED blob every few frames.

The receiver looks for that block in the luma profile (`r+g+b`, which must
span at least 30 counts), binarized at 15 % of its range with runs shorter
than 4 rows absorbed by their neighbours (a RAW Bayer profile toggles for a
few rows at every threshold crossing). A block is accepted when:

- there are three pulses whose widths are within 40 % of the first one's (at
  one threshold a dimmer die reads a narrower pulse);
- the two gaps are equal within the larger of 35 % of the gap and 10 % of a
  pulse width, and each is between 0.12 and 1.6 pulse widths (the exposure
  smear widens the pulses and narrows the gaps alike);
- a dark zone of at least half a gap (and at least 2 rows) lies on both sides;
- the colour of each pulse, taken from its brightest rows (luma within 25 % of
  the pulse's peak) minus the dark baseline, gives a usable 3×3 matrix. The
  baseline is the darker of the two outer dark zones: the gaps between the
  pulses do not reach the floor at a long exposure. The matrix columns are
  normalised to sum 1 and it must be well conditioned (`cond ≥ 0.35`).

An accepted matrix is smoothed into the stored one (EMA, 0.5) and inverted.
The calibration is used while a pilot has been seen in the last 10 s
(`RS_RX_CAL_TTL`); without one, or with a degenerate matrix, the receiver
falls back to the other modes below, so a board with a single LED works with
the same app.

## Reception

### Per-frame pipeline

For each camera frame (`rs_multi_process`):

1. **Segmentation** (`rs_frame_segment_rgb`): on a thumbnail of one cell per
   8×8 pixels (coarser for a frame that would not fit 256×144 cells), a
   vertical running maximum of ±112 pixels bridges the dark stripes the
   modulation carves into the blob (sync gaps, pilot blocks) so one light
   stays one component; threshold at 40 % of the thumbnail range, flood fill,
   reject specks, regions over half the frame, blobs under three cells wide
   or tall and dim blobs (peak under 48 counts, or under 40 above the
   background; a frame whose range is under 20 counts gives nothing). Up to 6
   blobs, brightest first.
2. **Tracking** (`rs_multi.c`): up to 4 tracks, each holding a complete
   receiver. A track takes every blob overlapping its box (grown by 24 px), so
   fragments of one source merge; it is dropped after `RS_TRACK_TTL` = 1 s
   without a detection, and reported once it has decoded something or has been
   seen for 12 frames.
3. **Profile per light** (`rs_frame_profile_rgb_blob2`): three per-row
   profiles (R, G, B), at most 4096 rows (`RS_DEC_MAX_ROWS`; a taller frame is
   read down to there), over the track's box extended by one blob height above
   and below and half a blob width each side (the halo keeps the short runs a
   clipped core loses), clipped to the frame and to the midpoint towards any
   neighbouring track. Each column is weighted by its **mean squared row-to-row
   difference** of luma — a high-pass matched to the stripes, so columns
   carrying the modulation dominate while a clipped core (flat at 255), the
   dark surround and noise-only columns (< 5 % of the peak weight) get nothing.
   Steps into or out of clipping are excluded from the weight. The weighted
   mean uses about 128 columns.
   Two variants exist — clipped columns kept with matched weights, or dropped
   outright (columns clipping in two or more sampled rows, as long as 8
   unclipped ones remain). They differ only when something in the blob clips;
   then the track compares them every 8 frames, or after an empty frame (at
   most every 3), and keeps the one that yields more packets on that frame.
4. **Decode** (`rs_rx_process`) on one or three profiles, see below. The
   three channels of a light are independent, so a platform can hand the
   receiver a parallel-for hook (`rs_multi_set_parallel`) and have them
   decoded on three threads; the core is then built with `RS_DEC_THREADS` so
   that the decoder's scratch is thread-local. The clock hint is the one at
   the start of the frame for every channel.
5. **Assemble**: in multi-source mode assembly is deferred
   (`defer_assembly`), the cross-talk filter runs first, then
   `rs_rx_assemble` feeds the surviving packets to `rs_asm_feed`.

### The three decoding modes of `rs_rx`

| mode | when | what is decoded |
|---|---|---|
| `RS_RX_MODE_LUMA` (0) | single-channel input, or no pilot lock and the light is not three-coloured | `(r+g+b)/3` |
| `RS_RX_MODE_RGB` (1) | a valid calibration and a pilot seen in the last 10 s | the three unmixed channels `inv · (r,g,b)` |
| `RS_RX_MODE_DIRECT` (2) | all three camera channels modulated (`rs_rx_three_coloured`) but no pilot lock | the camera's own R, G and B as the three streams |

The direct mode exists because a 3-die RGB LED seen defocused is three colour
discs offset by about 40 % of their diameter: the pilot pattern may never
appear in the combined profile, yet each camera channel already sees mostly
one die. Cross-talk between LED primaries is moderate and the packet CRC
rejects what it corrupts. `rs_rx_three_coloured` requires a range of at least
30 counts on the strongest channel and every channel range at least 30 % of
it.

The same (id, seed, payload) is counted once per frame: a board that repeats
its packets, and the three channels of an RGB LED in 1-channel mode, show the
same packet several times in a frame, and a copy carries no information.
Packet counts and rates are therefore of **distinct packets** (up to 96 per
light and frame).

### Packet decoding (`rs_decode_profile`)

The decoder is an exposure-aware maximum-likelihood detector: it does not
threshold the profile into chips, it fits chip templates that include the
camera's exposure smear to the analogue profile. One row integrates the LED
over the exposure window E, so a step becomes a ramp E rows long and a 3-chip
run at E = T has only 64 % of its amplitude; the templates reproduce that, so
the detector keeps working where the short runs no longer reach full
amplitude, up to an exposure of 3 chips (E = T).
The application passes the exposure in rows and the row time
(`rs_multi_set_camera`); half a chip is assumed when the exposure is unknown.

Per profile:

1. **Candidates, at every plausible scale** (2, 3.5, 6, 10, 17, 28 rows per
   chip; with a confirmed clock only the scales that cover it; never a scale
   at which a packet could not fit the profile). The profile is
   envelope-normalized for the scale (sliding min/max over 15 chips: the blob
   changes brightness several-fold along a packet) and correlated with the
   exposure-smeared sync template at 0.7×..1.3× of the scale (around a
   confirmed clock: at it and ±3 %). Peaks of at least 0.6 are candidates, up
   to 6 per scale, ranked globally across scales, best correlation first.
2. **Validation**, without spending a detector run. The ON run is measured
   between its two 0.5-crossings: it must be 10 chips ± 15 % and both edges
   must exist (the blob's own bright edge, which the correlation likes because
   the gap before it is dark too, fails this), and **its length is the clock
   estimate** (two edges 10 chips apart give about 1 %; a correlation grid
   cannot). The 5 chips before the gap must contain a bright chip (a real gap
   follows modulated signal). Candidates whose exposure exceeds 3 chips are
   dropped: the templates are flat and fit anything. The runs after the sync
   must be plausible data runs (3..8 chips at this clock).
3. **Detection**: a Viterbi search over the 12 states of the RLL(2,7) encoder
   (tree node × level). For each survivor and input bit the codeword's chip
   template is built at the survivor's position and clock, with the exposure
   smear and the survivor's last 8 chips as history, and compared to the
   profile at ±1 row of **timing slip**; the slip also corrects the survivor's
   own clock (a first-order PLL, gain 0.3·slip/chips, clamped ±6 %), so a
   clock hypothesis a few percent off converges instead of drifting out of the
   slip range. Metric: squared error per row. At ≥ 6 rows per chip the
   template is evaluated once per `rpc/4` rows against the profile's mean over
   those rows.
   **Early aborts**: at three points early in the packet a run is dropped
   when the best survivor's error is above `RS_ABORT_MSE` = 0.15. This is a
   cost limit, independent of `min_quality`: the first codewords fit worse
   than the rest while the clock PLL converges.
4. **Clock hypotheses**, tried until one passes the CRC: the receiver's
   confirmed clock, the sync's, ±2.5 % (also ±5 % with E > 2 chips).
5. **Exposure hypotheses**: the configured exposure first; only when it gives
   no valid CRC, the first clock hypothesis is also tried at 1.4× and 2× of
   it (phones report less exposure than their edges show).
6. **Checks**: CRC-12; a **rigid-grid re-fit** of the decoded chips with one
   clock = (end − start)/chips and a global phase of ±2 rows must not be more
   than 3× worse than the slipped fit (the slips can otherwise fit the stream
   at 2/3 of its clock, and such an alias can pass the CRC); and **quality** =
   1 / (1 + 30 · mse per row) ≥ `min_quality` (0.4).
7. **Framing beyond the sync.** The usable end of the signal is where the
   local amplitude falls below 60 % of the sync's (the blob's soft edge: the
   envelope normalization lags a fade shorter than its window, so rows with
   some contrast left are still garbage). When the packet after the sync runs
   past it:
   - **cyclic decode**: the tail is read one packet period earlier, before
     this sync, where the previous copy of a repeated packet lies. It is tried
     only where the signal one period earlier matches the rows after the sync
     (`repeats_one_period_earlier`: the receiver is not told whether the board
     repeats, it looks), or where too few rows overlap to tell. The seam sits
     half a normalization window before the fade, the wrapped rows must carry
     signal, and period offsets 0, ±2, ±4 rows are tried (the period is only
     known to the clock's precision), stopping at a valid CRC;
   - **backward decode**: the packet *before* the sync is decoded — its data
     field ends where this gap begins and takes clock and phase from this
     sync. This is done for every trusted sync (its own packet decoded, or did
     not fit), so every complete data field in the blob is read from the
     nearest sync, its own or the next one, and a blob one packet long yields
     a packet wherever the sync fell.
8. Results from different candidates are deduplicated by row (keeping the
   higher quality) and sorted. The budget is **24 validated candidates** per
   profile while the receiver has no confirmed clock, **6** once it has one; a
   candidate costs several detector runs (clock and exposure hypotheses, the
   cyclic offsets, the packet before the sync).

The receiver (`rs_rx`) confirms its clock after 3 concordant packets (within
15 %, EMA 0.1), passes it as `rows_per_chip_hint`, and releases it after 30
consecutive frames without a packet so that a change of T on the board is
picked up.

Debugging: build the core with `-DRS_DEC_DEBUG` (`RS_CFLAGS=-DRS_DEC_DEBUG` for
the Python tools) to trace candidates and hypotheses on stderr and read the
timing counters (`rs_decode_debug_counters`); `RS_LIB=<cached library>` runs
the Python tools against another core build for A/B comparisons.

### Stitching across frames (`rs_stitch.c`, experimental)

Stitching reads a packet in pieces over several frames, for a lit blob shorter
than a packet (a far or small light). It is **off by default**: a receiver
uses it only with `rs_rx_t.stitch_enabled` set, and no application sets it. It
needs:

- a board that repeats each packet for tens of copies (`rs_tx_set_repeat`), so
  that successive frames show successive pieces of the same 82-chip cycle, at
  phases that advance by the frame period modulo the packet period;
- the sensor's row time (`rs_camera_t.row_seconds`) and frame times precise to
  a fraction of a millisecond;
- frames that show a whole sync now and then, to anchor the phase.

Per channel, when a frame yielded no packet, the receiver:

1. takes the piece: the lit blob's rows (the longest stretch above a quarter
   of the raw profile's range, trimmed by a chip), normalized as the detector
   sees them;
2. anchors it when the frame shows a sync well inside the blob: the gap is
   chip 0 of the cycle. Only syncs at the stream's clock count (the median of
   the sync clocks seen; a sync cut by the blob's edge or a data run posing as
   one measures another clock);
3. learns the **chip period in seconds** from the anchored frames: their
   phases must all equal `phase0 + t/period` modulo the cycle. A frame period
   is hundreds of chips or more, so a wrong period scatters the phases; the
   period is found
   by a search of ±12 % around the row-time guess, outward from the guess
   because periods one cycle-per-frame apart fit the anchors equally, and is
   refined within ±2 % afterwards. Three agreeing anchors lock it;
4. predicts the phase of frames without a sync from the last anchor and the
   period, refined by correlating the piece against the cycle (±4 chips);
   a sync that disagrees with a lock by more than 3 chips is ignored (three
   in a row drop the lock);
5. keeps the last 12 pieces and composes the cycle from the newest ones that
   agree where they overlap (packets of one slot share their sync and their
   first data chips, so an older piece from the previous packet can agree on
   the prefix and still poison the rest; pieces from another packet are left
   out);
6. when every chip of the cycle is covered, lays the cycle out twice as a
   profile at 4 rows per chip and hands it to the ordinary detector; a decoded
   packet goes to the assembler like any other (`rs_rx_stitched` counts them).

### Assembly (`rs_assembler.c`)

Per slot the assembler keeps the linear system (pivots and values), the two
message CRCs, the pre-META queue (24) and a ring of the last 56 **raw** rows as
received.

- **Delivery**: when the rank reaches `len` the system is solved, and the
  message is delivered when **both** message CRCs have arrived and match. A
  slot whose text the board replaces while it is being received holds rows of
  two texts and solves to garbage, which one CRC-8 lets through once in 256;
  two make that 2⁻¹⁶. The FAULT slot is the exception (`crcs_needed`): **one**
  message CRC is enough there, because its text does not change and a pulsed
  fault LED shows few control packets in a short window.
- **Leave-one-out recovery**: if a solved system fails its CRCs (with both
  known), a row that passed its packet CRC by chance poisoned it. The
  assembler re-solves from the raw rows leaving one out at a time (≤ 56 small
  GF(2) eliminations). A solution is accepted only if **all the other rows
  agree with each other** and it satisfies both CRCs: the poisoned row is then
  dropped, the pivots are rebuilt without it and the message is delivered
  (counted in `rs_asm_recovered`). The agreement is what tells one poisoned
  row from a slot holding rows of two texts, which contradict each other
  whichever one is left out. If no candidate works, the slot is reset.
- **Slot replacement**: a META or message-CRC packet disagreeing with the
  stored one is ignored once (it may itself be corrupt) and resets the slot at
  the second consecutive disagreement; six rows inconsistent with the system
  also reset it, and so does a META with `len` 0.
- **Once per text**: a delivered slot stays delivered until it is reset, and
  only a different META, message CRC or contradicting rows reset it. The
  packets carry no message generation number, so a text identical to the one
  already delivered in a slot is not delivered again.

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
packets of the same slot seen within 20 ms of each other by tracks A and B,

```
seed_b − seed_a ∈ { n·dp + dch + s }   n ∈ {1, 3}, s ∈ {0, ±3, ±6}, ±1 tolerance
```

where `dp` is their distance in packets and `dch` their channel difference.
Inside a frame `dp` is the row distance over the packet height; between
packets of different frames the time between the frames is added, converted
with the sensor's row time (a chip lasts rows-per-chip × `t_row`). Without the
row time (`rs_camera_t.row_seconds` = 0) only packets of the same frame are
compared. Rows are time, so the relation holds whether the two blobs overlap
or sit far apart. Matches add 1 to a decayed pair score, mismatches subtract
0.3; above 2.0 the two tracks share a group (the smaller track id), below 0.5
they part again. Control packets (seed ≥ 124) are excluded, and a random pair
of boards matches only about 6 % of the time, so its score drifts negative.
Messages are reported under the group id, so one board is one source in the
UI.

`rs_multi_reset` forgets every track and message and keeps what the
application configured: the camera description, the parallel hook and the
minimum contrast (`rs_multi_set_min_contrast`).

## Fault record (payload of slot 7)

Text generated by the firmware, at most 31 characters:

| Text | Source |
|---|---|
| `HF p=XXXXXXXX l=XXXXXXXX c=XXXX` | hard fault on the Arduino library: stacked program counter, link register, low 16 bits of the fault status register (hex) |
| `ZF<reason> p=XXXXXXXX l=XXXXXXXX` | fatal error on Zephyr: the kernel's reason code, program counter, link register |
| `WDT reset @<checkpoint>` | watchdog reset, with the last checkpoint the firmware named (`?` if none) |
| `F<code>:<text>` | `fatal(code, text)` called by the application |

The reset cause is part of the STATUS a board announces at boot
(`boot#<n> rst=<cause> id=<board id>` on the Arduino library).

## Limits

- **Message length**: 31 bytes, up to 41 characters with the 6-bit packing. A
  longer text is several messages.
- **Identical text**: a text identical to the one already delivered in a slot
  is not delivered again (there is no message generation number on the wire).
  Identical log lines in a row go to different slots and are each delivered;
  the seventh lands in the slot of the first and is not. A STATUS set again
  to the same text is one message.
- **Replaced slots**: a slot whose text is replaced faster than a receiver
  collects it is never seen. The receiver delivers one of the texts or none,
  never a mixture.
- **Sources**: 4 tracks, 6 blobs per frame, 4096 rows per profile.
- **One receiver per process at a time**: the receiver and the profile
  extraction keep their scratch in static buffers (the decoder's alone is per
  thread with `RS_DEC_THREADS`), so two receivers must not process frames on
  two threads at once.
- **Time** is a `float` in seconds: keep it small (count from the first
  frame), the receiver needs it to a fraction of a millisecond.
- **Queues**: 24 packets before a slot's META, 16 undelivered messages per
  light, 96 distinct packets per light and frame.

## Recommended parameters

| Parameter | Value |
|---|---|
| T (`chip_us`) | 60 µs default (25 kbit/s gross per channel). 45 µs is the shortest the Arduino library runs; 90–120 µs with `repeat` 2–3 for a phone whose shortest exposure is 57 µs (see CALIBRATION.md in the umbrella repository) |
| phone exposure | the shortest available; it must stay below T: the detector drops candidates whose exposure, as the camera reports it, exceeds 3 chips |
| rows per chip | ≥ 1.2 for the detector; ≥ 3 for comfort |
| packet length | 82 chips = 1.64 ms at T = 60 µs |
| repetition | 1 when the blob is taller than a packet, 2–3 when it is about one packet |
| channels | 3 (RGB), pilot every 30 ms on average (36 chips) |
| visible blink | 150 ms on / 50 ms off |
| FAULT weight | 3 in the death loops |
| death loop timing | T = 120 µs, 3 copies, one stream on the fault LED, 150/50 ms bursts, full brightness, independent of the running configuration |
