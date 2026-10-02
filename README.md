# blinko-core

Portable C99 core of the Blinko optical log link (rolling-shutter camera
channel): protocol, transmitter with fountain coding and RGB channels,
decoder, message assembler, colour calibration, complete receiver, frame
segmentation and profiling. No allocation, no libc beyond stdint/stddef. Used
unchanged by the Arduino library, the Zephyr module, the iOS app, the Android
app and the UNO Q kiosk.

| File | Contents |
|---|---|
| `rs_proto.h` | packet format, line code, CRCs, coding mask, packet encoder (header only) |
| `rs_pack.[ch]` | lossless 6-bit text packing of messages |
| `rs_tx.[ch]` | transmitter: message slots, carousel, repetition, pilots, bursts, chip source |
| `rs_decoder.[ch]` | one profile → packets (exposure-aware maximum-likelihood detector) |
| `rs_assembler.[ch]` | packets → messages (GF(2) elimination, message CRCs, poisoned-row recovery) |
| `rs_rgb.[ch]` | pilot detection, colour matrix, unmixing of the three streams |
| `rs_rx.[ch]` | complete receiver of one light: modes, clock hint, assembly, message queue |
| `rs_stitch.[ch]` | a packet read in pieces over several frames (experimental, off by default) |
| `rs_frame.[ch]` | frame → blobs (segmentation) and per-row profiles |
| `rs_multi.[ch]` | several lights in one frame: tracking, one receiver per light, cross-talk filter, grouping of the lights of one board |
| `docs/PROTOCOL.md` | the wire format and the receiver pipeline |

A transmitter needs `rs_proto.h`, `rs_tx.[ch]` and `rs_pack.[ch]`; a receiver
needs everything.

## Requirements

Any C99 compiler for the library itself. The Python tools need Python 3 with
numpy, pillow and matplotlib, a C compiler reachable as `cc`, and `lz4` to
read compressed recordings. `make venv` creates `.venv` with the first three;
`make setup` in the umbrella repository installs all of them.

## Build and test

```sh
make venv        # .venv with the tool dependencies
make test        # the test suite, quick sweep (tools/test_core.py --quick)
make test-full   # the same with the full sweep
```

There is no build step for the C code: add the `rs_*.c` files to your project.
The Python tools compile them on first use into `build/` (one shared library
per state of the sources) and load it with ctypes.

`tools/test_core.py` checks, on the rolling-shutter simulator:

- the encoder against a Python reference, and the 6-bit packing round trip;
- the transmitter: a change of the number of channels in mid packet stays in
  bounds, packets prepared ahead give the same chip stream as packets prepared
  at the boundary, a long text is split where the packing ends, an empty line
  takes no slot;
- the assembler: messages complete from packets in random order; with 2 % of
  the packets corrupted no wrong message is delivered and the leave-one-out
  recovery runs; a STATUS replaced while it is being received is delivered as
  one of the texts sent, never a mixture;
- the receiver on three streams with colour cross-talk (calibration from the
  pilots), and on a single-stream board;
- the decoder over a sweep of phone-like conditions (3–15 rows per chip,
  exposure 0.3–2.8 chips, noise, short blobs, repeated packets, a saturating
  LED). The sweep is seeded. It fails when the **yield** falls under a floor
  (870 packets quick, 5650 full: 95 % of the reference yield) or when the
  **false accepts** (packets that are not in the stream) exceed 4 per thousand
  packets.

## API in one page

### Receiver (`rs_multi.h`)

An application with camera frames uses `rs_multi`: it segments the frame,
keeps one receiver per light and delivers messages tagged with their source.

| Call | Use |
|---|---|
| `rs_multi_init(m)` | once. `rs_multi_t` is large (four receivers): keep it static or on the heap; `rs_multi_sizeof()` gives its size to a binding |
| `rs_multi_reset(m)` | an application's "clear": forgets every track and message, keeps the camera description, the parallel hook and the minimum contrast |
| `rs_multi_set_camera(m, cam)` | `rs_camera_t { exposure_rows, row_seconds }`: the exposure in rows of the frames given (`exposure_us / row_us`; 0 = unknown, half a chip is assumed) and the time of one such row in seconds (0 = unknown). Call it whenever either changes |
| `rs_multi_set_parallel(m, fn, user)` | optional. `fn(user, count, job, ctx)` runs `job(ctx, i)` for `i = 0..count-1` on separate threads and returns when all are done: a light's three channels are then decoded concurrently. Build the core with `RS_DEC_THREADS` (thread-local decoder scratch) |
| `rs_multi_set_min_contrast(m, c)` | the decoder's minimum local contrast, in profile units (8-bit counts); 0 = the default (6) |
| `rs_multi_process(m, px, w, h, row_stride, pixel_stride, r_off, g_off, b_off, t)` | one frame: interleaved 8-bit RGB(A), `w × h` pixels of `pixel_stride` bytes, rows `row_stride` bytes apart, the R, G, B bytes at the given offsets within a pixel. **Rows must run along the rolling-shutter scan** (a row is an instant). `t`: frame time in seconds, a small `float` (count from the first frame). Returns the distinct packets decoded in this frame |
| `rs_multi_pop_message(m, &msg, &source)` | call until it returns 0 after every frame. `rs_message_t { id, level, len, text }`: slot, level, text length, NUL-terminated text (up to 63 characters). `source` is the logical source: the lights of one board share one |
| `rs_multi_track_count(m)`, `rs_multi_track(m, i)`, `rs_multi_track_info(m, i, …)`, `rs_multi_track_group(m, i)`, `rs_multi_track_rx(m, i)` | the lights being tracked, for a user interface: position and radius in pixels, decoding mode (`RS_RX_MODE_*`), packets, messages, pilots, logical source, and the track's receiver for the `rs_rx_*` accessors |

Context: every call from the thread that processes the frames, one frame at a
time **per process** (the receiver and the profile extraction keep their
scratch in static buffers). Limits: 4 lights, 6 blobs per frame, 4096 rows per
frame (a taller frame is read down to there).

A platform that already has per-row profiles of one light can use `rs_rx`
directly (`rs_rx_init`, `rs_rx_set_camera`, `rs_rx_process`,
`rs_rx_pop_message`), and `rs_decode_profile` / `rs_asm_feed` are the two
halves below it.

```c
static rs_multi_t m;
rs_multi_init(&m);
rs_camera_t cam = { exposure_us / row_us, row_us * 1e-6f };
rs_multi_set_camera(&m, cam);
/* per frame (BGRA here): */
rs_multi_process(&m, px, w, h, row_stride, 4, 2, 1, 0, t_seconds);
rs_message_t msg; int source;
while (rs_multi_pop_message(&m, &msg, &source)) printf("#%d [%u] %s\n", source, msg.level, msg.text);
```

### Transmitter (`rs_tx.h`)

The transmitter is a chip source for a periodic timer. **Units**: the timer
period is one chip = T / 3 (`RS_CELLS_PER_T`), where T is the value a board is
configured with; bursts and the pilot period are given in chips.

| Call | Use | Context |
|---|---|---|
| `rs_tx_init(tx)` | once, before the timer starts | — |
| `rs_tx_slot_prepare(&slot, level, text, len)` | step 1 of a new message: packs the start of `text` into a slot of the caller's and returns how many characters it took (up to 41 in the 31 bytes); call again with the rest for a longer text | anywhere, chip interrupt running |
| `rs_tx_log_slot(tx, &slot)` | step 2: a copy into the next rotating log slot (0..5); returns its id | chip interrupt masked |
| `rs_tx_put_slot(tx, id, &slot)` | step 2 for a fixed slot: `RS_SLOT_STATUS` (6), `RS_SLOT_FAULT` (7); an invalid slot clears it | chip interrupt masked |
| `rs_tx_clear_slot(tx, id)` | remove a message | chip interrupt masked |
| `rs_tx_log(tx, level, text, len)`, `rs_tx_set_slot(tx, id, level, text, len)` | both steps in one call (text beyond one message is dropped) | callers without an interrupt, tests |
| `rs_tx_set_burst(tx, on_chips, off_chips)` | visible blink: transmit for `on_chips`, dark for `off_chips`; 0 = continuous | chip interrupt masked |
| `rs_tx_set_channels(tx, nchan, pilot_period)` | 1 or 3 streams; mean chips between pilot blocks in 3-channel mode. Every channel restarts at a packet boundary | chip interrupt masked |
| `rs_tx_set_repeat(tx, n)` | copies of every packet, 1..100 | chip interrupt masked |
| `rs_tx_set_fault_weight(tx, w)` | FAULT visits per other visit, 1..4 | chip interrupt masked |
| `rs_tx_next_chips(tx, out)` | the chips of the three channels (`out[c]` = 1: LED on) for the next chip period; in 1-channel mode all three are equal | the timer interrupt, once per chip |
| `rs_tx_wants_prepare(tx)`, `rs_tx_prepare(tx)` | choose and encode the packets that follow the ones on air (thousands of instructions, several chip periods) | a context the chip interrupt can preempt: a lower-priority interrupt, or the bare loop of a fault handler |

The transmitter has no lock: "masked" means the caller masks the chip
interrupt, and whatever runs `rs_tx_prepare`, around the call. A port that
never calls `rs_tx_prepare` still works: the chip path then prepares at the
packet boundary itself and holds the LED for as long as that takes.

```c
static rs_tx_t tx;
rs_tx_init(&tx);
rs_slot_t s;
rs_tx_slot_prepare(&s, RS_LVL_INFO, "boot ok", 7);
mask_chip_irq(); rs_tx_log_slot(&tx, &s); unmask_chip_irq();

/* timer interrupt, every chip (T / 3): write the pins first, then compute the next chips */
static uint8_t next[RS_MAX_CHANNELS];
write_leds(next);
rs_tx_next_chips(&tx, next);
if (rs_tx_wants_prepare(&tx)) pend_low_priority_irq();   /* which calls rs_tx_prepare(&tx) */
```

## Tools (`tools/`)

Run them with the venv's interpreter, from anywhere:
`.venv/bin/python tools/replay.py …`.

| Script | Use |
|---|---|
| `test_core.py` | the test suite (`make test`, `make test-full`), see above |
| `replay.py` | run `.rsrec` recordings through the C receiver and print per-recording metrics (frames, fps, packets/s, messages, RGB lock, pilots). `--multi` uses the multi-source path the apps use; `--row-us US` gives the receiver the row time of the recorded rows, and with it the exposure (without it the exposure is unknown to the detector); `--json out.json` saves the metrics; `--png N out.png` extracts frame N |
| `decode_image.py` | decode packets and messages from a still photo or a video frame; `--plot out.png` draws the profile with the packets found |
| `loss_budget.py` | where packets go, against ground truth on the simulator: decoded, rescuable (the packet decodes at its true position) or lost |
| `simulate.py` | rolling-shutter camera model: chip stream → per-row profile (exposure box filter, blob, noise, saturation) |
| `synth2d.py` | synthetic 2-D frames: one or more defocused LED blobs with colour mixing, motion and saturation |
| `codelab.py` | lab script: line codes (Manchester, Miller, NRZ, 4B6B, 8B10B, PPM, RLL(2,7)) compared on the simulator under one detector |
| `v3lab.py` | lab script: a Python detector with a saturation model on recorded profiles, for trying detector ideas before they go into `rs_decoder.c` |
| `rscore.py` | ctypes wrapper around the C core (builds it on first use); the other tools import it |
| `rsrec.py` | `.rsrec` reader |

Two environment variables apply to every tool: `RS_CFLAGS` adds compiler
flags to the build (`RS_CFLAGS=-DRS_DEC_DEBUG` traces the detector on stderr),
and `RS_LIB=<path to a library built earlier>` loads that build instead of the
current sources, for A/B comparisons.
