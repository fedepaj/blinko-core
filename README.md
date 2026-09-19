# blinko-core

Portable C99 core of the Blinko optical log link (rolling-shutter camera
channel): protocol, transmitter with fountain coding and RGB channels,
decoder, message assembler, colour calibration, complete receiver, frame
profiling. No allocation, no libc beyond stdint. Used unchanged by the
Arduino library, the Zephyr module, the iOS app and the Android app.

- `rs_proto.h` packet format, CRCs, coding mask · `docs/PROTOCOL.md` spec
- `rs_tx.[ch]` message slots, carousel, chips, pilots, bursts
- `rs_decoder.[ch]` profile → packets · `rs_assembler.[ch]` packets → messages
- `rs_rgb.[ch]` pilot detection and unmixing · `rs_rx.[ch]` complete receiver
- `rs_multi.[ch]` segmentation and one receiver per light (several boards at once)
- `rs_frame.[ch]` luma/RGBA/YUV frame → per-row profiles

## Requirements

Any C99 compiler for the library itself. The Python tools need Python 3 with
numpy, scipy, pillow and matplotlib, which `make venv` installs into `.venv`.

## Build and test

```sh
make venv        # .venv with the tool dependencies
make test        # decoder robustness sweep against the simulator (quick)
make test-full   # the full sweep
```

There is no build step for the C code: add the `rs_*.c` files to your project
(the Python tools compile them on first use into `build/`).

## Minimal use

Transmitter — one chip per timer tick, one bit per two chips:

```c
rs_tx_t tx; rs_tx_init(&tx);
rs_tx_log(&tx, RS_LVL_INFO, "boot ok", 7);
/* in the timer ISR, every T_chip: */
uint8_t chips[RS_MAX_CHANNELS];
rs_tx_next_chips(&tx, chips);        /* chip 1 = LED on, one value per channel */
```

Receiver — the platform only supplies per-row profiles:

```c
rs_rx_t rx; rs_rx_init(&rx);
rs_frame_profile_rgb(px, w, h, row_stride, pixel_stride, 2, 1, 0, 0, r, g, b, &info);
rs_rx_process(&rx, r, g, b, info.count, t_seconds);   /* b == NULL: r is luma */
rs_message_t m;
while (rs_rx_pop_message(&rx, &m)) printf("[%u] %s\n", m.level, m.text);
```

## Tools (`tools/`)

| Script | Use |
|---|---|
| `test_core.py` | decoder robustness sweep on the simulator (`make test`) |
| `simulate.py`, `synth2d.py` | rolling-shutter models: chip stream → profile, and synthetic 2-D frames |
| `replay.py` | run a `.rsrec` recording through the C receiver exactly as the app does |
| `decode_image.py` | decode packets from a still photo or a video frame |
| `loss_budget.py` | where packets are lost: decoded / rescuable / damaged |
| `rscore.py`, `rsrec.py` | ctypes wrapper around the C core, `.rsrec` reader |
