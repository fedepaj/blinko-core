# rslog-core

Portable C99 core of the RSLog optical log link (rolling-shutter camera
channel): protocol, transmitter with fountain coding and RGB channels,
decoder, message assembler, colour calibration, complete receiver, frame
profiling. No allocation, no libc beyond stdint. Used unchanged by the
Arduino library, the Zephyr module, the iOS app and the Android app.

- `rs_proto.h` packet format, CRCs, coding mask · `docs/PROTOCOL.md` spec
- `rs_tx.[ch]` message slots, carousel, chips, pilots, bursts
- `rs_decoder.[ch]` profile → packets · `rs_assembler.[ch]` packets → messages
- `rs_rgb.[ch]` pilot detection and unmixing · `rs_rx.[ch]` complete receiver
- `rs_frame.[ch]` luma/RGBA/YUV frame → per-row profiles
- `tools/` Python simulator, tests, offline image decoding (`make venv && make test`)
