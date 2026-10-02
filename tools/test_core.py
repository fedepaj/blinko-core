"""Robustness sweep of the decoder against the rolling-shutter simulator.

Usage: .venv/bin/python tools/test_core.py [--quick]
"""
import sys, numpy as np
import ctypes
from rscore import Transmitter, Decoder, Assembler, Receiver, Packet, encode_packet, pack6, unpack6, RS_PKT_CHIPS, LEVELS, MODES, _lib
from simulate import rolling_shutter_profile, matches_truth


FLOOR_QUICK, FLOOR_FULL = 870, 5650     # packets the sweep must yield (--quick, full): 95 % of 920 and 5950


def make_stream():
    tx = Transmitter()
    tx.set_slot(6, 5, "up=12s rst=POR fw=1.0")
    tx.log(1, "boot ok")
    tx.log(2, "sensor timeout, retrying")
    tx.log(3, "I2C NACK @0x68")
    tx.set_slot(7, 6, "HF pc=0x0000a3f2 cfsr=0x400")
    chips, pk = tx.stream(200)
    return tx, chips, pk


def sanity():
    # encoder cross-check: C vs python
    for t in ["up=415s rst=POR n=82", "HF p=00004356 l=000049db c=8200", "sensor timeout, retrying", "I2C NACK @0x68", "Ünïcödé ok?", "x"]:
        pk = pack6(t)
        if pk:
            assert unpack6(pk) == t, (t, unpack6(pk))
        print(f"   pack6 {t!r}: {len(t)} -> {len(pk) if pk else 'raw'} bytes")
    tx = Transmitter(); tx.log(1, "abc")
    for _ in range(10):
        c = encode_packet(*tx.next_packet())
        assert len(c) == RS_PKT_CHIPS
    # a full cycle of chips from the C chip source must equal the encoder output
    tx = Transmitter(); tx.log(1, "xyz")
    tx2 = Transmitter(); tx2.log(1, "xyz")
    chips_c = [tx.next_chip() for _ in range(RS_PKT_CHIPS * 6)]
    chips_p = []
    for _ in range(6):
        chips_p += encode_packet(*tx2.next_packet())
    assert chips_c == chips_p, "C chip source differs from python encoder"
    print("sanity: encoder ok")


def sweep(quick=False):
    tx, chips, packets = make_stream()
    rows = 1080
    dec = Decoder()
    rng = np.random.default_rng(42)
    print(f"{'rpc':>5} {'exp':>5} {'noise':>5} {'blob':>5} {'rows':>5} | {'frames':>6} {'pkts':>5} {'wrong':>5} {'pkt/frame':>9} {'crcfail':>7}")
    worst = 0
    # (rows per cell, exposure in cells, noise, blob fraction, profile rows): phone-like conditions
    # 1080p phones (iPhone 14: 5.1 us rows, 15 us exposure -> 0.75 cells at T=60) and a RAW
    # 4000x3000 Android sensor (S21 FE: 2.65 us rows, 57 us exposure -> 1.3..2 cells at T=120..90)
    cases = []
    for rpc in ([4, 5, 6] if quick else [3, 4, 5, 6]):    # 5 stays in the quick list: the false accepts this sweep once had were all there
        for exp_frac in ([0.6, 1.0] if quick else [0.3, 0.6, 1.0]):
            for noise in [1.0, 3.0]:
                cases.append((rpc, exp_frac, noise, 0.6, 1080))
    for rpc in ([11, 15] if quick else [8, 11, 15]):
        for exp_frac in ([1.4, 1.9] if quick else [1.0, 1.4, 1.9]):
            cases.append((rpc, exp_frac, 2.0, 0.55, 3000))
    cases += [(3.7, 1.9, 2.0, 0.3, 1080), (3.7, 2.8, 2.0, 0.3, 1080)]            # S21 FE 1080p YUV: short blob, long exposure
    cases += [(8, 0.5, 2.0, 0.15, 1080), (8, 0.5, 2.0, 0.3, 1080), (8, 0.5, 6.0, 0.6, 1080), (4, 0.5, 4.0, 0.6, 1080)]
    rep_cases = [(15, 1.4, 2.0, 0.55, 3000), (11, 1.9, 2.0, 0.55, 3000), (3.7, 1.9, 2.0, 0.3, 1080), (6, 0.6, 2.0, 0.6, 1080)]   # packets repeated twice
    sat_cases = [(6, 0.5, 2.0, 0.6, 900.0), (6, 1.0, 2.0, 0.6, 2500.0), (4, 0.8, 2.0, 0.6, 1200.0)]   # saturating LED (amplitude >> 255)
    total_wrong = total_good = 0
    for rpc, exp_frac, noise, blob_frac, rows in cases:
        n_frames = 40 if quick else 120
        good = wrong = crcfail = 0
        dec.cfg.exposure_rows = exp_frac * rpc                 # the apps pass the camera's exposure
        for _ in range(n_frames):
            off = rng.uniform(0, len(chips))               # anywhere in the stream: every frame other packets
            b0 = rng.uniform(0.05, 0.95 - blob_frac)
            p = rolling_shutter_profile(chips, rows, rpc, exp_frac, off, blob=(b0, b0 + blob_frac),
                                        noise=noise, rng=rng)
            pkts = dec.decode(p)
            crcfail += dec.stats.crc_fail
            for pk in pkts:
                if matches_truth(pk, rpc, off, packets): good += 1
                else: wrong += 1
        total_wrong += wrong; total_good += good
        print(f"{rpc:5} {exp_frac:5.2f} {noise:5.1f} {blob_frac:5.2f} {rows:5} | {n_frames:6} {good:5} {wrong:5} {good/n_frames:9.2f} {crcfail:7}")
    # the firmware replays the same chips `repeat` times (rs_tx_set_repeat): same stream, each packet twice
    packets2 = [t for t in packets for _ in range(2)]
    chips2 = np.concatenate([np.array(encode_packet(*t), dtype=np.float32) for t in packets2])
    for rpc, exp_frac, noise, blob_frac, rows in rep_cases:
        n_frames = 40 if quick else 120
        good = wrong = crcfail = 0
        dec.cfg.exposure_rows = exp_frac * rpc
        for _ in range(n_frames):
            off = rng.uniform(0, len(chips2))
            b0 = rng.uniform(0.05, 0.95 - blob_frac)
            p = rolling_shutter_profile(chips2, rows, rpc, exp_frac, off, blob=(b0, b0 + blob_frac), noise=noise, rng=rng)
            pkts = dec.decode(p); crcfail += dec.stats.crc_fail
            for pk in pkts:
                if matches_truth(pk, rpc, off, packets2): good += 1
                else: wrong += 1
        total_wrong += wrong; total_good += good
        print(f"{rpc:5} {exp_frac:5.2f} {noise:5.1f} {blob_frac:5.2f} {rows:5} | {n_frames:6} {good:5} {wrong:5} {good/n_frames:9.2f} {crcfail:7}   repeat 2")
    rows = 1080
    for rpc, exp_frac, noise, blob_frac, amp in sat_cases:
        n_frames = 40 if quick else 120
        good = wrong = crcfail = 0
        dec.cfg.exposure_rows = exp_frac * rpc
        for _ in range(n_frames):
            off = rng.uniform(0, len(chips))               # anywhere in the stream: every frame other packets
            b0 = rng.uniform(0.05, 0.95 - blob_frac)
            p = rolling_shutter_profile(chips, rows, rpc, exp_frac, off, blob=(b0, b0 + blob_frac), amplitude=amp, noise=noise, rng=rng)
            pkts = dec.decode(p); crcfail += dec.stats.crc_fail
            for pk in pkts:
                if matches_truth(pk, rpc, off, packets): good += 1
                else: wrong += 1
        total_wrong += wrong; total_good += good
        print(f"{rpc:5} {exp_frac:5.2f} {noise:5.1f} {blob_frac:5.2f} {rows:5} | {n_frames:6} {good:5} {wrong:5} {good/n_frames:9.2f} {crcfail:7}   saturating (amplitude {amp:.0f})")
    print(f"sweep: {total_good} packets, {total_wrong} wrong")
    return total_good, total_wrong


def assembler_test():
    tx, chips, packets = make_stream()
    dec = Decoder(); asm = Assembler()
    rng = np.random.default_rng(7)
    got = {}
    frames = 0
    while len(got) < 5 and frames < 2000:
        frames += 1
        off = rng.uniform(0, len(chips))
        p = rolling_shutter_profile(chips, 1080, 5, 0.5, off, blob=(0.12, 0.88), noise=2.0, rng=rng)
        for pk in dec.decode(p):
            m = asm.feed(pk)
            if m:
                got[m[0]] = m
    print(f"assembler: {len(got)} messages after {frames} frames")
    for k in sorted(got):
        print("   ", got[k])
    exp = {6: "up=12s rst=POR fw=1.0", 0: "boot ok", 1: "sensor timeout, retrying", 2: "I2C NACK @0x68", 7: "HF pc=0x0000a3f2 cfsr=0x400"}
    for k, v in exp.items():
        assert got.get(k, (None, None, None))[2] == v, f"slot {k} mismatch: {got.get(k)}"
    print("assembler: ok")


def fountain_test():
    """Packets needed to complete a message when packets are received at random (fountain vs plain)."""
    import random
    rng = random.Random(3)
    text = "sensor timeout, retrying x3"
    tx = Transmitter(); tx.log(1, text)
    chips, packets = tx.stream(600)          # slot 0 only -> all packets belong to it
    n = tx.tx.slots[0].len
    trials = 200
    need = []
    for _ in range(trials):
        asm = Assembler(); got = None; count = 0
        order = list(range(len(packets))); rng.shuffle(order)
        for k in order:
            i, sd, pl = packets[k]
            from rscore import Packet
            p = Packet(); p.id = i; p.seed = sd; p.payload = pl
            count += 1
            m = asm.feed(p)
            if m: got = m; break
        assert got and got[2] == text, got
        need.append(count)
    # corruption: 2% of packets get a wrong payload (as if a packet-CRC false accept); never deliver garbage
    bad = 0; delivered = 0; rec_total = 0; reset_total = 0
    for _ in range(300):
        asm = Assembler(); order = list(range(len(packets))); rng.shuffle(order)
        for k in order[:120]:
            i, sd, pl = packets[k]
            if rng.random() < 0.02: pl ^= rng.randrange(1, 256)
            p = Packet(); p.id = i; p.seed = sd; p.payload = pl
            m = asm.feed(p)
            if m:
                delivered += 1
                if m[2] != text: bad += 1
        rec_total += asm.recovered; reset_total += asm.resets
    print(f"fountain: with 2% corrupted packets: {delivered} deliveries in 300 runs, {bad} wrong (recovered {rec_total}, resets {reset_total})")
    assert bad == 0, "garbage message delivered"
    assert rec_total > 0, "the leave-one-out recovery never ran"
    import statistics
    print(f"fountain: message of {n} packed bytes ({len(text)} chars): mean {statistics.mean(need):.1f} random packets to complete (min {min(need)}, max {max(need)}); coupon-collector would be ~{n*sum(1/i for i in range(1,n+1)):.0f}")


def rgb_test():
    """Three independent streams on R, G, B with Bayer crosstalk; the receiver must calibrate
    from the pilots and decode all channels."""
    tx = Transmitter()
    tx.set_slot(6, 5, "up=12s rst=POR fw=1.0"); tx.log(1, "boot ok"); tx.log(2, "sensor timeout, retrying")
    tx.log(3, "I2C NACK @0x68"); tx.set_slot(7, 6, "HF pc=0x0000a3f2 cfsr=0x400")
    tx.set_burst(5000, 1667)          # bursts and pauses, in chips (the firmware's 150 / 50 ms are 7500 / 2500 at T = 60 us)
    tx.set_channels(3, 3333)          # a pilot block every 3333 chips on average
    chips = tx.chips3(60000)          # the stream the frames are cut from
    M = np.array([[1.00, 0.12, 0.04],
                  [0.22, 1.00, 0.38],
                  [0.03, 0.10, 1.00]], dtype=np.float32)   # camera RGB response to LED R, G, B
    gains = np.array([110.0, 140.0, 90.0])
    rng = np.random.default_rng(11)
    rx = Receiver(); rpc = 6.0; frames = 0; got = {}; t = 0.0
    for f in range(600):                  # the length of the stream limits how fast the carousel completes, not the decoder
        off = rng.uniform(0, chips.shape[1] / rpc - 1200)  # offset in chips
        p = np.stack([rolling_shutter_profile(chips[c], 1080, rpc, 0.5, off, blob=(0.15, 0.85), amplitude=1.0, ambient=0.0, noise=0.0, quantize=False, rng=rng) for c in range(3)])
        obs = (M @ (p * gains[:, None])) + 10.0 + rng.normal(0, 2.0, p.shape)
        obs = np.clip(obs, 0, 255)
        n, msgs = rx.process(obs[0], obs[1], obs[2], t); t += 1 / 120
        frames += 1
        for m in msgs: got[m[0]] = m
        if len(got) >= 5: break
    print(f"rgb: mode={MODES[rx.mode]} pilots={rx.pilots} cond={rx.cond:.2f} packets={rx.packets} -> {len(got)} messages after {frames} frames")
    for k in sorted(got): print("   ", got[k])
    assert rx.mode == 1 and len(got) == 5, "rgb decode failed"
    # single-channel board (every LED carries the one stream): the same receiver must still deliver
    tx1 = Transmitter(); tx1.log(1, "mono board ok"); tx1.set_burst(5000, 1667)
    chips1 = tx1.chips3(40000)
    rx1 = Receiver(); got1 = None
    for f in range(300):
        off = rng.uniform(0, chips1.shape[1] / rpc - 1200)
        p = np.stack([rolling_shutter_profile(chips1[c], 1080, rpc, 0.5, off, blob=(0.15, 0.85), amplitude=1.0, ambient=0.0, noise=0.0, quantize=False, rng=rng) for c in range(3)])
        obs = np.clip((M @ (p * gains[:, None])) + 10.0 + rng.normal(0, 2.0, p.shape), 0, 255)
        n, msgs = rx1.process(obs[0], obs[1], obs[2], f / 120)
        if msgs: got1 = msgs[0]; break
    print(f"rgb: mono board -> mode={MODES[rx1.mode]} message={got1} after {f+1} frames")
    assert got1 and got1[2] == "mono board ok"


def tx_test():
    """Transmitter: a change of the number of channels in mid packet, long text, an empty line."""
    tx = Transmitter(); tx.set_slot(6, 5, "hello world")
    buf = (ctypes.c_uint8 * 3)()
    for _ in range(40): _lib.rs_tx_next_chips(ctypes.byref(tx.tx), buf)      # channel 0 is in mid packet
    tx.set_channels(3, 1500)
    for _ in range(200):
        _lib.rs_tx_next_chips(ctypes.byref(tx.tx), buf)
        assert max(tx.tx.chip_pos) <= RS_PKT_CHIPS and max(buf) <= 1, "channel switch: chip index past the packet"
    # the chip stream is the same whether the packets are prepared ahead (as a port does from a
    # lower-priority context) or by the chip path at the boundary; with repetition and pilots
    def chip_stream(ahead):
        t = Transmitter(); t.set_slot(6, 5, "up=12s rst=POR fw=1.0"); t.log(1, "boot ok"); t.set_channels(3, 700); t.set_repeat(2)
        out = []
        for _ in range(3000):
            if ahead: _lib.rs_tx_prepare(ctypes.byref(t.tx))
            _lib.rs_tx_next_chips(ctypes.byref(t.tx), buf); out.append(tuple(buf))
        return out
    assert chip_stream(True) == chip_stream(False), "preparing ahead changes the chip stream"
    # a text longer than one message is split where the 6-bit packing ends (up to 41 characters), not at 31
    tx = Transmitter(); text = "up=10000s rst=PIN n=2000 id=abcd and then some more text"
    took = tx.log_text(1, text)
    assert took[0] > 31 and sum(took) == len(text), took
    asm = Assembler(); got = []
    for _ in range(400):
        i, sd, pl = tx.next_packet(); p = Packet(); p.id = i; p.seed = sd; p.payload = pl
        m = asm.feed(p)
        if m: got.append(m[2])
    assert "".join(sorted(got, key=text.find)) == text, got
    # an empty line takes no log slot
    tx = Transmitter(); tx.log(1, "one"); tx.log(1, "")
    assert [bool(tx.tx.slots[i].valid) for i in range(2)] == [True, False] and tx.tx.next_log_slot == 1
    print(f"tx: channel switch in bounds; {len(text)} characters in {len(took)} messages of {took} characters")


def replaced_message_test():
    """A STATUS whose text changes while it is being received (same length, a counter) and half the
    packets lost: the receiver may deliver any of the texts, never a mixture of two."""
    import random
    rng = random.Random(5); tx = Transmitter(); asm = Assembler(); sent = set(); delivered = 0
    for n in range(4000):
        text = f"up={1000000 + n}s rst=POR t={n % 97:02d}"; sent.add(text); tx.set_slot(6, 5, text)
        for _ in range(60):
            i, sd, pl = tx.next_packet()
            if rng.random() < 0.5: continue
            p = Packet(); p.id = i; p.seed = sd; p.payload = pl
            m = asm.feed(p)
            if m:
                delivered += 1
                assert m[2] in sent, f"a text nobody sent: {m[2]!r}"
    assert delivered > 500, delivered
    print(f"replaced message: {delivered} deliveries over 4000 texts, all of them texts that were sent")


if __name__ == "__main__":
    quick = "--quick" in sys.argv
    sanity()
    tx_test()
    fountain_test()
    replaced_message_test()
    rgb_test()
    good, wrong = sweep(quick)
    assembler_test()
    # The sweep is seeded, so its numbers only move when the decoder does. A packet that is not in
    # the stream is a false accept: the CRC-12 lets one through per ~4096 completed detector runs
    # (about ten per packet), and the quality gate stops half of them: 1.5 per thousand packets
    # here and on recorded clips. Under 4 per thousand is chance; more is a detector fitting what
    # is not there. The floor is 95 % of today's yield: a change that loses packets shows.
    floor = FLOOR_QUICK if quick else FLOOR_FULL
    ok = wrong * 1000 <= 4 * good and good >= floor
    if not ok: print(f"FAILED: {good} packets (floor {floor}), {wrong} wrong (limit {4 * good // 1000})")
    sys.exit(0 if ok else 1)
