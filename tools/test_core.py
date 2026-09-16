"""Robustness sweep of the decoder against the rolling-shutter simulator.

Usage: .venv/bin/python tools/test_core.py [--quick]
"""
import sys, numpy as np
from rscore import Transmitter, Decoder, Assembler, Receiver, encode_packet, pack6, unpack6, RS_PKT_CHIPS, LEVELS
from simulate import rolling_shutter_profile, true_packet_at


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
    print(f"{'rpc':>5} {'exp':>5} {'noise':>5} {'blob':>5} | {'frames':>6} {'pkts':>5} {'wrong':>5} {'pkt/frame':>9} {'crcfail':>7}")
    worst = 0
    cases = []
    for rpc in ([4, 8, 16] if quick else [3, 4, 5, 6, 8, 12, 16, 24, 32, 40]):
        for exp_frac in [0.3, 0.6, 1.0]:
            for noise in [1.0, 3.0]:
                cases.append((rpc, exp_frac, noise, 0.6))
    cases += [(8, 0.5, 2.0, 0.15), (8, 0.5, 2.0, 0.3), (8, 0.5, 6.0, 0.6), (4, 0.5, 4.0, 0.6)]
    total_wrong = 0
    for rpc, exp_frac, noise, blob_frac in cases:
        n_frames = 40 if quick else 120
        good = wrong = crcfail = 0
        for _ in range(n_frames):
            off = rng.uniform(0, RS_PKT_CHIPS)
            b0 = rng.uniform(0.05, 0.95 - blob_frac)
            p = rolling_shutter_profile(chips, rows, rpc, exp_frac, off, blob=(b0, b0 + blob_frac),
                                        noise=noise, rng=rng)
            pkts = dec.decode(p)
            crcfail += dec.stats.crc_fail
            for pk in pkts:
                i, sd, pl, phase, err = true_packet_at(pk.row_start, rpc, off, packets)
                if (i, sd, pl) == (pk.id, pk.seed, pk.payload):
                    good += 1
                else:
                    wrong += 1
        total_wrong += wrong
        print(f"{rpc:5} {exp_frac:5.2f} {noise:5.1f} {blob_frac:5.2f} | {n_frames:6} {good:5} {wrong:5} {good/n_frames:9.2f} {crcfail:7}")
    print("total wrong packets:", total_wrong)
    return total_wrong


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
    # corruption: 2% of packets get a wrong payload (as if a CRC-8 false accept); never deliver garbage
    bad = 0; delivered = 0
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
    print(f"fountain: with 2% corrupted packets: {delivered} deliveries in 300 runs, {bad} wrong")
    assert bad == 0, "garbage message delivered"
    import statistics
    print(f"fountain: message of {n} packed bytes ({len(text)} chars): mean {statistics.mean(need):.1f} random packets to complete (min {min(need)}, max {max(need)}); coupon-collector would be ~{n*sum(1/i for i in range(1,n+1)):.0f}")


def rgb_test():
    """Three independent streams on R, G, B with Bayer crosstalk; the receiver must calibrate
    from the pilots and decode all channels."""
    tx = Transmitter()
    tx.set_slot(6, 5, "up=12s rst=POR fw=1.0"); tx.log(1, "boot ok"); tx.log(2, "sensor timeout, retrying")
    tx.log(3, "I2C NACK @0x68"); tx.set_slot(7, 6, "HF pc=0x0000a3f2 cfsr=0x400")
    tx.set_burst(5000, 1667)          # 150 / 50 ms at 30 us
    tx.set_channels(3, 3333)          # pilot every 100 ms
    chips = tx.chips3(60000)          # 1.8 s of stream
    M = np.array([[1.00, 0.12, 0.04],
                  [0.22, 1.00, 0.38],
                  [0.03, 0.10, 1.00]], dtype=np.float32)   # camera RGB response to LED R, G, B
    gains = np.array([110.0, 140.0, 90.0])
    rng = np.random.default_rng(11)
    rx = Receiver(); rpc = 6.0; frames = 0; got = {}; t = 0.0
    for f in range(400):
        off = rng.uniform(0, chips.shape[1] / rpc - 1200) * rpc / rpc  # offset in chips
        p = np.stack([rolling_shutter_profile(chips[c], 1080, rpc, 0.5, off, blob=(0.15, 0.85), amplitude=1.0, ambient=0.0, noise=0.0, quantize=False, rng=rng) for c in range(3)])
        obs = (M @ (p * gains[:, None])) + 10.0 + rng.normal(0, 2.0, p.shape)
        obs = np.clip(obs, 0, 255)
        n, msgs = rx.process(obs[0], obs[1], obs[2], t); t += 1 / 120
        frames += 1
        for m in msgs: got[m[0]] = m
        if len(got) >= 5: break
    print(f"rgb: mode={'rgb' if rx.mode else 'luma'} pilots={rx.pilots} cond={rx.cond:.2f} packets={rx.packets} -> {len(got)} messages after {frames} frames")
    for k in sorted(got): print("   ", got[k])
    assert rx.mode == 1 and len(got) == 5, "rgb decode failed"
    # single-channel board: same receiver must fall back to luma
    tx1 = Transmitter(); tx1.log(1, "mono board ok"); tx1.set_burst(5000, 1667)
    chips1 = tx1.chips3(40000)
    rx1 = Receiver(); got1 = None
    for f in range(300):
        off = rng.uniform(0, chips1.shape[1] / rpc - 1200)
        p = np.stack([rolling_shutter_profile(chips1[c], 1080, rpc, 0.5, off, blob=(0.15, 0.85), amplitude=1.0, ambient=0.0, noise=0.0, quantize=False, rng=rng) for c in range(3)])
        obs = np.clip((M @ (p * gains[:, None])) + 10.0 + rng.normal(0, 2.0, p.shape), 0, 255)
        n, msgs = rx1.process(obs[0], obs[1], obs[2], f / 120)
        if msgs: got1 = msgs[0]; break
    print(f"rgb: mono board -> mode={'rgb' if rx1.mode else 'luma'} message={got1} after {f+1} frames")
    assert got1 and got1[2] == "mono board ok"


if __name__ == "__main__":
    quick = "--quick" in sys.argv
    sanity()
    fountain_test()
    rgb_test()
    wrong = sweep(quick)
    assembler_test()
    sys.exit(1 if wrong > 2 else 0)  # CRC-8 admits ~1 false accept per few thousand candidates in extreme cases
