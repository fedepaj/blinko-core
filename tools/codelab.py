#!/usr/bin/env python3
"""Line-code bench for the rolling-shutter channel (protocol v3 brainstorming).

Compares codes (Manchester, Miller, NRZ+scrambler, 4B6B, balanced 8B10B, PPM) under the same
channel model and the same receiver: a maximum-likelihood Viterbi detector that knows the
exposure (the phone does) and the chip clock (from the sync / strobe calibration), with
per-packet CRC-8 acceptance. Every code gets its best shot, so the comparison is about physics
(minimum run vs exposure, packet length vs blob window), not about decoder quality.

  codelab.py sweep [--codes manchester,miller,...] [--quick]     yield vs E/T and window
  codelab.py one CODE --et 1.0 --window-ms 2.5 --chip-us 60 [--n 200]

Channel (simulate.rolling_shutter_profile): rows of 5.4 us (row time measured on iPhone 14 and
Samsung S21 FE), exposure E as a box filter, blob = window W (rows where the modulation is
readable), LED amplitude with optional saturation (clipping), gaussian noise, random packet phase.
Metric: whole packets accepted per frame and payload bits per ms of window.
"""
import argparse, itertools, sys
import numpy as np
from simulate import rolling_shutter_profile

ROW_US = 5.4
HDR_BITS = 3 + 7            # id + seed (v3 candidate: 26 data bits = 3+7+8+8)
PAYLOAD_BITS = 8
CRC_BITS = 8
DATA_BITS = HDR_BITS + PAYLOAD_BITS + CRC_BITS


def crc8(bits):
    crc = 0
    data = list(bits) + [0] * ((-len(bits)) % 8)
    for i in range(0, len(data), 8):
        byte = int("".join(map(str, data[i:i + 8])), 2)
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return [(crc >> (7 - k)) & 1 for k in range(8)]


def random_packet_bits(rng):
    d = list(rng.integers(0, 2, HDR_BITS + PAYLOAD_BITS))
    return d + crc8(d)


# ----------------------------------------------------------------------------------------------
# Codes. Each code maps a bit sequence to chips (0/1) and exposes a symbol trellis for the
# receiver: symbols(bit-group size), states, step(state, symbol) -> (chips, next_state).
# `sync` is a chip pattern that cannot occur inside coded data (run-length violation or a
# long unique word), followed by data starting from `start_state`.

class Code:
    name = "?"; bits_per_symbol = 1; sync = (); gap = 1
    cells_per_T = 1          # code cells per minimum run T (Manchester 1, Miller 2, RLL(2,7) 3)
    def start_state(self): return 0
    def step(self, state, sym): raise NotImplementedError
    def encode(self, bits):
        st = self.start_state(); out = []
        k = self.bits_per_symbol
        for i in range(0, len(bits), k):
            sym = 0
            for b in bits[i:i + k]: sym = sym * 2 + b
            chips, st = self.step(st, sym); out += chips
        return out, st
    def states(self): return [0]


class Manchester(Code):
    name = "manchester"; sync = (0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0)          # gap, 4 on, 4 off, start bit "0" = 10
    def step(self, state, bit): return ([0, 1] if bit else [1, 0]), 0


class Miller(Code):
    """Delay modulation: 1 = transition at mid-bit; 0 = transition at bit start only after a 0.
    Runs are T, 1.5T or 2T, so a 3T ON run is a unique sync. state = (level, last_bit_was_zero)."""
    name = "miller"; sync = (0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0)    # in half-chips: gap 2T, on 3T, off 2T; data starts OFF
    def start_state(self): return (0, 0)
    def states(self): return [(l, z) for l in (0, 1) for z in (0, 1)]
    def step(self, state, bit):
        level, last_zero = state
        if bit: chips = [level, 1 - level]; level = 1 - level; last_zero = 0
        else:
            if last_zero: level = 1 - level
            chips = [level, level]; last_zero = 1
        return chips, (level, last_zero)
    cells_per_T = 2          # each bit is 2 half-chips; the minimum run T is 2 cells


class NRZ(Code):
    """Scrambled NRZ (the 64b/66b idea at packet scale): 1 bit per chip, no run guarantee,
    sync = an 8-chip unique word found by correlation. Needs the chip clock from elsewhere."""
    name = "nrz"; sync = (0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0)
    LFSR = [1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0]
    def start_state(self): return 0
    def states(self): return list(range(len(self.LFSR)))
    def step(self, state, bit): return [bit ^ self.LFSR[state % len(self.LFSR)]], state + 1


class FourBSixB(Code):
    """IEEE 802.15.7 4B6B: 4 bits -> 6 chips with three ones; max run 4 -> sync run of 5."""
    name = "4b6b"; bits_per_symbol = 4; sync = (0, 0, 1, 1, 1, 1, 1, 0, 0)
    TABLE = ["011100", "011010", "010110", "011001", "010101", "100101", "101001", "011000"[::-1],
             "110001", "110010", "101010", "100110", "110100", "101100", "100011", "010011"]
    def __init__(self):
        # replace the one non-balanced placeholder with a balanced word not yet used
        used = set(self.TABLE); cands = ["".join(p) for p in itertools.product("01", repeat=6) if p.count("1") == 3]
        self.TABLE = [w if w.count("1") == 3 else next(c for c in cands if c not in used) for w in self.TABLE]
    def step(self, state, sym): return [int(c) for c in self.TABLE[sym]], 0


class EightBTenB(Code):
    """Balanced 8B10B (constructed, not the IBM tables): 256 ten-chip words of weight 5 (252) plus
    4 of weight 4/6, all with runs <= 4 inside the word; sync = run of 6. Rate 0.8 like the real one."""
    name = "8b10b"; bits_per_symbol = 8; sync = (0, 0, 1, 1, 1, 1, 1, 1, 0, 0)
    def __init__(self):
        def maxrun(w): return max(len(list(g)) for _, g in itertools.groupby(w))
        words = ["".join(p) for p in itertools.product("01", repeat=10)]
        five = [w for w in words if w.count("1") == 5 and maxrun(w) <= 4]
        four = [w for w in words if w.count("1") in (4, 6) and maxrun(w) <= 3]
        self.TABLE = (five + four)[:256]
        assert len(self.TABLE) == 256
    def step(self, state, sym): return [int(c) for c in self.TABLE[sym]], 0


class PPM(Code):
    """M-PPM: one ON pulse of `pulse` chips in one of M slots, then a guard of `guard` OFF chips.
    log2(M) bits per symbol; information is in the rising-edge position."""
    def __init__(self, m=4, pulse=1, guard=1):
        self.m = m; self.pulse = pulse; self.guard = guard
        self.bits_per_symbol = int(np.log2(m)); self.name = f"ppm{m}"
        self.sync = (0, 0) + (1,) * (pulse + 2) + (0, 0)        # a pulse longer than any data pulse
    def step(self, state, sym):
        chips = [0] * (self.m * self.pulse + self.guard)
        for k in range(self.pulse): chips[sym * self.pulse + k] = 1
        return chips, 0


class RLL27(Code):
    """RLL(2,7) as on hard disks: variable-length table (2 bits -> 4 cells, 3 -> 6, 4 -> 8), rate 1/2,
    NRZI (a 1 in the codeword = transition). Transitions are >= 3 and <= 8 cells apart: minimum run
    T = 3 cells, 1.5 bits per T. Modelled as a bit-driven state machine (prefix tree node, level):
    internal nodes emit nothing, leaves emit their codeword, so the generic Viterbi decodes it
    without any parsing ambiguity. sync = OFF 3, ON 10, OFF 3 cells (a run of 10 cannot occur)."""
    name = "rll27"; cells_per_T = 3; variable = True; sync = (0, 0, 0) + (1,) * 10 + (0, 0, 0)
    TABLE = {"10": "0100", "11": "1000", "000": "000100", "010": "100100", "011": "001000", "0010": "00100100", "0011": "00001000"}
    rll = (2, 7)
    def start_state(self): return ("", 0)
    def states(self): return [(n, l) for n in ("", "0", "1", "00", "01", "001") for l in (0, 1)]
    def step(self, state, bit):
        node, level = state
        node += str(bit)
        if node in self.TABLE:
            cells = []
            for c in self.TABLE[node]:
                if c == "1": level = 1 - level
                cells.append(level)
            return cells, ("", level)
        return [], (node, level)
    def encode(self, bits):
        st = self.start_state(); out = []
        for b in list(bits) + [0, 0, 0]:              # flush the tree with zeros like the decoder does
            cells, st = self.step(st, b); out += cells
            if st[0] == "" and len(out) >= 2 * len(bits): break
        return out, st


CODES = {"manchester": Manchester, "miller": Miller, "rll27": RLL27, "nrz": NRZ, "4b6b": FourBSixB, "8b10b": EightBTenB,
         "ppm4": lambda: PPM(4, 1, 1), "ppm8": lambda: PPM(8, 1, 1)}


# ----------------------------------------------------------------------------------------------
# Channel + receiver

def padded(bits, k):
    return list(bits) + [0] * ((-len(bits)) % k)


def chips_of_packet(code, bits):
    data, _ = code.encode(padded(bits, code.bits_per_symbol))
    return list(code.sync) + data


_tcache = {}


def smear_template(chips_before, chips, rpc, exp_chips, sub=8):
    key = (tuple(chips_before), tuple(chips), round(rpc, 4), round(exp_chips, 4))
    t = _tcache.get(key)
    if t is None:
        t = _smear_template(chips_before, chips, rpc, exp_chips, sub); _tcache[key] = t
    return t


def _smear_template(chips_before, chips, rpc, exp_chips, sub=8):
    """Expected profile (per row, mean LED level 0..1) over the rows of `chips`, given the chips
    that precede them (their tail leaks in through the exposure). Box exposure, row centres."""
    seq = list(chips_before) + list(chips)
    n_rows = int(round(len(chips) * rpc))
    dt = 1.0 / (rpc * sub)
    fine = np.repeat(seq, int(round(rpc * sub)))
    C = np.concatenate([[0.0], np.cumsum(fine) * dt])
    def integ(x):
        i = np.clip((x / dt).astype(np.int64), 0, len(fine) - 1); frac = x / dt - i
        return C[i] + frac * fine[i] * dt
    t0 = len(chips_before) / 1.0                      # chip time where `chips` start
    r = np.arange(n_rows)
    ts = t0 + r / rpc - exp_chips                     # row r integrates [ts, ts+E] ending at its readout time
    ts = np.clip(ts, 0, None)
    e = max(exp_chips, dt)
    return (integ(ts + e) - integ(ts)) / e


def viterbi_decode(profile, start_row, code, nbits, rpc, exp_chips, lo, hi, track=0.12):
    """ML sequence detection over the code trellis with exposure-aware templates.
    Per symbol the position may slip by -track/0/+track chips (a PLL: the chip clock from the
    sync is only known to ~2 %). Codes with variable-length emission (RLL tables) are keyed by
    (state, cells emitted) and flushed with zero bits at the end. Returns (bits, metric)."""
    k = code.bits_per_symbol; nsym = -(-nbits // k)
    variable = getattr(code, "variable", False)
    tail = int(np.ceil(exp_chips)) + 1
    scale = max(hi - lo, 1e-3)
    slips = (0.0, -track * rpc, track * rpc) if track > 0 else (0.0,)
    # survivors: key -> (metric, bits, chips history tail, position in rows, cells emitted)
    surv = {(code.start_state(), 0): (0.0, [], list(code.sync)[-tail:], float(start_row), 0)}

    def advance(surv, syms):
        new = {}
        for (st, _), (metric, bits, hist, pos, ncell) in surv.items():
            for sym in syms:
                chips, nst = code.step(st, sym)
                newbits = bits + [(sym >> (k - 1 - j)) & 1 for j in range(k)]
                if not chips:
                    key = (nst, ncell if variable else 0)
                    if key not in new or metric < new[key][0]: new[key] = (metric, newbits, hist, pos, ncell)
                    continue
                tmpl = smear_template(hist[-tail:], chips, rpc, exp_chips)
                best = None
                for d in slips:
                    a = int(round(pos + d)); b = a + len(tmpl)
                    if a < 0 or b > len(profile): continue
                    obs = (profile[a:b] - lo) / scale
                    m = float(np.sum((obs - tmpl) ** 2)) + (0.02 if d else 0.0)
                    if best is None or m < best[0]: best = (m, pos + d + len(chips) * rpc)
                if best is None: continue
                m = metric + best[0]
                key = (nst, ncell + len(chips) if variable else 0)
                if key not in new or m < new[key][0]:
                    new[key] = (m, newbits, (hist + chips)[-tail:], best[1], ncell + len(chips))
        return new

    syms = range(2 ** k)
    for s in range(nsym):
        surv = advance(surv, syms)
        if not surv: return None, np.inf
    if variable:   # flush: pad with zero bits until every survivor sits at the tree root
        for _ in range(4):
            pending = {kk: v for kk, v in surv.items() if kk[0][0] != ""}
            if not pending: break
            done = {kk: v for kk, v in surv.items() if kk[0][0] == ""}
            surv = {**done, **advance(pending, [0])}
        surv = {kk: v for kk, v in surv.items() if kk[0][0] == ""} or surv
        best = min(surv.values(), key=lambda v: v[0] / max(v[4], 1))
    else:
        best = min(surv.values(), key=lambda v: v[0])
    return best[1][:nbits], best[0]


def envelope_normalize(profile, rpc, win_chips=5.0, min_amp=6.0):
    """Local [0,1] normalization with sliding min/max over `win_chips` (any window that long holds
    both levels in Manchester/Miller data and in the sync), like the classic decoder's local
    thresholds: the blob envelope changes the LED brightness several-fold along one packet."""
    n = len(profile); w = max(3, int(round(win_chips * rpc)))
    from numpy.lib.stride_tricks import sliding_window_view
    pad = np.pad(profile, (w // 2, w - w // 2 - 1), mode="edge")
    v = sliding_window_view(pad, w)
    lo = v.min(axis=1); hi = v.max(axis=1)
    amp = hi - lo
    x = np.where(amp >= min_amp, (profile - lo) / np.maximum(amp, 1e-3), 0.5)
    return x.astype(np.float32), amp


def find_syncs(profile, code, rpc, exp_chips, lo, hi, thr=0.25):
    """Rows where the smeared sync template correlates best (normalized), one per local maximum."""
    tmpl = smear_template([0] * 2, code.sync, rpc, exp_chips)
    tmpl = tmpl - tmpl.mean()
    x = (profile - lo) / max(hi - lo, 1e-3)
    n = len(tmpl)
    if len(x) <= n: return []
    win = np.lib.stride_tricks.sliding_window_view(x, n)
    wc = win - win.mean(axis=1, keepdims=True)
    corr = (wc @ tmpl) / (np.linalg.norm(wc, axis=1) * np.linalg.norm(tmpl) + 1e-6)
    out = []
    for i in range(1, len(corr) - 1):
        if corr[i] > thr and corr[i] >= corr[i - 1] and corr[i] >= corr[i + 1]: out.append((i, corr[i]))
    out.sort(key=lambda t: -t[1])
    return out


REAL = dict(noise=6.0, rpc_err=0.02, exp_err=0.10, amplitude=200.0)   # realistic receiver knowledge / camera


def trial(code, chip_us, exp_us, window_ms, rows=1080, amplitude=200.0, noise=6.0, rpc_err=0.02, exp_err=0.10, n=100, seed=0, verbose=False):
    """n random frames: a stream of random packets, random phase, blob covering `window_ms`.
    The receiver uses a chip clock off by `rpc_err` (relative, random sign per frame) and an
    exposure estimate off by `exp_err`. Returns (packets accepted, false accepts, frames)."""
    rng = np.random.default_rng(seed)
    rpc = chip_us / ROW_US / code.cells_per_T; exp_chips = exp_us / chip_us * code.cells_per_T   # per code cell
    win_rows = window_ms * 1000 / ROW_US
    if win_rows >= rows: rows = int(win_rows * 1.4)
    blob0 = 0.5 - win_rows / rows / 2; blob1 = blob0 + win_rows / rows
    # stream of packets
    pkts = [random_packet_bits(rng) for _ in range(12)]
    chips = []; starts = []
    for b in pkts: starts.append(len(chips)); chips += chips_of_packet(code, b)
    chips = np.array(chips, dtype=np.float64); period = len(chips)
    ok = 0; false = 0
    pk_chips = len(chips_of_packet(code, pkts[0]))
    for f in range(n):
        offset = rng.uniform(0, period)
        prof = rolling_shutter_profile(chips, rows, rpc, exp_chips, offset, blob=(blob0, blob1), amplitude=amplitude, noise=noise, edge_softness=0.01, rng=rng)
        r0 = int(blob0 * rows) + 2; r1 = int(blob1 * rows) - 2
        seg = prof[r0:r1]
        lo, hi = np.percentile(seg, 5), np.percentile(seg, 95)
        if hi - lo < 10: continue
        # what the receiver believes: chip clock and exposure slightly wrong
        rpc_rx = rpc * (1 + rpc_err * rng.choice([-1, 1])); exp_rx = exp_chips * (1 + exp_err * rng.choice([-1, 1]))
        accepted = set()
        for row, c in find_syncs(seg, code, rpc_rx, exp_rx, lo, hi)[:6]:
            # timing hypotheses like the real decoder: chip clock +-3 % in 1 % steps, sync phase +-1 row
            bits = None
            for drpc in (0, -0.01, 0.01, -0.02, 0.02, -0.03, 0.03):
                r = rpc_rx * (1 + drpc)
                for dph in (0, -1, 1):
                    start = row + dph + len(code.sync) * r
                    span = (2 * DATA_BITS + 8) if getattr(code, "variable", False) else (-(-DATA_BITS // code.bits_per_symbol)) * len(code.step(code.start_state(), 0)[0])
                    if start + span * r > len(seg): continue
                    b, metric = viterbi_decode(seg, start, code, DATA_BITS, r, exp_rx, lo, hi)
                    if b is not None and crc8(b[:-CRC_BITS]) == b[-CRC_BITS:]: bits = b; break
                if bits is not None: break
            if bits is None: continue
            # ground truth: which packet starts at this chip position?
            chip_pos = (row + r0) / rpc + offset
            k = int(round(chip_pos)) % period
            truth = None
            for i, s in enumerate(starts):
                if abs(((k - s + period / 2) % period) - period / 2) <= max(1.5, exp_chips): truth = pkts[i]
            key = int(round(chip_pos / pk_chips))
            if truth is not None and bits == truth:
                if key not in accepted: accepted.add(key); ok += 1
            else: false += 1
    return ok, false, n


def sweep(codes, quick=False, chip_us=60.0, ideal=False):
    ets = [0.5, 1.0, 1.5, 2.0] if not quick else [1.0, 1.5, 2.0]
    windows = [1.6, 2.5, 3.5] if not quick else [2.5]
    n = 40 if quick else 120
    kw = dict(noise=2.5, rpc_err=0.0, exp_err=0.0, amplitude=160.0) if ideal else REAL
    print(f"minimum run T = {chip_us:.0f} us, row {ROW_US} us, {DATA_BITS} data bits/packet, {n} frames per cell, {'ideal' if ideal else 'realistic'} receiver {kw}; cell = packets/frame (false accepts)")
    print(f"{'code':10s} {'pkt chips':>9s} {'ms':>5s} " + " ".join(f"E/T={et:<4} W={w:<4}" for et in ets for w in windows))
    for name in codes:
        code = CODES[name]()
        pk = len(chips_of_packet(code, random_packet_bits(np.random.default_rng(1))))
        unit = chip_us / code.cells_per_T
        row = f"{name:10s} {pk:9d} {pk * unit / 1000:5.2f} "
        for et in ets:
            for w in windows:
                ok, false, nn = trial(code, chip_us, et * chip_us, w, n=n, **kw)
                row += f"{ok / nn:8.2f}({false:2d})     "
        print(row, flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sw = sub.add_parser("sweep"); sw.add_argument("--codes", default=",".join(CODES)); sw.add_argument("--quick", action="store_true")
    sw.add_argument("--chip-us", type=float, default=60); sw.add_argument("--ideal", action="store_true")
    one = sub.add_parser("one"); one.add_argument("code"); one.add_argument("--et", type=float, default=1.0); one.add_argument("--window-ms", type=float, default=2.5)
    one.add_argument("--chip-us", type=float, default=60); one.add_argument("--n", type=int, default=200); one.add_argument("--amplitude", type=float, default=200)
    one.add_argument("--noise", type=float, default=6); one.add_argument("--rpc-err", type=float, default=0.02); one.add_argument("--exp-err", type=float, default=0.10)
    a = ap.parse_args()
    if a.cmd == "sweep": sweep(a.codes.split(","), a.quick, a.chip_us, a.ideal)
    else:
        code = CODES[a.code]()
        ok, false, n = trial(code, a.chip_us, a.et * a.chip_us, a.window_ms, n=a.n, amplitude=a.amplitude, noise=a.noise, rpc_err=a.rpc_err, exp_err=a.exp_err)
        pk = len(chips_of_packet(code, random_packet_bits(np.random.default_rng(1))))
        unit = a.chip_us / code.cells_per_T
        print(f"{a.code}: packet {pk} chips = {pk * unit / 1000:.2f} ms; E/T {a.et}, window {a.window_ms} ms: {ok / n:.2f} packets/frame, {false} false accepts, "
              f"{ok / n * PAYLOAD_BITS / a.window_ms:.1f} payload bits per ms of window")


if __name__ == "__main__":
    main()
