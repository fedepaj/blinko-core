#!/usr/bin/env python3
"""Protocol v3 packets on real recordings with the Python ML detector (codelab), extended with a
saturation model: template = min(1, gain * smeared level). Used to validate detector ideas before
they go into rs_decoder.c.

  v3lab.py REC.rsrec --t-us 60 [--row-us 5.44] [--frames 0:20] [--gains 1,2,4,8] [--channels g,r,b,luma]
"""
import argparse, sys
import numpy as np
sys.path.insert(0, ".")
from rsrec import Recording
from replay import profiles_bgra
import codelab
from codelab import RLL27, envelope_normalize, find_syncs, smear_template
from rscore import crc12

DATA_BITS = 30


class V3(RLL27):
    """The real v3 packet: sync 3/10/3, 30 bits (id 3, seed 7, payload 8, crc12), rate 1/2 + flush."""
    name = "v3"


def viterbi_gain(profile, start_row, code, nbits, rpc, exp_chips, gain, track=0.12):
    """codelab.viterbi_decode with clipped templates min(1, gain * t)."""
    k = 1; nsym = nbits
    tail = int(np.ceil(exp_chips)) + 1
    slips = (0.0, -max(track * rpc, 0.3), max(track * rpc, 0.3))
    surv = {(code.start_state(), 0): (0.0, [], list(code.sync)[-tail:], float(start_row), 0)}
    def advance(surv, syms):
        new = {}
        for (st, _), (metric, bits, hist, pos, ncell) in surv.items():
            for sym in syms:
                chips, nst = code.step(st, sym)
                nb = bits + [sym]
                if not chips:
                    key = (nst, ncell)
                    if key not in new or metric < new[key][0]: new[key] = (metric, nb, hist, pos, ncell)
                    continue
                tmpl = np.minimum(1.0, gain * smear_template(hist[-tail:], chips, rpc, exp_chips))
                best = None
                for d in slips:
                    a = int(round(pos + d)); b = a + len(tmpl)
                    if a < 0 or b > len(profile): continue
                    m = float(np.sum((profile[a:b] - tmpl) ** 2)) + (0.02 if d else 0.0)
                    if best is None or m < best[0]: best = (m, pos + d + len(chips) * rpc)
                if best is None: continue
                m = metric + best[0]; key = (nst, ncell + len(chips))
                if key not in new or m < new[key][0]: new[key] = (m, nb, (hist + chips)[-tail:], best[1], ncell + len(chips))
        return new
    for s in range(nsym):
        surv = advance(surv, (0, 1))
        if not surv: return None, np.inf
    for _ in range(4):
        pending = {kk: v for kk, v in surv.items() if kk[0][0] != ""}
        if not pending: break
        done = {kk: v for kk, v in surv.items() if kk[0][0] == ""}
        surv = {**done, **advance(pending, [0])}
    surv = {kk: v for kk, v in surv.items() if kk[0][0] == ""} or surv
    best = min(surv.values(), key=lambda v: v[0] / max(v[4], 1))
    return best[1][:nbits], best[0] / max(best[4], 1)


def decode_profile(prof, rpc, exp_chips, gains, max_cand=6):
    code = V3()
    x, amp = envelope_normalize(prof, rpc, win_chips=15, min_amp=6)
    out = []
    # sync candidates: ON runs of 8..16 cells between OFF runs (stretched by saturation), by run lengths
    binr = (x >= 0.5).astype(int); runs = []; s0 = 0
    for k in range(1, len(binr) + 1):
        if k == len(binr) or binr[k] != binr[k - 1]: runs.append((binr[s0], s0, k - s0)); s0 = k
    for i in range(1, len(runs) - 1):
        lv, st, ln = runs[i]
        if lv != 1 or not (8 * rpc <= ln <= 16 * rpc) or runs[i - 1][2] < 1.5 * rpc or runs[i + 1][2] < 1.0 * rpc: continue
        if np.median(amp[st:st + ln]) < 6: continue
        best = None
        for gain in gains:
            stretch = exp_chips * (1 - 1.0 / gain)                       # ON runs grow by E(1-1/g) cells
            rpc_est = (ln - stretch * rpc) / 10.0
            for r in (rpc_est, rpc, rpc * 0.98, rpc * 1.02):
                # rising 0.5-crossing at st; with gain g the ramp crosses 0.5 after E/(2g) instead of E/2
                t0 = st - exp_chips * r / (2 * gain)
                start = t0 + 13 * r
                if start + 66 * r > len(x): continue
                for dph in (-1, 0, 1):
                    bits, mse = viterbi_gain(x, start + dph, code, DATA_BITS, r, exp_chips, gain)
                    if bits is None: continue
                    if best is None or mse < best[0]: best = (mse, bits, gain, r, dph)
        if best is None: continue
        mse, bits, gain, r, dph = best
        v = int("".join(map(str, bits)), 2)
        fields, crc = v >> 12, v & 0xFFF
        ok = crc12(fields, 18) == crc
        out.append(dict(row=st, ok=ok, mse=round(mse, 3), gain=gain, rpc=round(r, 2), id=fields >> 15, seed=(fields >> 8) & 127, payload=fields & 255))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec"); ap.add_argument("--t-us", type=float, required=True); ap.add_argument("--row-us", type=float, default=5.44)
    ap.add_argument("--frames", default="0:20"); ap.add_argument("--gains", default="1,2,4,8"); ap.add_argument("--channels", default="g,r,b,luma"); ap.add_argument("--exp-us", type=float, default=0)
    a = ap.parse_args()
    rec = Recording(a.rec); exp_us = a.exp_us or float(rec.header.get("exposureUs", 0))
    cell_us = a.t_us / 3; rpc = cell_us / a.row_us; exp_chips = exp_us / cell_us
    gains = [float(g) for g in a.gains.split(",")]
    print(f"{a.rec.split('/')[-1]}: T {a.t_us} us, cell {cell_us:.1f} us = {rpc:.2f} rows, exposure {exp_us:.1f} us = {exp_chips:.2f} cells, gains {gains}")
    f0, f1 = [int(v) for v in a.frames.split(":")]
    ok_total = 0; cand_total = 0; by_gain = {}
    for k in range(f0, min(f1, len(rec))):
        ts, g, ac, fr = rec.frame(k); r, gg, b, info = profiles_bgra(fr, "rows")
        chans = {"g": gg, "r": r, "b": b, "luma": (r + gg + b) / 3}
        for name in a.channels.split(","):
            for p in decode_profile(chans[name], rpc, exp_chips, gains):
                cand_total += 1
                if p["ok"]:
                    ok_total += 1; by_gain[p["gain"]] = by_gain.get(p["gain"], 0) + 1
                    print(f"  frame {k} {name}: OK id{p['id']} seed{p['seed']} pl{p['payload']:02x} gain {p['gain']} rpc {p['rpc']} mse {p['mse']} row {p['row']}")
    print(f"valid packets {ok_total} of {cand_total} candidates; by gain {by_gain}")


if __name__ == "__main__":
    main()
