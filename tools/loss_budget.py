#!/usr/bin/env python3
"""Where do the packets go? Ground-truth loss budget of the decoder on the 1-D simulator.

For every frame the packets that lie entirely inside the lit rows are "available". Each one
is either decoded, rescuable (rs_decode_at at its true position passes the CRC: a perfect
temporal/phase prediction would get it without a sync), or lost (the bits themselves are
damaged: smear, saturation, noise).

  loss_budget.py [--frames 60]
"""
import argparse, ctypes, sys
import numpy as np
from rscore import _lib, Decoder, Packet, DecCfg, RS_PKT_CHIPS
from simulate import rolling_shutter_profile, matches_truth
from test_core import make_stream

_lib.rs_decode_at.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.POINTER(DecCfg), ctypes.c_float, ctypes.c_float, ctypes.POINTER(Packet)]
_lib.rs_decode_at.restype = ctypes.c_int


def budget(chips, packets, rpc, exp, noise, blob_frac, amp, frames, rng, hint=True):
    rows = 1080
    dec = Decoder()
    if hint: dec.cfg.rows_per_chip_hint = rpc
    avail = decoded = rescuable = lost = 0
    for _ in range(frames):
        off = rng.uniform(0, RS_PKT_CHIPS)
        b0 = rng.uniform(0.05, 0.95 - blob_frac)
        p = rolling_shutter_profile(chips, rows, rpc, exp, off, blob=(b0, b0 + blob_frac), amplitude=amp, noise=noise, rng=rng)
        got = dec.decode(p)
        got_rows = [pk.row_start for pk in got if matches_truth(pk, rpc, off, packets)]
        pf = np.ascontiguousarray(p, dtype=np.float32).ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        # truth packets whose rows [start, end] lie inside the lit band (with a 3-chip margin for the soft edge)
        lo, hi = (b0 * rows) + 3 * rpc, (b0 + blob_frac) * rows - 3 * rpc
        k0 = int(np.floor(off / RS_PKT_CHIPS)) - 1
        for k in range(k0, k0 + int(rows / rpc / RS_PKT_CHIPS) + 3):
            start = (k * RS_PKT_CHIPS - off) * rpc
            end = start + RS_PKT_CHIPS * rpc
            if start < lo or end > hi: continue
            avail += 1
            if any(abs(r - start) < 2.0 * rpc for r in got_rows): decoded += 1; continue
            pk = Packet(); ok = 0
            for d in (0.0, -0.25, 0.25, -0.5, 0.5):
                if _lib.rs_decode_at(pf, rows, ctypes.byref(dec.cfg), start + d * rpc, rpc, ctypes.byref(pk)) and matches_truth(pk, rpc, off, packets):
                    ok = 1; break
            if ok: rescuable += 1
            else: lost += 1
    return avail, decoded, rescuable, lost


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--frames", type=int, default=60); a = ap.parse_args()
    tx, chips, packets = make_stream()
    rng = np.random.default_rng(7)
    cases = [(6, 0.5, 2.0, 0.6, 120.0), (6, 1.0, 2.0, 0.6, 120.0), (6, 0.5, 6.0, 0.6, 120.0), (6, 0.5, 2.0, 0.6, 900.0),
             (6, 1.0, 2.0, 0.6, 2500.0), (4, 0.8, 2.0, 0.6, 1200.0), (8, 0.5, 3.0, 0.4, 120.0), (5, 0.3, 4.0, 0.5, 400.0)]
    print(f"{'rpc':>4} {'exp':>4} {'noise':>5} {'blob':>4} {'amp':>5} | {'avail':>5} {'decoded':>7} {'rescuable':>9} {'lost':>5}   decoded%  +rescue%")
    for rpc, exp, noise, blob, amp in cases:
        av, de, re, lo = budget(chips, packets, rpc, exp, noise, blob, amp, a.frames, rng)
        print(f"{rpc:4} {exp:4.1f} {noise:5.1f} {blob:4.1f} {amp:5.0f} | {av:5} {de:7} {re:9} {lo:5}   {100*de/max(av,1):6.0f}%   {100*(de+re)/max(av,1):6.0f}%")


if __name__ == "__main__":
    main()
