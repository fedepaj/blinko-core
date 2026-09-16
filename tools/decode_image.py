"""Decode RSLog packets from a still photo or a video frame (offline debugging).

Usage: .venv/bin/python tools/decode_image.py IMAGE [--axis rows|columns] [--plot out.png]
The image is converted to luma; the ROI on the cross axis is the bright blob.
"""
import sys, argparse, numpy as np
from PIL import Image
from rscore import Decoder, Assembler, LEVELS


def profile_from_image(img, axis):
    y = np.asarray(img.convert("L"), dtype=np.float32)  # rows x cols
    if axis == "columns":
        y = y.T
    cross = y.mean(axis=0)
    mn, mx = cross.min(), cross.max()
    if mx - mn < 8:
        lo, hi = 0, y.shape[1]
    else:
        thr = mn + 0.4 * (mx - mn)
        peak = int(np.argmax(cross))
        lo = peak
        while lo > 0 and cross[lo - 1] > thr: lo -= 1
        hi = peak
        while hi < len(cross) - 1 and cross[hi + 1] > thr: hi += 1
        hi += 1
    return y[:, lo:hi].mean(axis=1), (lo, hi)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--axis", default="rows", choices=["rows", "columns"])
    ap.add_argument("--plot")
    a = ap.parse_args()
    img = Image.open(a.image)
    p, roi = profile_from_image(img, a.axis)
    dec = Decoder()
    pkts = dec.decode(p)
    st = dec.stats
    print(f"profile: {len(p)} samples, roi {roi}, contrast {st.contrast:.0f}, syncs {st.syncs}, crc_fail {st.crc_fail}, truncated {st.truncated}")
    asm = Assembler()
    for pk in pkts:
        kind = "META" if pk.idx == 31 else f"idx {pk.idx:2d}"
        print(f"  slot {pk.id} {kind} data 0x{pk.data:02x} rows {pk.row_start:7.1f}-{pk.row_end:7.1f} rpc {pk.rows_per_chip:5.2f} q {pk.quality:.2f}")
        m = asm.feed(pk)
        if m: print("  -> message", m)
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(12, 3))
        ax.plot(p, lw=0.7)
        for pk in pkts:
            ax.axvspan(pk.row_start, pk.row_end, color="g", alpha=0.2)
        ax.set_xlabel(a.axis); ax.set_ylabel("luma")
        fig.tight_layout(); fig.savefig(a.plot, dpi=120)
        print("plot:", a.plot)


if __name__ == "__main__":
    main()
