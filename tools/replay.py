"""Replay a .rsrec recording through the C receiver exactly as the app does.

  replay.py REC.rsrec [--axis rows|columns] [--json out.json] [--png N out.png] [--quiet]

Prints per-recording metrics: frames, fps, packets/s, messages, RGB lock, pilots, resets.
"""
import argparse, ctypes, json, sys, time
import numpy as np
from rscore import _lib, Receiver, LEVELS
from rsrec import Recording


class Info(ctypes.Structure):
    _fields_ = [("roi_start", ctypes.c_int), ("roi_end", ctypes.c_int), ("count", ctypes.c_int),
                ("peak", ctypes.c_int), ("sat_frac", ctypes.c_float)]


_lib.rs_frame_profile_rgb.argtypes = [ctypes.POINTER(ctypes.c_uint8)] + [ctypes.c_int] * 8 + [ctypes.POINTER(ctypes.c_float)] * 3 + [ctypes.POINTER(Info)]
_lib.rs_rx_resets.argtypes = [ctypes.c_void_p]; _lib.rs_rx_resets.restype = ctypes.c_uint32


def profiles_bgra(a, axis):
    """BGRA frame (h, w, 4) -> r, g, b per-row profiles + info, via the shared C code."""
    h, w, _ = a.shape
    a = np.ascontiguousarray(a)
    n = max(w, h)
    r = np.zeros(n, np.float32); g = np.zeros(n, np.float32); b = np.zeros(n, np.float32); info = Info()
    fp = lambda x: x.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
    _lib.rs_frame_profile_rgb(a.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), w, h, w * 4, 4, 2, 1, 0,
                              0 if axis == "rows" else 1, fp(r), fp(g), fp(b), ctypes.byref(info))
    c = info.count
    return r[:c], g[:c], b[:c], info


def replay(path, axis="rows", quiet=False, rx=None):
    rec = Recording(path)
    rx = rx or Receiver()
    frames = len(rec); t0 = rec.frames[0][1] if frames else 0
    packets = 0; messages = []; per_frame = []; modes = []; conds = []; peaks = []; sats = []
    for k in range(frames):
        ts, gyro, accel, a = rec.frame(k)
        r, g, b, info = profiles_bgra(a, axis)
        n, msgs = rx.process(r, g, b, ts)
        packets += n; per_frame.append(n); modes.append(rx.mode); conds.append(rx.cond); peaks.append(info.peak); sats.append(info.sat_frac)
        for m in msgs:
            messages.append((round(ts - t0, 3), m[0], m[1], m[2]))
            if not quiet: print(f"  {ts - t0:6.3f}s  slot {m[0]} {m[1]:6s} {m[2]}")
    dur = rec.duration or 1e-9
    summary = {
        "file": path.split('/')[-1], "note": rec.header.get("note", ""), "frames": frames, "duration_s": round(dur, 3),
        "fps": round((frames - 1) / dur, 1) if frames > 1 else 0, "packets": packets, "pkt_per_s": round(packets / dur, 1),
        "frames_with_packets": int(sum(1 for n in per_frame if n)), "messages": len(messages),
        "rgb_frames": int(sum(modes)), "pilots": rx.pilots, "cond_mean": round(float(np.mean(conds)), 2) if conds else 0,
        "resets": int(_lib.rs_rx_resets(rx.buf)), "peak_max": int(max(peaks) if peaks else 0),
        "sat_mean": round(float(np.mean(sats)), 3) if sats else 0,
        "exposure_us": rec.header.get("exposureUs"), "iso": rec.header.get("iso"), "cam_fps": rec.header.get("fps"),
        "message_list": messages,
    }
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+"); ap.add_argument("--axis", default="rows")
    ap.add_argument("--json"); ap.add_argument("--png", nargs=2, metavar=("FRAME", "OUT")); ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    if a.png:
        from PIL import Image
        rec = Recording(a.files[0]); _, _, _, fr = rec.frame(int(a.png[0]))
        Image.fromarray(fr[:, :, [2, 1, 0]]).save(a.png[1]); print("saved", a.png[1]); return
    results = []
    for f in a.files:
        print(f"== {f.split('/')[-1]}")
        s = replay(f, a.axis, a.quiet)
        results.append(s)
        print(f"   note={s['note']!r} frames={s['frames']} fps={s['fps']} pkt/s={s['pkt_per_s']} frames_with_pkts={s['frames_with_packets']} "
              f"msgs={s['messages']} rgb_frames={s['rgb_frames']} pilots={s['pilots']} cond={s['cond_mean']} resets={s['resets']} peak={s['peak_max']} sat={s['sat_mean']}")
    if a.json:
        json.dump(results, open(a.json, 'w'), indent=1); print("wrote", a.json)


if __name__ == "__main__":
    main()
