"""2-D synthetic camera frames: one or more defocused LED blobs transmitting RSLog streams,
rolling-shutter exposure, Bayer-like colour mixing, optional motion (the blob slides while
the frame is read out) and saturation. Produces BGRA frames like the iPhone recorder.
"""
import numpy as np
from simulate import rolling_shutter_profile


class Blob:
    def __init__(self, chips3, cx, cy, radius, gains=(110.0, 140.0, 90.0), vx=0.0, vy=0.0, rows_per_chip=6.0, exposure_chips=0.5):
        self.chips3 = chips3          # (3, N) chip streams for R, G, B LEDs (identical rows for a mono LED)
        self.cx, self.cy, self.r = cx, cy, radius
        self.gains = np.array(gains, dtype=np.float32)
        self.vx, self.vy = vx, vy     # pixels per frame
        self.rpc, self.exp = rows_per_chip, exposure_chips
        self.phase = 0.0              # chip offset advancing frame by frame


M_CAM = np.array([[1.00, 0.12, 0.04],
                  [0.22, 1.00, 0.38],
                  [0.03, 0.10, 1.00]], dtype=np.float32)   # camera RGB response to LED R, G, B


def render(blobs, w=960, h=1080, readout_fraction=0.66, ambient=10.0, noise=2.0, sat_scale=1.0, rng=None):
    """One BGRA frame. Each blob's centre moves linearly during the readout (rows are time)."""
    rng = rng or np.random.default_rng(0)
    img = np.zeros((h, w, 3), dtype=np.float32) + ambient
    ys = np.arange(h, dtype=np.float32)[:, None]; xs = np.arange(w, dtype=np.float32)[None, :]
    for b in blobs:
        # per-row LED level for each colour (time axis = rows), including exposure smear
        chips_per_frame = h / b.rpc
        led = np.stack([rolling_shutter_profile(b.chips3[c], h, b.rpc, b.exp, b.phase, blob=(-1, 2), amplitude=1.0,
                                                ambient=0.0, noise=0.0, quantize=False, rng=rng) for c in range(3)])  # (3, h) in 0..1
        b.phase += chips_per_frame / readout_fraction          # next frame starts later (blanking included)
        # blob centre drifts along rows (motion during the readout)
        cx_row = b.cx + b.vx * readout_fraction * (ys - h / 2) / h
        cy_row = b.cy + b.vy * readout_fraction * (ys - h / 2) / h
        d2 = (xs - cx_row) ** 2 + (ys - cy_row) ** 2
        disc = 1.0 / (1.0 + np.exp((np.sqrt(d2) - b.r) / (0.06 * b.r)))    # soft-edged disc
        for cam in range(3):
            resp = sum(M_CAM[cam, k] * b.gains[k] * led[k] for k in range(3))    # (h,)
            img[:, :, cam] += disc * resp[:, None] * sat_scale
        b.cx += b.vx; b.cy += b.vy
    img += rng.normal(0, noise, img.shape)
    img = np.clip(img, 0, 255).astype(np.uint8)
    return img[:, :, ::-1].copy()   # BGR order like the recorder (BGRA without alpha: add it)


def to_bgra(bgr):
    h, w, _ = bgr.shape
    out = np.zeros((h, w, 4), dtype=np.uint8); out[:, :, :3] = bgr; out[:, :, 3] = 255
    return out
