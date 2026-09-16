"""Rolling-shutter camera model: chip stream -> per-row brightness profile.

Row r integrates the LED signal over [r*t_row + t_off, r*t_row + t_off + t_exp].
All times are expressed in chips (T_chip = 1).
"""
import numpy as np


def rolling_shutter_profile(chips, rows, rows_per_chip, exposure_chips, offset_chips,
                            blob=(0.2, 0.8), amplitude=120.0, ambient=12.0, noise=2.0,
                            edge_softness=0.03, quantize=True, rng=None):
    """chips: 0/1 array (periodic stream). Returns float32 profile of length `rows`.

    blob: (start, end) as fractions of the frame height covered by the defocused LED.
    edge_softness: width of the blob's soft edge (fraction of frame height).
    """
    rng = rng or np.random.default_rng(0)
    oversample = 8
    n_chips = len(chips)
    # fine time grid in chips, dt = 1/(rows_per_chip*oversample)
    dt = 1.0 / (rows_per_chip * oversample)
    t_end = rows / rows_per_chip + exposure_chips + offset_chips + 2
    n_fine = int(np.ceil(t_end / dt)) + 2
    t_fine = np.arange(n_fine) * dt
    L = chips[(t_fine.astype(np.int64)) % n_chips]
    C = np.concatenate([[0.0], np.cumsum(L) * dt])

    def integ(x):
        i = np.clip((x / dt).astype(np.int64), 0, n_fine - 1)
        frac = x / dt - i
        return C[i] + frac * L[np.minimum(i, n_fine - 1)] * dt

    r = np.arange(rows)
    t_start = r / rows_per_chip + offset_chips
    exp = max(exposure_chips, dt)
    led = (integ(t_start + exp) - integ(t_start)) / exp  # 0..1 mean LED level during exposure

    x = r / rows
    s = max(edge_softness, 1e-6)
    envelope = 0.5 * (np.tanh((x - blob[0]) / s) - np.tanh((x - blob[1]) / s))
    p = ambient + amplitude * envelope * led + rng.normal(0, noise, rows)
    if quantize:
        p = np.clip(np.round(p), 0, 255)
    return p.astype(np.float32)


def true_packet_at(row, rows_per_chip, offset_chips, packets):
    """Map a decoded packet's row_start (gap chip) to the ground-truth (id, seed, payload).
    Returns (id, seed, payload, phase, chip_phase_error)."""
    chip = (row / rows_per_chip + offset_chips)
    k = int(np.round(chip))
    n = len(packets)
    from rscore import RS_PKT_CHIPS
    idx = (k // RS_PKT_CHIPS) % n
    phase = k % RS_PKT_CHIPS
    return packets[idx][0], packets[idx][1], packets[idx][2], phase, abs(chip - k)
