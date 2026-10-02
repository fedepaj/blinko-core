"""ctypes wrapper around the portable C core (core/). Builds it on first use."""
import ctypes, os, subprocess, sys, hashlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the blinko-core repo
CORE = ROOT
BUILD = os.path.join(ROOT, "build")
SRCS = ["rs_tx.c", "rs_decoder.c", "rs_assembler.c", "rs_pack.c", "rs_rgb.c", "rs_rx.c", "rs_frame.c", "rs_multi.c", "rs_stitch.c"]

RS_PKT_CHIPS = 82
RS_NUM_SLOTS = 8
RS_MSG_MAX_LEN = 31
RS_SEED_META = 511
RS_SEED_META_PACKED = 510
RS_TEXT_MAX = 64
LEVELS = ["DEBUG", "INFO", "WARN", "ERROR", "FATAL", "STATUS", "FAULT", "RSVD"]


def _build():
    os.makedirs(BUILD, exist_ok=True)
    h = hashlib.sha1()
    for f in sorted(os.listdir(CORE)):
        if not (f.endswith(".c") or f.endswith(".h")):
            continue
        with open(os.path.join(CORE, f), "rb") as fh:
            h.update(fh.read())
    extra = os.environ.get("RS_CFLAGS", "").split()
    h.update(" ".join(extra).encode())
    lib = os.path.join(BUILD, f"librscore-{h.hexdigest()[:10]}.dylib")
    if not os.path.exists(lib):
        cmd = ["cc", "-std=c99", "-O2", "-fPIC", "-shared", "-o", lib] + extra + [os.path.join(CORE, s) for s in SRCS]
        subprocess.check_call(cmd)
    return lib


class Slot(ctypes.Structure):
    _fields_ = [("valid", ctypes.c_uint8), ("len", ctypes.c_uint8), ("level", ctypes.c_uint8), ("packed", ctypes.c_uint8),
                ("next_seed", ctypes.c_uint16), ("seq", ctypes.c_uint32), ("data", ctypes.c_uint8 * RS_MSG_MAX_LEN)]


class Tx(ctypes.Structure):
    _fields_ = [("slots", Slot * RS_NUM_SLOTS), ("seq_counter", ctypes.c_uint32), ("next_log_slot", ctypes.c_uint8),
                ("round", ctypes.c_uint8 * (RS_NUM_SLOTS * 4)), ("fault_weight", ctypes.c_uint8), ("round_len", ctypes.c_uint8), ("round_pos", ctypes.c_uint8),
                ("cur", Slot), ("cur_id", ctypes.c_uint8), ("cur_sent", ctypes.c_uint8), ("visit_len", ctypes.c_uint8),
                ("burst_on", ctypes.c_uint32), ("burst_off", ctypes.c_uint32), ("burst_pos", ctypes.c_uint32), ("in_pause", ctypes.c_uint8),
                ("nchan", ctypes.c_uint8), ("pilot_period", ctypes.c_uint32), ("pilot_pos", ctypes.c_uint32), ("in_pilot", ctypes.c_uint8), ("pilot_idx", ctypes.c_uint32),
                ("chips", (ctypes.c_uint8 * RS_PKT_CHIPS) * 3), ("chip_pos", ctypes.c_uint8 * 3), ("packets_sent", ctypes.c_uint32),
                ("repeat", ctypes.c_uint8), ("rep_left", ctypes.c_uint8 * 3)]


class DecCfg(ctypes.Structure):
    _fields_ = [("min_rows_per_chip", ctypes.c_float), ("max_rows_per_chip", ctypes.c_float), ("sync_tol", ctypes.c_float), ("min_contrast", ctypes.c_float),
                ("track_timing", ctypes.c_int), ("min_quality", ctypes.c_float), ("rows_per_chip_hint", ctypes.c_float),
                ("timing_retries", ctypes.c_int), ("grid_decode", ctypes.c_int), ("exposure_rows", ctypes.c_float),
                ("parallel", ctypes.c_void_p), ("parallel_user", ctypes.c_void_p)]


class Packet(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint8), ("seed", ctypes.c_uint16), ("payload", ctypes.c_uint8),
                ("row_start", ctypes.c_float), ("row_end", ctypes.c_float),
                ("rows_per_chip", ctypes.c_float), ("quality", ctypes.c_float), ("amplitude", ctypes.c_float)]


class Stats(ctypes.Structure):
    _fields_ = [("syncs", ctypes.c_int), ("crc_ok", ctypes.c_int), ("crc_fail", ctypes.c_int),
                ("truncated", ctypes.c_int),
                ("rows_per_chip", ctypes.c_float), ("contrast", ctypes.c_float)]


class AsmSlot(ctypes.Structure):
    _fields_ = [("have_meta", ctypes.c_uint8), ("len", ctypes.c_uint8), ("level", ctypes.c_uint8), ("packed", ctypes.c_uint8),
                ("delivered", ctypes.c_uint8), ("have_crc", ctypes.c_uint8), ("msg_crc", ctypes.c_uint8), ("have_crc2", ctypes.c_uint8), ("msg_crc2", ctypes.c_uint8),
                ("rank", ctypes.c_uint8), ("pivot_have", ctypes.c_uint32),
                ("pivot_mask", ctypes.c_uint32 * 32), ("pivot_val", ctypes.c_uint8 * 32),
                ("pend_seed", ctypes.c_uint16 * 24), ("pend_val", ctypes.c_uint8 * 24), ("npend", ctypes.c_uint8),
                ("data", ctypes.c_uint8 * RS_MSG_MAX_LEN),
                ("raw_mask", ctypes.c_uint32 * 56), ("raw_val", ctypes.c_uint8 * 56), ("nraw", ctypes.c_uint8), ("raw_head", ctypes.c_uint8),
                ("packets", ctypes.c_uint32), ("resets", ctypes.c_uint32), ("recovered", ctypes.c_uint32), ("contradictions", ctypes.c_uint8), ("meta_strikes", ctypes.c_uint8)]


class Asm(ctypes.Structure):
    _fields_ = [("slots", AsmSlot * RS_NUM_SLOTS), ("packets_total", ctypes.c_uint32), ("messages_total", ctypes.c_uint32)]


class Message(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint8), ("level", ctypes.c_uint8), ("len", ctypes.c_uint8),
                ("text", ctypes.c_char * RS_TEXT_MAX)]


_lib = ctypes.CDLL(os.environ.get("RS_LIB") or _build())   # RS_LIB: a cached build, for A/B comparisons
_lib.rs_tx_init.argtypes = [ctypes.POINTER(Tx)]
_lib.rs_tx_log.argtypes = [ctypes.POINTER(Tx), ctypes.c_uint8, ctypes.c_char_p, ctypes.c_size_t]
_lib.rs_tx_log.restype = ctypes.c_uint8
_lib.rs_tx_set_slot.argtypes = [ctypes.POINTER(Tx), ctypes.c_uint8, ctypes.c_uint8, ctypes.c_char_p, ctypes.c_size_t]
_lib.rs_tx_next_packet.argtypes = [ctypes.POINTER(Tx), ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(ctypes.c_uint8)]
_lib.rs_tx_set_burst.argtypes = [ctypes.POINTER(Tx), ctypes.c_uint32, ctypes.c_uint32]
_lib.rs_tx_set_channels.argtypes = [ctypes.POINTER(Tx), ctypes.c_uint8, ctypes.c_uint32]
_lib.rs_tx_set_repeat.argtypes = [ctypes.POINTER(Tx), ctypes.c_uint8]
_lib.rs_tx_next_chips.argtypes = [ctypes.POINTER(Tx), ctypes.POINTER(ctypes.c_uint8)]
_lib.rs_rx_sizeof.restype = ctypes.c_size_t
_lib.rs_rx_init.argtypes = [ctypes.c_void_p]
_lib.rs_rx_process.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.c_float]
_lib.rs_rx_process.restype = ctypes.c_int
class Camera(ctypes.Structure):
    _fields_ = [("exposure_rows", ctypes.c_float), ("row_seconds", ctypes.c_float)]
for _name, _args, _ret in (("rs_rx_set_camera", [ctypes.c_void_p, Camera], None), ("rs_multi_set_camera", [ctypes.c_void_p, Camera], None), ("rs_rx_set_parallel", [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p], None),
                           ("rs_rx_stitched", [ctypes.c_void_p], ctypes.c_uint32)):
    if hasattr(_lib, _name): getattr(_lib, _name).argtypes = _args; getattr(_lib, _name).restype = _ret   # absent in older builds loaded via RS_LIB
_lib.rs_rx_pop_message.argtypes = [ctypes.c_void_p, ctypes.POINTER(Message)]
_lib.rs_rx_pop_message.restype = ctypes.c_int
for name, rt in (("rs_rx_mode", ctypes.c_int), ("rs_rx_rows_per_chip", ctypes.c_float), ("rs_rx_pilots", ctypes.c_int), ("rs_rx_cal_cond", ctypes.c_float), ("rs_rx_packets", ctypes.c_uint32), ("rs_rx_messages", ctypes.c_uint32)):
    getattr(_lib, name).argtypes = [ctypes.c_void_p]; getattr(_lib, name).restype = rt
_lib.rs_pack6.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t]
_lib.rs_pack6.restype = ctypes.c_size_t
_lib.rs_unpack6.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t, ctypes.c_char_p, ctypes.c_size_t]
_lib.rs_unpack6.restype = ctypes.c_size_t
_lib.rs_tx_next_chip.argtypes = [ctypes.POINTER(Tx)]
_lib.rs_tx_next_chip.restype = ctypes.c_uint8
_lib.rs_dec_cfg_default.argtypes = [ctypes.POINTER(DecCfg)]
_lib.rs_decode_profile.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.POINTER(DecCfg),
                                   ctypes.POINTER(Packet), ctypes.c_int, ctypes.POINTER(Stats)]
_lib.rs_decode_profile.restype = ctypes.c_int
_lib.rs_asm_init.argtypes = [ctypes.POINTER(Asm)]
_lib.rs_asm_sizeof.restype = ctypes.c_size_t
_lib.rs_asm_recovered.argtypes = [ctypes.POINTER(Asm)]; _lib.rs_asm_recovered.restype = ctypes.c_uint32
assert ctypes.sizeof(Asm) == _lib.rs_asm_sizeof(), f"rs_asm_t layout mismatch: python {ctypes.sizeof(Asm)} vs C {_lib.rs_asm_sizeof()}"
_lib.rs_asm_feed.argtypes = [ctypes.POINTER(Asm), ctypes.POINTER(Packet), ctypes.POINTER(Message)]
_lib.rs_asm_feed.restype = ctypes.c_int


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc12(v: int, nbits: int) -> int:
    """CRC-12 (poly 0x80F, init 0xFFF) over the nbits low bits of v, as rs_crc12_bits."""
    crc = 0xFFF
    for i in range(nbits - 1, -1, -1):
        inb = ((v >> i) & 1) ^ ((crc >> 11) & 1)
        crc = (crc << 1) & 0xFFF
        if inb: crc ^= 0x80F
    return crc


def crc_fields(id_: int, seed: int, payload: int) -> int:
    return crc12(((id_ & 7) << 15) | ((seed & 127) << 8) | (payload & 255), 18)


_lib.rs_tx_encode.argtypes = [ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint8)]
_lib.rs_tx_encode.restype = ctypes.c_int


def encode_packet(id_: int, seed: int, payload: int) -> list:
    """Chips of one v3 packet, from the C encoder (RLL(2,7), sync, CRC-12)."""
    buf = (ctypes.c_uint8 * RS_PKT_CHIPS)()
    n = _lib.rs_tx_encode(id_, seed, payload, buf)
    return list(buf[:n])


def pack6(text: str) -> bytes:
    buf = (ctypes.c_uint8 * 64)()
    n = _lib.rs_pack6(text.encode(), len(text.encode()), buf, 64)
    return bytes(buf[:n])


def unpack6(data: bytes) -> str:
    buf = (ctypes.c_uint8 * len(data))(*data)
    out = ctypes.create_string_buffer(64)
    _lib.rs_unpack6(buf, len(data), out, 64)
    return out.value.decode(errors="replace")


class Transmitter:
    def __init__(self):
        self.tx = Tx()
        _lib.rs_tx_init(ctypes.byref(self.tx))

    def log(self, level: int, text: str) -> int:
        b = text.encode()
        return _lib.rs_tx_log(ctypes.byref(self.tx), level, b, len(b))

    def set_slot(self, slot: int, level: int, text: str):
        b = text.encode()
        _lib.rs_tx_set_slot(ctypes.byref(self.tx), slot, level, b, len(b))

    def next_packet(self):
        i, s, p = ctypes.c_uint8(), ctypes.c_uint16(), ctypes.c_uint8()
        _lib.rs_tx_next_packet(ctypes.byref(self.tx), ctypes.byref(i), ctypes.byref(s), ctypes.byref(p))
        return i.value, s.value, p.value

    def set_fault_weight(self, w: int):
        _lib.rs_tx_set_fault_weight(ctypes.byref(self.tx), w)

    def set_burst(self, on_chips: int, off_chips: int):
        _lib.rs_tx_set_burst(ctypes.byref(self.tx), on_chips, off_chips)

    def set_repeat(self, n: int): _lib.rs_tx_set_repeat(ctypes.byref(self.tx), n)
    def set_channels(self, nchan: int, pilot_period: int):
        _lib.rs_tx_set_channels(ctypes.byref(self.tx), nchan, pilot_period)

    def chips3(self, n: int):
        """n ticks of the 3-channel chip source -> ndarray (3, n)."""
        import numpy as np
        out = np.zeros((3, n), dtype=np.float32); buf = (ctypes.c_uint8 * 3)()
        for i in range(n):
            _lib.rs_tx_next_chips(ctypes.byref(self.tx), buf)
            out[0, i], out[1, i], out[2, i] = buf[0], buf[1], buf[2]
        return out

    def next_chip(self) -> int:
        return _lib.rs_tx_next_chip(ctypes.byref(self.tx))

    def stream(self, n_packets: int):
        """Return (chips ndarray, list of (id, seed, payload)) for n_packets consecutive packets."""
        import numpy as np
        pk, chips = [], []
        for _ in range(n_packets):
            t = self.next_packet()
            pk.append(t)
            chips += encode_packet(*t)
        return np.array(chips, dtype=np.float32), pk


class Decoder:
    def __init__(self, **overrides):
        self.cfg = DecCfg()
        _lib.rs_dec_cfg_default(ctypes.byref(self.cfg))
        for k, v in overrides.items():
            setattr(self.cfg, k, v)
        self.stats = Stats()

    def decode(self, profile, max_out=64):
        import numpy as np
        p = np.ascontiguousarray(profile, dtype=np.float32)
        out = (Packet * max_out)()
        n = _lib.rs_decode_profile(p.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), len(p),
                                   ctypes.byref(self.cfg), out, max_out, ctypes.byref(self.stats))
        return [out[i] for i in range(n)]


class Assembler:
    def __init__(self):
        self.a = Asm()
        _lib.rs_asm_init(ctypes.byref(self.a))

    @property
    def recovered(self): return _lib.rs_asm_recovered(ctypes.byref(self.a))

    @property
    def resets(self): return sum(self.a.slots[i].resets for i in range(RS_NUM_SLOTS))

    def feed(self, pkt: Packet):
        m = Message()
        if _lib.rs_asm_feed(ctypes.byref(self.a), ctypes.byref(pkt), ctypes.byref(m)):
            return (m.id, LEVELS[m.level], m.text.decode(errors="replace"))
        return None


class Receiver:
    """Complete C receiver (rs_rx): calibration, unmixing, decoding, assembly."""
    def __init__(self):
        self.buf = ctypes.create_string_buffer(_lib.rs_rx_sizeof())
        _lib.rs_rx_init(self.buf)

    @property
    def cfg(self): return DecCfg.from_address(ctypes.addressof(self.buf))   # cfg is the first field of rs_rx_t
    def set_camera(self, exposure_rows=0.0, row_seconds=0.0): _lib.rs_rx_set_camera(self.buf, Camera(exposure_rows, row_seconds))
    def set_row_time(self, seconds): self.set_camera(self.cfg.exposure_rows, seconds)
    def set_parallel_off(self): _lib.rs_rx_set_parallel(self.buf, None, None)
    @property
    def stitched(self): return _lib.rs_rx_stitched(self.buf)

    def process(self, r, g=None, b=None, t=0.0):
        import numpy as np
        r = np.ascontiguousarray(r, dtype=np.float32)
        pr = r.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        if g is None:
            n = _lib.rs_rx_process(self.buf, pr, None, None, len(r), t)
        else:
            g = np.ascontiguousarray(g, dtype=np.float32); b = np.ascontiguousarray(b, dtype=np.float32)
            n = _lib.rs_rx_process(self.buf, pr, g.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), b.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), len(r), t)
        msgs = []
        m = Message()
        while _lib.rs_rx_pop_message(self.buf, ctypes.byref(m)):
            msgs.append((m.id, LEVELS[m.level], m.text.decode(errors="replace")))
        return n, msgs

    @property
    def mode(self): return _lib.rs_rx_mode(self.buf)
    @property
    def pilots(self): return _lib.rs_rx_pilots(self.buf)
    @property
    def cond(self): return _lib.rs_rx_cal_cond(self.buf)
    @property
    def packets(self): return _lib.rs_rx_packets(self.buf)


_lib.rs_multi_sizeof.restype = ctypes.c_size_t
_lib.rs_multi_init.argtypes = [ctypes.c_void_p]
_lib.rs_multi_process.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8)] + [ctypes.c_int] * 7 + [ctypes.c_float]
_lib.rs_multi_process.restype = ctypes.c_int
_lib.rs_multi_pop_message.argtypes = [ctypes.c_void_p, ctypes.POINTER(Message), ctypes.POINTER(ctypes.c_int)]
_lib.rs_multi_pop_message.restype = ctypes.c_int
_lib.rs_multi_track_count.argtypes = [ctypes.c_void_p]; _lib.rs_multi_track_count.restype = ctypes.c_int
_lib.rs_multi_track_info.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
                                     ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_int)]
_lib.rs_multi_track_info.restype = ctypes.c_int


class Multi:
    """Multi-source receiver: segmentation + tracking + one receiver per light."""
    def __init__(self):
        self.buf = ctypes.create_string_buffer(_lib.rs_multi_sizeof())
        _lib.rs_multi_init(self.buf)

    def set_camera(self, exposure_rows=0.0, row_seconds=0.0): _lib.rs_multi_set_camera(self.buf, Camera(exposure_rows, row_seconds))

    def process(self, bgra, t=0.0):
        import numpy as np
        a = np.ascontiguousarray(bgra); h, w, ch = a.shape
        n = _lib.rs_multi_process(self.buf, a.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), w, h, w * ch, ch, 2, 1, 0, t)
        msgs = []; m = Message(); tid = ctypes.c_int()
        while _lib.rs_multi_pop_message(self.buf, ctypes.byref(m), ctypes.byref(tid)):
            msgs.append((tid.value, m.id, LEVELS[m.level], m.text.decode(errors="replace")))
        return n, msgs

    def tracks(self):
        out = []
        for i in range(_lib.rs_multi_track_count(self.buf)):
            tid = ctypes.c_int(); cx = ctypes.c_float(); cy = ctypes.c_float(); rad = ctypes.c_float(); mode = ctypes.c_int()
            pk = ctypes.c_uint32(); ms = ctypes.c_uint32(); pil = ctypes.c_int()
            if _lib.rs_multi_track_info(self.buf, i, ctypes.byref(tid), ctypes.byref(cx), ctypes.byref(cy), ctypes.byref(rad), ctypes.byref(mode), ctypes.byref(pk), ctypes.byref(ms), ctypes.byref(pil)):
                out.append(dict(id=tid.value, group=_lib.rs_multi_track_group(self.buf, i), cx=round(cx.value), cy=round(cy.value), radius=round(rad.value), mode="rgb" if mode.value else "luma", packets=pk.value, messages=ms.value, pilots=pil.value))
        return out
