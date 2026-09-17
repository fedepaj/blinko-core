"""Reader for .rsrec recordings written by the iOS app (Recorder.swift).

Layout: "RSREC001" | u32 headerLen | header JSON | frames:
  "FRME" | f64 timestamp | f32 gyro[3] | f32 accel[3] | u32 rawSize | u32 compSize | u8 codec | bytes
codec 1 = Apple Compression framework LZ4 (sequence of 'bv41'/'bv4-' blocks), 0 = raw.
Frames are BGRA, rows at full resolution, columns subsampled by header["columnStep"].
"""
import json, struct
import numpy as np


def _apple_lz4(buf, expected):
    import lz4.block
    out = bytearray(); i = 0
    while i + 4 <= len(buf):
        magic = bytes(buf[i:i + 4]); i += 4
        if magic == b'bv4$':
            break
        if magic == b'bv41':
            n, c = struct.unpack_from('<II', buf, i); i += 8
            # Apple's blocks are linked: matches may reference the previously decoded data
            out += lz4.block.decompress(bytes(buf[i:i + c]), uncompressed_size=n, dict=bytes(out[-65536:])); i += c
        elif magic == b'bv4-':
            n, = struct.unpack_from('<I', buf, i); i += 4
            out += buf[i:i + n]; i += n
        else:
            raise ValueError(f"bad lz4 block magic {magic!r} at {i - 4}")
    if len(out) != expected:
        raise ValueError(f"decoded {len(out)} bytes, expected {expected}")
    return bytes(out)


class Recording:
    def __init__(self, path):
        self.path = path
        with open(path, 'rb') as f:
            self.buf = f.read()
        assert self.buf[:8] == b'RSREC001', "not an rsrec file"
        hl, = struct.unpack_from('<I', self.buf, 8)
        self.header = json.loads(self.buf[12:12 + hl])
        self.width, self.height = self.header['width'], self.header['height']
        self.frames = []           # (offset, timestamp, gyro, accel, raw, comp, codec)
        i = 12 + hl
        while i + 4 <= len(self.buf) and self.buf[i:i + 4] == b'FRME':
            ts, gx, gy, gz, ax, ay, az, raw, comp = struct.unpack_from('<dffffffII', self.buf, i + 4)
            codec = self.buf[i + 4 + 8 + 24 + 8]
            data_off = i + 4 + 8 + 24 + 8 + 1
            self.frames.append((data_off, ts, (gx, gy, gz), (ax, ay, az), raw, comp, codec))
            i = data_off + comp

    def __len__(self):
        return len(self.frames)

    def frame(self, k):
        """Return (timestamp, gyro, accel, bgra ndarray (h, w, 4))."""
        off, ts, gyro, accel, raw, comp, codec = self.frames[k]
        blob = self.buf[off:off + comp]
        data = _apple_lz4(blob, raw) if codec == 1 else bytes(blob)
        a = np.frombuffer(data, dtype=np.uint8).reshape(self.height, self.width, 4)
        return ts, gyro, accel, a

    @property
    def duration(self):
        if len(self.frames) < 2: return 0.0
        return self.frames[-1][1] - self.frames[0][1]
