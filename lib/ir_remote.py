# ir_remote.py
# MicroPython IR receiver (NEC + SONY SIRC)
# Author: ChatGPT (TiCLE Lite)
# License: MIT

from machine import Pin
import time
from collections import deque

def us():
    return time.ticks_us()

def diff(a, b):
    return time.ticks_diff(a, b)

def in_range(v, target, tol=0.25):
    lo = target * (1.0 - tol)
    hi = target * (1.0 + tol)
    return lo <= v <= hi

NEC_HDR_MARK   = 9000
NEC_HDR_SPACE  = 4500
NEC_RPT_SPACE  = 2250
NEC_BIT_MARK   = 560
NEC_ZERO_SPACE = 560
NEC_ONE_SPACE  = 1690
NEC_FRAME_BITS = 32
NEC_TRAIL_MARK = 560
NEC_GAP        = 30000

SIRC_HDR_MARK  = 2400
SIRC_HDR_SPACE = 600
SIRC_ONE_MARK  = 1200
SIRC_ZERO_MARK = 600
SIRC_BIT_SPACE = 600
SIRC_GAP       = 15000


class IRFrame:
    def __init__(self, protocol, address=None, command=None, bits=None, value=None, repeat=False, raw=None):
        self.protocol = protocol
        self.address = address
        self.command = command
        self.bits = bits
        self.value = value
        self.repeat = repeat
        self.raw = raw or []

    def __repr__(self):
        return "<IRFrame proto=%s addr=%s cmd=%s bits=%s repeat=%s>" % (
            self.protocol, self.address, self.command, self.bits, self.repeat
        )


class IRReceiver:
    def __init__(self, pin, active_low=True, buf_len=256, gap_us=None, cb=None):
        self.pin = pin if isinstance(pin, Pin) else Pin(pin, Pin.IN, Pin.PULL_UP if active_low else Pin.PULL_DOWN)
        self.active_low = active_low
        self.buf = deque((),buf_len)
        self.last_edge = us()
        self.last_level = self.pin.value()
        self.gap_us = gap_us or NEC_GAP
        self.cb = cb
        self._last_nec = None
        self.pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._irq)

    def _irq(self, pin):
        t = us()
        lv = pin.value()
        dt = diff(t, self.last_edge)
        try:
            self.buf.append((dt, self.last_level))
        except IndexError:
            pass
        self.last_edge = t
        self.last_level = lv

    def _drain_frame(self):
        if not self.buf:
            return None
        local = []
        n = len(self.buf)
        gap_idx = -1
        for i in range(n):
            dt, lvl = self.buf[i]
            if dt >= self.gap_us:
                gap_idx = i
                break
        if gap_idx < 0:
            return None
        take = gap_idx
        for _ in range(take):
            local.append(self.buf.popleft())
        if self.buf:
            self.buf.popleft()
        marks_spaces = []
        for dt, lvl in local:
            is_mark = (lvl == 0) if self.active_low else (lvl == 1)
            marks_spaces.append((dt, 'M' if is_mark else 'S'))
        while marks_spaces and marks_spaces[0][1] == 'S':
            marks_spaces.pop(0)
        seq = [int(dt) for dt, _ in marks_spaces]
        if len(seq) < 4:
            return None
        return seq

    def _decode_nec(self, seq):
        if len(seq) < 4:
            return None
        mark0, space0 = seq[0], seq[1]
        if in_range(mark0, NEC_HDR_MARK) and in_range(space0, NEC_RPT_SPACE):
            rpt = IRFrame("NEC", repeat=True, raw=seq)
            if self._last_nec:
                rpt.address = self._last_nec.address
                rpt.command = self._last_nec.command
                rpt.bits = self._last_nec.bits
                rpt.value = self._last_nec.value
            return rpt
        if not (in_range(mark0, NEC_HDR_MARK) and in_range(space0, NEC_HDR_SPACE)):
            return None
        bits = []
        i = 2
        while i + 1 < len(seq):
            m = seq[i]
            s = seq[i + 1]
            if in_range(m, NEC_BIT_MARK):
                if in_range(s, NEC_ONE_SPACE):
                    bits.append(1)
                elif in_range(s, NEC_ZERO_SPACE):
                    bits.append(0)
                else:
                    if in_range(m, NEC_TRAIL_MARK) and i+1 == len(seq)-1:
                        break
                    return None
                i += 2
            else:
                if in_range(m, NEC_TRAIL_MARK) and (i == len(seq)-1):
                    break
                return None
        if len(bits) not in (NEC_FRAME_BITS,):
            return None
        value = 0
        for idx, b in enumerate(bits):
            value |= (b & 1) << idx
        addr     =  value        & 0xFF
        addr_inv = (value >> 8)  & 0xFF
        cmd      = (value >> 16) & 0xFF
        cmd_inv  = (value >> 24) & 0xFF
        frame = IRFrame("NEC",
                        address=addr,
                        command=cmd,
                        bits=len(bits),
                        value=value,
                        repeat=False,
                        raw=seq)
        self._last_nec = frame
        return frame

    def _decode_sirc(self, seq):
        if len(seq) < 4:
            return None
        mark0, space0 = seq[0], seq[1]
        if not (in_range(mark0, SIRC_HDR_MARK) and in_range(space0, SIRC_HDR_SPACE)):
            return None
        bits = []
        i = 2
        while i + 1 < len(seq):
            m = seq[i]
            s = seq[i + 1]
            if not in_range(s, SIRC_BIT_SPACE):
                return None
            if in_range(m, SIRC_ONE_MARK):
                bits.append(1)
            elif in_range(m, SIRC_ZERO_MARK):
                bits.append(0)
            else:
                return None
            i += 2
        if len(bits) not in (12, 15, 20):
            return None
        value = 0
        for idx, b in enumerate(bits):
            value |= (b & 1) << idx
        if len(bits) == 12:
            cmd = value & 0x7F
            addr = (value >> 7) & 0x1F
        elif len(bits) == 15:
            cmd = value & 0x7F
            addr = (value >> 7) & 0xFF
        else:
            cmd = value & 0x7F
            addr = (value >> 7) & 0xFFFF
        return IRFrame("SIRC", address=addr, command=cmd, bits=len(bits), value=value, raw=seq)

    def read(self, timeout_ms=0):
        t0 = time.ticks_ms()
        while True:
            seq = self._drain_frame()
            if seq:
                for decoder in (self._decode_nec, self._decode_sirc):
                    frame = decoder(seq)
                    if frame:
                        if self.cb:
                            try:
                                self.cb(frame)
                            except:
                                pass
                        return frame
                return IRFrame("UNKNOWN", raw=seq)
            if timeout_ms <= 0:
                return None
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                return None
            time.sleep_ms(1)

    def clear_buffer(self):
        self.buf.clear()

    def set_callback(self, cb):
        self.cb = cb
