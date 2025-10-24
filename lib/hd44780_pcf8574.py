__version__ = "1.0.0"
__author__ = "PlanX Lab Development Team"

from ticle import (
    utime,
    micropython,
    I2c
)

class HD44780_PCF8574:
    MODE_TEXT = 0
    MODE_GFX  = 1

    def __init__(self, scl:int, sda:int, *, addr:int=0x3f, freq:int=400_000, cols:int=16, rows:int=2):
        self.__i2c = I2c(sda=sda, scl=scl, addr=addr, freq=freq)

        self.__cols = int(cols)
        self.__rows = int(rows)

        self.__display = True
        self.__cursor = False
        self.__blink = False
        self.__x = 0
        self.__y = 0
        self.__dwidth = 40  # DDRAM width
        self.__shift = 0
        self.__ax = 0
        self.__ay = 0
        self.__tx4 = bytearray(4)

        self.__bar_row_mask = 0xFF
        self.__bar_cfg = None          # (rtl_bool, row_mask) cache
        
        self.__gw = self.__cols * 5
        self.__gh = self.__rows * 8
        self.__gfb = bytearray(self.__gw * self.__gh)

        self.__cb_slots = [None] * 8     # slot -> pattern(tuple-of-8-rows)
        self.__cb_map = {}               # pattern(tuple) -> slot
        
        self.__i2c.writeto(bytes([0x00]))
        utime.sleep_us(20_000)
        for _ in range(3):
            self.__i2c.writeto(bytes((0x30 | 0x04, 0x30)))  # EN pulse packed
            utime.sleep_us(5_000)
        self.__i2c.writeto(bytes((0x20 | 0x04, 0x20)))
        utime.sleep_us(1_000)

        self.__cmd(0x20 | (0x08 if self.__rows > 1 else 0x00))
        self.set_display(False)
        self.clear()
        self.__cmd(0x04 | 0x02)
        
        self.set_display(True, cursor=False, blink=False)
        self.__mode = self.MODE_TEXT

    def deinit(self):
        self.set_display(cursor=False, blink=False)
        self.clear()

    @property
    def mode(self):
        return self.__mode

    @mode.setter
    def mode(self, n):
        self.__mode = n

    def __write(self, data: int, rs: int):
        b0 = ((rs & 0x01) | (data & 0xF0))
        b1 = ((rs & 0x01) | ((data & 0x0F) << 4))
        tx = self.__tx4
        tx[0] = b0 | 0x04
        tx[1] = b0
        tx[2] = b1 | 0x04
        tx[3] = b1
        self.__i2c.writeto(tx)

        if rs == 0 and data <= 3:
            utime.sleep_us(5_000)

    def __cmd(self, c: int):
        self.__write(c & 0xFF, 0)

    def __data(self, d: int):
        self.__write(d & 0xFF, 0x01)

    def __move_to(self, x:int, y:int):
        ROW_ADDR = (0x00, 0x40, 0x14, 0x54)
        x = max(0, min(self.__cols-1, int(x)))
        y = max(0, min(self.__rows-1, int(y)))
        self.__x, self.__y = x, y
        real_col = (self.__shift + x) % self.__dwidth
        pos = 0x80 | (ROW_ADDR[y] + real_col)
        self.__cmd(pos)

    def __ensure_bar_tiles(self, rtl:bool):
        key = (True if rtl else False, self.__bar_row_mask)
        if self.__bar_cfg == key:
            return
        for seg in range(6):
            px = seg
            if px <= 0:    colmask = 0x00
            elif px >= 5:  colmask = 0x1F
            else:
                colmask = ((0x1F << (5 - px)) & 0x1F) if not rtl else (0x1F >> (5 - px))
            rows = []
            m = self.__bar_row_mask
            for rr in range(8):
                use = (m >> (7 - rr)) & 1
                rows.append(colmask if use else 0)
            self.create_char(seg, rows)
        self.__bar_cfg = key

    def __cell_pattern(self, cx:int, cy:int):
        x0 = cx*5; y0 = cy*8; gw = self.__gw; fb = self.__gfb
        rows = []
        for rr in range(8):
            b = 0; base = (y0+rr)*gw + x0
            if fb[base+0]: b |= 0x10
            if fb[base+1]: b |= 0x08
            if fb[base+2]: b |= 0x04
            if fb[base+3]: b |= 0x02
            if fb[base+4]: b |= 0x01
            rows.append(b)
        return tuple(rows)

    def __set_fb(self, x:int, y:int, v:int):
        if 0 <= x < self.__gw and 0 <= y < self.__gh:
            self.__gfb[y * self.__gw + x] = 1 if v else 0

    def __cb_assign(self, need_patterns):
        prev_slots = list(self.__cb_slots)
        prev_map   = dict(self.__cb_map)

        new_slots = list(prev_slots)
        new_map   = dict(prev_map)

        for p in list(new_map.keys()):
            if p not in need_patterns:
                s = new_map.pop(p)
                new_slots[s] = None

        free_slots = [s for s in range(8) if new_slots[s] is None]

        for p in need_patterns:
            if p in new_map:
                continue
            if not free_slots:
                break
            s = free_slots.pop(0)
            new_slots[s] = p
            new_map[p]   = s

        changed = []
        for s in range(8):
            if new_slots[s] != prev_slots[s] and new_slots[s] is not None:
                changed.append((s, new_slots[s]))

        return new_slots, new_map, changed


    def __cb_slot_of(self, pattern):
        s = self.__cb_map.get(pattern, None)
        if s is not None:
            return s
        # nearest (hamming) fallback
        best_s, best_score = 0, 1_000
        for si, p in enumerate(self.__cb_slots):
            if p is None:
                continue
            score = 0
            for a, b in zip(pattern, p):
                x = a ^ b
                cnt = (x & 1) + ((x>>1)&1) + ((x>>2)&1) + ((x>>3)&1) + ((x>>4)&1)
                score += cnt
            if score < best_score:
                best_score, best_s = score, si
        return best_s

    @micropython.native
    def clear(self):
        self.__cmd(0x01)
        self.__x = self.__ax = 0
        self.__y = self.__ay = 0
        self.__shift = 0

    def home(self):
        self.__cmd(0x02)
        self.__x = self.__ax = 0
        self.__y = self.__ay = 0
        self.__shift = 0

    def set_display(self, on: bool|None = None, cursor: bool|None = None, blink: bool|None = None):
        if on is None: on = self.__display
        else: self.__display = on
        if cursor is None: cursor = self.__cursor
        else: self.__cursor = cursor
        if blink is None: blink = self.__blink
        else: self.__blink = blink
        self.__cmd(0x08 | (0x04 if on else 0) | (0x02 if cursor else 0) | (0x01 if blink else 0))

    def text(self, text: str|bytes, x: int|None = None, y: int|None = None, *, wrap: bool = False):
        ROW_ADDR = (0x00, 0x40, 0x14, 0x54)
        if isinstance(text, (bytes, bytearray)):
            text = text.decode()
        if x is not None and y is not None:
            self.__move_to(x, y)
            if not wrap:
                self.__ay = y
                self.__ax = (self.__shift + x) % self.__dwidth
        for ch in str(text):
            if ch == '\n':
                self.__ay = (self.__ay + 1) % self.__rows
                self.__ax = 0
                self.__move_to(0, self.__ay)
                continue
            oc = ord(ch)
            if wrap:
                self.__data(oc)
                self.__x += 1
                if self.__x >= self.__cols:
                    self.__x = 0
                    self.__y = (self.__y + 1) % self.__rows
                    self.__move_to(self.__x, self.__y)
            else:
                if self.__ax < self.__dwidth:
                    real_col = self.__ax % self.__dwidth
                    pos = 0x80 | (ROW_ADDR[self.__ay] + real_col)
                    self.__cmd(pos)
                    self.__data(oc)
                    if self.__ax < self.__cols:
                        self.__x = self.__ax; self.__y = self.__ay
                    else:
                        self.__x = self.__cols - 1; self.__y = self.__ay
                    self.__ax += 1
                else:
                    break

    def scroll_left(self, n:int=1):
        for _ in range(max(1, n)):
            self.__cmd(0x10 | 0x08 | 0x00)
            self.__shift = (self.__shift + 1) % self.__dwidth

    def scroll_right(self, n:int=1):
        for _ in range(max(1, n)):
            self.__cmd(0x10 | 0x08 | 0x04)
            self.__shift = (self.__shift - 1) % self.__dwidth

    def create_char(self, slot:int, pattern8):
        slot &= 7
        if len(pattern8) != 8: raise ValueError("pattern8 must have 8 rows")
        self.__cmd(0x40 | (slot << 3))
        utime.sleep_us(40)
        for v in pattern8:
            self.__data(int(v) & 0x1F)
            utime.sleep_us(40)
        self.__move_to(self.__x, self.__y)

    @property
    def bar_patterns(self) -> int:
        return self.__bar_row_mask

    @bar_patterns.setter
    def bar_patterns(self, mask:int):
        self.__bar_row_mask = int(mask) & 0xFF
        self.__bar_cfg = None

    def bar(self, row:int, value:int, *, max_value:int=100, start_col:int=0, end_col:int=-1):
        if self.__mode == self.MODE_GFX:
            raise RuntimeError("Cannot use bar() in Graphics mode")

        r = 0 if row < 0 else (self.__rows-1 if row >= self.__rows else row)

        def norm_col(c:int) -> int:
            if c == -1: return self.__cols - 1
            if c < 0:   return 0
            if c >= self.__cols: return self.__cols - 1
            return c

        sc = norm_col(start_col)
        ec = norm_col(end_col)

        rtl = sc > ec
        step = -1 if rtl else +1
        cells = abs(ec - sc) + 1
        if cells <= 0:
            return

        total_px = cells * 5
        if max_value <= 0: max_value = 1
        if value < 0: value = 0
        if value > max_value: value = max_value
        filled_px = int(value * total_px / max_value)

        self.__ensure_bar_tiles(rtl)

        for i in range(cells):
            px = filled_px - i*5
            if px < 0: px = 0
            if px > 5: px = 5
            ch = px
            col = sc + i*step
            self.__move_to(col, r)
            self.__data(ch)

    @property
    def g_width(self) -> int:
        return self.__gw

    @property
    def g_height(self) -> int:
        return self.__gh

    def g_clear(self, on=False):
        v = 1 if on else 0
        fb = self.__gfb
        for i in range(len(fb)):
            fb[i] = v

    def g_point(self, x:int, y:int, on=True):
        self.__set_fb(x, y, 1 if on else 0)

    def g_rect(self, x:int, y:int, w:int, h:int, fill=False, on=True):
        if w <= 0 or h <= 0: 
            return
        x1, y1 = x + w - 1, y + h - 1
        gw, gh = self.__gw, self.__gh
        if x1 < 0 or y1 < 0 or x >= gw or y >= gh:
            return
        x  = 0 if x  < 0 else x
        y  = 0 if y  < 0 else y
        x1 = gw - 1 if x1 >= gw else x1
        y1 = gh - 1 if y1 >= gh else y1
        v = 1 if on else 0
        fb = self.__gfb

        if fill:
            span = x1 - x + 1
            for yy in range(y, y1+1):
                base = yy * gw + x
                for off in range(span):
                    fb[base + off] = v
        else:
            for xx in range(x, x1+1):
                fb[y  * gw + xx] = v
                fb[y1 * gw + xx] = v
            for yy in range(y, y1+1):
                fb[yy * gw + x ] = v
                fb[yy * gw + x1] = v

    def g_line(self, x0:int, y0:int, x1:int, y1:int, on=True):
        v = 1 if on else 0
        gw, gh = self.__gw, self.__gh
        dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            if 0 <= x0 < gw and 0 <= y0 < gh:
                self.__gfb[y0 * gw + x0] = v
            if x0 == x1 and y0 == y1:
                break
            e2 = err << 1
            if e2 >= dy:
                err += dy; x0 += sx
            if e2 <= dx:
                err += dx; y0 += sy


    # --- Hamming helpers for 5x8 tiles (popcount LUT for 0..31) ---
    __POP5 = bytes((0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5))

    def __pat_dist(self, a, b):
        pop = self.__POP5
        s = 0
        for i in range(8):
            s += pop[(a[i] ^ b[i]) & 31]
        return s

    def __select_k_prototypes(self, uniq_patterns, counts, k=8):
        n = len(uniq_patterns)
        if n <= k:
            return list(uniq_patterns)

        dist = [[0]*n for _ in range(n)]
        for i in range(n):
            ai = uniq_patterns[i]
            di = dist[i]
            for j in range(i+1, n):
                d = self.__pat_dist(ai, uniq_patterns[j])
                di[j] = d
                dist[j][i] = d

        w = [counts[uniq_patterns[i]] for i in range(n)]

        first = max(range(n), key=lambda i: w[i])
        centers = [first]
        best_d = dist[first][:]

        for _ in range(1, k):
            best_gain, best_j = -1, None
            for j in range(n):
                if j in centers:
                    continue
                dj = dist[j]
                gain = 0

                for i in range(n):
                    d = dj[i]
                    bd = best_d[i]
                    if d < bd:
                        gain += (bd - d) * w[i]
                if gain > best_gain:
                    best_gain, best_j = gain, j

            if best_j is None:
                break
            centers.append(best_j)
            dj = dist[best_j]
            for i in range(n):
                if dj[i] < best_d[i]:
                    best_d[i] = dj[i]

        return [uniq_patterns[i] for i in centers]

    def g_update(self):
        if self.__mode != self.MODE_GFX:
            raise RuntimeError("Cannot use g_update() in Text mode")

        cols, rows = self.__cols, self.__rows

        counts = {}
        tiles  = []
        for r in range(rows):
            for c in range(cols):
                pat = self.__cell_pattern(c, r)
                tiles.append(pat)
                counts[pat] = counts.get(pat, 0) + 1

        uniq = list(counts.keys())
        if len(uniq) <= 8:
            prototypes = uniq
        else:
            prototypes = self.__select_k_prototypes(uniq, counts, 8)

        while len(prototypes) < 8:
            prototypes.append((0,0,0,0,0,0,0,0))

        new_slots, new_map, changed = self.__cb_assign(prototypes)

        prev_disp = self.__display
        if changed and prev_disp:
            self.set_display(on=False)

        for s, p in changed:
            self.create_char(s, p)

        if changed and prev_disp:
            self.set_display(on=True)

        self.__cb_slots = new_slots
        self.__cb_map   = new_map

        idx = 0
        for r in range(rows):
            self.__move_to(0, r)
            for c in range(cols):
                t = tiles[idx]; idx += 1
                slot = self.__cb_slot_of(t)
                self.__data(slot)
