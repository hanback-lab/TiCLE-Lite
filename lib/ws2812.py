__version__ = "1.1.0-stripped"
__author__ = "PlanX Lab Development Team"

import math, utime, array, gc, machine, micropython, rp2

gc.threshold(20480)

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def __ws2812_pio():
    T1, T2, T3 = 2, 5, 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()

class WS2812Matrix:
    def __init__(self, pin_sm_pairs, panel_width=16, panel_height=16,
                 grid_width=1, grid_height=1, zigzag=False, origin="top_left",
                 brightness=0.25):
        if origin not in ("top_left", "top_right", "bottom_left", "bottom_right"):
            raise ValueError("origin must be top_left/top_right/bottom_left/bottom_right")

        for _, sm_id in pin_sm_pairs:
            if not (0 <= sm_id <= 11):
                raise ValueError("State machine ID out of range (0-11)")

        self.__panel_width = int(panel_width)
        self.__panel_height = int(panel_height)
        self.__grid_width = int(grid_width)
        self.__grid_height = int(grid_height)

        self._fb_width = self.__panel_width * self.__grid_width
        self._fb_height = self.__panel_height * self.__grid_height
        self._fb_length = self._fb_width * self._fb_height
        self._fb = array.array("I", [0] * self._fb_length)
        self._fb_dirty = False

        self.__zigzag = bool(zigzag)
        self.__origin = origin

        self.brightness = float(brightness)

        self.__sms = []
        self.__tx_bufs = []

        self.__panels_per_sm = math.ceil((self.__grid_width * self.__grid_height) / len(pin_sm_pairs))
        self.__pixels_per_panel = self.__panel_width * self.__panel_height

        for pin_no, sm_id in pin_sm_pairs:
            try:
                pin = machine.Pin(pin_no, machine.Pin.OUT, machine.Pin.PULL_DOWN)
                sm = rp2.StateMachine(sm_id, __ws2812_pio, freq=8_000_000, sideset_base=pin)
                sm.active(1)
                self.__sms.append(sm)
                buf_len = self.__pixels_per_panel * self.__panels_per_sm
                self.__tx_bufs.append(array.array("I", [0] * buf_len))
            except OSError as e:
                for sm2 in self.__sms:
                    try:
                        sm2.active(0)
                    except:
                        pass
                raise OSError("Failed init pin {} / SM {}: {}".format(pin_no, sm_id, e))

        self.__sm_ids = [sm_id for _, sm_id in pin_sm_pairs]
        self.__dmas = [rp2.DMA() for _ in self.__sms]

        self.__build_pix_maps()

    class _PixelView:
        def __init__(self, parent, x, y):
            self._parent = parent
            self._x = x
            self._y = y

        @property
        def value(self):
            fb = self._parent._fb
            w = self._parent._fb_width
            packed = fb[self._y * w + self._x]
            g = (packed >> 24) & 0xFF
            r = (packed >> 16) & 0xFF
            b = (packed >> 8) & 0xFF
            return (r, g, b)

        @value.setter
        def value(self, color):
            self._parent._set_pixel(self._x, self._y, color)

    def __getitem__(self, pos):
        if isinstance(pos, tuple) and len(pos) == 2:
            x, y = pos
            if not (0 <= x < self._fb_width and 0 <= y < self._fb_height):
                raise IndexError("Pixel coordinates out of bounds")
            return WS2812Matrix._PixelView(self, x, y)
        else:
            raise TypeError("Index must be a tuple of (x, y)")

    @micropython.native
    def _set_pixel(self, x, y, color):
        r, g, b = self.__normalize_color(color)
        packed = self.__pack_grb(r, g, b)
        if 0 <= x < self._fb_width and 0 <= y < self._fb_height:
            self._fb[y * self._fb_width + x] = packed
            self._fb_dirty = True

    @property
    def width(self):
        return self._fb_width

    @property
    def height(self):
        return self._fb_height

    @property
    def brightness(self):
        return self.__brightness

    @brightness.setter
    def brightness(self, value):
        self.__brightness = max(0.0, min(value, 1.0))
        self.__btab = bytes(int(i * self.__brightness + 0.5) for i in range(256))

    @micropython.native
    def update(self, wait=True):
        dma_busy = any(dma.active() for dma in self.__dmas)
        if dma_busy and not wait:
            return
        if dma_busy and wait:
            while any(dma.active() for dma in self.__dmas):
                pass

        if self._fb_dirty:
            self.__flush_fb_to_txb()
            self._fb_dirty = False

        for sm_id, sm, src, dma in zip(self.__sm_ids, self.__sms, self.__tx_bufs, self.__dmas):
            if not dma.active():
                ctrl = dma.pack_ctrl(size=2, inc_write=False, treq_sel=self.__pio_dreq(sm_id), bswap=False)
                dma.config(read=src, write=sm, count=len(src), ctrl=ctrl, trigger=True)

        if wait:
            while any(dma.active() for dma in self.__dmas):
                pass

    def clear(self):
        self.fill(0)
        self.update(True)

    @micropython.native
    def fill(self, color):
        r, g, b = self.__normalize_color(color)
        packed = self.__pack_grb(r, g, b)
        self.__fill32(self._fb, 0, packed, self._fb_length)
        self._fb_dirty = True

    def deinit(self):
        self.clear()
        utime.sleep_ms(50)
        for sm in self.__sms:
            sm.active(0)

    @micropython.native
    def draw_line(self, x0, y0, x1, y1, color):
        r, g, b = self.__normalize_color(color)
        packed = self.__pack_grb(r, g, b)
        w = self._fb_width
        h = self._fb_height
        fb = self._fb

        if y0 == y1:
            y = y0
            if 0 <= y < h:
                if x0 > x1:
                    x0, x1 = x1, x0
                x0 = max(0, x0)
                x1 = min(w - 1, x1)
                if x0 <= x1:
                    span = x1 - x0 + 1
                    base = y * w + x0
                    self.__fill32(fb, base, packed, span)
                    self._fb_dirty = True
            return

        if x0 == x1:
            x = x0
            if not (0 <= x < w):
                return
            if y0 > y1:
                y0, y1 = y1, y0
            y0 = max(0, y0)
            y1 = min(h - 1, y1)
            if y0 > y1:
                return
            span = y1 - y0 + 1
            offset = y0 * w + x
            self.__fill32(fb, offset, packed, span)
            self._fb_dirty = True
            return

        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            if 0 <= x0 < w and 0 <= y0 < h:
                fb[y0 * w + x0] = packed
            if x0 == x1 and y0 == y1:
                break
            e2 = err << 1
            if e2 >= dy:
                err += dy
                x0  += sx
            if e2 <= dx:
                err += dx
                y0  += sy
        self._fb_dirty = True

    @micropython.native
    def draw_line_polar(self, cx, cy, length, angle_deg, color):
        w, h = self._fb_width, self._fb_height
        fb = self._fb
        half = length >> 1
        if half <= 0:
            return
        angle = angle_deg % 360

        r, g, b = self.__normalize_color(color)
        packed = self.__pack_grb(r, g, b)

        if angle % 90 == 0:
            if angle == 0 or angle == 180:
                y = cy
                if not (0 <= y < h):
                    return
                x0 = cx - half
                x1 = cx + half
                if angle == 180:
                    x0, x1 = x1, x0
                if x0 > x1:
                    x0, x1 = x1, x0
                x0 = max(0, x0)
                x1 = min(w - 1, x1)
                if x0 <= x1:
                    span = x1 - x0 + 1
                    base = y * w + x0
                    fb[base: base + span] = array.array('I', [packed] * span)
                    self._fb_dirty = True
                return
            else:
                x = cx
                if not (0 <= x < w):
                    return
                y0 = cy - half
                y1 = cy + half
                if angle == 270:
                    y0, y1 = y1, y0
                if y0 > y1:
                    y0, y1 = y1, y0
                y0 = max(0, y0)
                y1 = min(h - 1, y1)
                if y0 <= y1:
                    for py in range(y0, y1 + 1):
                        fb[py * w + x] = packed
                    self._fb_dirty = True
                return

        rad  = math.radians(angle)
        dxh  = int(round(math.cos(rad) * half))
        dyh  = int(round(math.sin(rad) * half))
        x0 = cx - dxh; y0 = cy - dyh
        x1 = cx + dxh; y1 = cy + dyh
        if (x0 < 0 and x1 < 0) or (x0 >= w and x1 >= w): return
        if (y0 < 0 and y1 < 0) or (y0 >= h and y1 >= h): return
        self.draw_line(x0, y0, x1, y1, color)

    @micropython.native
    def draw_rect(self, x, y, w, h, outline, fill=None):
        if w <= 0 or h <= 0:
            return
        x1, y1 = x, y
        x2, y2 = x + w - 1, y + h - 1
        dw, dh = self._fb_width, self._fb_height
        fb = self._fb
        pix_out = self.__pack_grb(*self.__normalize_color(outline))
        pix_fill = (self.__pack_grb(*self.__normalize_color(fill)) if fill is not None else None)

        def hspan(y_row, xa, xb, packed):
            if not (0 <= y_row < dh):
                return
            if xb < 0 or xa >= dw:
                return
            if xa < 0:
                xa = 0
            if xb >= dw:
                xb = dw - 1
            if xa > xb:
                return
            self.__fill32(fb, y_row*dw + xa, packed, xb - xa + 1)

        hspan(y1, x1, x2, pix_out)
        if y2 != y1:
            hspan(y2, x1, x2, pix_out)

        if w >= 2:
            xa, xb = x1, x2
            for py in range(max(y1, 0)+1, min(y2, dh-1)):
                row = py * dw
                if 0 <= xa < dw:
                    fb[row + xa] = pix_out
                if 0 <= xb < dw and xb != xa:
                    fb[row + xb] = pix_out

        if pix_fill is not None and w > 2 and h > 2:
            fill_xa = x1 + 1
            fill_xb = x2 - 1
            for py in range(max(y1+1, 0), min(y2, dh-1)):
                hspan(py, fill_xa, fill_xb, pix_fill)

        self._fb_dirty = True

    @micropython.native
    def draw_rect_polar(self, cx, cy, w, h, angle_deg, outline, fill=None):
        if w <= 0 or h <= 0:
            return
        if angle_deg % 90 == 0:
            self.draw_rect(cx - (w >> 1), cy - (h >> 1), w, h, outline, fill=fill)
            return

        half_w = (w - 1) / 2.0
        half_h = (h - 1) / 2.0
        vx = (-half_w, +half_w, +half_w, -half_w)
        vy = (-half_h, -half_h, +half_h, +half_h)

        rad = math.radians(angle_deg)
        ca, sa = math.cos(rad), math.sin(rad)

        def rot(ix, iy):
            return cx + ix * ca - iy * sa, cy + ix * sa + iy * ca

        pts = [rot(vx[i], vy[i]) for i in range(4)]

        fb = self._fb
        dw, dh = self._fb_width, self._fb_height
        p_out = self.__pack_grb(*self.__normalize_color(outline))
        p_fill = (self.__pack_grb(*self.__normalize_color(fill)) if fill is not None else None)

        tris = [(pts[0], pts[1], pts[2]), (pts[0], pts[2], pts[3])]

        def edge_x(y_scan, v0, v1):
            x0, y0 = v0;  x1, y1 = v1
            if y0 == y1:
                return None
            if y0 > y1:
                x0, y0, x1, y1 = x1, y1, x0, y0
            if not (y0 <= y_scan < y1):
                return None
            t = (y_scan - y0) / (y1 - y0)
            return x0 + t * (x1 - x0)

        y_min = max(0, int(math.ceil(min(p[1] for p in pts))))
        y_max = min(dh - 1, int(math.floor(max(p[1] for p in pts))))

        if p_fill is not None:
            for py in range(y_min, y_max + 1):
                xs = []
                for v0, v1, v2 in tris:
                    for e0, e1 in ((v0, v1), (v1, v2), (v2, v0)):
                        xx = edge_x(py + 0.5, e0, e1)
                        if xx is not None:
                            xs.append(xx)
                if len(xs) < 2:
                    continue
                xs.sort()
                xa = int(math.ceil(xs[0])) + 1
                xb = int(math.floor(xs[-1])) - 1
                if xa <= xb:
                    if xa < 0: xa = 0
                    if xb >= dw: xb = dw - 1
                    if xa <= xb:
                        self.__fill32(fb, py * dw + xa, p_fill, xb - xa + 1)

        def put(px, py):
            if 0 <= px < dw and 0 <= py < dh:
                fb[py * dw + px] = p_out

        for py in range(y_min, y_max + 1):
            xs = []
            for v0, v1, v2 in tris:
                for e0, e1 in ((v0, v1), (v1, v2), (v2, v0)):
                    xx = edge_x(py + 0.5, e0, e1)
                    if xx is not None:
                        xs.append(xx)
            if len(xs) < 2:
                continue
            xs.sort()
            xl = int(math.floor(xs[0] + 0.5))
            xr = int(math.floor(xs[-1] + 0.5))
            put(xl, py)
            if xr != xl:
                put(xr, py)

        self._fb_dirty = True

    @micropython.native
    def draw_ellipse(self, cx, cy, rx, ry=None, outline=(255, 255, 255), fill=None, angle_deg=0.0):
        if ry is None:
            ry = rx
        if rx <= 0 or ry <= 0:
            return

        dw, dh = self._fb_width, self._fb_height
        fb = self._fb
        p_edge = self.__pack_grb(*self.__normalize_color(outline))
        p_fill = (self.__pack_grb(*self.__normalize_color(fill)) if fill is not None else None)

        if angle_deg == 0.0:
            rx2, ry2 = rx * rx, ry * ry
            two_rx2  = rx2 << 1
            two_ry2  = ry2 << 1
            x, y = 0, ry
            dx, dy = 0, two_rx2 * y
            d1 = ry2 - rx2 * ry + (rx2 >> 2)

            def pset(px, py):
                if 0 <= px < dw and 0 <= py < dh:
                    fb[py * dw + px] = p_edge

            while dx < dy:
                if p_fill is not None:
                    xa, xb = cx - x, cx + x
                    if xa < 0: xa = 0
                    if xb >= dw: xb = dw - 1
                    if xa <= xb:
                        span = xb - xa + 1
                        self.__fill32(fb, (cy + y) * dw + xa, p_fill, span)
                        self.__fill32(fb, (cy - y) * dw + xa, p_fill, span)

                pset(cx + x, cy + y);  pset(cx - x, cy + y)
                pset(cx + x, cy - y);  pset(cx - x, cy - y)
                x  += 1
                dx += two_ry2
                if d1 < 0:
                    d1 += dx + ry2
                else:
                    y  -= 1
                    dy -= two_rx2
                    d1 += dx - dy + ry2

            d2 = (ry2 * (x + 0.5)**2) + (rx2 * (y - 1)**2) - rx2 * ry2
            while y >= 0:
                if p_fill is not None:
                    xa, xb = cx - x, cx + x
                    if xa < 0: xa = 0
                    if xb >= dw: xb = dw - 1
                    if xa <= xb:
                        span = xb - xa + 1
                        self.__fill32(fb, (cy + y) * dw + xa, p_fill, span)
                        self.__fill32(fb, (cy - y) * dw + xa, p_fill, span)

                pset(cx + x, cy + y);  pset(cx - x, cy + y)
                pset(cx + x, cy - y);  pset(cx - x, cy - y)
                y -= 1
                if d2 > 0:
                    d2 += rx2 - (two_rx2 * y)
                else:
                    x += 1
                    d2 += two_ry2 * x + rx2 - two_rx2 * y

            self._fb_dirty = True
            return

        th  = math.radians(angle_deg)
        ct, st = math.cos(th), math.sin(th)
        inv_rx2, inv_ry2 = 1 / (rx * rx), 1 / (ry * ry)
        A = ct * ct * inv_rx2 + st * st * inv_ry2
        B = 2 * ct * st * (inv_rx2 - inv_ry2)
        C = st * st * inv_rx2 + ct * ct * inv_ry2
        h_bound = abs(rx * st) + abs(ry * ct)
        y0 = max(0, int(cy - h_bound) - 1)
        y1 = min(dh - 1, int(cy + h_bound) + 1)

        for py in range(y0, y1 + 1):
            dy = py - cy
            By = B * dy
            K  = C * dy * dy - 1
            D  = By * By - 4 * A * K
            if D < 0:
                continue
            sqrtD = math.sqrt(D)
            xL = (-By - sqrtD) / (2 * A)
            xR = (-By + sqrtD) / (2 * A)
            oxl = int(math.ceil(cx + xL))
            oxr = int(math.floor(cx + xR))
            if oxr < 0 or oxl >= dw:
                continue
            xl = max(0, oxl)
            xr = min(dw - 1, oxr)
            if xl > xr:
                continue
            if p_fill is not None:
                self.__fill32(fb, py * dw + xl, p_fill, xr - xl + 1)
            if 0 <= oxl < dw:
                fb[py * dw + oxl] = p_edge
            if oxr != oxl and 0 <= oxr < dw:
                fb[py * dw + oxr] = p_edge

        self._fb_dirty = True

    def draw_circle(self, cx, cy, r, color, fill=None):
        self.draw_ellipse(cx, cy, r, r, color, fill=fill, angle_deg=0.0)

    @micropython.viper
    def __fill32(self, buf: ptr32, off: int, value: uint, count: int):
        i: int = 0
        while i < count:
            buf[off + i] = value
            i += 1

    @micropython.viper
    def __flush_fb_to_txb(self):
        fb = ptr32(self._fb)
        map_sm = ptr8(self.__pix_map_sm)
        map_idx = ptr16(self.__pix_map_idx)
        bufs = self.__tx_bufs
        length = int(self._fb_length)
        i = 0
        while i < length:
            sm = int(map_sm[i])
            idx = int(map_idx[i])
            bufs[sm][idx] = fb[i]
            i += 1

    @micropython.native
    def __build_pix_maps(self):
        dw = self._fb_width
        dh = self._fb_height
        n = dw * dh
        sm_t = array.array("B", [0] * n)
        idx_t = array.array("H", [0] * n)
        for y in range(dh):
            base = y * dw
            for x in range(dw):
                sm_i, bi = self.__coord_to_index(x, y)
                i = base + x
                sm_t[i] = sm_i
                idx_t[i] = bi
        self.__pix_map_sm = sm_t
        self.__pix_map_idx = idx_t

    @micropython.native
    def __coord_to_index(self, x, y):
        if not (0 <= x < self._fb_width and 0 <= y < self._fb_height):
            raise IndexError("pixel out of range")

        panel_col = x // self.__panel_width
        panel_row = y // self.__panel_height
        panel_id = panel_row * self.__grid_width + panel_col

        lx = x % self.__panel_width
        ly = y % self.__panel_height
        if self.__origin.startswith("bottom"):
            ly = self.__panel_height - 1 - ly
        if self.__origin.endswith("right"):
            lx = self.__panel_width - 1 - lx
        if self.__zigzag and (ly % 2):
            lx = self.__panel_width - 1 - lx

        sm_idx = panel_id // self.__panels_per_sm
        local_id = panel_id % self.__panels_per_sm
        buf_idx = local_id * self.__pixels_per_panel + ly * self.__panel_width + lx
        return sm_idx, buf_idx

    @micropython.native
    def __pack_grb(self, r, g, b):
        bt = self.__btab
        return (bt[g] << 24) | (bt[r] << 16) | (bt[b] << 8)

    @micropython.native
    def __normalize_color(self, color):
        if isinstance(color, (tuple, list)):
            if len(color) != 3:
                raise ValueError("Color tuple/list must have 3 elements")
            r, g, b = color
            if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
                raise ValueError("RGB must be 0-255 ints")
            return int(r), int(g), int(b)
        elif isinstance(color, int):
            if not (0 <= color <= 0xFFFFFF):
                raise ValueError("hex 0x000000-0xFFFFFF")
            return (color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF
        else:
            raise TypeError("Color must be tuple/list/int")

    @micropython.native
    def __pio_dreq(self, sm_id):
        if sm_id < 4:
            pio = 0
        elif sm_id < 8:
            pio = 1
        elif sm_id < 12:
            pio = 2
        else:
            raise ValueError("SM id out of range")
        return (pio << 3) | (sm_id & 0x03)
