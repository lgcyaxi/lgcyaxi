#!/usr/bin/env python3
"""Render profile/pipeline.svg, the animated research banner in the profile README.

A roadside unit (LiDAR + camera on a pole) watches a signalised junction in a
holographic city and loops through four 6-second stages:

  01 calibrate   camera frame, 2D <-> 3D detection matching, extrinsics converge
  02 perceive    the LiDAR sweep reveals a ray-cast point cloud and range image
  03 fuse        painted points, 3D boxes and tracks on a bird's-eye map
  04 real->sim   wipe into a CARLA/SUMO-style twin where MARL agents cross the
                 junction without signals

Everything is procedural and seeded. No dependencies:
    python profile/pipeline.py [out.svg]
"""
from __future__ import annotations

import math
import random
import sys
import xml.dom.minidom
from pathlib import Path

W, H = 1200, 675
T = 24.0                          # loop length (s): four 6 s stages
S = 6.0                           # isometric scale, px per metre
IC = S * math.cos(math.pi / 6)
IS = S * math.sin(math.pi / 6)
CX, CY = 430.0, 352.0             # screen position of the junction centre
STATIC_T = 14.0                   # frame shown when the viewer prefers reduced motion
RSU = (11.0, 11.0)                # pole position (m)
LIDAR = (11.0, 11.0, 7.4)
CAM = (9.9, 9.9, 6.7)
STOP = -13.5                      # stop line on every approach's own axis
T_SIM = 20.5                      # real traffic is hidden under the twin after this

CYAN, SKY, MAG, AMBER, LIME, ROSE = "#22d3ee", "#38bdf8", "#e879f9", "#fbbf24", "#a3e635", "#fb7185"
INK, MUTED = "#e2e8f0", "#94a3b8"

rng = random.Random(20260923)


# ------------------------------------------------------------------ helpers
def iso(x, y, z=0.0):
    return CX + (x - y) * IC, CY + (x + y) * IS - z * S


def n(v):
    s = f"{v:.1f}"
    if s.endswith(".0"):
        s = s[:-2]
    return "0" if s == "-0" else s


def P(pts):
    return " ".join(f"{n(x)} {n(y)}" for x, y in pts)


def poly(pts):
    return f"M{P(pts)}Z"


def ipoly(pts):
    return "M" + " ".join(f"{round(x)} {round(y)}" for x, y in pts) + "Z"


def pct(t):
    return f"{100 * t / T:.2f}".rstrip("0").rstrip(".") + "%"


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def hexmix(a, b, t):
    ca = [int(a[i:i + 2], 16) for i in (1, 3, 5)]
    cb = [int(b[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{round(x + (y - x) * t):02x}" for x, y in zip(ca, cb))


def turbo(x):
    x = min(1.0, max(0.0, x))
    r = 0.13572138 + x * (4.61539260 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))))
    g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))))
    b = 0.10667330 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))))
    return "#" + "".join(f"{round(min(1, max(0, c)) * 255):02x}" for c in (r, g, b))


def sub(a, b):
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def add(a, b):
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def mul(a, k):
    return a[0] * k, a[1] * k, a[2] * k


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]


def unit(a):
    k = math.sqrt(dot(a, a))
    return a[0] / k, a[1] / k, a[2] / k


def text(x, y, s, size=13, fill=INK, weight=None, anchor=None, mono=False, cls=None, extra=""):
    attrs = [f'x="{n(x)}"', f'y="{n(y)}"', f'font-size="{size:g}"', f'fill="{fill}"']
    if weight:
        attrs.append(f'font-weight="{weight}"')
    if anchor:
        attrs.append(f'text-anchor="{anchor}"')
    classes = " ".join(c for c in ("m" if mono else "", cls or "") if c)
    if classes:
        attrs.append(f'class="{classes}"')
    return f'<text {" ".join(attrs)}{extra}>{esc(s)}</text>'


def dots(xy, scale=2):
    """One round-capped zero-length subpath per point, in scale-x integer coords."""
    q = sorted({(round(x * scale), round(y * scale)) for x, y in xy}, key=lambda p: (p[1] // 24, p[0]))
    if not q:
        return ""
    out, (px, py) = [f"M{q[0][0]} {q[0][1]}h0"], q[0]
    for x, y in q[1:]:
        out.append(f"m{x - px}{'' if y - py < 0 else ' '}{y - py}h0")
        px, py = x, y
    return "".join(out)


def rdp(pts, eps):
    """Ramer-Douglas-Peucker on (t, s) samples, measuring the error in s."""
    if len(pts) < 3:
        return list(pts)
    (t0, s0), (t1, s1) = pts[0], pts[-1]
    worst, idx = 0.0, 0
    for i in range(1, len(pts) - 1):
        t, s = pts[i]
        d = abs(s - (s0 + (s1 - s0) * (t - t0) / (t1 - t0)))
        if d > worst:
            worst, idx = d, i
    if worst <= eps:
        return [pts[0], pts[-1]]
    return rdp(pts[:idx + 1], eps)[:-1] + rdp(pts[idx:], eps)


def interp(frames, t):
    for (t0, v0), (t1, v1) in zip(frames, frames[1:]):
        if t0 <= t <= t1:
            return v0 if t1 == t0 else v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    return frames[-1][1]


def hull(points):
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]
    return half(pts) + half(pts[::-1])


# --------------------------------------------------------------- animation
class Css:
    """Collects keyframes; every class loops over the shared T-second timeline."""

    def __init__(self):
        self.rules, self.memo = [], {}

    def anim(self, frames, static=""):
        key = (tuple(frames), static)
        if key not in self.memo:
            name = f"k{len(self.memo)}"
            self.memo[key] = name
            body = "".join(f"{pct(t)}{{{d}{';animation-timing-function:' + e if e else ''}}}" for t, d, e in frames)
            self.rules.append(f"@keyframes {name}{{{body}}}")
            self.rules.append(f".{name}{{{static + ';' if static else ''}animation:{name} {T:g}s linear infinite}}")
        return self.memo[key]

    def vis(self, on, off, fade=0.45, lo=0.0, hi=1.0):
        if on <= 0 and off >= T:
            pts = [(0, hi), (T, hi)]
        elif on <= 0:
            pts = [(0, hi), (off - fade, hi), (off, lo), (T - fade, lo), (T, hi)]
        elif off >= T:
            pts = [(0, lo), (on, lo), (on + fade, hi), (T - fade, hi), (T, lo)]
        else:
            pts = [(0, lo), (on, lo), (on + fade, hi), (off - fade, hi), (off, lo), (T, lo)]
        return self.anim([(t, f"opacity:{v:g}", None) for t, v in pts],
                         static=f"opacity:{interp(pts, STATIC_T):g}")

    def move(self, frames):
        """frames: [(t, dx, dy)] screen offsets, linear in between."""
        kf = [(t, f"transform:translate({n(x)}px,{n(y)}px)", None) for t, x, y in frames]
        sx = interp([(t, x) for t, x, _ in frames], STATIC_T)
        sy = interp([(t, y) for t, _, y in frames], STATIC_T)
        return self.anim(kf, static=f"transform:translate({n(sx)}px,{n(sy)}px)")


def stage(css, i):
    return css.vis(6.0 * i + 0.12, 6.0 * (i + 1) - 0.12, 0.33)      # dip between stages, no text overlap


# -------------------------------------------------------------------- city
LOTS = [(12, 22), (25, 35), (38, 48), (56, 66), (69, 79), (82, 92),
        (100, 110), (113, 123), (126, 136), (144, 154), (157, 167)]


class Box:
    def __init__(self, x0, x1, y0, y1, h, sty="c", roof=""):
        self.x0, self.x1, self.y0, self.y1, self.h = x0, x1, y0, y1, h
        self.sty, self.roof = sty, roof

    @property
    def quad(self):
        return ("-" if self.x1 <= 0 else "+") + ("-" if self.y1 <= 0 else "+")


def visible(x0, x1, y0, y1, h, pad=12):
    xs, ys = zip(*(iso(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (0, h)))
    return max(xs) > -pad and min(xs) < W + pad and max(ys) > -pad and min(ys) < H + pad


def lot_height(sx, sy, a0, b0):
    if sx > 0 and sy > 0:                          # foreground stays low and open
        if (a0, b0) == (12, 12) or rng.random() < 0.3:
            return None
        return rng.uniform(3.0, 7.0) + 0.04 * (a0 + b0)
    if sx < 0 and sy < 0:                          # skyline behind the junction
        h = (10 + 0.40 * (a0 + b0)) * rng.uniform(0.55, 1.05)
        return min(h * (1.6 if rng.random() < 0.16 else 1.0), 118)
    if rng.random() < 0.08:
        return None
    back = a0 if sx < 0 else b0                    # side blocks rise toward the back
    return (5 + 0.30 * back + 0.07 * (a0 + b0 - back)) * rng.uniform(0.6, 1.25)


def footprints(x0, x1, y0, y1):
    m = 1.2
    x0, x1, y0, y1 = x0 + m, x1 - m, y0 + m, y1 - m
    if rng.random() < 0.38:
        if rng.random() < 0.5:
            xm = (x0 + x1) / 2 + rng.uniform(-1.5, 1.5)
            return [(x0, xm - 0.7, y0, y1), (xm + 0.7, x1, y0, y1)]
        ym = (y0 + y1) / 2 + rng.uniform(-1.5, 1.5)
        return [(x0, x1, y0, ym - 0.7), (x0, x1, ym + 0.7, y1)]
    return [(x0, x1, y0, y1)]


def build_city():
    blds, plazas = [], []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for a0, a1 in LOTS:
                for b0, b1 in LOTS:
                    x0, x1 = sorted((sx * a0, sx * a1))
                    y0, y1 = sorted((sy * b0, sy * b1))
                    h = lot_height(sx, sy, a0, b0)
                    if h is None:
                        if visible(x0, x1, y0, y1, 0):
                            plazas.append((x0 + 1.2, x1 - 1.2, y0 + 1.2, y1 - 1.2))
                        continue
                    for fx0, fx1, fy0, fy1 in footprints(x0, x1, y0, y1):
                        hh = max(3.0, h * rng.uniform(0.72, 1.12))
                        sty = rng.choices("cwf", (0.55, 0.2, 0.25))[0]
                        roof = "beam" if hh > 44 and rng.random() < 0.35 else "blink" if hh > 20 and rng.random() < 0.25 else ""
                        if visible(fx0, fx1, fy0, fy1, hh):
                            blds.append(Box(fx0, fx1, fy0, fy1, hh, sty, roof))
    blds.sort(key=lambda b: b.x0 + b.x1 + b.y0 + b.y1)
    return blds, plazas


def box_faces(x0, x1, y0, y1, z0, z1):
    top = [iso(x0, y0, z1), iso(x1, y0, z1), iso(x1, y1, z1), iso(x0, y1, z1)]
    left = [iso(x0, y1, z0), iso(x1, y1, z0), iso(x1, y1, z1), iso(x0, y1, z1)]     # +y face
    right = [iso(x1, y1, z0), iso(x1, y0, z0), iso(x1, y0, z1), iso(x1, y1, z1)]    # +x face
    return top, left, right


def box_outline(x0, x1, y0, y1, z0, z1):
    return [iso(x0, y1, z0), iso(x1, y1, z0), iso(x1, y0, z0), iso(x1, y0, z1), iso(x0, y0, z1), iso(x0, y1, z1)]


def box_edges(x0, x1, y0, y1, z0, z1):
    a, b, c, d = iso(x0, y0, z1), iso(x1, y0, z1), iso(x1, y1, z1), iso(x0, y1, z1)
    return f"M{P([a, b, c, d])}Z M{P([c, iso(x1, y1, z0)])} M{P([d, iso(x0, y1, z0)])} M{P([b, iso(x1, y0, z0)])}"


def box_wire(x0, x1, y0, y1, z0, z1):
    """All 12 edges, for bounding boxes."""
    c = {(i, j, k): iso((x0, x1)[i], (y0, y1)[j], (z0, z1)[k]) for i in (0, 1) for j in (0, 1) for k in (0, 1)}
    ring = lambda k: f"M{P([c[0, 0, k], c[1, 0, k], c[1, 1, k], c[0, 1, k]])}Z"
    verts = "".join(f"M{P([c[i, j, 0], c[i, j, 1]])}" for i in (0, 1) for j in (0, 1))
    return ring(0) + ring(1) + verts


# ------------------------------------------------------------------- LiDAR
CH, NAZ, RMAX = 32, 300, 105.0
ELEV = [math.radians(-36 + i * 42 / (CH - 1)) for i in range(CH)]   # -36 .. +6 deg
NB = 12
BIN = [turbo(0.14 + 0.8 * (k + 0.5) / NB) for k in range(NB)]
VIEW = unit((1.0, 1.0, 1.0))                                        # toward the iso viewer


def rbin(r, near=6.0, far=72.0):
    return max(0, min(NB - 1, int((r - near) / (far - near) * NB)))


def ray_box(o, d, b, tmax):
    t0, t1 = 1e-6, tmax
    for lo, hi, oi, di in ((b.x0, b.x1, o[0], d[0]), (b.y0, b.y1, o[1], d[1]), (0.0, b.h, o[2], d[2])):
        if abs(di) < 1e-12:
            if oi < lo or oi > hi:
                return None
            continue
        ta, tb = (lo - oi) / di, (hi - oi) / di
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return None
    return t0


def first_hit(o, d, blds, tmax):
    best, hit = tmax, None
    for b in blds:
        t = ray_box(o, d, b, best)
        if t is not None and t < best:
            best, hit = t, b
    return best, hit


def az_buckets(blds, origin, nbins=NAZ):
    """Buildings grouped by the azimuth bins they cover, seen from origin."""
    buckets = [[] for _ in range(nbins)]
    for b in blds:
        angs = [math.atan2(y - origin[1], x - origin[0]) for x in (b.x0, b.x1) for y in (b.y0, b.y1)]
        if max(angs) - min(angs) > math.pi:            # straddles the -x axis
            angs = [a + 2 * math.pi if a < 0 else a for a in angs]
        j0 = math.floor(min(angs) / (2 * math.pi) * nbins) - 1
        j1 = math.ceil(max(angs) / (2 * math.pi) * nbins) + 1
        for j in range(j0, j1 + 1):
            buckets[j % nbins].append(b)
    return buckets


def az_bin(d, nbins=NAZ):
    return round(math.atan2(d[1], d[0]) / (2 * math.pi) * nbins) % nbins


def lidar_scan(blds):
    """Ray-cast every beam; returns [(point, range, kind)] and {(ch, az): range}."""
    pts, grid = [], {}
    buckets = az_buckets(blds, LIDAR)
    for j in range(NAZ):
        az = 2 * math.pi * j / NAZ
        ca, sa = math.cos(az), math.sin(az)
        for i, el in enumerate(ELEV):
            d = (math.cos(el) * ca, math.cos(el) * sa, math.sin(el))
            tmax, kind = RMAX, None
            if d[2] < 0 and -LIDAR[2] / d[2] < tmax:
                tmax, kind = -LIDAR[2] / d[2], "g"
            t, b = first_hit(LIDAR, d, buckets[j], tmax)
            if b is not None:
                tmax, kind = t, "b"
            if kind:
                grid[i, j] = tmax
                pts.append((add(LIDAR, mul(d, tmax)), tmax, kind))
    return pts, grid


class Viewer:
    """Occlusion toward the isometric viewer, prefiltered by screen bounding boxes."""

    def __init__(self, blds):
        self.items = []
        for b in blds:
            xs, ys = zip(*box_outline(b.x0, b.x1, b.y0, b.y1, 0, b.h))
            self.items.append((b, min(xs), min(ys), max(xs), max(ys)))

    def sees(self, p, X, Y):
        o = add(p, mul(VIEW, 0.05))
        return not any(x0 <= X <= x1 and y0 <= Y <= y1 and ray_box(o, VIEW, b, 400.0) is not None
                       for b, x0, y0, x1, y1 in self.items)


# ------------------------------------------------------------------ camera
class Camera:
    def __init__(self, pos, target, hfov, w, h):
        self.c, self.w, self.h = pos, w, h
        f = unit(sub(target, pos))
        r = unit((-f[1], f[0], 0.0))
        u = cross(r, f)
        self.f, self.r, self.u = f, r, (mul(u, -1) if u[2] < 0 else u)
        self.fx = (w / 2) / math.tan(math.radians(hfov) / 2)

    def cam(self, p):
        v = sub(p, self.c)
        return dot(v, self.r), dot(v, self.u), dot(v, self.f)

    def px(self, q):
        x, y, z = q
        return (max(-3000.0, min(3000.0, self.w / 2 + self.fx * x / z)),
                max(-3000.0, min(3000.0, self.h / 2 - self.fx * y / z)))

    def project(self, p):
        q = self.cam(p)
        return self.px(q) if q[2] > 0.3 else None

    def polygon(self, world, zn=0.35):
        q = [self.cam(p) for p in world]
        out = []
        for i, a in enumerate(q):
            b = q[(i + 1) % len(q)]
            if a[2] >= zn:
                out.append(a)
            if (a[2] >= zn) != (b[2] >= zn):
                t = (zn - a[2]) / (b[2] - a[2])
                out.append(tuple(a[k] + (b[k] - a[k]) * t for k in range(3)))
        if len(out) < 3:
            return None, 0.0
        return [self.px(p) for p in out], sum(p[2] for p in out) / len(out)

    def ray(self, u, v):
        x, y = (u - self.w / 2) / self.fx, -(v - self.h / 2) / self.fx
        return unit(add(add(self.f, mul(self.r, x)), mul(self.u, y)))


def world_faces(x0, x1, y0, y1, z0, z1):
    return [
        ([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], (0, 0, 1)),
        ([(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)], (1, 0, 0)),
        ([(x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)], (-1, 0, 0)),
        ([(x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)], (0, 1, 0)),
        ([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)], (0, -1, 0)),
    ]


# ----------------------------------------------------------------- traffic
V0, HEADWAY, AMAX, BCOMF, GAP0 = 11.0, 1.3, 2.4, 3.2, 2.2


def light(road, t):
    if road == "A":
        return "G" if t < 4.8 else "Y" if t < 5.6 else "R"
    return "R" if t < 7.0 else "G"


class Car:
    def __init__(self, road, sign, lane, s, v=V0, kind="car"):
        self.road, self.sign, self.lane, self.kind = road, sign, lane, kind
        self.s = self.s0 = s
        self.v = self.v0 = v
        self.L, self.Wd, self.Hc = (10.5, 2.5, 3.1) if kind == "bus" else (4.6, 1.9, 1.5)
        self.track, self.committed, self.id = [], False, 0

    def at(self, s):
        return (self.sign * s, self.lane) if self.road == "A" else (self.lane, self.sign * s)

    def extent(self, s):
        x, y = self.at(s)
        hx, hy = (self.L / 2, self.Wd / 2) if self.road == "A" else (self.Wd / 2, self.L / 2)
        return x - hx, x + hx, y - hy, y + hy

    def s_at(self, t):
        return interp(self.frames, t)


def simulate(cars, dt=0.02):
    lanes = {}
    for c in cars:
        lanes.setdefault((c.road, c.sign, c.lane), []).append(c)
    for k in range(int(round(T_SIM / dt)) + 1):
        t = k * dt
        acc = {}
        for c in cars:
            c.track.append((t, c.s))
        for group in lanes.values():
            group.sort(key=lambda c: c.s)
            for i, c in enumerate(group):
                front = c.s + c.L / 2
                gap, dv = 1e9, 0.0
                if i + 1 < len(group):
                    lead = group[i + 1]
                    gap, dv = lead.s - lead.L / 2 - front, c.v - lead.v
                state, to_stop = light(c.road, t), STOP - front
                if state == "Y" and to_stop > 0 and to_stop < c.v * c.v / (2 * BCOMF):
                    c.committed = True
                if state != "G" and not c.committed and 0 < to_stop < gap:
                    gap, dv = to_stop, c.v
                ss = GAP0 + max(0.0, c.v * HEADWAY + c.v * dv / (2 * math.sqrt(AMAX * BCOMF)))
                acc[c] = AMAX * (1 - (c.v / V0) ** 4 - (ss / max(gap, 0.05)) ** 2)
        for c in cars:
            c.v = max(0.0, c.v + acc[c] * dt)
            c.s += c.v * dt
    for c in cars:
        f = rdp(c.track, 0.12)
        pre = c.s0 - c.v0 * (T - T_SIM - 0.02)          # glide back into the loop start unseen
        c.frames = f + [(T_SIM + 0.02, pre), (T, c.s0)]


def real_traffic():
    q = [-18.0, -24.8, -31.6, -38.4, -45.2]
    cars = [Car("A", 1, 1.75, s) for s in (-92, -64, -41, -19, 4)]
    cars += [Car("A", 1, 5.25, s, kind="bus" if s == -55 else "car") for s in (-80, -55, -30, -6)]
    cars += [Car("A", -1, -1.75, s) for s in (-85, -60, -36, -12)]
    cars += [Car("A", -1, -5.25, s) for s in (-72, -47, -22)]
    cars += [Car("B", 1, -1.75, s, 0.0) for s in q[:4]] + [Car("B", 1, -1.75, s) for s in (-120, -150)]
    cars += [Car("B", 1, -5.25, s, 0.0, kind="bus" if s == -24.8 else "car") for s in (-18.0, -24.8)]
    cars += [Car("B", 1, -5.25, -140.0)]
    cars += [Car("B", -1, 1.75, s, 0.0) for s in q[:2]] + [Car("B", -1, 1.75, -125.0)]
    cars += [Car("B", -1, 5.25, -18.0, 0.0)]
    for c in cars:
        if c.kind == "bus" and c.road == "B":
            c.s = c.s0 = c.s0 - 3.0
    simulate(cars)
    shown = []
    for c in cars:
        if any(-30 < iso(*c.at(s))[0] < W + 30 and -30 < iso(*c.at(s))[1] < H + 30 for _, s in c.track[::10]):
            shown.append(c)
    for i, c in enumerate(shown):
        c.id = 3 + (i * 7) % 97
    return shown


class Agent(Car):
    """Twin vehicle crossing at constant speed; tc is when it reaches the centre."""
    V = 9.0

    def __init__(self, road, sign, lane, kind):
        super().__init__(road, sign, lane, 0.0, self.V, "car")
        self.type, self.tc = kind, 0.0

    def s_at(self, t):
        return self.V * (t - self.tc)


def schedule_agents():
    specs = [("A", 1, 1.75, "marl"), ("B", 1, -1.75, "marl"), ("A", -1, -1.75, "av"), ("B", -1, 1.75, "marl"),
             ("A", 1, 5.25, "human"), ("B", 1, -5.25, "av"), ("A", -1, -5.25, "marl"), ("B", -1, 5.25, "human"),
             ("A", 1, 1.75, "av"), ("B", 1, -1.75, "marl"), ("A", -1, -1.75, "marl"), ("B", -1, 1.75, "human")]
    agents = []

    def clash(a, b):
        if (a.road, a.sign, a.lane) == (b.road, b.sign, b.lane):
            return abs(a.tc - b.tc) < 1.2
        for k in range(160):
            t = min(a.tc, b.tc) - 1.6 + k * 0.03
            ax0, ax1, ay0, ay1 = a.extent(a.s_at(t))
            bx0, bx1, by0, by1 = b.extent(b.s_at(t))
            if ax0 < bx1 + 0.9 and bx0 < ax1 + 0.9 and ay0 < by1 + 0.9 and by0 < ay1 + 0.9:
                return True
        return False

    for road, sign, lane, kind in specs:
        a = Agent(road, sign, lane, kind)
        a.tc = 19.4
        while any(clash(a, b) for b in agents):
            a.tc += 0.05
        agents.append(a)
    return agents


# ------------------------------------------------------------------- build
def build(out_path):
    css = Css()
    blds, plazas = build_city()
    cars = real_traffic()
    agents = schedule_agents()
    scan, grid = lidar_scan(blds)
    cam = Camera(CAM, (-3.4, -25.0, 0.6), 42, 334, 188)

    defs, real, sim, hud, over = [], [], [], [], []
    PX, PY, PW, PH = 812, 20, 366, 540            # HUD panel

    def under_panel(b):
        xs, ys = zip(*box_outline(b.x0, b.x1, b.y0, b.y1, 0, b.h))
        return min(xs) > PX + 6 and max(xs) < PX + PW - 4 and min(ys) > PY + 6 and max(ys) < PY + PH - 6
    drawn = [b for b in blds if not under_panel(b)]
    G = f"matrix({IC:.4f} {IS:.4f} {-IC:.4f} {IS:.4f} {n(CX)} {n(CY)})"      # ground plane in metres

    # ---------------------------------------------------------------- defs
    defs.append(f'<clipPath id="card"><rect width="{W}" height="{H}" rx="18"/></clipPath>')
    defs.append(f'<radialGradient id="bg" cx="{n(CX)}" cy="{n(CY - 70)}" r="860" gradientUnits="userSpaceOnUse">'
                '<stop offset="0" stop-color="#0d2446"/><stop offset=".55" stop-color="#06122a"/><stop offset="1" stop-color="#02060f"/></radialGradient>')
    defs.append('<linearGradient id="gT" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#1b4a8c"/><stop offset="1" stop-color="#123463"/></linearGradient>'
                '<linearGradient id="gL" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#103164"/><stop offset="1" stop-color="#040c1e"/></linearGradient>'
                '<linearGradient id="gR" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#174585"/><stop offset="1" stop-color="#061532"/></linearGradient>')
    ml = f"matrix({IC:.4f} {IS:.4f} 0 {-S:g} {n(CX)} {n(CY)})"               # +y facades: u = x, v = z
    mr = f"matrix({-IC:.4f} {IS:.4f} 0 {-S:g} {n(CX)} {n(CY)})"              # +x facades: u = y, v = z
    tiles = {
        "c": (3.0, 3.5, '<rect x=".5" y=".7" width="2" height="1.9" fill="#7dd3fc" fill-opacity=".24"/>'
                        '<rect y="3.3" width="3" height=".2" fill="#38bdf8" fill-opacity=".3"/>'),
        "w": (2.4, 3.2, '<rect x=".4" y=".8" width="1.4" height="1.5" fill="#fcd34d" fill-opacity=".2"/>'),
        "f": (1.4, 60, '<rect width=".3" height="60" fill="#93c5fd" fill-opacity=".28"/>'),
    }
    for k, (tw, th, body) in tiles.items():
        for side, m in (("l", ml), ("r", mr)):
            defs.append(f'<pattern id="p{side}{k}" width="{tw:g}" height="{th:g}" patternUnits="userSpaceOnUse" patternTransform="{m}">{body}</pattern>')
    defs.append('<pattern id="grid" width="10" height="10" patternUnits="userSpaceOnUse"><path d="M10 0H0V10" fill="none" stroke="#38bdf8" stroke-opacity=".2" stroke-width=".14"/></pattern>'
                '<pattern id="sgrid" width="10" height="10" patternUnits="userSpaceOnUse"><path d="M10 0H0V10" fill="none" stroke="#94a3b8" stroke-opacity=".13" stroke-width=".12"/></pattern>'
                '<pattern id="trees" width="4" height="4" patternUnits="userSpaceOnUse"><circle cx="2" cy="2" r=".7" fill="#2dd4bf" fill-opacity=".45"/></pattern>'
                '<pattern id="strees" width="4" height="4" patternUnits="userSpaceOnUse"><circle cx="2" cy="2" r=".9" fill="#4d7c63"/></pattern>')
    defs.append('<radialGradient id="beam" cx="0" cy="0" r="92" gradientUnits="userSpaceOnUse">'
                '<stop offset="0" stop-color="#cffafe" stop-opacity=".95"/><stop offset=".35" stop-color="#22d3ee" stop-opacity=".45"/>'
                '<stop offset="1" stop-color="#22d3ee" stop-opacity="0"/></radialGradient>')
    for name, (a, b, c, d) in {"hxp": (0, 0, 1, 1), "hxn": (1, 1, 0, 0), "hyp": (1, 0, 0, 1), "hyn": (0, 1, 1, 0)}.items():
        defs.append(f'<linearGradient id="{name}" x1="{a}" y1="{b}" x2="{c}" y2="{d}">'
                    '<stop offset="0" stop-color="#fef3c7" stop-opacity=".42"/><stop offset="1" stop-color="#fef3c7" stop-opacity="0"/></linearGradient>')
    defs.append('<linearGradient id="beamV" x1="0" y1="1" x2="0" y2="0"><stop offset="0" stop-color="#67e8f9" stop-opacity=".75"/><stop offset="1" stop-color="#67e8f9" stop-opacity="0"/></linearGradient>'
                '<linearGradient id="scan" x1="0" y1="0" x2="1" y2="0"><stop offset="0" stop-color="#e879f9" stop-opacity="0"/><stop offset=".8" stop-color="#e879f9" stop-opacity=".35"/>'
                '<stop offset=".97" stop-color="#fdf4ff"/><stop offset="1" stop-color="#fdf4ff" stop-opacity="0"/></linearGradient>'
                '<linearGradient id="fog" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#02060f" stop-opacity=".78"/><stop offset="1" stop-color="#02060f" stop-opacity="0"/></linearGradient>'
                '<linearGradient id="scrimT" x1="0" y1="0" x2="1" y2="0"><stop offset="0" stop-color="#02060f" stop-opacity=".92"/><stop offset=".7" stop-color="#02060f" stop-opacity=".55"/><stop offset="1" stop-color="#02060f" stop-opacity="0"/></linearGradient>'
                '<linearGradient id="scrimB" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#02060f" stop-opacity="0"/><stop offset=".45" stop-color="#02060f" stop-opacity=".82"/><stop offset="1" stop-color="#02060f" stop-opacity=".95"/></linearGradient>'
                '<linearGradient id="panelB" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="#67e8f9" stop-opacity=".7"/><stop offset=".5" stop-color="#6366f1" stop-opacity=".25"/><stop offset="1" stop-color="#e879f9" stop-opacity=".55"/></linearGradient>'
                '<linearGradient id="sky" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#0a1730"/><stop offset="1" stop-color="#2a4470"/></linearGradient>'
                '<radialGradient id="vig" cx=".42" cy=".5" r=".75"><stop offset=".6" stop-color="#02060f" stop-opacity="0"/><stop offset="1" stop-color="#02060f" stop-opacity=".6"/></radialGradient>')

    # ------------------------------------------------------- ground + roads
    def roads(sim_style):
        road, street = ("#1b2436", "#161e2d") if sim_style else ("#0a1528", "#07101f")
        edge, lane, centre = ("#64748b", "#e2e8f0", "#fde68a") if sim_style else ("#38bdf8", "#7dd3fc", "#fbbf24")
        eo, lo, co = (".9", ".8", ".85") if sim_style else (".35", ".55", ".4")
        g = [f'<g transform="{G}">']
        for a, b in ((48, 56), (92, 100), (136, 144)):
            for s in (-1, 1):
                lo_, hi_ = sorted((s * a, s * b))
                g.append(f'<rect x="-200" y="{lo_}" width="400" height="{hi_ - lo_}" fill="{street}"/>'
                         f'<rect x="{lo_}" y="-200" width="{hi_ - lo_}" height="400" fill="{street}"/>')
        g.append(f'<rect x="-200" y="-7" width="400" height="14" fill="{road}"/><rect x="-7" y="-200" width="14" height="400" fill="{road}"/>')
        for x0, x1, y0, y1 in plazas:
            g.append(f'<rect x="{n(x0)}" y="{n(y0)}" width="{n(x1 - x0)}" height="{n(y1 - y0)}" '
                     + ('fill="#14231f"/>' if sim_style else 'fill="#082032" stroke="#2dd4bf" stroke-opacity=".35" stroke-width=".2"/>')
                     + f'<rect x="{n(x0 + 1.5)}" y="{n(y0 + 1.5)}" width="{n(x1 - x0 - 3)}" height="{n(y1 - y0 - 3)}" fill="url(#{"strees" if sim_style else "trees"})"/>')
        ln = []
        for s in (-1, 1):                     # curbs, lane dividers and centre lines outside the box
            for k in (-1, 1):
                ln.append(f'<path d="M{s * 200} {k * 7}H{s * 10}M{k * 7} {s * 200}V{s * 10}" stroke="{edge}" stroke-opacity="{eo}" stroke-width=".18"/>')
                ln.append(f'<path d="M{s * 200} {k * 0.18:g}H{s * 13.5}M{k * 0.18:g} {s * 200}V{s * 13.5}" stroke="{centre}" stroke-opacity="{co}" stroke-width=".14"/>')
        flow = "" if sim_style else ' class="flow"'
        for s in (-1, 1):
            # dividers drawn in the direction of travel so the flow animation runs with traffic
            ln.append(f'<path{flow} d="M{-200 * s} {3.5 * s}H{-13.5 * s}M{13.5 * s} {3.5 * s}H{200 * s}" stroke="{lane}" stroke-opacity="{lo}" stroke-width=".16" stroke-dasharray="3 6"/>')
            ln.append(f'<path{flow} d="M{-3.5 * s} {-200 * s}V{-13.5 * s}M{-3.5 * s} {13.5 * s}V{200 * s}" stroke="{lane}" stroke-opacity="{lo}" stroke-width=".16" stroke-dasharray="3 6"/>')
        g.append(f'<g fill="none">{"".join(ln)}</g>')
        zebra = []
        for s in (-1, 1):
            lo_, hi_ = sorted((s * 9.8, s * 12.8))
            for k in range(13):
                c = -7 + 0.3 + k * 1.1
                zebra.append(f'<rect x="{n(lo_)}" y="{n(c)}" width="3" height=".55"/><rect x="{n(c)}" y="{n(lo_)}" width=".55" height="3"/>')
        g.append(f'<g fill="{"#e2e8f0" if sim_style else "#bae6fd"}" fill-opacity="{".75" if sim_style else ".28"}">{"".join(zebra)}</g>')
        stops = [f'M{STOP} 0V7', f'M{-STOP} -7V0', f'M-7 {STOP}H0', f'M0 {-STOP}H7']
        g.append(f'<path d="{"".join(stops)}" stroke="{"#f8fafc" if sim_style else "#e0f2fe"}" stroke-opacity="{".9" if sim_style else ".55"}" stroke-width=".5"/>')
        g.append("</g>")
        return "".join(g)

    real.append(f'<rect width="{W}" height="{H}" fill="url(#bg)"/>')
    real.append(f'<g transform="{G}"><rect x="-130" y="-130" width="260" height="260" fill="url(#grid)"/></g>')
    city_dim = css.anim([(0, "opacity:1", None), (5.6, "opacity:1", None), (6.4, "opacity:.3", None),
                         (17.6, "opacity:.3", None), (19.0, "opacity:.3", None), (22.6, "opacity:1", None), (T, "opacity:1", None)],
                        static="opacity:.3")
    real.append(f'<g class="{city_dim}">{roads(False)}</g>')

    # LiDAR sweep on the ground (drawn under the buildings so they occlude it)
    px, py = iso(*RSU)
    wedges = []
    for a0, a1, op in ((-3, 0, .55), (-8, -3, .32), (-16, -8, .18), (-30, -16, .09), (-55, -30, .04)):
        r0, r1 = math.radians(a0), math.radians(a1)
        wedges.append(f'<path fill="url(#beam)" fill-opacity="{op}" d="M0 0L{n(92 * math.cos(r0))} {n(92 * math.sin(r0))}'
                      f'A92 92 0 0 1 {n(92 * math.cos(r1))} {n(92 * math.sin(r1))}Z"/>')
    rings = "".join(f'<circle r="4" fill="none" stroke="#67e8f9" stroke-width=".3"><animate attributeName="r" values="3;88" dur="3s" begin="{b}s" repeatCount="indefinite"/>'
                    f'<animate attributeName="stroke-opacity" values=".6;0" dur="3s" begin="{b}s" repeatCount="indefinite"/></circle>' for b in (0, 1, 2))
    sweep_cls = css.vis(6.0, 19.5, 0.5)
    real.append(f'<g class="{sweep_cls}"><g transform="matrix({IC:.4f} {IS:.4f} {-IC:.4f} {IS:.4f} {n(px)} {n(py)})">'
                f'<g>{"".join(wedges)}<animateTransform attributeName="transform" type="rotate" from="0" to="360" dur="3s" repeatCount="indefinite"/></g>{rings}</g></g>')

    # ------------------------------------------------------------ buildings
    cb = {q: [] for q in ("--", "-+", "+-", "++")}
    for b in drawn:
        top, left, right = (ipoly(f) for f in box_faces(b.x0, b.x1, b.y0, b.y1, 0, b.h))
        s = [f'<path class="bt" d="{top}"/><path class="bl" d="{left}"/><path class="br" d="{right}"/>']
        if b.h > 6:
            s.append(f'<path fill="url(#pl{b.sty})" d="{left}"/><path fill="url(#pr{b.sty})" d="{right}"/>')
        edges = box_edges(b.x0, b.x1, b.y0, b.y1, 0, b.h)
        if (b.x1 - b.x0) * (b.y1 - b.y0) > 55:
            m = 1.1
            edges += f"M{P([iso(b.x0 + m, b.y0 + m, b.h), iso(b.x1 - m, b.y0 + m, b.h), iso(b.x1 - m, b.y1 - m, b.h), iso(b.x0 + m, b.y1 - m, b.h)])}Z"
        s.append(f'<path class="be" d="{edges}"/>')
        rx, ry = iso((b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, b.h)
        if b.roof == "beam":
            dur = rng.uniform(2.6, 4.2)
            s.append(f'<rect x="{n(rx - 1.3)}" y="{n(ry - 150)}" width="2.6" height="150" fill="url(#beamV)" class="pulse" style="animation-duration:{dur:.1f}s"/>'
                     f'<ellipse cx="{n(rx)}" cy="{n(ry)}" rx="7" ry="4" fill="#67e8f9" fill-opacity=".35"/>')
        elif b.roof == "blink":
            s.append(f'<circle cx="{n(rx)}" cy="{n(ry - 2)}" r="1.7" fill="{ROSE}" class="blink" style="animation-delay:-{rng.uniform(0, 2):.1f}s"/>')
        cb[b.quad].append("".join(s))

    # ---------------------------------------------------------- point cloud
    shown, viewer = [], Viewer(blds)
    for p, r, kind in scan:
        X, Y = iso(*p)
        if -4 < X < W + 4 and -4 < Y < H + 4 and viewer.sees(p, X, Y):
            shown.append((X, Y, r, kind))
    groups = {}
    for X, Y, r, kind in shown:
        groups.setdefault((kind, rbin(r)), []).append((X, Y))
    uses_range, uses_paint = [], []
    for (kind, k), xy in sorted(groups.items()):
        pid = f"pc{kind}{k}"
        defs.append(f'<path id="{pid}" d="{dots(xy)}"/>')
        uses_range.append(f'<use href="#{pid}" stroke="{BIN[k]}"/>')
        uses_paint.append(f'<use href="#{pid}" stroke="{"#818cf8" if kind == "g" else "#2dd4bf"}"/>')
    circ = 2 * math.pi * 60
    defs.append(f'<mask id="reveal" maskUnits="userSpaceOnUse" x="0" y="0" width="{W}" height="{H}">'
                f'<g transform="matrix({IC:.4f} {IS:.4f} {-IC:.4f} {IS:.4f} {n(px)} {n(py)})">'
                f'<circle r="60" fill="none" stroke="#fff" stroke-width="121" stroke-dasharray="{circ:.2f} {circ:.2f}" stroke-dashoffset="{circ:.2f}">'
                f'<animate attributeName="stroke-dashoffset" values="{circ:.2f};{circ:.2f};0;0" keyTimes="0;.25;.375;1" dur="{T:g}s" repeatCount="indefinite"/>'
                '</circle></g></mask>')
    pc_range = css.vis(6.0, 12.6, 0.6)
    pc_paint = css.vis(12.0, 19.5, 0.6, hi=0.95)
    cloud = (f'<g class="{pc_range}" mask="url(#reveal)"><g transform="scale(.5)" fill="none" stroke-linecap="round" stroke-width="3.3" stroke-opacity=".95">{"".join(uses_range)}</g></g>'
             f'<g class="{pc_paint}"><g transform="scale(.5)" fill="none" stroke-linecap="round" stroke-width="3.3">{"".join(uses_paint)}</g></g>')

    # ----------------------------------------------------------------- cars
    p1_cls = css.vis(0.5, 5.7, 0.35)
    p3_cls = css.vis(12.1, 19.4, 0.4)
    p23_cls = css.vis(6.3, 19.4, 0.5)
    queue_ids = {}

    def car_body(c, s_ref, palette):
        x0, x1, y0, y1 = c.extent(s_ref)
        top_c, left_c, right_c, edge_c = palette
        zb, zt = 0.25, c.Hc * (0.62 if c.kind == "car" else 1.0)
        out = []
        faces = box_faces(x0, x1, y0, y1, zb, zt)
        out.append(f'<path fill="{top_c}" d="{poly(faces[0])}"/><path fill="{left_c}" d="{poly(faces[1])}"/><path fill="{right_c}" d="{poly(faces[2])}"/>')
        edges = box_edges(x0, x1, y0, y1, zb, zt)
        if c.kind == "car":                      # cabin, shifted toward the rear
            if c.road == "A":
                span = x1 - x0
                cx0, cx1 = (x0 + 0.18 * span, x1 - 0.32 * span) if c.sign > 0 else (x0 + 0.32 * span, x1 - 0.18 * span)
                cy0, cy1 = y0 + 0.12, y1 - 0.12
            else:
                span = y1 - y0
                cy0, cy1 = (y0 + 0.18 * span, y1 - 0.32 * span) if c.sign > 0 else (y0 + 0.32 * span, y1 - 0.18 * span)
                cx0, cx1 = x0 + 0.12, x1 - 0.12
            f2 = box_faces(cx0, cx1, cy0, cy1, zt, c.Hc)
            out.append(f'<path fill="{top_c}" d="{poly(f2[0])}"/><path fill="{left_c}" fill-opacity=".8" d="{poly(f2[1])}"/><path fill="{right_c}" fill-opacity=".8" d="{poly(f2[2])}"/>')
            edges += box_edges(cx0, cx1, cy0, cy1, zt, c.Hc)
        out.append(f'<path fill="none" stroke="{edge_c}" stroke-opacity=".7" stroke-width=".7" d="{edges}"/>')
        return "".join(out), (x0, x1, y0, y1)

    def car_group(c, sim_style=False):
        ref = 0.0
        x0, x1, y0, y1 = c.extent(ref)
        g = []
        if not sim_style:                           # headlight cone on the road ahead
            fwd = 1 if c.sign > 0 else -1
            if c.road == "A":
                xf = x1 if fwd > 0 else x0
                cone = [iso(xf, y0 + 0.2), iso(xf + fwd * 11, y0 - 1.2), iso(xf + fwd * 11, y1 + 1.2), iso(xf, y1 - 0.2)]
                grad = "hxp" if fwd > 0 else "hxn"
            else:
                yf = y1 if fwd > 0 else y0
                cone = [iso(x0 + 0.2, yf), iso(x0 - 1.2, yf + fwd * 11), iso(x1 + 1.2, yf + fwd * 11), iso(x1 - 0.2, yf)]
                grad = "hyp" if fwd > 0 else "hyn"
            g.append(f'<path fill="url(#{grad})" d="{poly(cone)}"/>')
            trail = []                              # track history dots (fusion stage)
            for k in range(1, 6):
                d = -fwd * (c.L / 2 + 2.2 * k)
                cx, cy = (d, c.lane) if c.road == "A" else (c.lane, d)
                X, Y = iso(cx, cy, 0.1)
                trail.append(f'<circle cx="{n(X)}" cy="{n(Y)}" r="{1.9 - 0.25 * k:.2f}" fill-opacity="{0.75 - 0.13 * k:.2f}"/>')
            g.append(f'<g class="{p3_cls}" fill="{AMBER}">{"".join(trail)}</g>')
        if sim_style:
            col = {"marl": ("#d9f99d", "#65a30d", "#84cc16", "#ecfccb"), "av": ("#bae6fd", "#0369a1", "#0ea5e9", "#e0f2fe"),
                   "human": ("#f1f5f9", "#64748b", "#94a3b8", "#ffffff")}[c.type]
            if c.type == "marl":
                cx_, cy_ = iso(*c.at(ref))
                g.append(f'<ellipse cx="{n(cx_)}" cy="{n(cy_)}" rx="{n(3.6 * IC)}" ry="{n(3.6 * IS)}" fill="none" stroke="{LIME}" stroke-width="1.2" class="halo"/>')
            body, _ = car_body(c, ref, col)
        else:
            body, _ = car_body(c, ref, ("#e0f2fe", "#7dd3fc", "#38bdf8", "#f0f9ff") if c.kind == "car" else ("#ede9fe", "#a78bfa", "#8b5cf6", "#f5f3ff"))
        g.append(body)
        if not sim_style:
            # head/tail lights on whichever end faces the viewer
            end_x = c.road == "A"
            visible_front = c.sign > 0
            lamp = "#fef9c3" if visible_front else ROSE
            if end_x:
                xe = x1
                pts = [iso(xe, y0 + 0.35, 0.65), iso(xe, y1 - 0.35, 0.65)]
            else:
                ye = y1
                pts = [iso(x0 + 0.35, ye, 0.65), iso(x1 - 0.35, ye, 0.65)]
            g.append("".join(f'<circle cx="{n(X)}" cy="{n(Y)}" r="1.25" fill="{lamp}"/>' for X, Y in pts))
            # LiDAR returns on the body (stages 2-3)
            pts3 = []
            for i in range(5):
                for j in range(2):
                    u, v = (i + 0.5) / 5, (j + 0.5) / 2
                    pts3.append(iso(x0 + (x1 - x0) * u, y0 + (y1 - y0) * v, c.Hc * 0.62 + 0.05))
                    if c.road == "A":
                        pts3.append(iso(x0 + (x1 - x0) * u, y1, 0.35 + 0.5 * j))
                    else:
                        pts3.append(iso(x1, y0 + (y1 - y0) * u, 0.35 + 0.5 * j))
            dd = dots(pts3)
            g.append(f'<g class="{p23_cls}" transform="scale(.5)" fill="none" stroke-linecap="round" stroke-width="3.2">'
                     f'<path class="{pc_range}" stroke="{BIN[3]}" d="{dd}"/><path class="{pc_paint}" stroke="{AMBER}" d="{dd}"/></g>')
            # 3D boxes: magenta for the calibration matches, amber tracks in fusion
            wire = box_wire(x0 - 0.3, x1 + 0.3, y0 - 0.3, y1 + 0.3, 0, c.Hc + 0.3)
            lx, ly = iso(*c.at(ref), c.Hc + 1.9)
            if c.id in queue_ids:
                g.append(f'<g class="{p1_cls}"><path d="{wire}" fill="none" stroke="{MAG}" stroke-width="1.1"/>'
                         f'<circle cx="{n(lx)}" cy="{n(ly)}" r="7.5" fill="#1e0b2e" stroke="{MAG}"/>{text(lx, ly + 3.6, str(queue_ids[c.id]), 10.5, "#fdf4ff", 700, "middle", True)}</g>')
            g.append(f'<g class="{p3_cls}"><path d="{wire}" fill="{AMBER}" fill-opacity=".06" stroke="{AMBER}" stroke-width="1"/>'
                     f'<rect x="{n(lx - 15)}" y="{n(ly - 7)}" width="30" height="13" rx="3" fill="#1c1406" stroke="{AMBER}" stroke-opacity=".7"/>'
                     f'{text(lx, ly + 3, f"#{c.id:02d}", 9.5, "#fde68a", 600, "middle", True)}</g>')
        return "".join(g)

    # which queued cars does the camera see? (they get matched in stage 1)
    visible_q = []
    for c in cars:
        if c.road == "B" and c.sign > 0 and c.v0 == 0 and c.kind == "car":
            x, y = c.at(c.s0)
            uv = cam.project((x, y, 0.8))
            if uv and 20 < uv[0] < cam.w - 20 and 20 < uv[1] < cam.h - 10:
                visible_q.append(c)
    visible_q.sort(key=lambda c: -c.s0)
    for i, c in enumerate(visible_q[:4]):
        queue_ids[c.id] = i + 1

    layers = {"A": [], "B": []}
    for c in cars:
        frames = [(t, *(a - b for a, b in zip(iso(*c.at(s)), iso(*c.at(0))))) for t, s in c.frames]
        c.move_cls = css.move(frames)
        layers[c.road].append(f'<g class="{c.move_cls}">{car_group(c)}</g>')

    # --------------------------------------------------- pole, lights, camera
    def pole(v2x=False):
        bx, by = iso(*RSU)
        tx, ty = iso(RSU[0], RSU[1], 7.0)
        ax, ay = iso(CAM[0], CAM[1], CAM[2] + 0.2)
        g = [f'<ellipse cx="{n(bx)}" cy="{n(by)}" rx="5" ry="3" fill="{MAG if v2x else CYAN}" fill-opacity=".35"/>',
             f'<path d="M{n(bx)} {n(by)}V{n(ty)}L{n(ax)} {n(ay)}" stroke="#e2e8f0" stroke-width="2.2" fill="none" stroke-linecap="round"/>']
        cx0, cy0 = iso(*LIDAR)
        rx, ry = 0.5 * S * 1.2247, 0.5 * S * 0.7071
        g.append(f'<rect x="{n(cx0 - rx)}" y="{n(cy0 - 4)}" width="{n(2 * rx)}" height="4" fill="#0f172a" stroke="#e2e8f0" stroke-width=".8"/>'
                 f'<ellipse cx="{n(cx0)}" cy="{n(cy0 - 4)}" rx="{n(rx)}" ry="{n(ry)}" fill="#1e293b" stroke="#a5f3fc" stroke-width=".9"/>'
                 f'<ellipse cx="{n(cx0)}" cy="{n(cy0 - 2)}" rx="{n(rx + 2)}" ry="{n(ry + 1.2)}" fill="none" stroke="{MAG if v2x else CYAN}" stroke-width="1.2" class="pulse"/>')
        kx, ky = iso(*CAM)
        g.append(f'<rect x="{n(kx - 3.5)}" y="{n(ky - 2.5)}" width="7" height="5" rx="1" fill="#0f172a" stroke="#e2e8f0" stroke-width=".8"/>'
                 f'<circle cx="{n(kx - 2.2)}" cy="{n(ky - 0.6)}" r="1.4" fill="#a5f3fc"/>')
        return "".join(g)

    lights = []
    for sx, sy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
        x, y = 9.6 * sx, 9.6 * sy
        if (sx, sy) == (1, 1):
            x, y = 12.6, 8.6
        bx, by = iso(x, y)
        tx, ty = iso(x, y, 4.6)
        lamps = []
        for dx, road in ((-3.2, "A"), (3.2, "B")):
            if road == "A":
                g_on, r_on = css.vis(0, 4.8, 0.15), css.vis(5.6, 23.8, 0.15)
                y_on = css.vis(4.8, 5.6, 0.1)
            else:
                g_on, r_on, y_on = css.vis(7.0, 23.8, 0.15), css.vis(0, 7.0, 0.15), None
            lamps.append(f'<circle cx="{n(tx + dx)}" cy="{n(ty)}" r="2.3" fill="#10b981" class="{g_on}"/><circle cx="{n(tx + dx)}" cy="{n(ty)}" r="2.3" fill="#f43f5e" class="{r_on}"/>'
                         + (f'<circle cx="{n(tx + dx)}" cy="{n(ty)}" r="2.3" fill="#f59e0b" class="{y_on}"/>' if y_on else ""))
        lights.append(f'<path d="M{n(bx)} {n(by)}V{n(ty)}" stroke="#94a3b8" stroke-width="1.3"/><rect x="{n(tx - 6.2)}" y="{n(ty - 3.4)}" width="12.4" height="6.8" rx="3.4" fill="#020617" stroke="#475569" stroke-width=".6"/>{"".join(lamps)}')
    # Occlusion by draw order instead of masks: road-A cars only ever overlap blocks with y > 0 in
    # front of them and road-B cars blocks with x > 0, and the (+,-)/(-,+) blocks never overlap the
    # other road on screen, so  (-,-) < A cars < (-,+) < B cars < (+,-) < (+,+)  is a valid order.
    dim = lambda q: f'<g class="{city_dim}">{"".join(cb[q])}</g>'
    real.append(dim("--") + f'<g class="{city_dim}">{"".join(lights)}</g>' + "".join(layers["A"]) + dim("-+")
                + "".join(layers["B"]) + dim("+-") + dim("++") + pole() + cloud)

    # camera frustum (stage 1)
    far = []
    for u, v in ((0, 0), (cam.w, 0), (cam.w, cam.h), (0, cam.h)):
        d = cam.ray(u, v)
        t = 40.0 if d[2] >= 0 else min(40.0, -CAM[2] / d[2])
        far.append(add(CAM, mul(d, t)))
    k0 = iso(*CAM)
    fr = [iso(*p) for p in far]
    fr_cls = css.vis(0.3, 5.8, 0.4)
    footprint = []                                 # visible ground: border rays that land within 48 m
    border = [(u, cam.h) for u in range(0, cam.w + 1, 16)] + [(0, v) for v in range(0, cam.h, 8)] + [(cam.w, v) for v in range(0, cam.h, 8)]
    for u, v in border:
        d = cam.ray(u, v)
        flat_d = math.hypot(d[0], d[1])
        t = 48.0 / flat_d if d[2] >= 0 else min(48.0 / flat_d, -CAM[2] / d[2])
        footprint.append((CAM[0] + d[0] * t, CAM[1] + d[1] * t))
    foot = [iso(x, y) for x, y in hull(footprint + [CAM[:2]])]
    frustum = f'<path d="{poly(foot)}" fill="{CYAN}" fill-opacity=".08" stroke="#67e8f9" stroke-opacity=".3"/>'
    frustum += "".join(f'<path d="M{P([k0, fr[i], fr[(i + 1) % 4]])}Z" fill="{CYAN}" fill-opacity=".04"/>' for i in range(4))
    frustum += f'<path d="{"".join(f"M{P([k0, q])}" for q in fr)}M{P(fr)}Z" fill="none" stroke="#67e8f9" stroke-opacity=".75" stroke-width=".9" stroke-dasharray="4 3"/>'
    real.append(f'<g class="{fr_cls}">{frustum}</g>')

    # ambient particles and a light fog on the far skyline
    parts = []
    for _ in range(26):
        x, y = rng.uniform(20, 1180), rng.uniform(90, 560)
        parts.append(f'<circle cx="{n(x)}" cy="{n(y)}" r="{rng.uniform(.6, 1.5):.1f}" fill="#a5f3fc" class="float" '
                     f'style="animation-duration:{rng.uniform(7, 13):.1f}s;animation-delay:-{rng.uniform(0, 13):.1f}s"/>')
    real.append(f'<g>{"".join(parts)}</g><rect width="{W}" height="260" fill="url(#fog)"/>')

    # ----------------------------------------------------------- sim layer
    sim.append(f'<rect width="{W}" height="{H}" fill="#0b111c"/>')
    sim.append(f'<g transform="{G}"><rect x="-200" y="-200" width="400" height="400" fill="url(#sgrid)"/></g>')
    sim.append(roads(True))
    lanes = []                                     # SUMO lane graph
    for road, sign, lane in (("A", 1, 1.75), ("A", 1, 5.25), ("A", -1, -1.75), ("A", -1, -5.25),
                             ("B", 1, -1.75), ("B", 1, -5.25), ("B", -1, 1.75), ("B", -1, 5.25)):
        c = Car(road, sign, lane, 0)
        a, b = c.at(-160), c.at(160)
        lanes.append(f"M{n(a[0])} {n(a[1])}L{n(b[0])} {n(b[1])}")
        for s in range(-140, 160, 18):
            p0, p1, p2 = c.at(s - 1.2), c.at(s), c.at(s - 1.2)
            if c.road == "A":
                p0, p2 = (p0[0], p0[1] - 0.8), (p2[0], p2[1] + 0.8)
            else:
                p0, p2 = (p0[0] - 0.8, p0[1]), (p2[0] + 0.8, p2[1])
            lanes.append(f"M{n(p0[0])} {n(p0[1])}L{n(p1[0])} {n(p1[1])}L{n(p2[0])} {n(p2[1])}")
    sim.append(f'<g transform="{G}"><path d="{"".join(lanes)}" fill="none" stroke="#4ade80" stroke-opacity=".55" stroke-width=".28" stroke-linejoin="round"/></g>')
    shadow = []
    for b in drawn:
        off = (0.12 * b.h, 0.5 * b.h)
        fp = [(b.x0, b.y0), (b.x1, b.y0), (b.x1, b.y1), (b.x0, b.y1)]
        shadow.append(ipoly([iso(x, y) for x, y in hull(fp + [(x + off[0], y + off[1]) for x, y in fp])]))
    sim.append(f'<path fill="#000" fill-opacity=".38" d="{"".join(shadow)}"/>')
    corridors = []
    for a in agents:
        t_in = a.tc - (9 + a.L / 2) / a.V
        t_out = a.tc + (9 + a.L / 2) / a.V
        c0, c1 = a.at(-9.5), a.at(9.5)
        if a.road == "A":
            q = [iso(c0[0], a.lane - 1.5), iso(c1[0], a.lane - 1.5), iso(c1[0], a.lane + 1.5), iso(c0[0], a.lane + 1.5)]
        else:
            q = [iso(a.lane - 1.5, c0[1]), iso(a.lane + 1.5, c0[1]), iso(a.lane + 1.5, c1[1]), iso(a.lane - 1.5, c1[1])]
        col = {"marl": LIME, "av": SKY, "human": "#e2e8f0"}[a.type]
        corridors.append(f'<path class="{css.vis(max(18.1, t_in - 0.7), min(23.9, t_out), 0.25)}" d="{poly(q)}" fill="{col}" fill-opacity=".22" stroke="{col}" stroke-opacity=".6" stroke-width=".8"/>')
    sim.append("".join(corridors))
    sb = {q: [] for q in ("--", "-+", "+-", "++")}
    for b in drawn:
        top, left, right = (ipoly(f) for f in box_faces(b.x0, b.x1, b.y0, b.y1, 0, b.h))
        sb[b.quad].append(f'<path class="sa" d="{top}"/><path class="sb" d="{left}"/><path class="sc" d="{right}"/>')
    sdim = lambda q: f'<g class="se">{"".join(sb[q])}</g>'
    alay = {"A": [], "B": []}
    v2x = []
    rsu_top = iso(*LIDAR)
    for a in agents:
        f0 = (0, *(p - q for p, q in zip(iso(*a.at(a.s_at(0))), iso(*a.at(0)))))
        f1 = (T, *(p - q for p, q in zip(iso(*a.at(a.s_at(T))), iso(*a.at(0)))))
        alay[a.road].append(f'<g class="{css.move([f0, f1])}">{car_group(a, True)}</g>')
        if a.type == "marl":
            e0, e1 = iso(*a.at(a.s_at(0)), 1.6), iso(*a.at(a.s_at(T)), 1.6)
            on, off = a.tc - 38 / a.V, a.tc + 26 / a.V
            v2x.append(f'<line class="{css.vis(max(18.4, on), min(23.9, off), 0.3)}" x1="{n(rsu_top[0])}" y1="{n(rsu_top[1])}" x2="{n(e0[0])}" y2="{n(e0[1])}" '
                       f'stroke="#f0abfc" stroke-width="1.5" stroke-dasharray="4 3"><animate attributeName="x2" values="{n(e0[0])};{n(e1[0])}" dur="{T:g}s" repeatCount="indefinite"/>'
                       f'<animate attributeName="y2" values="{n(e0[1])};{n(e1[1])}" dur="{T:g}s" repeatCount="indefinite"/>'
                       f'<animate attributeName="stroke-dashoffset" values="0;-24" dur="1s" repeatCount="indefinite"/></line>')
    sim.append(sdim("--") + "".join(alay["A"]) + sdim("-+") + "".join(alay["B"]) + sdim("+-") + sdim("++"))
    rings = "".join(f'<circle r="3" fill="none" stroke="{MAG}" stroke-width=".35"><animate attributeName="r" values="2;34" dur="2.4s" begin="{b}s" repeatCount="indefinite"/>'
                    f'<animate attributeName="stroke-opacity" values=".8;0" dur="2.4s" begin="{b}s" repeatCount="indefinite"/></circle>' for b in (0, .8, 1.6))
    sim.append(f'<g transform="matrix({IC:.4f} {IS:.4f} {-IC:.4f} {IS:.4f} {n(px)} {n(py)})">{rings}</g>{pole(True)}{"".join(v2x)}')

    # ------------------------------------------------------------ camera frame
    cw, ch = cam.w, cam.h
    img = [f'<rect width="{cw}" height="{ch}" fill="url(#sky)"/>']
    prims = []                                     # (depth, svg) painter's list

    def cam_poly(world, fill, extra=""):
        pts2, depth = cam.polygon(world)
        if pts2:
            prims.append((depth, f'<path fill="{fill}"{extra} d="{poly(pts2)}"/>'))

    def flat(world, fill, extra=""):
        pts2, _ = cam.polygon(world)
        return f'<path fill="{fill}"{extra} d="{poly(pts2)}"/>' if pts2 else ""

    ground_svg = [flat([(-260, -260, 0), (260, -260, 0), (260, 260, 0), (-260, 260, 0)], "#0a1222")]
    ground_svg.append(flat([(-260, -7, 0), (260, -7, 0), (260, 7, 0), (-260, 7, 0)], "#131d31"))
    ground_svg.append(flat([(-7, -260, 0), (7, -260, 0), (7, 260, 0), (-7, 260, 0)], "#131d31"))
    for x0, x1, y0, y1 in plazas:
        ground_svg.append(flat([(x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0)], "#0d2331"))
    marks = []
    for s in (-1, 1):
        for lat in (-3.5, 3.5):
            for k in range(24):
                a0_, a1_ = sorted((s * (13.5 + 9 * k), s * (16.5 + 9 * k)))
                marks.append([(a0_, lat - 0.08, 0), (a1_, lat - 0.08, 0), (a1_, lat + 0.08, 0), (a0_, lat + 0.08, 0)])
                marks.append([(lat - 0.08, a0_, 0), (lat + 0.08, a0_, 0), (lat + 0.08, a1_, 0), (lat - 0.08, a1_, 0)])
        lo_, hi_ = sorted((s * 9.8, s * 12.8))
        for k in range(13):
            c = -7 + 0.3 + k * 1.1
            marks.append([(lo_, c, 0), (hi_, c, 0), (hi_, c + 0.55, 0), (lo_, c + 0.55, 0)])
            marks.append([(c, lo_, 0), (c + 0.55, lo_, 0), (c + 0.55, hi_, 0), (c, hi_, 0)])
    ground_svg.append(f'<g fill="#cbd5e1" fill-opacity=".55">{"".join(flat(m, "#cbd5e1") for m in marks)}</g>')
    tones = {(0, 0, 1): "#3d567d", (1, 0, 0): "#2d4468", (-1, 0, 0): "#293f62", (0, 1, 0): "#24395b", (0, -1, 0): "#213555"}
    for b in blds:
        for world, nrm in world_faces(b.x0, b.x1, b.y0, b.y1, 0, b.h):
            centre = tuple(sum(p[i] for p in world) / 4 for i in range(3))
            if dot(nrm, sub(CAM, centre)) <= 0:
                continue
            pts2, depth = cam.polygon(world)
            if not pts2 or all(p[0] < -40 or p[0] > cw + 40 for p in pts2) or all(p[1] < -40 or p[1] > ch + 40 for p in pts2):
                continue
            fill = hexmix(tones[nrm], "#4f6a96", min(0.8, (depth / 170) ** 0.85))
            prims.append((depth, f'<path fill="{fill}" stroke="#93c5fd" stroke-opacity="{max(0.1, 0.55 - depth / 260):.2f}" stroke-width=".6" d="{poly(pts2)}"/>'))
            if nrm[2] == 0 and depth < 120:          # floor slabs and a scatter of lit windows
                a, bb = world[0], world[1]
                floors, glow = [], []
                for z in [3.5 * k for k in range(1, int(b.h / 3.5))]:
                    pa, pb = cam.project((a[0], a[1], z)), cam.project((bb[0], bb[1], z))
                    if pa and pb:
                        floors.append(f"M{n(pa[0])} {n(pa[1])}L{n(pb[0])} {n(pb[1])}")
                    span = math.dist(a[:2], bb[:2])
                    for k in range(int(span / 2.6)):
                        if rng.random() < 0.22:
                            u = (k + 0.5) * 2.6 / span
                            q = cam.project((a[0] + (bb[0] - a[0]) * u, a[1] + (bb[1] - a[1]) * u, z - 1.6))
                            if q and 0 <= q[0] <= cw and 0 <= q[1] <= ch:
                                glow.append(q)
                if floors:
                    prims.append((depth - 1e-3, f'<path fill="none" stroke="#bfdbfe" stroke-opacity="{max(0.06, 0.3 - depth / 400):.2f}" stroke-width=".5" d="{"".join(floors)}"/>'))
                if glow:
                    warm = rng.random() < 0.4
                    prims.append((depth - 2e-3, f'<path fill="none" stroke="{"#fde68a" if warm else "#bae6fd"}" stroke-opacity=".75" stroke-linecap="round" '
                                                f'stroke-width="{max(1.2, 3.6 - depth / 40):.1f}" transform="scale(.5)" d="{dots(glow)}"/>'))
    boxes2d = {}
    for c in visible_q[:4]:
        x0, x1, y0, y1 = c.extent(c.s0)
        for world, nrm in world_faces(x0, x1, y0, y1, 0.25, c.Hc):
            centre = tuple(sum(p[i] for p in world) / 4 for i in range(3))
            if dot(nrm, sub(CAM, centre)) > 0:
                cam_poly(world, hexmix({(0, 0, 1): "#dbe7f3", (1, 0, 0): "#9fb3c8", (-1, 0, 0): "#8ea3ba", (0, 1, 0): "#7f95ad", (0, -1, 0): "#7890a8"}[nrm], "#34496e", 0.25))
        uv = [cam.project((x, y, z)) for x in (x0, x1) for y in (y0, y1) for z in (0.2, c.Hc)]
        us, vs = [p[0] for p in uv], [p[1] for p in uv]
        boxes2d[c.id] = (min(us) - 2, min(vs) - 2, max(us) + 2, max(vs) + 2)
    prims.sort(key=lambda p: -p[0])
    img.append("".join(ground_svg) + "".join(svg for _, svg in prims))
    defs.append(f'<g id="camImg">{"".join(img)}</g>')

    # LiDAR returns reprojected into the camera, coloured by depth
    cam_pts, cam_buckets = {}, az_buckets(blds, CAM)
    for p, r, kind in scan[::2]:
        q = cam.cam(p)
        if q[2] < 2:
            continue
        u, v = cam.px(q)
        if not (0 <= u <= cw and 0 <= v <= ch):
            continue
        ray = sub(p, CAM)
        dist = math.sqrt(dot(ray, ray))
        if first_hit(CAM, unit(ray), cam_buckets[az_bin(ray)], dist * 0.985)[1] is not None:
            continue
        cam_pts.setdefault(rbin(q[2], 4, 70), []).append((u, v))
    for c in visible_q[:4]:
        x0, x1, y0, y1 = c.extent(c.s0)
        for i in range(4):
            for j in range(3):
                uv = cam.project((x0 + (x1 - x0) * (j + 0.5) / 3, y1, 0.4 + 0.3 * i))
                if uv:
                    cam_pts.setdefault(1, []).append(uv)
    cam_dots = "".join(f'<path stroke="{BIN[k]}" d="{dots(xy)}"/>' for k, xy in sorted(cam_pts.items()))

    # ------------------------------------------------------------------ HUD
    X0, CWD = PX + 16, PW - 32
    hud.append(f'<rect x="{PX}" y="{PY}" width="{PW}" height="{PH}" rx="16" fill="#050d1f" fill-opacity=".82"/>'
               f'<rect x="{PX + .5}" y="{PY + .5}" width="{PW - 1}" height="{PH - 1}" rx="15.5" fill="none" stroke="url(#panelB)"/>')
    heads = [("01", "LiDAR ↔ Camera Calibration", "targetless · detection-based matching"),
             ("02", "Sensor Perception", "multi-beam LiDAR sweep + camera stream"),
             ("03", "Sensor Fusion", "painted points → 3D detection & tracking"),
             ("04", "Real → Sim", "digital twin in CARLA + SUMO co-simulation")]
    for i, (num, title, subt) in enumerate(heads):
        hud.append(f'<g class="{stage(css, i)}">{text(X0, 62, num, 30, CYAN, 700, mono=True)}{text(X0 + 48, 52, title, 18, "#f8fafc", 700)}'
                   f'{text(X0 + 48, 71, subt, 12.5, MUTED)}</g>')
    hud.append(f'<path d="M{X0} 86H{X0 + CWD}" stroke="#1e3a5f"/>')

    def chips(items, y, color=CYAN):
        out, x = [], X0
        for label in items:
            w = 14 + len(label) * 6.6
            if x + w > X0 + CWD:
                x, y = X0, y + 28
            out.append(f'<rect x="{n(x)}" y="{y}" width="{n(w)}" height="21" rx="10.5" fill="{color}" fill-opacity=".1" stroke="{color}" stroke-opacity=".5"/>'
                       + text(x + w / 2, y + 14.5, label, 11.5, "#cffafe" if color == CYAN else INK, anchor="middle"))
            x += w + 7
        return "".join(out)

    # stage 1: camera frame, matching, extrinsics
    iy = 96
    defs.append(f'<clipPath id="ci"><rect width="{cw}" height="{ch}" rx="8"/></clipPath>')
    ox, oy = cw / 2, ch / 2
    mis = "translate({a}px,{b}px) rotate({r}deg) translate({c}px,{d}px) translate({e}px,{f}px)"
    off = mis.format(a=n(ox), b=n(oy), r="-2.6", c=n(-ox), d=n(-oy), e="13", f="-8")
    ok = mis.format(a=n(ox), b=n(oy), r="0", c=n(-ox), d=n(-oy), e="0", f="0")
    align = css.anim([(0, f"transform:{off}", None), (3.1, f"transform:{off}", "cubic-bezier(.3,0,.2,1)"),
                      (4.5, f"transform:{ok}", None), (T, f"transform:{ok}", None)], static=f"transform:{ok}")
    pts_cls = css.vis(1.1, 5.8, 0.4)
    b2d = []
    for c in visible_q[:4]:
        u0, v0, u1, v1 = boxes2d[c.id]
        k = queue_ids[c.id]
        b2d.append(f'<rect x="{n(u0)}" y="{n(v0)}" width="{n(u1 - u0)}" height="{n(v1 - v0)}" fill="none" stroke="{MAG}" stroke-width="1.2"/>'
                   f'<circle class="{css.vis(1.6 + 0.25 * queue_ids[c.id], 5.7, 0.3)}" cx="{n(u0)}" cy="{n((v0 + v1) / 2)}" r="2.6" fill="{MAG}"/>'
                   f'<rect x="{n(u0)}" y="{n(v0 - 12)}" width="13" height="12" fill="{MAG}"/>{text(u0 + 6.5, v0 - 2.6, str(k), 9.5, "#1e0b2e", 700, "middle", True)}')
    hud.append(f'<g class="{stage(css, 0)}"><g transform="translate({X0} {iy})"><g clip-path="url(#ci)"><use href="#camImg"/>'
               f'<g class="{pts_cls}"><g class="{align}"><g transform="scale(.5)" fill="none" stroke-linecap="round" stroke-width="3.2">{cam_dots}</g></g></g>'
               f'<g class="{css.vis(0.6, 5.8, 0.3)}">{"".join(b2d)}</g></g>'
               f'<rect width="{cw}" height="{ch}" rx="8" fill="none" stroke="#67e8f9" stroke-opacity=".45"/>'
               f'<rect x="8" y="8" width="58" height="17" rx="4" fill="#020617" fill-opacity=".8"/>{text(37, 20, "CAMERA", 9.5, "#a5f3fc", 600, "middle", True)}'
               f'<g class="{pts_cls}"><rect x="{cw - 104}" y="8" width="96" height="17" rx="4" fill="#020617" fill-opacity=".8"/>{text(cw - 56, 20, "LiDAR → image", 9.5, "#fde68a", 600, "middle", True)}</g>'
               f'<g class="{css.vis(4.5, 5.8, 0.25)}"><rect x="{cw - 92}" y="{ch - 26}" width="84" height="18" rx="4" fill="#052e16" stroke="{LIME}" stroke-opacity=".7"/>{text(cw - 50, ch - 13.5, "aligned ✓", 10.5, "#d9f99d", 700, "middle")}</g>'
               '</g></g>')
    my = iy + ch + 22
    cells = []
    for i in range(4):
        for j in range(4):
            v = 0.9 if i == j else rng.uniform(0.05, 0.3)
            cls = css.vis(1.9 + 0.12 * (i + j), 5.8, 0.25, hi=v) if i != j else css.vis(2.6 + 0.2 * i, 5.8, 0.25, hi=v)
            cells.append(f'<rect class="{cls}" x="{X0 + 16 + j * 18}" y="{my + 12 + i * 18}" width="16" height="16" rx="2" fill="{MAG if i == j else "#a78bfa"}"/>')
    labels = "".join(text(X0 + 24 + j * 18, my + 8, str(j + 1), 9, MUTED, anchor="middle", mono=True) for j in range(4))
    labels += "".join(text(X0 + 7, my + 24 + i * 18, str(i + 1), 9, MUTED, anchor="middle", mono=True) for i in range(4))
    steps = [("detect cars in both sensors", 0.5, 2.0), ("match 2D ↔ 3D detections", 1.8, 3.2),
             ("PnP + RANSAC → [R | t]", 3.0, 4.5), ("points snap onto the image", 4.4, 5.8)]
    st = []
    for k, (label, a, b) in enumerate(steps):
        y = my + 18 + k * 21
        st.append(f'<circle cx="{X0 + 108}" cy="{y - 4}" r="3" fill="#334155"/><circle class="{css.vis(a, 5.8, 0.25)}" cx="{X0 + 108}" cy="{y - 4}" r="3" fill="{CYAN}"/>'
                  + text(X0 + 118, y, label, 12, "#64748b") + f'<g class="{css.vis(a, b, 0.25)}">{text(X0 + 118, y, label, 12, INK)}</g>')
    bar = css.anim([(0, f"transform:translate({X0}px,0) scaleX(1) translate({-X0}px,0)", None),
                    (3.1, f"transform:translate({X0}px,0) scaleX(1) translate({-X0}px,0)", "cubic-bezier(.3,0,.2,1)"),
                    (4.5, f"transform:translate({X0}px,0) scaleX(.07) translate({-X0}px,0)", None),
                    (T, f"transform:translate({X0}px,0) scaleX(.07) translate({-X0}px,0)", None)],
                   static=f"transform:translate({X0}px,0) scaleX(.07) translate({-X0}px,0)")
    ey = my + 118
    hud.append(f'<g class="{stage(css, 0)}">{text(X0, my, "2D ↔ 3D similarity", 11, MUTED)}{labels}{"".join(cells)}{"".join(st)}'
               f'{text(X0, ey, "reprojection error", 11, MUTED)}<rect x="{X0}" y="{ey + 7}" width="{CWD}" height="6" rx="3" fill="#1e293b"/>'
               f'<rect class="{bar}" x="{X0}" y="{ey + 7}" width="{CWD}" height="6" rx="3" fill="{MAG}"/>'
               f'{chips(["targetless", "ONNX · TensorRT", "Jetson edge", "one-shot adaptation"], ey + 30)}'
               f'{text(X0, PY + PH - 16, "radar-lab / XCalib", 11, "#64748b", mono=True)}</g>')

    # stage 2: range image + thumbnails
    sy0 = 112
    strip = {}
    for (i, j), r in grid.items():
        if i % 2 == 0 and j % 3 == 0:
            strip.setdefault(rbin(r), []).append((X0 + CWD * j / NAZ, sy0 + (CH - 2 - i) / 2 * 5 + 2))
    defs.append(f'<clipPath id="cs"><rect x="{X0}" y="{sy0 - 2}" width="0" height="86"><animate attributeName="width" values="0;0;{CWD};{CWD}" keyTimes="0;.25;.375;1" dur="{T:g}s" repeatCount="indefinite"/></rect></clipPath>')
    strip_svg = "".join(f'<path stroke="{BIN[k]}" d="{dots(xy)}"/>' for k, xy in sorted(strip.items()))
    ticks = "".join(text(X0 + CWD * a / 360, sy0 + 98, f"{a}°", 9, "#64748b", anchor="middle" if 0 < a < 360 else ("start" if a == 0 else "end"), mono=True) for a in (0, 90, 180, 270, 360))
    th_y, th_w, th_h = sy0 + 120, (CWD - 8) / 2, (CWD - 8) / 2 * ch / cw
    tcx, tcy, tk = X0 + th_w + 8 + th_w / 2, th_y + th_h / 2 - 6, 1.35

    def bev_small(x, y):
        return tcx + (x - y) * tk / math.sqrt(2), tcy + (x + y) * tk / math.sqrt(2)

    bev_groups = {}
    for p, r, kind in scan[::5]:
        u, v = bev_small(p[0], p[1])
        if X0 + th_w + 8 < u < X0 + CWD and th_y < v < th_y + th_h:
            bev_groups.setdefault(rbin(r), []).append((u, v))
    bsx, bsy = bev_small(*RSU)
    defs.append(f'<clipPath id="cb2"><rect x="{n(X0 + th_w + 8)}" y="{th_y}" width="{n(th_w)}" height="{n(th_h)}" rx="6"/></clipPath>'
                f'<clipPath id="cc2"><rect width="{cw}" height="{ch}" rx="12"/></clipPath>')
    hud.append(f'<g class="{stage(css, 1)}">{text(X0, sy0 - 12, "LiDAR range image · 360°", 11, MUTED)}'
               f'<rect x="{X0}" y="{sy0 - 2}" width="{CWD}" height="86" rx="4" fill="#020617"/>'
               f'<g clip-path="url(#cs)"><g transform="scale(.5)" fill="none" stroke-linecap="round" stroke-width="3.6">{strip_svg}</g></g>'
               f'<rect x="{X0}" y="{sy0 - 4}" width="2" height="90" fill="#f8fafc" fill-opacity=".85" class="cursor"/>{ticks}'
               f'<g transform="translate({X0} {th_y}) scale({th_w / cw:.4f})" clip-path="url(#cc2)"><use href="#camImg"/></g>'
               f'<rect x="{X0}" y="{th_y}" width="{n(th_w)}" height="{n(th_h)}" rx="6" fill="none" stroke="#67e8f9" stroke-opacity=".4"/>'
               f'<rect x="{n(X0 + th_w + 8)}" y="{th_y}" width="{n(th_w)}" height="{n(th_h)}" rx="6" fill="#020617"/>'
               f'<g clip-path="url(#cb2)"><g transform="scale(.5)" fill="none" stroke-linecap="round" stroke-width="2.4">'
               + "".join(f'<path stroke="{BIN[k]}" d="{dots(xy)}"/>' for k, xy in sorted(bev_groups.items())) + '</g>'
               f'<g transform="translate({n(bsx)} {n(bsy)})"><path d="M0 0L70 0" stroke="#a5f3fc" stroke-opacity=".8" stroke-width="1.2">'
               f'<animateTransform attributeName="transform" type="rotate" from="45" to="405" dur="3s" repeatCount="indefinite"/></path>'
               f'<circle r="2.6" fill="{CYAN}"/></g></g>'
               f'<rect x="{n(X0 + th_w + 8)}" y="{th_y}" width="{n(th_w)}" height="{n(th_h)}" rx="6" fill="none" stroke="#67e8f9" stroke-opacity=".4"/>'
               f'{text(X0, th_y + th_h + 16, "camera", 11, MUTED)}{text(X0 + th_w + 8, th_y + th_h + 16, "bird’s-eye returns", 11, MUTED)}'
               f'{text(X0, th_y + th_h + 44, "every beam return → x, y, z, range", 12.5, INK)}'
               f'{text(X0, th_y + th_h + 64, "camera frames time-synced to the sweep", 12.5, INK)}'
               f'{chips(["point cloud", "range view", "camera stream", "time sync"], th_y + th_h + 84)}</g>')

    # stage 3: bird's-eye fusion map with live tracks
    mx0, my0, mw, mh = X0, 96, CWD, 250
    mcx, mcy, kk = mx0 + mw / 2, my0 + mh / 2, 3.0

    def bev(x, y):
        return mcx + (x - y) * kk / math.sqrt(2), mcy + (x + y) * kk / math.sqrt(2)

    m = [f'<rect x="{mx0}" y="{my0}" width="{mw}" height="{mh}" rx="8" fill="#020617"/>']
    for x0, x1, y0, y1 in ((-200, 200, -7, 7), (-7, 7, -200, 200)):
        m.append(f'<path fill="#0f1b30" d="{poly([bev(x0, y0), bev(x1, y0), bev(x1, y1), bev(x0, y1)])}"/>')
    m.append(f'<path fill="none" stroke="#1e3a5f" stroke-width=".8" d="{"".join(poly([bev(b.x0, b.y0), bev(b.x1, b.y0), bev(b.x1, b.y1), bev(b.x0, b.y1)]) for b in blds)}"/>')
    paint = {}
    for p, r, kind in scan[::3]:
        u, v = bev(p[0], p[1])
        if mx0 < u < mx0 + mw and my0 < v < my0 + mh:
            paint.setdefault(kind, []).append((u, v))
    m.append('<g transform="scale(.5)" fill="none" stroke-linecap="round" stroke-width="2.2" stroke-opacity=".7">'
             + "".join(f'<path stroke="{"#818cf8" if k == "g" else "#2dd4bf"}" d="{dots(xy)}"/>' for k, xy in paint.items()) + "</g>")
    d0, d1 = cam.ray(0, cam.h * 0.7), cam.ray(cam.w, cam.h * 0.7)
    fov = [bev(*CAM[:2]), bev(CAM[0] + d0[0] * 40, CAM[1] + d0[1] * 40), bev(CAM[0] + d1[0] * 40, CAM[1] + d1[1] * 40)]
    m.append(f'<path d="{poly(fov)}" fill="{CYAN}" fill-opacity=".06" stroke="#67e8f9" stroke-opacity=".45" stroke-dasharray="3 3"/>')
    sxy = bev(*RSU)
    m.append(f'<circle cx="{n(sxy[0])}" cy="{n(sxy[1])}" r="3.5" fill="{CYAN}"/><circle cx="{n(sxy[0])}" cy="{n(sxy[1])}" r="8" fill="none" stroke="{CYAN}" class="pulse"/>')
    tracks = []
    ax, ay = kk / math.sqrt(2) / IC, kk / math.sqrt(2) / IS      # BEV offset = iso offset scaled per axis
    for c in cars:
        x0, x1, y0, y1 = c.extent(0)
        box = poly([bev(x0, y0), bev(x1, y0), bev(x1, y1), bev(x0, y1)])
        tail = []
        for k in range(5):
            u, v = bev(*c.at(-(c.L / 2 + 1.6 + 2.6 * k)))
            tail.append(f'<circle cx="{n(u)}" cy="{n(v)}" r="{1.4 - 0.2 * k:.1f}" fill-opacity="{0.7 - 0.12 * k:.2f}"/>')
        tail = "".join(tail)
        lx, ly = bev(*c.at(0))
        tracks.append(f'<g transform="scale({ax:.5f} {ay:.5f})"><g class="{c.move_cls}"><g transform="scale({1 / ax:.5f} {1 / ay:.5f})">'
                      f'<g fill="{AMBER}">{tail}</g><path d="{box}" fill="{AMBER}" fill-opacity=".35" stroke="{AMBER}" stroke-width="1"/>'
                      f'{text(lx + 7, ly - 5, f"#{c.id:02d}", 8.5, "#fde68a", 600, mono=True)}</g></g></g>')
    defs.append(f'<clipPath id="cm"><rect x="{mx0}" y="{my0}" width="{mw}" height="{mh}" rx="8"/></clipPath>')
    ly3 = my0 + mh + 22
    hud.append(f'<g class="{stage(css, 2)}"><g clip-path="url(#cm)">{"".join(m)}{"".join(tracks)}</g>'
               f'<rect x="{mx0}" y="{my0}" width="{mw}" height="{mh}" rx="8" fill="none" stroke="#67e8f9" stroke-opacity=".4"/>'
               f'<rect x="{mx0 + 8}" y="{my0 + 8}" width="104" height="17" rx="4" fill="#020617" fill-opacity=".85"/>{text(mx0 + 60, my0 + 20, "BEV · FUSED", 9.5, "#a5f3fc", 600, "middle", True)}'
               f'<rect x="{X0}" y="{ly3 - 9}" width="12" height="8" fill="{AMBER}" fill-opacity=".4" stroke="{AMBER}"/>{text(X0 + 18, ly3, "tracked car", 11, MUTED)}'
               f'<circle cx="{X0 + 110}" cy="{ly3 - 5}" r="3" fill="#2dd4bf"/>{text(X0 + 118, ly3, "building", 11, MUTED)}'
               f'<circle cx="{X0 + 186}" cy="{ly3 - 5}" r="3" fill="#818cf8"/>{text(X0 + 194, ly3, "road", 11, MUTED)}'
               f'<path d="M{X0 + 246} {ly3 - 1}l6 -9l6 9z" fill="{CYAN}" fill-opacity=".2" stroke="#67e8f9"/>{text(X0 + 262, ly3, "camera", 11, MUTED)}'
               f'{text(X0, ly3 + 28, "paint LiDAR points with image semantics,", 12.5, INK)}'
               f'{text(X0, ly3 + 48, "then 3D boxes → multi-object tracking", 12.5, INK)}'
               f'{chips(["point painting", "3D detection", "multi-object tracking", "BEV"], ly3 + 68)}</g>')

    # stage 4: co-simulation diagram + MARL return curve
    def node(x, y, w, h, title, subt, color):
        return (f'<rect x="{n(x)}" y="{n(y)}" width="{n(w)}" height="{n(h)}" rx="7" fill="{color}" fill-opacity=".1" stroke="{color}" stroke-opacity=".6"/>'
                + text(x + w / 2, y + 16, title, 12, "#f8fafc", 700, "middle") + text(x + w / 2, y + 30, subt, 10, MUTED, anchor="middle"))

    dy = 100
    diag = [node(X0, dy, 150, 38, "real junction", "RSU · tracks", CYAN), node(X0 + CWD - 150, dy, 150, 38, "digital twin", "map · actors · V2X", MAG),
            node(X0, dy + 74, 150, 38, "CARLA", "3D world · sensors", SKY), node(X0 + CWD - 150, dy + 74, 150, 38, "SUMO", "traffic · lanes", "#4ade80")]
    ar = [f"M{X0 + 152} {dy + 19}H{X0 + CWD - 154}", f"M{X0 + CWD - 75} {dy + 40}V{dy + 72}", f"M{X0 + CWD - 100} {dy + 40}L{X0 + 110} {dy + 72}",
          f"M{X0 + 152} {dy + 88}H{X0 + CWD - 154}", f"M{X0 + CWD - 154} {dy + 100}H{X0 + 152}"]
    diag.append('<g fill="none" stroke="#94a3b8" stroke-width="1.2" marker-end="url(#arr)">'
                + "".join(f'<path d="{a}"/>' for a in ar) + "</g>")
    diag.append(text(X0 + CWD / 2, dy + 124, "co-sim sync ⇄", 10, "#86efac", anchor="middle", mono=True))
    defs.append('<marker id="arr" viewBox="0 0 8 8" refX="7" refY="4" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0L8 4L0 8z" fill="#94a3b8"/></marker>')
    cy0, chh = dy + 150, 104
    curve, raw = [], []
    val, rs = 0.0, random.Random(7)
    for i in range(120):
        target = 1 - math.exp(-i / 34)
        val += (target - val) * 0.25
        y = val + rs.gauss(0, 0.05) * (1.1 - val)
        raw.append((X0 + 10 + i * (CWD - 20) / 119, cy0 + chh - 12 - max(-0.05, min(1.05, y)) * (chh - 30)))
        curve.append((X0 + 10 + i * (CWD - 20) / 119, cy0 + chh - 12 - val * (chh - 30)))
    def drawn(pts):
        length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:])) + 2
        cls = css.anim([(0, f"stroke-dashoffset:{length:.0f}", None), (18.6, f"stroke-dashoffset:{length:.0f}", None),
                        (22.8, "stroke-dashoffset:0", None), (T, "stroke-dashoffset:0", None)], static="stroke-dashoffset:0")
        return f'class="{cls}" stroke-dasharray="{length:.0f}"'
    ly4 = cy0 + chh + 24
    hud.append(f'<g class="{stage(css, 3)}">{"".join(diag)}'
               f'<rect x="{X0}" y="{cy0}" width="{CWD}" height="{chh}" rx="8" fill="#020617"/>{text(X0 + 10, cy0 + 16, "MARL training return", 11, MUTED)}'
               f'{text(X0 + CWD - 10, cy0 + chh - 2, "episodes →", 9, "#64748b", anchor="end", mono=True)}'
               f'<path d="M{P(raw)}" fill="none" stroke="{LIME}" stroke-opacity=".25" stroke-width="1" {drawn(raw)}/>'
               f'<path d="M{P(curve)}" fill="none" stroke="{LIME}" stroke-width="2" {drawn(curve)}/>'
               f'<circle cx="{X0 + 4}" cy="{ly4 - 4}" r="4" fill="{LIME}"/>{text(X0 + 12, ly4, "MARL agent", 11, MUTED)}'
               f'<circle cx="{X0 + 102}" cy="{ly4 - 4}" r="4" fill="{SKY}"/>{text(X0 + 110, ly4, "rule-based AV", 11, MUTED)}'
               f'<circle cx="{X0 + 216}" cy="{ly4 - 4}" r="4" fill="#e2e8f0"/>{text(X0 + 224, ly4, "human-driven", 11, MUTED)}'
               f'{text(X0, ly4 + 26, "signal-free junction, V2X-coordinated", 12.5, INK)}'
               f'{chips(["OpenCDA-MARL", "RA-L 2026", "V2X", "CTDE", "mixed autonomy"], ly4 + 44, MAG)}</g>')

    # matching lines from the camera boxes to the 3D boxes in the scene (stage 1)
    lines = []
    for c in visible_q[:4]:
        u0, v0, u1, v1 = boxes2d[c.id]
        a = (X0 + u0, iy + (v0 + v1) / 2)
        x, y = c.at(c.s0)
        b = iso(x, y, c.Hc + 0.3)
        mid = ((a[0] + b[0]) / 2, min(a[1], b[1]) - 60)
        k = queue_ids[c.id]
        lines.append(f'<g class="{css.vis(1.6 + 0.25 * k, 5.7, 0.3)}"><path class="dash" d="M{n(a[0])} {n(a[1])}Q{n(mid[0])} {n(mid[1])} {n(b[0])} {n(b[1])}" '
                     f'fill="none" stroke="{MAG}" stroke-width="1.2" stroke-dasharray="5 4"/><circle cx="{n(b[0])}" cy="{n(b[1])}" r="2.4" fill="{MAG}"/></g>')
    under = f'<g class="{stage(css, 0)}">{"".join(lines)}</g>'       # under the HUD: they emerge from the panel edge

    # ------------------------------------------------------ title + stepper
    over.append(f'<rect width="700" height="126" fill="url(#scrimT)"/><rect y="560" width="{W}" height="115" fill="url(#scrimB)"/>')
    over.append(text(28, 42, "RESEARCH PIPELINE", 12, "#67e8f9", 600, extra=' letter-spacing="3"')
                + text(28, 74, "Roadside sensing → digital twin", 27, "#f8fafc", 800)
                + text(28, 98, "LiDAR–camera calibration · perception · fusion · real-to-sim", 14, MUTED))
    names = [("Calibrate", "targetless LiDAR ↔ camera"), ("Perceive", "LiDAR sweep · camera"),
             ("Fuse", "3D detection · tracking"), ("Real → Sim", "CARLA + SUMO · MARL")]
    step_svg = [f'<path d="M36 596H1164" stroke="#1e3a5f" stroke-width="2"/>']
    for i, (a, b) in enumerate(names):
        x = 56 + i * 288
        on = stage(css, i)
        step_svg.append(f'<rect class="{on}" x="{x - 20}" y="595" width="276" height="3" rx="1.5" fill="{CYAN}"/>'
                        f'<circle cx="{x}" cy="630" r="15" fill="#020617" stroke="#334155" stroke-width="1.5"/>'
                        f'<g class="{on}"><circle cx="{x}" cy="630" r="15" fill="{CYAN}" fill-opacity=".16" stroke="{CYAN}" stroke-width="1.5"/>'
                        f'<circle cx="{x}" cy="630" r="21" fill="none" stroke="{CYAN}" stroke-opacity=".35" class="pulse"/></g>'
                        + text(x, 634.5, f"0{i + 1}", 12, "#e2e8f0", 700, "middle", True)
                        + text(x + 26, 626, a, 15.5, "#f8fafc", 700) + text(x + 26, 644, b, 12, MUTED))
    head = css.anim([(0, "transform:translate(0px,0)", None), (T, "transform:translate(1128px,0)", None)], static="transform:translate(564px,0)")
    step_svg.append(f'<g class="{head}"><circle cx="36" cy="596.5" r="3.5" fill="#f8fafc"/></g>')
    over.append("".join(step_svg))

    # ------------------------------------------------------------ assemble
    wipe = ('<clipPath id="wipe"><rect x="0" y="0" width="0" height="675">'
            f'<animate attributeName="width" values="0;0;{W};{W}" keyTimes="0;.75;.8083;1" calcMode="spline" keySplines="0 0 1 1;.45 0 .55 1;0 0 1 1" dur="{T:g}s" repeatCount="indefinite"/>'
            '</rect></clipPath>')
    defs.append(wipe)
    sim_fade = css.anim([(0, "opacity:1", None), (23.35, "opacity:1", None), (T, "opacity:0", None)], static="opacity:0")
    scan_line = (f'<g opacity="0"><animate attributeName="opacity" values="0;0;1;1;0;0" keyTimes="0;.749;.75;.806;.812;1" dur="{T:g}s" repeatCount="indefinite"/>'
                 f'<g><animateTransform attributeName="transform" type="translate" values="0 0;0 0;{W} 0;{W} 0" keyTimes="0;.75;.8083;1" calcMode="spline" '
                 f'keySplines="0 0 1 1;.45 0 .55 1;0 0 1 1" dur="{T:g}s" repeatCount="indefinite"/>'
                 f'<rect x="-90" width="92" height="{H}" fill="url(#scan)"/>'
                 f'<rect x="-64" y="300" width="58" height="20" rx="4" fill="#2e1065"/>{text(-35, 314, "SIM ▸", 10.5, "#f5d0fe", 700, "middle", True)}</g></g>')
    style = f"""
text{{font-family:ui-sans-serif,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif}}
.m{{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}}
.bt{{fill:url(#gT)}}.bl{{fill:url(#gL)}}.br{{fill:url(#gR)}}
.sa{{fill:#8f9cb1}}.sb{{fill:#3a475a}}.sc{{fill:#586880}}.se path{{stroke:#e2e8f0;stroke-opacity:.22;stroke-width:.6;stroke-linejoin:round}}
.be{{fill:none;stroke:#67e8f9;stroke-opacity:.5;stroke-width:.75;stroke-linejoin:round}}
.flow{{animation:flow 1.6s linear infinite}}@keyframes flow{{to{{stroke-dashoffset:-9}}}}
.pulse{{animation:pulse 2.4s ease-in-out infinite}}@keyframes pulse{{50%{{opacity:.25}}}}
.blink{{animation:blink 2.2s steps(1) infinite}}@keyframes blink{{0%{{opacity:.15}}75%{{opacity:1}}90%{{opacity:.15}}}}
.float{{animation:float 10s ease-in-out infinite}}@keyframes float{{0%,100%{{transform:translateY(0);opacity:.15}}50%{{transform:translateY(-26px);opacity:.8}}}}
.halo{{animation:pulse 1.2s ease-in-out infinite}}
.dash{{animation:dash 1s linear infinite}}@keyframes dash{{to{{stroke-dashoffset:-18}}}}
.cursor{{animation:cursor 3s linear infinite}}@keyframes cursor{{to{{transform:translateX({CWD}px)}}}}
{"".join(css.rules)}
@media (prefers-reduced-motion:reduce){{*{{animation:none!important}}}}
"""
    desc = ("Animated loop of a roadside LiDAR and camera unit at a city junction: targetless LiDAR-camera calibration, "
            "LiDAR sweep and range image, sensor fusion with 3D boxes and tracks, then a wipe into a CARLA and SUMO "
            "digital twin where MARL agents cross the junction without signals.")
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" aria-labelledby="ttl dsc">'
           f'<title id="ttl">Roadside sensing to digital twin: calibration, perception, fusion, real-to-sim</title><desc id="dsc">{desc}</desc>'
           f'<style>{style}</style><defs>{"".join(defs)}</defs>'
           f'<g clip-path="url(#card)"><g>{"".join(real)}</g>'
           f'<g clip-path="url(#wipe)"><g class="{sim_fade}">{"".join(sim)}</g></g>{scan_line}'
           f'<rect width="{W}" height="{H}" fill="url(#vig)"/>{under}{"".join(hud)}{"".join(over)}</g>'
           f'<rect x=".5" y=".5" width="{W - 1}" height="{H - 1}" rx="17.5" fill="none" stroke="#67e8f9" stroke-opacity=".22"/></svg>')
    xml.dom.minidom.parseString(svg)              # GitHub serves it to <img>, which needs well-formed XML
    Path(out_path).write_text(svg, encoding="utf-8", newline="\n")
    return len(svg), len(blds), len(shown), len(cars), len(agents)


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else Path(__file__).with_name("pipeline.svg")
    size, nb, npts, nc, na = build(out)
    print(f"{out}: {size / 1024:.0f} KiB, {nb} buildings, {npts} points, {nc} cars, {na} agents")
