"""A demonstration plan out of the Howcast overhand clip: the rope's centreline, its crossings, the hands, the grasps.

    /srv/home/bohanlyu/miniconda3/bin/python3 video/overhand/demo_extract.py [--debug DIR]

Reads ``SwdJ5op25SM.mp4`` beside this file and writes ``demo_plan.json`` and ``demo_plan.png`` beside it.

Pipeline, per sampled frame (10 fps over 15-38 s):

1. Shot: the clip is cut into shots by frame differencing; each shot is named and given the equivalent camera of
   ``REAL2SIM.md`` where one was calibrated there. Metres per pixel are measured from the rope's own width (2 x the
   distance transform at the skeleton, median, for a 10 mm cord) and cross-checked against that camera
   (f = 240 / tan(22.5 deg) = 579.4 px; table metres per pixel = camera height / f).
2. Rope: blue/red ratio (b - r > -14), not cyan (g - r < 15: the title box and the "STEP n" text are cyan-blue),
   not bright (V < 200: logo, text); the pale braid flecks closed with a disc of ~1/3 of the rope width; small
   components dropped; title graphics masked by time and place.
3. Centreline: skeleton (scikit-image), traced into branches between endpoints and junctions; spurs pruned; at a
   crossing (a junction cluster with four branches) the branches are paired by direction; strands are chained
   through crossings and then joined across hand occlusions (nearest compatible endpoint). Oriented from the
   working end's tip where it is visible.
4. Crossings: where the chained strands cross. Over/under: the intensity along each strand's axis through the
   crossing; the under strand's profile is cut by the over strand's two dark edges, the over strand's is not.
5. Hands: skin (HSV hue <= 10, S >= 100, YCrCb Cr >= 148), the two largest blobs, left/right by image x; the
   fingertip region is where a hand mask touches the rope mask; a grasp is a fingertip region that stays on the
   rope while the hand moves or holds.

Where the automatic ordering fails (under the hands, motion blur, the tuck at 29.6-30.4 s), the key shapes use a
few MANUAL seed points (``MANUAL_SEEDS``): clearly marked, and their reliability is "estimated" in the JSON.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy import ndimage
from skimage.morphology import skeletonize

HERE = Path(__file__).resolve().parent
CLIP = HERE / "SwdJ5op25SM.mp4"
FPS_SAMPLE = 10.0
SPAN = (15.0, 38.0)
ROPE_D = 0.010  # m
ROPE_LEN_ASSUMED = 1.05  # m, REAL2SIM.md's assumption (the far end is never seen)
F_PX = 240.0 / math.tan(math.radians(22.5))  # 579.4 px: REAL2SIM's 45 deg vertical fov at 640x480

# ----------------------------------------------------------------------------------------------- shots

# REAL2SIM.md's equivalent cameras: height over the table [m], image-up direction in table xy.
CAMERAS = {
    "phone": {"height": 0.077, "up": (1.0, 0.0), "centre": (0.0, 0.0)},
    "loop": {"height": 0.367, "up": (0.819152, -0.573576), "centre": (0.040, 0.0)},
    "loose": {"height": 0.205, "up": (1.0, 0.0), "centre": (-0.021, 0.0)},
    "wide": {"height": 0.495, "up": (1.0, 0.0), "centre": (0.010, 0.0)},
}

# Shots, as cut by the frame differencing below (the times are what it finds; names and cameras are ours).
SHOTS = [
    {"name": "intro_knot", "t0": 14.0, "t1": 15.8, "camera": None, "step": 0, "reliable": False,
     "note": "the finished knot (title card); not used"},
    {"name": "straight_closeup", "t0": 15.8, "t1": 19.3, "camera": "phone", "step": 0, "reliable": True,
     "note": "macro of the straight rope; the 'YOU WILL NEED / Length of rope' light-blue box 16.2-19.2 s is masked"},
    {"name": "wide_step1", "t0": 19.3, "t1": 23.8, "camera": "wide", "step": 1, "reliable": True,
     "note": "top-down wide shot of step 1; the SAME framing as the step-3 wide shot (SIFT on the wood: scale 1.000, 0.0 deg, shifted 30.5 px), so REAL2SIM's 'wide' camera; no zoom inside the shot"},
    {"name": "loop_closeup", "t0": 23.8, "t1": 26.0, "camera": "loop", "step": 1, "reliable": True,
     "note": "rotated close-up of the finished loop (REAL2SIM 'loop' camera, image up = table (0.819,-0.574))"},
    {"name": "step2_closeup", "t0": 26.0, "t1": 31.65, "camera": None, "step": 2, "reliable": True,
     "note": "macro of the tuck; no REAL2SIM camera: per-frame scale from the rope width (the shot zooms in x1.5 over 26-31 s, SIFT on the wood); the loop's far end leaves the frame; its image is taken as unrotated (image up = away from the person: the loop points up here as in the wide shot) -- an assumption"},
    {"name": "loose_closeup", "t0": 31.65, "t1": 33.5, "camera": "loose", "step": 2, "reliable": True,
     "note": "the same camera as the tuck (no cut: SIFT scale 1.00 from 31.0 to 32.0 s; the knot moved when the hand let go); still zooming (x1.07 to 33.4 s); motion blur; the working end leaves the frame on the right"},
    {"name": "wide_step3", "t0": 33.5, "t1": 36.4, "camera": "wide", "step": 3, "reliable": True,
     "note": "top-down wide shot of the pull"},
    {"name": "oblique_tight", "t0": 36.4, "t1": 39.2, "camera": None, "step": 3, "reliable": False,
     "note": "oblique beauty shot of the tight knot: not metric"},
]


def shot_at(t: float) -> dict:
    for s in SHOTS:
        if s["t0"] <= t < s["t1"]:
            return s
    return SHOTS[-1]


def detect_cuts(cap, t0=14.0, t1=39.5, fps_s=10.0, thr=12.0) -> list[float]:
    """Times where the mean absolute difference of consecutive (downsized, grey) samples exceeds ``thr``."""
    prev, cuts = None, []
    for t in np.arange(t0, t1, 1.0 / fps_s):
        im = read_frame(cap, t)
        g = cv2.cvtColor(cv2.resize(im, (160, 120)), cv2.COLOR_BGR2GRAY).astype(np.float32)
        if prev is not None and float(np.abs(g - prev).mean()) > thr:
            cuts.append(round(float(t), 2))
        prev = g
    return cuts


def read_frame(cap, t: float) -> np.ndarray:
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(round(t * fps)))
    ok, im = cap.read()
    if not ok:
        raise RuntimeError(f"no frame at {t}")
    return im


# ----------------------------------------------------------------------------------------------- masks


def graphics_mask(t: float, shape) -> np.ndarray:
    """Pixels that belong to Howcast's overlays at time t (True = not scene)."""
    h, w = shape[:2]
    m = np.zeros((h, w), bool)
    m[20:110, 480:640] = True  # the Howcast logo / "howcast.com", top right
    if 16.2 <= t < 19.25:
        m[110:320, 100:540] = True  # the light-blue "Length of rope" box (it is blue: never rope)
        m[40:100, 50:310] = True  # "YOU WILL NEED"
    return m


def badge_mask(im: np.ndarray) -> np.ndarray:
    """The "STEP n" chevrons, bottom left: dark brown-black with cyan text; found by colour inside their corner."""
    h, w = im.shape[:2]
    m = np.zeros((h, w), bool)
    roi = im[360:450, 40:260].astype(np.int32)
    dark = (roi.max(axis=2) < 70) & (roi[..., 2] >= roi[..., 0] - 5)  # the chevrons' dark fill
    cyan = (roi[..., 1] - roi[..., 2] > 15) & (roi[..., 0] > 120)  # the text
    sel = dark | cyan
    if sel.sum() > 400:
        ys, xs = np.nonzero(sel)
        x0, x1 = xs.min(), xs.max()
        y0, y1 = ys.min(), ys.max()
        m[360 + y0 - 4: 360 + y1 + 5, 40 + x0 - 4: 40 + x1 + 5] = True
    return m


def rope_mask(im: np.ndarray, t: float, width_px: float | None = None) -> np.ndarray:
    f = im.astype(np.int32)
    b, g, r = f[..., 0], f[..., 1], f[..., 2]
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    m = (b - r > -14) & (g - r < 15) & (hsv[..., 2] < 200)
    m &= ~graphics_mask(t, im.shape) & ~badge_mask(im)
    m = m.astype(np.uint8) * 255
    k = max(3, int(round((width_px or 12) / 3)) | 1)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    keep = np.zeros(m.shape, bool)
    min_area = 6.0 * (width_px or 12) ** 2
    for i in range(1, n):
        if st[i, 4] >= min_area:
            keep |= lab == i
    # Holes smaller than a rope width squared are braid flecks, not loop eyes.
    holes = ndimage.binary_fill_holes(keep) & ~keep
    hl, nh = ndimage.label(holes)
    if nh:
        sizes = ndimage.sum(holes, hl, range(1, nh + 1))
        for i, sz in enumerate(sizes, 1):
            if sz < 1.5 * (width_px or 12) ** 2:
                keep |= hl == i
    return keep


def skin_mask(im: np.ndarray, t: float, min_area: int = 3000) -> np.ndarray:
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    ycc = cv2.cvtColor(im, cv2.COLOR_BGR2YCrCb)
    m = ((hsv[..., 0] <= 10) | (hsv[..., 0] >= 175)) & (hsv[..., 1] >= 100) & (ycc[..., 1] >= 148)
    m &= ~graphics_mask(t, im.shape)
    m = m.astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    keep = np.zeros(m.shape, bool)
    for i in range(1, n):
        if st[i, 4] >= min_area:
            keep |= lab == i
    return keep


def rope_width_px(mask: np.ndarray, skel: np.ndarray | None = None) -> float:
    dt = ndimage.distance_transform_edt(mask)
    if skel is None:
        skel = skeletonize(mask)
    v = dt[skel]
    v = v[v > 1.5]
    return float(2.0 * np.median(v)) if len(v) else float("nan")


# ----------------------------------------------------------------------------------------------- skeleton graph

NB = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _neighbours(sk: np.ndarray, y: int, x: int):
    h, w = sk.shape
    for dy, dx in NB:
        yy, xx = y + dy, x + dx
        if 0 <= yy < h and 0 <= xx < w and sk[yy, xx]:
            yield yy, xx


def skeleton_graph(sk: np.ndarray, merge_r: float = 6.0):
    """Branches of a skeleton between its nodes (endpoints and junction clusters).

    Returns (nodes, branches): nodes = list of dicts {xy, kind: end|junction, branches: [...]}; branches = list of
    dicts {pts: (N,2) float array of (x, y) from node a to node b, a, b}.
    """
    sk = sk.copy().astype(bool)
    deg = ndimage.convolve(sk.astype(np.int32), np.ones((3, 3), np.int32), mode="constant") - 1
    deg[~sk] = 0
    junc = sk & (deg >= 3)
    ends = sk & (deg == 1)
    # Cluster junction pixels (those within merge_r of each other are one node).
    jl, nj = ndimage.label(ndimage.binary_dilation(junc, iterations=max(1, int(merge_r // 2))) & sk)
    node_of = -np.ones(sk.shape, np.int32)
    nodes = []
    for i in range(1, nj + 1):
        ys, xs = np.nonzero((jl == i) & junc)
        if len(xs) == 0:
            continue
        idx = len(nodes)
        nodes.append({"xy": (float(xs.mean()), float(ys.mean())), "kind": "junction", "branches": []})
        node_of[(jl == i)] = idx
    for y, x in zip(*np.nonzero(ends)):
        if node_of[y, x] >= 0:
            continue
        node_of[y, x] = len(nodes)
        nodes.append({"xy": (float(x), float(y)), "kind": "end", "branches": []})
    visited = np.zeros(sk.shape, bool)
    branches = []
    # Walk from every node-pixel neighbour that is not in a node.
    for ni, nd in enumerate(nodes):
        ys, xs = np.nonzero(node_of == ni)
        for y0, x0 in zip(ys, xs):
            for y, x in _neighbours(sk, y0, x0):
                if node_of[y, x] >= 0 or visited[y, x]:
                    continue
                path = [(x0, y0), (x, y)]
                visited[y, x] = True
                py, px = y0, x0
                cy, cx = y, x
                end_node = -1
                while True:
                    nxt = None
                    for yy, xx in _neighbours(sk, cy, cx):
                        if (yy, xx) == (py, px):
                            continue
                        if node_of[yy, xx] >= 0 and node_of[yy, xx] != ni or (node_of[yy, xx] == ni and len(path) > 3):
                            end_node = node_of[yy, xx]
                            path.append((xx, yy))
                            nxt = None
                            break
                        if not visited[yy, xx] and node_of[yy, xx] < 0:
                            nxt = (yy, xx)
                            break
                    if end_node >= 0 or nxt is None:
                        break
                    py, px = cy, cx
                    cy, cx = nxt
                    visited[cy, cx] = True
                    path.append((cx, cy))
                bi = len(branches)
                branches.append({"pts": np.asarray(path, float), "a": ni, "b": end_node})
                nd["branches"].append(bi)
                if end_node >= 0 and end_node != ni:
                    nodes[end_node]["branches"].append(bi)
    # Loops with no node at all (a closed ring): one branch from an arbitrary pixel.
    rest = sk & ~visited & (node_of < 0)
    rl, nr = ndimage.label(rest, structure=np.ones((3, 3)))
    for i in range(1, nr + 1):
        ys, xs = np.nonzero(rl == i)
        if len(xs) < 10:
            continue
        pts = _order_ring(np.stack([xs, ys], 1))
        branches.append({"pts": pts.astype(float), "a": -1, "b": -1, "ring": True})
    return nodes, branches


def _order_ring(p: np.ndarray) -> np.ndarray:
    """Order the pixels of a closed 1-px ring by walking it."""
    s = {tuple(q) for q in p.tolist()}
    cur = tuple(p[0].tolist())
    out = [cur]
    seen = {cur}
    while True:
        nxt = None
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
            q = (cur[0] + dx, cur[1] + dy)
            if q in s and q not in seen:
                nxt = q
                break
        if nxt is None:
            break
        out.append(nxt)
        seen.add(nxt)
        cur = nxt
    return np.asarray(out)


def _dir_at(pts: np.ndarray, from_start: bool, span: int) -> np.ndarray:
    """Unit direction leaving a branch's end (pointing away from that end, into the node)."""
    q = pts if from_start else pts[::-1]
    a = q[min(span, len(q) - 1)]
    b = q[0]
    d = b - a
    n = np.linalg.norm(d)
    return d / n if n > 0 else d


def strands(mask: np.ndarray, width: float, min_len: float | None = None):
    """Polylines of the rope: skeleton branches chained straight through crossings, spurs pruned.

    Returns a list of dicts {pts (N,2) x,y, closed: bool} and the crossing junctions {xy, pair: [(i_strand, k), ...]}.
    """
    sk = skeletonize(mask)
    nodes, branches = skeleton_graph(sk, merge_r=max(4.0, 0.9 * width))
    spur = 1.2 * width
    alive = [True] * len(branches)
    # Prune spurs: a short branch from a junction to an endpoint.
    for _ in range(3):
        for bi, b in enumerate(branches):
            if not alive[bi] or b.get("ring"):
                continue
            L = float(np.linalg.norm(np.diff(b["pts"], axis=0), axis=1).sum())
            ka = nodes[b["a"]]["kind"] if b["a"] >= 0 else "end"
            kb = nodes[b["b"]]["kind"] if b["b"] >= 0 else "end"
            if L < spur and (ka == "end") != (kb == "end"):
                alive[bi] = False
        for nd in nodes:
            nd["live"] = [bi for bi in nd["branches"] if alive[bi]]
    # Pair branches at each junction by direction.
    pairing = {}  # (bi, end) -> (bj, end)
    crossings = []
    span = int(max(4, 1.5 * width))
    for ni, nd in enumerate(nodes):
        if nd["kind"] != "junction":
            continue
        ends = []
        for bi in nd.get("live", nd["branches"]):
            b = branches[bi]
            for at_start in (True, False):
                if (b["a"] if at_start else b["b"]) == ni:
                    ends.append((bi, at_start, _dir_at(b["pts"], at_start, span)))
        if len(ends) < 2:
            continue
        used = set()
        cand = []
        for i in range(len(ends)):
            for j in range(i + 1, len(ends)):
                if ends[i][0] == ends[j][0] and ends[i][1] == ends[j][1]:
                    continue
                # Two ends continue each other when they point at each other: directions anti-parallel.
                cand.append((float(np.dot(ends[i][2], ends[j][2])), i, j))
        cand.sort()
        for c, i, j in cand:
            if i in used or j in used or c > -0.3:
                continue
            used |= {i, j}
            pairing[(ends[i][0], ends[i][1])] = (ends[j][0], ends[j][1])
            pairing[(ends[j][0], ends[j][1])] = (ends[i][0], ends[i][1])
        if len(ends) >= 4:
            crossings.append({"xy": nd["xy"], "node": ni})
    # Chain.
    done = [False] * len(branches)
    out = []
    for bi, b in enumerate(branches):
        if done[bi] or not alive[bi]:
            continue
        if b.get("ring"):
            done[bi] = True
            out.append({"pts": b["pts"], "closed": True})
            continue
        # Walk backwards to a chain start.
        cur, at_start = bi, True
        seen = {bi}
        while (cur, at_start) in pairing:
            nb, nend = pairing[(cur, at_start)]
            if nb in seen:
                break
            seen.add(nb)
            cur, at_start = nb, not nend
        # Now walk forward from (cur, entering at at_start's opposite).
        chain = []
        c, s = cur, at_start  # traverse c starting from its 'start' if s else from its end
        closed = False
        visited_local = set()
        while True:
            if c in visited_local:
                closed = True
                break
            visited_local.add(c)
            done[c] = True
            p = branches[c]["pts"] if s else branches[c]["pts"][::-1]
            chain.append(p)
            exit_key = (c, not s)
            if exit_key not in pairing:
                break
            nb, nend = pairing[exit_key]
            c, s = nb, nend
        pts = np.vstack(chain)
        if len(pts) >= 3:
            out.append({"pts": pts, "closed": closed})
    min_len = min_len if min_len is not None else 3.0 * width
    out = [o for o in out if float(np.linalg.norm(np.diff(o["pts"], axis=0), axis=1).sum()) >= min_len]
    return out, crossings, sk


def join_gaps(polys: list[dict], max_gap: float, max_turn_deg: float = 70.0) -> list[dict]:
    """Join open polylines end to end across gaps (hands) when the ends face each other."""
    polys = [dict(p) for p in polys]
    changed = True
    while changed:
        changed = False
        best = None
        opens = [i for i, p in enumerate(polys) if not p.get("closed")]
        for ii in range(len(opens)):
            for jj in range(ii + 1, len(opens)):
                i, j = opens[ii], opens[jj]
                for ei in (0, 1):
                    for ej in (0, 1):
                        pi, pj = polys[i]["pts"], polys[j]["pts"]
                        a = pi[0] if ei == 0 else pi[-1]
                        b = pj[0] if ej == 0 else pj[-1]
                        gap = float(np.linalg.norm(a - b))
                        if gap > max_gap:
                            continue
                        da = _dir_at(pi, ei == 0, 12)  # pointing out of polyline i at that end
                        db = _dir_at(pj, ej == 0, 12)
                        g = (b - a) / max(gap, 1e-6)
                        turn = math.degrees(math.acos(np.clip(np.dot(da, g), -1, 1))) + math.degrees(
                            math.acos(np.clip(np.dot(db, -g), -1, 1)))
                        if gap > 6 and turn > 2 * max_turn_deg:
                            continue
                        score = gap * (1.0 + turn / 90.0)
                        if best is None or score < best[0]:
                            best = (score, i, j, ei, ej)
        if best is not None:
            _, i, j, ei, ej = best
            pi, pj = polys[i]["pts"], polys[j]["pts"]
            a = pi[::-1] if ei == 0 else pi
            b = pj if ej == 0 else pj[::-1]
            gap_pts = np.linspace(a[-1], b[0], max(2, int(np.linalg.norm(b[0] - a[-1]) // 3) + 2))[1:-1]
            new = {"pts": np.vstack([a, gap_pts, b]), "closed": False,
                   "gaps": polys[i].get("gaps", []) + polys[j].get("gaps", []) + [[a[-1].tolist(), b[0].tolist()]]}
            polys = [p for k, p in enumerate(polys) if k not in (i, j)] + [new]
            changed = True
    return polys


def resample(pts: np.ndarray, step: float) -> np.ndarray:
    d = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    if d[-1] <= 0:
        return pts[:1]
    s = np.arange(0, d[-1] + 1e-9, step)
    return np.stack([np.interp(s, d, pts[:, 0]), np.interp(s, d, pts[:, 1])], 1)


def smooth(pts: np.ndarray, k: int = 5) -> np.ndarray:
    if len(pts) < 2 * k + 1:
        return pts
    ker = np.ones(k) / k
    x = np.convolve(np.pad(pts[:, 0], (k // 2, k // 2), mode="edge"), ker, mode="valid")
    y = np.convolve(np.pad(pts[:, 1], (k // 2, k // 2), mode="edge"), ker, mode="valid")
    return np.stack([x, y], 1)


def arc(pts: np.ndarray) -> np.ndarray:
    return np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])


# ----------------------------------------------------------------------------------------------- crossings


def self_crossings(pts: np.ndarray, min_sep: float) -> list[dict]:
    """Where a polyline crosses itself: segment intersections whose arc-length separation is > min_sep."""
    s = arc(pts)
    out = []
    n = len(pts)
    P0, P1 = pts[:-1], pts[1:]
    for i in range(n - 1):
        a0, a1 = P0[i], P1[i]
        da = a1 - a0
        # Vectorised against all later segments.
        j = np.arange(i + 2, n - 1)
        if len(j) == 0:
            continue
        b0, b1 = P0[j], P1[j]
        db = b1 - b0
        den = da[0] * db[:, 1] - da[1] * db[:, 0]
        ok = np.abs(den) > 1e-9
        w = b0 - a0
        ta = np.where(ok, (w[:, 0] * db[:, 1] - w[:, 1] * db[:, 0]) / np.where(ok, den, 1), -1)
        tb = np.where(ok, (w[:, 0] * da[1] - w[:, 1] * da[0]) / np.where(ok, den, 1), -1)
        hit = ok & (ta >= 0) & (ta <= 1) & (tb >= 0) & (tb <= 1)
        for jj in np.nonzero(hit)[0]:
            jj_ = int(j[jj])
            sa = s[i] + ta[jj] * (s[i + 1] - s[i])
            sb = s[jj_] + tb[jj] * (s[jj_ + 1] - s[jj_])
            if sb - sa < min_sep:
                continue
            xy = a0 + ta[jj] * da
            out.append({"xy": xy, "sa": float(sa), "sb": float(sb), "ia": i, "ib": jj_})
    # Merge duplicates (a crossing found on adjacent segments).
    merged = []
    for c in out:
        if any(np.linalg.norm(c["xy"] - m["xy"]) < 3 and abs(c["sa"] - m["sa"]) < 6 for m in merged):
            continue
        merged.append(c)
    return merged


def _line_fit(pts: np.ndarray, s: np.ndarray, s0: float, half: float):
    sel = np.abs(s - s0) < half
    p = pts[sel] if sel.sum() >= 3 else pts[max(0, int(np.searchsorted(s, s0)) - 3): int(np.searchsorted(s, s0)) + 3]
    c = p.mean(0)
    _, _, vt = np.linalg.svd(p - c)
    return c, vt[0]


def _samp(arr: np.ndarray, q) -> float:
    h, w = arr.shape
    if 0 <= q[0] < w - 1 and 0 <= q[1] < h - 1:
        return float(cv2.getRectSubPix(arr, (1, 1), (float(q[0]), float(q[1])))[0, 0])
    return float("nan")


def over_under(gray: np.ndarray, rope: np.ndarray, pts: np.ndarray, c: dict, width: float) -> dict:
    """Which strand is on top at a crossing: the under strand's axis is cut by the over strand's two dark edges.

    The crossing's centre is refined to the intersection of straight-line fits to each strand (+-1.5 w of arc).
    Along each strand's axis, from -1.5 w to +1.5 w, the grey level (Gaussian-smoothed with sigma w/6 to wash out
    the braid's pale flecks, averaged over +-w/4 across the axis) is compared with its value at the centre:
    ``dip`` = centre - darkest in 0.25 w < |t| < 0.85 w. The over strand's pixels are continuous through the
    crossing (no dip); the under strand passes beneath the over strand's two boundaries (two dark seams, a dip).
    On the six crossings checked by eye (22.7, 24.4, 25.0 s; 32.0 s x 3) this got all six, by 2-27 grey levels.
    """
    s = arc(pts)
    ca, da = _line_fit(pts, s, c["sa"], 1.5 * width)
    cb, db = _line_fit(pts, s, c["sb"], 1.5 * width)
    try:
        tt = np.linalg.solve(np.array([da, -db]).T, cb - ca)
        x = ca + tt[0] * da
        if np.linalg.norm(x - np.asarray(c["xy"])) > width:
            x = np.asarray(c["xy"], float)
    except np.linalg.LinAlgError:
        x = np.asarray(c["xy"], float)
    g = cv2.GaussianBlur(gray, (0, 0), max(0.6, width / 6.0))

    def dip(t_axis):
        n = np.array([-t_axis[1], t_axis[0]])
        ts = np.linspace(-1.5 * width, 1.5 * width, 61)
        v = []
        for tt_ in ts:
            vals = [_samp(g, x + tt_ * t_axis + bb * n) for bb in np.linspace(-0.25 * width, 0.25 * width, 5)]
            vals = [u for u in vals if np.isfinite(u)]
            v.append(np.mean(vals) if vals else np.nan)
        v = np.asarray(v)
        ref = np.nanmedian(v[np.abs(ts) < 0.25 * width])
        sel = (np.abs(ts) > 0.25 * width) & (np.abs(ts) < 0.85 * width)
        return float(ref - np.nanmin(v[sel]))

    pa, pb = dip(da), dip(db)
    over = "a" if pb > pa else "b"
    conf = abs(pb - pa) / (abs(pa) + abs(pb) + 1.0)
    return {"over": over, "confidence": round(float(conf), 3), "dip_a": round(pa, 1), "dip_b": round(pb, 1),
            "centre": [round(float(x[0]), 1), round(float(x[1]), 1)],
            "angle_deg": round(math.degrees(math.acos(min(1.0, abs(float(np.dot(da, db)))))), 1)}


# ----------------------------------------------------------------------------------------------- hands


def _wrists(sk: np.ndarray, min_run: int = 25, gap: int = 90) -> list[np.ndarray]:
    """Where arms enter the frame: runs of skin along the bottom, left and right borders (a 3-px band)."""
    h, w = sk.shape
    # Walk the border clockwise from the top-left: left side downwards, bottom left-to-right, right side upwards.
    path = [(y, 1) for y in range(h)] + [(h - 2, x) for x in range(w)] + [(y, w - 2) for y in range(h - 1, -1, -1)]
    on = np.array([sk[max(0, y - 1): y + 2, max(0, x - 1): x + 2].any() for y, x in path])
    runs, cur, last_on = [], [], -10**9
    for i, v in enumerate(on):
        if v:
            if cur and i - last_on > gap:
                runs.append(cur)
                cur = []
            cur.append(i)
            last_on = i
    if cur:
        runs.append(cur)
    out = []
    for r in runs:
        if len(r) >= min_run:
            ys = np.array([path[i][0] for i in r], float)
            xs = np.array([path[i][1] for i in r], float)
            out.append(np.array([xs.mean(), ys.mean()]))
    return out


def hands(im: np.ndarray, t: float, rope: np.ndarray, width: float) -> list[dict]:
    """The hands: skin blobs split per arm (watershed from where each arm enters the frame), left/right by entry.

    The person sits at the bottom of the frame; the left arm enters on the left (left border or bottom-left), the
    right arm on the right. ``fingertip``: the hand's pixel farthest from its wrist. ``rope_contacts``: rope pixels
    the hand's boundary touches (dilated by w/2), clustered, farthest from the wrist first.
    """
    from skimage.segmentation import watershed

    sk = skin_mask(im, t, min_area=int(max(2500, 25 * width * width)))
    h, w = sk.shape
    if not sk.any():
        return []
    wr = _wrists(sk)
    lab_cc, ncc = ndimage.label(sk)
    if len(wr) >= 2:
        # Keep the two wrists with the most skin behind them, left one first.
        markers = np.zeros(sk.shape, np.int32)
        for k, p in enumerate(wr, 1):
            cv2.circle(markers, (int(p[0]), int(p[1])), 6, k, -1)
        markers[~sk] = 0
        dist = ndimage.distance_transform_edt(sk)
        lab = watershed(-dist, markers, mask=sk)
        ids = list(range(1, len(wr) + 1))
        # A hand cut off from its forearm (a shadow, a fast move) has no marker: join it to the nearest arm.
        for i in range(1, ncc + 1):
            comp = lab_cc == i
            if (lab[comp] > 0).any() or comp.sum() < 1500:
                continue
            best, bd = 0, float("inf")
            for k in ids:
                other = lab == k
                if not other.any():
                    continue
                d = ndimage.distance_transform_edt(~other)[comp].min()
                if d < bd:
                    best, bd = k, d
            if best and bd < 120:
                lab[comp] = best
    else:
        lab, ids = lab_cc, list(range(1, ncc + 1))
        wr = []
    blobs = []
    for k in ids:
        blob = lab == k
        n = int(blob.sum())
        if n < 1500:
            continue
        ys, xs = np.nonzero(blob)
        if wr and k - 1 < len(wr):
            wrist = wr[k - 1]
        else:
            border = (ys >= h - 3) | (xs <= 2) | (xs >= w - 3)
            if border.any():
                wrist = np.array([xs[border].mean(), ys[border].mean()])
            else:
                i = int(np.argmax(ys))
                wrist = np.array([xs[i], ys[i]], float)
        blobs.append((n, blob, xs, ys, wrist))
    blobs.sort(key=lambda b: -b[0])
    blobs = blobs[:2]
    out = []
    for n, blob, xs, ys, wrist in blobs:
        ring = ndimage.binary_dilation(blob, iterations=int(max(2, width / 2))) & ~blob
        touch = ring & rope
        tips = []
        if touch.any():
            tl, nt = ndimage.label(ndimage.binary_dilation(touch, iterations=2))
            for j in range(1, nt + 1):
                ty, tx = np.nonzero((tl == j) & touch)
                if len(tx) < 3:
                    continue
                p = np.array([tx.mean(), ty.mean()])
                tips.append({"xy": p.round(1).tolist(), "n": int(len(tx)), "reach": float(np.linalg.norm(p - wrist))})
            tips.sort(key=lambda d: -d["reach"])
        d2 = (xs - wrist[0]) ** 2 + (ys - wrist[1]) ** 2
        far = d2 >= np.percentile(d2, 97)
        out.append({"centroid": [float(xs.mean()), float(ys.mean())], "area": n,
                    "bbox": [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
                    "wrist": np.asarray(wrist).round(1).tolist(),
                    "fingertip": [float(xs[far].mean()), float(ys[far].mean())], "rope_contacts": tips, "_mask": blob})
    # Left/right by entry point along the border: left border / bottom-left first.
    def entry_key(d):
        x, y = d["wrist"]
        return x - (w if x < 3 else 0) + (w if x > w - 4 else 0)
    out.sort(key=entry_key)
    if len(out) == 2:
        out[0]["hand"], out[1]["hand"] = "left", "right"
    elif len(out) == 1:
        out[0]["hand"] = "left" if out[0]["wrist"][0] < w / 2 else "right"
    return out


# ----------------------------------------------------------------------------------------------- per frame


def free_ends(polys: list[dict], hand_list: list[dict], shape, width: float) -> list[dict]:
    """Polyline ends that are real rope ends: not at the image border, not under or next to a hand."""
    h, w = shape[:2]
    hm = np.zeros((h, w), bool)
    for hd in hand_list:
        hm |= hd["_mask"]
    near = ndimage.binary_dilation(hm, iterations=int(max(3, 1.2 * width)))
    out = []
    for i, p in enumerate(polys):
        if p.get("closed"):
            continue
        for e in (0, -1):
            x, y = p["pts"][e]
            if x < 6 or y < 6 or x > w - 7 or y > h - 7:
                continue
            if near[int(round(y)), int(round(x))]:
                continue
            out.append({"poly": i, "end": e, "xy": [float(x), float(y)]})
    return out


PRIOR_WIDTH = {"straight_closeup": 75.0, "wide_step1": 11.0, "loop_closeup": 15.0, "step2_closeup": 29.0,
               "loose_closeup": 30.0, "wide_step3": 11.5, "oblique_tight": 12.0, "intro_knot": 30.0}


def process(im: np.ndarray, t: float) -> dict:
    m0 = rope_mask(im, t, PRIOR_WIDTH.get(shot_at(t)["name"]))
    w = rope_width_px(m0)
    if not np.isfinite(w):
        w = PRIOR_WIDTH.get(shot_at(t)["name"], 12.0)
    m = rope_mask(im, t, w)
    w = rope_width_px(m)
    sts, juncs, sk = strands(m, w)
    hl = hands(im, t, m, w)
    polys = join_gaps(sts, max_gap=6.0 * w)
    ends = free_ends(polys, hl, im.shape, w)
    return {"t": t, "width_px": w, "mask": m, "strands": sts, "polys": polys, "junctions": juncs, "hands": hl,
            "ends": ends}


# ----------------------------------------------------------------------------------------------- guided routes


def trace_route(mask: np.ndarray, width: float, waypoints, hidden_cost: float = 5.0) -> dict:
    """The rope's centreline through ``waypoints`` (pixel x, y, in rope order), following the mask between them.

    Between consecutive waypoints the minimum-cost path: on the rope the cost is lowest along its axis (distance
    transform), off it (under a hand, out of the mask) it is ``hidden_cost`` per pixel, so a path bridges an
    occlusion straight. Returns {pts (N,2) resampled every 2 px, visible (N,) bool}.
    """
    from skimage.graph import route_through_array

    dt = ndimage.distance_transform_edt(mask)
    r = max(1.0, width / 2.0)
    cost = np.where(mask, 1.0 + 3.0 * (1.0 - np.clip(dt / r, 0, 1)) ** 2, hidden_cost).astype(np.float64)
    h, w = mask.shape
    path = []
    wp = [(float(np.clip(x, 0, w - 1)), float(np.clip(y, 0, h - 1))) for x, y in waypoints]
    for (x0, y0), (x1, y1) in zip(wp[:-1], wp[1:]):
        idx, _ = route_through_array(cost, (int(round(y0)), int(round(x0))), (int(round(y1)), int(round(x1))),
                                     fully_connected=True, geometric=True)
        seg = np.asarray([(c, r_) for r_, c in idx], float)
        path.append(seg if not path else seg[1:])
    pts = np.vstack(path)
    pts = smooth(pts, k=max(3, int(width) | 1))
    pts = resample(pts, 2.0)
    vis = np.array([mask[int(round(y)), int(round(x))] if 0 <= int(round(y)) < h and 0 <= int(round(x)) < w else False
                    for x, y in pts])
    return {"pts": pts, "visible": vis}


# ----------------------------------------------------------------------------------------------- pixels to table


def up_right(shot: dict) -> tuple[np.ndarray, np.ndarray]:
    cam = CAMERAS.get(shot.get("camera") or "", None)
    up = np.asarray(cam["up"] if cam else (1.0, 0.0), float)
    up /= np.linalg.norm(up)
    right = np.array([up[1], -up[0]])  # image right: up turned -90 deg about table z (seen from above)
    return up, right


# The two wide shots (19.3-23.8 s and 33.5-36.4 s) are ONE framing: SIFT on the bare wood matched them at scale
# 1.000, rotation 0.0 deg, with step 1's image 30.5 px to the right of step 3's. Step-1 pixels are shifted by this
# so both wide shots share step 3's table frame (REAL2SIM's 'wide' camera). Within each shot SIFT found no zoom
# (scale 1.000 at 19.4 -> 23.5 s and 33.6 -> 36.2 s). The close-up 25.9-33.5 s is one continuous shot that zooms in
# (x1.50 from 26.0 to 31.0 s, x1.07 from 32.0 to 33.4 s; the knot moved at 31.4-31.6 s, the camera did not).
WIDE_STEP1_DU = -30.5


def to_table(px, shot: dict, m_per_px: float) -> np.ndarray:
    """Image pixels -> table metres in the shot's own frame: origin at the image centre, x away from the person,
    y to the person's left (REAL2SIM's table axes, the camera's look-at point moved to the origin). The step-1 wide
    shot is expressed in the step-3 wide shot's frame (same camera, shifted 30.5 px)."""
    p = np.atleast_2d(np.asarray(px, float))
    up, right = up_right(shot)
    du, dv = p[:, 0] - 320.0, 240.0 - p[:, 1]
    if shot["name"] == "wide_step1":
        du = du + WIDE_STEP1_DU
    return m_per_px * (du[:, None] * right[None, :] + dv[:, None] * up[None, :])


# ----------------------------------------------------------------------------------------------- key frames
#
# MANUAL route waypoints, pixel (x, y), in rope order starting from the working-end side ("tip" when the tip is in
# view). They were read off gridded stills by eye; between them the centreline is traced on the rope mask
# (trace_route), and where the rope is under a hand the trace bridges straight (those samples are "hidden").
# Everything measured from them (arc lengths, crossings, over/under, sizes) is computed, not typed in.

KEYS = [
    {"t": 19.4, "label": "straight", "start": "right edge", "note": "both ends off-frame; the left hand pinches the rope at the left",
     "wps": [[639, 252], [560, 254], [480, 255], [400, 256], [300, 250], [200, 243], [140, 238], [100, 228], [60, 212],
             [30, 200], [2, 193]]},
    {"t": 20.0, "label": "fold", "start": "tip", "note": "the right hand has taken the rope near its right end and folded the end back leftward, on the far side (+x) of the standing part",
     "wps": [[382, 230], [420, 240], [460, 240], [520, 232], [555, 230], [590, 240], [605, 270], [585, 305], [560, 312],
             [500, 321], [420, 320], [360, 305], [280, 270], [200, 240], [160, 222], [120, 195], [70, 178], [30, 180], [2, 187]]},
    {"t": 20.4, "label": "take_end", "start": "tip", "note": "the left hand has let the standing part go and pinches the end's run in the middle; the right hand still holds the fold",
     "wps": [[232, 210], [280, 234], [340, 246], [400, 242], [450, 228], [495, 228], [520, 250], [515, 300], [495, 335],
             [420, 346], [360, 332], [300, 302], [240, 270], [150, 216], [80, 186], [20, 180], [1, 190]]},
    {"t": 20.9, "label": "loop_oval", "start": "tip (under the left hand)", "note": "the oval between the hands; X1 and the tip are under the left hand, the fold under the right",
     "wps": [[222, 240], [236, 200], [260, 184], [300, 178], [400, 182], [455, 190], [485, 205], [495, 232], [475, 260],
             [440, 280], [360, 292], [300, 286], [245, 268], [205, 245], [160, 215], [110, 198], [60, 189], [25, 193],
             [6, 215], [2, 260], [2, 330], [2, 400]]},
    {"t": 21.3, "label": "turn_loop", "start": "tip", "note": "X1 made (under the left hand), the end hangs toward the person; the right hand lifts the fold away (+x) to turn the loop",
     "wps": [[400, 366], [340, 340], [290, 310], [262, 280], [250, 250], [262, 215], [300, 160], [360, 120], [420, 100],
             [470, 105], [495, 130], [490, 180], [470, 220], [430, 245], [360, 262], [300, 262], [255, 258], [230, 245],
             [200, 225], [140, 215], [90, 230], [50, 270], [20, 330], [4, 400]]},
    {"t": 22.7, "label": "loop", "start": "tip", "note": "teardrop loop pointing away (+x); X1 at its bottom by the left hand; the right hand holds the end",
     "wps": [[598, 318], [520, 313], [470, 313], [420, 318], [380, 316], [350, 302], [328, 262], [322, 180], [340, 100],
             [400, 75], [450, 100], [460, 160], [445, 210], [400, 245], [355, 275], [300, 306], [230, 312], [160, 316],
             [120, 330], [80, 352], [40, 392], [8, 445]]},
    {"t": 25.0, "label": "loop_closeup", "start": "right hand (the end; tip off-frame)", "note": "rotated close-up of the same loop: the crossing at 15.6 px per rope width",
     "wps": [[400, 240], [360, 236], [300, 224], [250, 196], [212, 162], [196, 110], [210, 70], [260, 45], [320, 40],
             [370, 60], [398, 100], [385, 145], [355, 185], [330, 210], [302, 240], [282, 270], [268, 300], [258, 330]]},
    {"t": 29.3, "label": "tuck_over", "start": "tip", "note": "step 2: the right hand has brought the end up from the lower right and across the loop's right side into the eye (over it); only the run from the tip to the right hand is traced",
     "wps": [[290, 166], [350, 190], [410, 215], [480, 245], [560, 275], [636, 298]]},
    {"t": 32.0, "label": "loose", "start": "right edge (the working end; tip off-frame right)", "note": "the loose knot, no hands; motion-blurred zoom",
     "wps": [[639, 286], [610, 293], [560, 310], [500, 330], [455, 346], [420, 368], [390, 385], [340, 377], [300, 352],
             [275, 322], [265, 300], [262, 250], [270, 180], [300, 120], [360, 90], [440, 85], [520, 100], [580, 150],
             [610, 220], [606, 280], [590, 330], [540, 366], [480, 375], [440, 370], [400, 357], [340, 336], [290, 318],
             [262, 308], [200, 300], [100, 292], [2, 285]]},
    {"t": 33.6, "label": "pull_start", "start": "tip", "note": "step 3 begins: the right hand holds the end right of the knot, the left the standing part left of it; the knot body's order is read from 32.0 (too small here to trace)",
     "wps": [[505, 238], [480, 250], [455, 262], [425, 272], [402, 280]], "tail_only": True},
    {"t": 34.3, "label": "pulling", "start": "tip", "note": "both hands drawing the tails apart; tail traced from the tip to the knot",
     "wps": [[575, 212], [540, 228], [500, 248], [460, 262], [430, 272]], "tail_only": True},
    {"t": 35.8, "label": "tight", "start": "right hand (tip at the right edge)", "note": "the knot ~4 cm across; the left hand has left the frame",
     "wps": [[636, 224], [600, 238], [560, 246], [500, 248], [420, 246], [362, 246]], "tail_only": True},
]


SCALE: dict = {}
"""Set by main(): per shot, the metres per pixel used ({name: m_per_px}), and per frame for the zooming close-up."""


def m_per_px_at(t: float, width_px: float | None = None) -> float:
    name = shot_at(t)["name"]
    if name in ("step2_closeup", "loose_closeup") and "zoom" in SCALE:
        ts, ws = SCALE["zoom"]
        return ROPE_D / float(np.interp(t, ts, ws))
    if name in SCALE:
        return SCALE[name]
    return ROPE_D / (width_px or PRIOR_WIDTH.get(name, 12.0))


def analyse_key(cap, key: dict) -> dict:
    t = key["t"]
    shot = shot_at(t)
    im = read_frame(cap, t)
    fr = process(im, t)
    w = fr["width_px"]
    mpp = m_per_px_at(t, w)
    route = trace_route(fr["mask"], w, key["wps"])
    pts = route["pts"]
    s_px = arc(pts)
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).astype(np.float32)
    crossings = []
    if not key.get("tail_only"):
        hand_px = np.zeros(fr["mask"].shape, bool)
        for hd in fr["hands"]:
            hand_px |= hd["_mask"]
        hand_px = ndimage.binary_dilation(hand_px, iterations=int(max(2, 0.5 * w)))
        for c in sorted(self_crossings(pts, min_sep=3.0 * w), key=lambda c: c["sa"]):
            ou = over_under(gray, fr["mask"], pts, c, w)
            cx, cy = int(round(ou["centre"][0])), int(round(ou["centre"][1]))
            occluded = bool(hand_px[min(max(cy, 0), hand_px.shape[0] - 1), min(max(cx, 0), hand_px.shape[1] - 1)])
            crossings.append({"xy_px": [round(float(v), 1) for v in ou["centre"]],
                              "xy": to_table(ou["centre"], shot, mpp)[0].round(4).tolist(),
                              "s_start_m": [round(c["sa"] * mpp, 4), round(c["sb"] * mpp, 4)],
                              "over": ou["over"] if not occluded else key.get("occluded_over"),
                              "measured": not occluded,
                              "evidence": ({k: ou[k] for k in ("dip_a", "dip_b", "confidence", "angle_deg")} if not occluded
                                           else "under a hand: not measured here")})
    # Label crossings in the order met from the start, and the Gauss code.
    events = sorted([(c["s_start_m"][0], i, {"a": "O", "b": "U"}.get(c["over"], "?")) for i, c in enumerate(crossings)]
                    + [(c["s_start_m"][1], i, {"a": "U", "b": "O"}.get(c["over"], "?")) for i, c in enumerate(crossings)])
    label_of, code = {}, []
    for s_, i, ou in events:
        if i not in label_of:
            label_of[i] = len(label_of) + 1
        code.append(f"{label_of[i]}{ou}")
    for i, c in enumerate(crossings):
        c["label"] = label_of.get(i)
    # Hands at this frame and the rope under each (hidden route samples inside the dilated hand mask).
    hl = []
    for hd in fr["hands"]:
        near = ndimage.binary_dilation(hd["_mask"], iterations=int(max(2, 0.6 * w)))
        inside = np.array([near[int(round(y)), int(round(x))] for x, y in pts])
        runs = []
        k = 0
        while k < len(pts):
            if inside[k]:
                j = k
                while j + 1 < len(pts) and inside[j + 1]:
                    j += 1
                if s_px[j] - s_px[k] > 0.5 * w:
                    runs.append((k, j))
                k = j + 1
            else:
                k += 1
        hl.append({"hand": hd["hand"], "centroid": to_table(hd["centroid"], shot, mpp)[0].round(4).tolist(),
                   "fingertip": to_table(hd["fingertip"], shot, mpp)[0].round(4).tolist(),
                   "fingertip_px": [round(v, 1) for v in hd["fingertip"]],
                   "rope_under": [{"s_start_m": [round(s_px[a] * mpp, 4), round(s_px[b] * mpp, 4)],
                                   "mid_px": pts[(a + b) // 2].round(1).tolist()} for a, b in runs]})
    tab = to_table(pts, shot, mpp)
    x1c = next((c for c in crossings if c.get("label") == len(crossings)), None) if crossings else None
    origin = np.asarray(x1c["xy"]) if x1c else tab.mean(axis=0)
    out = {"t": t, "label": key["label"], "shot": shot["name"], "start": key["start"], "note": key["note"],
           "width_px": round(w, 2), "m_per_px": round(mpp, 6),
           "arc_visible_m": round(float((np.diff(s_px)[route["visible"][1:]]).sum() * mpp), 4),
           "arc_traced_m": round(float(s_px[-1] * mpp), 4),
           "centreline": resample(tab, 0.01).round(4).tolist(),
           "centreline_centred": (resample(tab, 0.01) - origin).round(4).tolist(),
           "centred_on": "X1 (the end-over-standing crossing; for the loose knot the last one met from the end)" if x1c else "the traced centreline's centroid",
           "crossings": crossings, "gauss_from_start": " ".join(code), "hands": hl,
           "_pts": pts, "_visible": route["visible"], "_im": im, "_frame": fr}
    return out


# ----------------------------------------------------------------------------------------------- tracks


def track(cap, t0: float, t1: float, fps: float = FPS_SAMPLE) -> list[dict]:
    rows = []
    for t in np.arange(t0, t1 + 1e-6, 1.0 / fps):
        t = round(float(t), 2)
        shot = shot_at(t)
        im = read_frame(cap, t)
        fr = process(im, t)
        w = fr["width_px"]
        row = {"t": t, "shot": shot["name"], "width_px": round(float(w), 2) if np.isfinite(w) else None,
               "rope_px": int(fr["mask"].sum()), "free_ends_px": [e["xy"] for e in fr["ends"]], "hands": {}}
        for hd in fr["hands"]:
            row["hands"][hd["hand"]] = {"centroid_px": [round(v, 1) for v in hd["centroid"]],
                                        "fingertip_px": [round(v, 1) for v in hd["fingertip"]],
                                        "wrist_px": hd["wrist"], "area": hd["area"],
                                        "on_rope": len(hd["rope_contacts"]) > 0}
        rows.append(row)
    return rows


def shot_scale(rows: list[dict]) -> dict:
    """Median metres per pixel of each shot, from the rope's width (frames with enough rope in view)."""
    out = {}
    for s in SHOTS:
        ws = [r["width_px"] for r in rows if r["shot"] == s["name"] and r["width_px"] and r["rope_px"] > 2000]
        if ws:
            out[s["name"]] = {"width_px_median": round(float(np.median(ws)), 2), "m_per_px": round(ROPE_D / float(np.median(ws)), 6),
                              "n": len(ws)}
        cam = CAMERAS.get(s["camera"] or "")
        if cam and s["name"] in out:
            out[s["name"]]["camera_m_per_px"] = round(cam["height"] / F_PX, 6)
    return out


# ----------------------------------------------------------------------------------------------- the plan


def _mid(run):
    return round(0.5 * (run[0] + run[1]), 4)


def _under(key_out: dict, hand: str, pick: str = "first") -> list | None:
    """The [s0, s1] of rope (from the key's start) under ``hand`` at that key frame."""
    for h in key_out["hands"]:
        if h["hand"] == hand and h["rope_under"]:
            runs = [r["s_start_m"] for r in h["rope_under"]]
            return runs[0] if pick == "first" else (runs[-1] if pick == "last" else min(runs, key=lambda r: r[0]))
    return None


def _path(rows, hand, t0, t1, every=2):
    p = hand_path(rows, hand, t0, t1, {}, key="fingertip_px")
    p = p[::every] + ([p[-1]] if p and (len(p) - 1) % every else [])
    return [[q[1], q[2]] for q in p], [q[0] for q in p]


def hand_path(rows, hand: str, t0: float, t1: float, scale: dict, key: str = "fingertip_px") -> list:
    pts = []
    for r in rows:
        if t0 - 1e-6 <= r["t"] <= t1 + 1e-6 and hand in r["hands"]:
            shot = shot_at(r["t"])
            p = to_table(r["hands"][hand][key], shot, m_per_px_at(r["t"], r["width_px"]))[0]
            pts.append([r["t"], round(float(p[0]), 4), round(float(p[1]), 4)])
    return pts


def _tip_path(rows, t0, t1, seed_px, max_jump=60.0):
    """The working-end tip followed through the free rope ends of each frame (nearest to the last one)."""
    out, cur = [], np.asarray(seed_px, float)
    for r in rows:
        if not (t0 - 1e-6 <= r["t"] <= t1 + 1e-6) or not r["free_ends_px"]:
            continue
        e = np.asarray(r["free_ends_px"], float)
        d = np.linalg.norm(e - cur, axis=1)
        k = int(np.argmin(d))
        if d[k] <= max_jump:
            cur = e[k]
            p = to_table(cur, shot_at(r["t"]), m_per_px_at(r["t"], r["width_px"]))[0]
            out.append([r["t"], round(float(p[0]), 4), round(float(p[1]), 4), cur.round(1).tolist()])
    return out


def build_plan(cap, rows) -> dict:
    K = {k["label"]: analyse_key(cap, k) for k in KEYS}
    f = lambda s: None if s is None else round(s / ROPE_LEN_ASSUMED, 3)  # noqa: E731

    # ---- numbers out of the key frames (arc from the working-end tip unless the key says otherwise)
    fold = _under(K["fold"], "right")  # the right hand's rope at 20.0: [first contact, where the standing part leaves]
    left_s1 = _under(K["fold"], "left", "last")  # the left hand on the standing part at 20.0
    take = _under(K["take_end"], "left", "first")  # the left hand on the end's run at 20.4
    oval = K["loop_oval"]
    oval_fold = _under(oval, "right")
    oval_left = [r["s_start_m"] for h in oval["hands"] if h["hand"] == "left" for r in h["rope_under"]]
    tl = K["turn_loop"]
    loop = K["loop"]
    x1 = loop["crossings"][0] if loop["crossings"] else None
    lc = K["loop_closeup"]
    loose = K["loose"]
    lx = {c["label"]: c for c in loose["crossings"]}  # labels in order met from the working end: 1 = X3, 2 = X2, 3 = X1
    right_end = _under(loop, "right")
    tuck = K["tuck_over"]
    ps = K["pull_start"]
    pulling = K["pulling"]
    tight = K["tight"]
    r_pull = _under(ps, "right")
    # The loose knot's tip is off-frame: its tail (tip -> where the end enters the knot, X3) is read at 33.6 s.
    tail = ps["arc_traced_m"]
    off = tail - (lx[1]["s_start_m"][0] if 1 in lx else 0.0)  # tip-to-right-edge offset for the 32.0 s arcs

    def loose_s(label, which):
        return round(off + lx[label]["s_start_m"][which], 4) if label in lx else None

    def bbox(c):
        a = np.asarray(c)
        return [round(float(np.ptp(a[:, 0])), 3), round(float(np.ptp(a[:, 1])), 3)]

    # Loop extent from the traced loop part only (between the crossing's two passes).
    def loop_box(key, x):
        if not x:
            return None
        pts = np.asarray(key["centreline"])
        i0, i1 = int(x["s_start_m"][0] / 0.01), int(x["s_start_m"][1] / 0.01)
        seg = pts[i0: i1 + 1]
        return bbox(seg) if len(seg) > 3 else None

    oval_loop = None
    if oval_fold and oval_left:
        # The oval's loop runs from where the end leaves the left hand (first run's end) to where the standing part
        # enters it (second run's start); the 6-8 cm under the left hand itself are not counted.
        oval_loop = round(oval_left[-1][0] - oval_left[0][1], 4)
    knot_len = round(lx[3]["s_start_m"][1] - lx[1]["s_start_m"][0], 4) if 1 in lx and 3 in lx else None
    loose_ring = None
    if 1 in lx and 3 in lx:  # the ring of the loose knot: from X1's end-side pass round to X3's loop-side pass
        cl = np.asarray(loose["centreline"])
        seg = cl[int(lx[3]["s_start_m"][0] / 0.01): int(lx[1]["s_start_m"][1] / 0.01) + 1]
        loose_ring = bbox(seg) if len(seg) > 3 else None

    # Knot size in the step-3 wide shots: the ring's span, from the rope mask between the hands (rows above the line).
    def ring_size(key, x_range):
        """The ring's [width, height] in a wide shot: rope-mask pixels in a column window that stand above the
        straight line the tails make (the line's own row found at the window's two edges)."""
        m = key["_frame"]["mask"].copy()
        for hd in key["_frame"]["hands"]:
            m &= ~ndimage.binary_dilation(hd["_mask"], iterations=4)
        w = key["width_px"]
        x0, x1 = x_range
        edge_rows = []
        for xx in (x0, x1):
            ys = np.nonzero(m[:, max(0, xx - 3): xx + 3].any(axis=1))[0]
            ys = ys[(ys > 150) & (ys < 360)]
            if len(ys):
                edge_rows.append(float(np.median(ys)))
        if not edge_rows:
            return None
        y_line = float(np.mean(edge_rows))
        sub = m[:, x0: x1]
        ys, xs = np.nonzero(sub)
        above = ys < y_line - 1.2 * w
        if not above.any():
            return None
        mpp = key["m_per_px"]
        # [extent along table x (image vertical), extent along table y (image horizontal)], as bbox() gives
        return [round(float((y_line - ys[above].min()) * mpp), 3), round(float((xs[above].max() - xs[above].min() + 1) * mpp), 3)]

    pull_l, tl_ = _path(rows, "left", 33.6, 35.3)  # the left hand leaves the frame after 35.3 s
    pull_r, tr_ = _path(rows, "right", 33.6, 35.7)  # at 35.8 s the right hand's far point jumps to its wrist edge
    disp = lambda p: round(float(np.linalg.norm(np.subtract(p[-1], p[0]))), 3) if len(p) > 1 else None  # noqa: E731

    tip_tuck = _tip_path(rows, 29.0, 29.5, (436, 219))
    s_lefthold3 = None
    if 3 in lx:
        # Standing side: X1's standing pass, plus the standing part from the knot to the left hand at 33.6 s (the hand
        # at ~(232, 268) px, the knot's X1 at ~(245, 300) px: ~0.03 m along the rope; estimated).
        s_lefthold3 = round(loose_s(3, 1) + 0.03, 4)

    events = []

    def ev(**kw):
        kw["i"] = len(events) + 1
        kw.setdefault("path", [])
        if "s_from_tip_m" in kw:
            kw["rope_fraction"] = f(kw["s_from_tip_m"])
        events.append(kw)

    p, _ = _path(rows, "left", 19.3, 20.1)
    ev(step=1, hand="left", action="hold", s_from_tip_m=_mid(left_s1) if left_s1 else None, path=p, t0=19.3, t1=20.1,
       shot="wide_step1", reliability="estimated",
       note="pinches the standing part well left of where the loop will be (its arc from the tip read at 20.0 s, when the tip is in view)")
    ev(step=1, hand="right", action="pinch", s_from_tip_m=round(fold[0], 4) if fold else None, s_range_m=fold, path=_path(rows, "right", 19.9, 20.0)[0],
       t0=19.8, t1=19.9, shot="wide_step1", reliability="measured",
       note="takes the rope near its right end (the end is off-frame right until 19.9 s); at 20.0 s the hand holds the fold: the end's run enters it at the first value, the standing part leaves it at the second")
    p, _ = _path(rows, "right", 19.9, 20.3)
    ev(step=1, hand="right", action="carry", s_from_tip_m=round(fold[0], 4) if fold else None, path=p, t0=19.9, t1=20.3,
       shot="wide_step1", reliability="measured",
       note="folds the end back to the left in the air and lays it on the FAR side (+x) of the standing part, pointing left (20.0 s); no crossing yet")
    ev(step=1, hand="left", action="release", t0=20.1, t1=20.2, shot="wide_step1", reliability="measured",
       note="lets the standing part go and moves to the middle")
    ev(step=1, hand="left", action="pinch", s_from_tip_m=_mid(take) if take else None, s_range_m=take, path=_path(rows, "left", 20.4, 20.4)[0],
       t0=20.3, t1=20.4, shot="wide_step1", reliability="measured", note="takes the end's run ~0.15-0.18 m from the tip, between the tip and the right hand's fold")
    p, _ = _path(rows, "left", 20.4, 21.0)
    ev(step=1, hand="left", action="carry", s_from_tip_m=_mid(take) if take else None, path=p, t0=20.4, t1=21.0,
       shot="wide_step1", reliability="measured",
       note="draws the end left and lays it ACROSS the standing part from the far side to the near side: crossing X1, the end OVER the standing part, made under the left hand's fingers (seen at 21.0-21.3 s: the end hangs toward the person from X1)")
    ev(step=1, hand="left", action="hold", s_from_tip_m=round(x1["s_start_m"][0], 4) if x1 else None, path=_path(rows, "left", 21.0, 21.0)[0],
       t0=21.0, t1=31.4, shot="wide_step1 -> loop_closeup -> step2_closeup", reliability="measured",
       note="pinches X1 (thumb on top) from here to the end of step 2; it slides the rope through X1 while the loop is sized (21.3-25.0 s)")
    p, _ = _path(rows, "right", 20.9, 21.4)
    ev(step=1, hand="right", action="carry", s_from_tip_m=_mid(oval_fold) if oval_fold else None, path=p, t0=20.9, t1=21.4,
       shot="wide_step1", reliability="measured",
       note="still holding the fold (the loop's far end), swings it away from the person (+x): the oval loop turns into a loop pointing away")
    ev(step=1, hand="right", action="release", t0=21.4, t1=21.6, shot="wide_step1", reliability="measured",
       note="lets the loop go; it lies on the table, pointing away from the person")
    p, _ = _path(rows, "right", 21.8, 23.7)
    ev(step=1, hand="right", action="pinch", s_from_tip_m=_mid(right_end) if right_end else None, s_range_m=right_end, path=p,
       t0=21.8, t1=25.8, shot="wide_step1 -> loop_closeup", reliability="measured",
       note="takes the end close to its tip and draws it out while the left hand holds X1: the loop shrinks (0.66 m of rope at 21.3 s -> 0.48 m at 22.7 s -> 0.39 m at 25.0 s)")
    ev(step=2, hand="left", action="lift", s_from_tip_m=loose_s(2, 1), path=_path(rows, "left", 26.0, 26.0)[0],
       t0=25.9, t1=31.4, shot="step2_closeup", reliability="estimated",
       note="the left hand keeps X1 pinched between thumb and fingers; the strands at X1, and the loop strand just past it on the standing-part side (where the end will pass under), are held off the table by the fingers. Stationary through the tuck (fingertip moves < 1 cm 26.0-29.7 s). The strand it holds up is the one the end goes under: s_from_tip is that strand's pass at X2, read at 32.0 s")
    ev(step=2, hand="right", action="pinch", s_from_tip_m=round(tuck["arc_traced_m"], 4), path=[], t0=28.3, t1=28.9,
       shot="step2_closeup", reliability="estimated",
       note="takes the end ~0.13 m from its tip (the hand is mostly off the right of the frame; the value is the tip-to-frame-edge arc at 29.3 s, a lower bound)")
    ev(step=2, hand="right", action="carry", s_from_tip_m=0.0, path=[[q[1], q[2]] for q in tip_tuck], t_path=[q[0] for q in tip_tuck],
       t0=28.6, t1=29.5, shot="step2_closeup", reliability="measured",
       note="lifts the end from the lower right and lays it up and across the loop's right side into the eye (OVER that side); the path is the tip's")
    p, _ = _path(rows, "right", 29.8, 30.0)
    ev(step=2, hand="right", action="carry", s_from_tip_m=0.0, path=p, t0=29.5, t1=29.9, shot="step2_closeup", reliability="estimated",
       note="brings the tip down across the eye to just beside the left hand's X1")
    p, _ = _path(rows, "right", 29.9, 30.2)
    ev(step=2, hand="right", action="push_under", s_from_tip_m=0.0, under_s_from_tip_m=loose_s(2, 1), path=p, t0=29.9, t1=30.2,
       shot="step2_closeup", reliability="estimated",
       note="the index finger tucks the tip UNDER the loop strand next to X1, the one the left hand holds up (crossing X2: the end under). Mostly occluded by the fingers; the result is read at 32.0 s")
    p, _ = _path(rows, "right", 30.2, 30.5)
    ev(step=2, hand="right", action="pull_through", s_from_tip_m=0.0, over_s_from_tip_m=loose_s(1, 1), path=p, t0=30.2, t1=30.7,
       shot="step2_closeup", reliability="measured",
       note="draws the tip up through the eye and out to the right, OVER the loop's far (right) side (crossing X3: the end over); the knot is formed, loose")
    ev(step=2, hand="right", action="release", t0=30.6, t1=30.8, shot="step2_closeup", reliability="measured", note="the right hand leaves the frame")
    ev(step=2, hand="left", action="release", t0=31.3, t1=31.5, shot="step2_closeup", reliability="measured", note="lets X1 go; the loose knot lies on its own (31.7-33.4 s)")
    ev(step=3, hand="left", action="pinch", s_from_tip_m=s_lefthold3, path=pull_l[:1], t0=33.5, t1=33.6, shot="wide_step3", reliability="estimated",
       note="takes the standing part just left of the knot (its arc from the tip composed from the 33.6 s tail and the 32.0 s knot)")
    ev(step=3, hand="right", action="pinch", s_from_tip_m=_mid(r_pull) if r_pull else None, s_range_m=r_pull, path=pull_r[:1], t0=33.5, t1=33.6,
       shot="wide_step3", reliability="measured", note="takes the end right of the knot, ~5-9 cm from the tip")
    ev(step=3, hand="left", action="pull", s_from_tip_m=s_lefthold3, path=pull_l, distance_m=disp(pull_l), t0=33.6, t1=35.8,
       shot="wide_step3", reliability="measured", note="draws the standing part away to the left (+y) along the rope; it leaves the frame at ~35.3 s, so the distance (to 35.3 s) is a lower bound")
    ev(step=3, hand="right", action="pull", s_from_tip_m=_mid(r_pull) if r_pull else None, path=pull_r, distance_m=disp(pull_r), t0=33.6, t1=35.8,
       shot="wide_step3", reliability="measured", note="draws the end away to the right (-y) along the rope (distance to 35.7 s); the knot closes to ~4-5 cm")

    key_shapes = []
    for k in KEYS:
        o = K[k["label"]]
        key_shapes.append({kk: o[kk] for kk in ("t", "label", "shot", "start", "note", "width_px", "m_per_px", "arc_visible_m",
                                                  "arc_traced_m", "centreline", "centreline_centred", "centred_on",
                                                  "crossings", "gauss_from_start")}
                          | {"hands": [{kk: h[kk] for kk in ("hand", "fingertip", "rope_under")} for h in o["hands"]]})

    retarget = {
        "basis": "arc length from the working end's tip [m], table metres; the working end is the person's RIGHT end",
        "d_grab_step1": {"value": round(fold[0], 4) if fold else None, "range": fold,
                         "how": "20.0 s: the right hand's fold; first value = where the end's run enters the hand (measured), second = where the standing part leaves it; the apex lies between"},
        "d_take_end_step1": {"value": _mid(take) if take else None, "how": "20.4 s: the left hand's pinch on the end's run, the point it carries over the standing part"},
        "X1_end_side": {"value": round(x1["s_start_m"][0], 4) if x1 else None, "t": 22.7},
        "X1_standing_side": {"value": round(x1["s_start_m"][1], 4) if x1 else None, "t": 22.7},
        "loop_circumference": {"oval_20.9": oval_loop, "turned_21.3": round(tl["crossings"][0]["s_start_m"][1] - tl["crossings"][0]["s_start_m"][0], 4) if tl["crossings"] else None,
                               "teardrop_22.7": round(x1["s_start_m"][1] - x1["s_start_m"][0], 4) if x1 else None,
                               "closeup_25.0": round(lc["crossings"][0]["s_start_m"][1] - lc["crossings"][0]["s_start_m"][0], 4) if lc["crossings"] else None,
                               "note": "the right hand draws the end out while the left holds X1, so the loop shrinks; 20.9 s excludes the rope under the left hand"},
        "loop_size_wxh": {"teardrop_22.7": loop_box(loop, x1), "closeup_25.0": loop_box(lc, lc["crossings"][0] if lc["crossings"] else None),
                          "note": "extent [x, y] of the loop between X1's two passes, table metres"},
        "d_grab_step2": {"value": round(tuck["arc_traced_m"], 4), "how": "29.3 s: tip to where the end leaves the frame under the right hand (lower bound)"},
        "loose_knot_32.0": {
            "tail_tip_to_X3": round(tail, 4),
            "X3_end_side": loose_s(1, 0), "X2_end_side": loose_s(2, 0), "X1_end_side": loose_s(3, 0),
            "X3_loop_side": loose_s(1, 1), "X2_loop_side": loose_s(2, 1), "X1_standing_side": loose_s(3, 1),
            "how": "arcs within the knot measured at 32.0 s (from where the end leaves the frame); the tail from the tip to X3 measured at 33.6 s (wide shot) and added",
        },
        "knot_rope_first_to_last_crossing": knot_len,
        "tail_lengths": {"working_tail_tip_to_first_crossing": round(tail, 4),
                         "standing_tail": "not measurable: the far end is never in view (REAL2SIM assumed 1.05 m in all)"},
        "step3": {"right_grasp_s": _mid(r_pull) if r_pull else None, "left_grasp_s": s_lefthold3,
                  "right_pull_m": disp(pull_r), "left_pull_m": disp(pull_l),
                  "ring_33.6_wxh": ring_size(ps, (215, 425)), "ring_34.3_wxh": ring_size(pulling, (280, 455)),
                  "ring_35.8_wxh": ring_size(tight, (290, 380)), "ring_32.0_loose_wxh": loose_ring,
                  "note": "pull = fingertip displacement from 33.6 s to 35.3 s (left: it then leaves the frame, a lower bound) / 35.7 s (right). Ring extents [along x (away), along y (sideways)] from the rope mask above the tails' line. The 32.0 s ring (close-up scale) and the 33.6 s ring (wide scale) differ by ~50%: the hands re-handled the knot across the cut, and each shot's scale is only good to ~10-15%"},
    }

    crossing_structure = {
        "after_step1": {"t": [22.7, 25.0], "n": 1, "gauss_from_tip": loop["gauss_from_start"], "closeup_gauss_from_right_hand": lc["gauss_from_start"],
                        "words": "one crossing X1: the end OVER the standing part; walking from the standing part: X1 under, up the loop's right side, across its far end right-to-left, down its left side, X1 over, then the end toward the person and right",
                        "evidence": [x1["evidence"] if x1 else None, lc["crossings"][0]["evidence"] if lc["crossings"] else None]},
        "after_step2": {"t": 32.0, "n": len(loose["crossings"]), "gauss_from_tip": loose["gauss_from_start"],
                        "gauss_from_standing_part": "1U 2O 3U 1O 2U 3O",
                        "words": "X3 (met first from the tip): the end OVER the loop's far side; X2: the end UNDER the loop strand just past X1 on the standing side; X1: the end OVER the standing part. An alternating 3-crossing diagram: the overhand knot (trefoil)",
                        "evidence": [c["evidence"] for c in loose["crossings"]]},
        "after_step3": {"t": 35.8, "n": 3, "gauss_from_tip": loose["gauss_from_start"],
                        "words": "the same three crossings drawn tight (~4 cm): too small in the wide shot to read over/under (assumed unchanged: pulling the tails does not change the code)"},
    }
    return {"events": events, "key_shapes": key_shapes, "retarget_numbers": retarget, "crossing_structure": crossing_structure, "_K": K}


# ----------------------------------------------------------------------------------------------- the picture sheet

PANELS = [  # (t, key label or None for an automatic panel, title)
    (19.4, "straight", "straight: L holds the standing part"),
    (20.0, "fold", "R folds the end back (far side)"),
    (20.4, "take_end", "L takes the end's run"),
    (20.9, "loop_oval", "L lays the end over the standing part"),
    (21.3, "turn_loop", "X1 made, end over; R turns the loop"),
    (22.7, "loop", "after step 1: the loop, 1 crossing"),
    (29.3, "tuck_over", "step 2: R lays the end over the far side"),
    (30.1, None, "R tucks the tip under (X2); L holds X1"),
    (30.6, None, "R draws the tip out over the far side (X3)"),
    (32.0, "loose", "after step 2: the loose overhand"),
    (34.3, "pulling", "step 3: both hands pull the tails apart"),
    (35.8, "tight", "pulled: the knot ~4-5 cm"),
]


def _grad(i, n):
    u = i / max(1, n - 1)
    return (int(255 * u), int(60 + 120 * (1 - abs(2 * u - 1))), int(255 * (1 - u)))  # red (start) -> blue (far)


def panel(cap, t, key_out, title) -> np.ndarray:
    im = read_frame(cap, t) if key_out is None else key_out["_im"]
    fr = process(im, t) if key_out is None else key_out["_frame"]
    vis = (0.72 * im).astype(np.uint8)
    for hd in fr["hands"]:
        c = np.array((235, 140, 40) if hd["hand"] == "left" else (40, 140, 255), np.float32)  # BGR: left blue, right orange
        vis[hd["_mask"]] = (0.55 * vis[hd["_mask"]] + 0.45 * c).astype(np.uint8)
        q = tuple(int(v) for v in hd["fingertip"])
        cv2.circle(vis, q, 7, (255, 255, 255), 2)
        cv2.putText(vis, hd["hand"][0].upper(), (q[0] + 8, q[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    if key_out is not None:
        pts, visb = key_out["_pts"], key_out["_visible"]
        n = len(pts)
        for i in range(n - 1):
            p0, p1 = tuple(np.int32(pts[i])), tuple(np.int32(pts[i + 1]))
            if visb[i]:
                cv2.line(vis, p0, p1, _grad(i, n), 3, cv2.LINE_AA)
            elif i % 6 < 3:
                cv2.line(vis, p0, p1, (235, 235, 235), 2, cv2.LINE_AA)
        cv2.circle(vis, tuple(np.int32(pts[0])), 7, (0, 0, 255), -1)
        cv2.putText(vis, "tip" if key_out["start"].startswith("tip") else "from", tuple(np.int32(pts[0]) + [6, 22]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        names = {}
        if key_out["label"] == "loose":
            names = {1: "X3", 2: "X2", 3: "X1"}
        for c in key_out["crossings"]:
            q = tuple(np.int32(c["xy_px"]))
            cv2.drawMarker(vis, q, (0, 255, 255), cv2.MARKER_TILTED_CROSS, 18, 3)
            who = {"a": "end over", "b": "end under"}.get(c["over"], "?") if c.get("measured", True) else "(hidden)"
            if key_out["label"] in ("loop", "loop_closeup", "turn_loop"):
                who = {"a": "end over", "b": "standing over"}.get(c["over"], "?") if c.get("measured", True) else "under hand"
            txt = f"{names.get(c['label'], 'X1' if key_out['label'] != 'loose' else '')} {who}"
            (tw, _), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.62, 2)
            tx = min(q[0] + 10, 636 - tw)
            ty = q[1] - 12 if q[1] > 90 else q[1] + 28
            cv2.putText(vis, txt, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 5)
            cv2.putText(vis, txt, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)
        for h in key_out["hands"]:
            for r in h["rope_under"]:
                q = tuple(int(v) for v in r["mid_px"])
                cv2.rectangle(vis, (q[0] - 7, q[1] - 7), (q[0] + 7, q[1] + 7), (255, 255, 255), 2)
                s_ = 0.5 * (r["s_start_m"][0] + r["s_start_m"][1])
                lab_ = f"{h['hand'][0].upper()} s={s_:.2f}" + ("" if key_out["start"].startswith("tip") else "*")
                cv2.putText(vis, lab_, (q[0] - 30, q[1] + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 5)
                cv2.putText(vis, lab_, (q[0] - 30, q[1] + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
    else:
        for i, p in enumerate(fr["polys"]):
            cv2.polylines(vis, [p["pts"].astype(np.int32).reshape(-1, 1, 2)], bool(p.get("closed")), (255, 200, 80), 3, cv2.LINE_AA)
        for e in fr["ends"]:
            cv2.drawMarker(vis, tuple(np.int32(e["xy"])), (0, 0, 255), cv2.MARKER_STAR, 14, 2)
    head = f"{t:.1f}s  {shot_at(t)['name']}"
    cv2.rectangle(vis, (0, 0), (640, 62), (0, 0, 0), -1)
    cv2.putText(vis, head, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(vis, title, (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (120, 230, 255), 2)
    if key_out is not None and key_out["gauss_from_start"]:
        g = f"Gauss from {'tip' if key_out['start'].startswith('tip') else 'working end'}: {key_out['gauss_from_start']}"
        cv2.rectangle(vis, (0, 446), (640, 480), (0, 0, 0), -1)
        cv2.putText(vis, g, (8, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (120, 255, 120), 2)
    return vis


def sheet(cap, K, path: Path) -> None:
    tiles = []
    for t, label, title in PANELS:
        tiles.append(cv2.resize(panel(cap, t, K.get(label) if label else None, title), (480, 360), interpolation=cv2.INTER_AREA))
    rows_ = [np.hstack(tiles[i: i + 6]) for i in range(0, len(tiles), 6)]
    img = np.vstack(rows_)
    legend = np.zeros((44, img.shape[1], 3), np.uint8)
    cv2.putText(legend, "centreline red (tip / working end) -> blue; dashed white = under a hand; yellow x = crossing; "
                "blue tint L hand, orange R hand; square = rope under the hand (s = arc from the tip, m; * = from the panel's red start)",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), np.vstack([img, legend]))


# ----------------------------------------------------------------------------------------------- main


def _json_clean(o):
    if isinstance(o, dict):
        return {k: _json_clean(v) for k, v in o.items() if not str(k).startswith("_")}
    if isinstance(o, (list, tuple)):
        return [_json_clean(v) for v in o]
    if isinstance(o, (np.floating,)):
        return round(float(o), 5)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.ndarray):
        return _json_clean(o.tolist())
    if isinstance(o, float):
        return round(o, 5)
    return o


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(HERE / "demo_plan.json"))
    ap.add_argument("--png", default=str(HERE / "demo_plan.png"))
    a = ap.parse_args()
    cap = cv2.VideoCapture(str(CLIP))
    rows = track(cap, SPAN[0], SPAN[1])
    sc = shot_scale(rows)
    # Scales: the two wide shots are one framing (pooled); the loop close-up its own; the zooming close-up per frame.
    wide = [r["width_px"] for r in rows if r["shot"] in ("wide_step1", "wide_step3") and r["width_px"] and r["rope_px"] > 2000]
    SCALE.clear()
    SCALE["wide_step1"] = SCALE["wide_step3"] = ROPE_D / float(np.median(wide))
    for name in ("straight_closeup", "loop_closeup"):
        if name in sc:
            SCALE[name] = sc[name]["m_per_px"]
    zt = [r["t"] for r in rows if r["shot"] in ("step2_closeup", "loose_closeup") and r["width_px"] and r["rope_px"] > 5000]
    zw = [r["width_px"] for r in rows if r["shot"] in ("step2_closeup", "loose_closeup") and r["width_px"] and r["rope_px"] > 5000]
    zw = ndimage.median_filter(np.asarray(zw), size=9, mode="nearest")
    SCALE["zoom"] = (np.asarray(zt), zw)
    plan = build_plan(cap, rows)
    shots = []
    for s in SHOTS:
        if s["t1"] < SPAN[0] or s["t0"] > SPAN[1] + 1:
            continue
        up, right = up_right(s)
        mpp = SCALE.get(s["name"])
        entry = {"t0": s["t0"], "t1": s["t1"], "name": s["name"], "step": s["step"], "camera": s["camera"],
                 "m_per_px": round(mpp, 6) if mpp else ("per frame (zoom): " + ", ".join(f"{t:.1f}s {ROPE_D / w * 1000:.3f}mm"
                                                                              for t, w in list(zip(*SCALE["zoom"]))[::10])
                                                        if s["name"] in ("step2_closeup", "loose_closeup") else None),
                 "rope_width_px_median": sc.get(s["name"], {}).get("width_px_median"),
                 "camera_m_per_px": sc.get(s["name"], {}).get("camera_m_per_px"),
                 "image_to_table": {"x": f"{up[0]:+.3f}*(240-v)*m {right[0]:+.3f}*(u-320{'-30.5' if s['name'] == 'wide_step1' else ''})*m",
                                    "y": f"{up[1]:+.3f}*(240-v)*m {right[1]:+.3f}*(u-320{'-30.5' if s['name'] == 'wide_step1' else ''})*m",
                                    "m": "m_per_px", "origin": "image centre (wide shots: the step-3 image centre)"},
                 "reliable": s["reliable"], "note": s["note"]}
        shots.append(entry)
    out = {
        "clip": str(CLIP.name),
        "source": "Howcast, 'How to Tie an Overhand Knot' (youtube SwdJ5op25SM), 640x480 29.97 fps",
        "span": list(SPAN),
        "sampled_fps": FPS_SAMPLE,
        "made_by": "video/overhand/demo_extract.py",
        "cuts_detected": detect_cuts(cap, thr=20.0),
        "scale_cross_check": {k: v for k, v in sc.items()},
        "shots": shots,
        "rope": {"diameter_m": ROPE_D, "length_assumed_m": ROPE_LEN_ASSUMED,
                 "working_end": "the person's RIGHT end (table -y; image right in the top-down shots); its tip is the darker end, in view from 20.0 s",
                 "fraction_basis": "rope_fraction = s_from_tip_m / 1.05 (REAL2SIM's assumed length; the far end is never seen) -- use s_from_tip_m",
                 "frames": "path / centreline / xy: table metres in the shot's frame, origin at the image centre, x away from the person, y to the person's left"},
        "events": plan["events"],
        "key_shapes": plan["key_shapes"],
        "crossing_structure": plan["crossing_structure"],
        "retarget_numbers": plan["retarget_numbers"],
        "tracks": [{"t": r["t"], "shot": r["shot"], "width_px": r["width_px"],
                    "hands": {h: {"fingertip": to_table(d["fingertip_px"], shot_at(r["t"]), m_per_px_at(r["t"], r["width_px"]))[0].round(4).tolist(),
                                  "on_rope": d["on_rope"]} for h, d in r["hands"].items()}} for r in rows],
        "manual": "key-frame route waypoints (KEYS in demo_extract.py) were read off gridded stills by eye; the traced centrelines, "
                  "arc lengths, crossings and over/under come from the image (mask, distance transform, intensity profiles)",
    }
    Path(a.out).write_text(json.dumps(_json_clean(out), indent=1))
    sheet(cap, plan["_K"], Path(a.png))
    print("wrote", a.out, a.png)


if __name__ == "__main__":
    main()
