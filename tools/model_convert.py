#!/usr/bin/env python3
"""
model_convert.py - converts any 3D model (glTF/GLB, OBJ, FBX-via-glTF, STL...) into the
R3DM binary used by the ESP32-S3 real-time renderer.

What it does (all automatic, overridable with a JSON file):
  1. loads the scene, applies node transforms, keeps part + material names
  2. classifies every material into a shading preset (paint, glass, chrome, tire, ...)
  3. normalises orientation (length along +X, front at +X, Y up, wheels on y=0)
  4. removes geometry that can never be seen (interior behind opaque glass, hidden parts)
  5. decimates to a triangle budget, spending triangles where they are visible
  6. rebuilds normals with crease angles, bakes per-vertex ambient occlusion
  7. renders a studio-lit "matcap" per material (car paint clear-coat, chrome, glass...)
  8. writes model.bin (flash partition image) + a JSON report

Usage:
  python model_convert.py audi_a4.glb -o model.bin
  python model_convert.py audi_a4.glb -o model.bin --budget 12000 --paint "#B0101A"
  python model_convert.py audi_a4.glb -o model.bin --config audi_a4.json

Requires: pip install numpy trimesh pygltflib fast-simplification scipy pillow
          (+ DracoPy if the GLB uses Draco compression)
"""
import argparse
import json
import math
import re
import struct
import sys
import zlib

import numpy as np
from scipy.spatial import cKDTree

try:
    import trimesh
except ImportError:
    sys.exit("pip install trimesh pygltflib fast-simplification scipy pillow numpy")

try:
    import fast_simplification
except ImportError:
    fast_simplification = None
try:
    import meshoptimizer as meshopt          # preferred: high quality, keeps original vertices
except ImportError:
    meshopt = None

# --------------------------------------------------------------------------------------
# Shading classes (must match r3d_model.h)
# --------------------------------------------------------------------------------------
CLS_PAINT, CLS_GLASS, CLS_CHROME, CLS_METAL, CLS_TIRE, CLS_PLASTIC, CLS_LIGHT_RED, \
    CLS_LIGHT_CLEAR, CLS_INTERIOR, CLS_CARBON, CLS_GENERIC = range(11)
CLS_NAMES = ["paint", "glass", "chrome", "metal", "tire", "plastic", "light_red",
             "light_clear", "interior", "carbon", "generic"]
CLS_DROP = -1

# triangle-budget weight per class (paint gets the most detail)
CLS_WEIGHT = {CLS_PAINT: 1.6, CLS_GLASS: 0.8, CLS_CHROME: 1.0, CLS_METAL: 1.0, CLS_TIRE: 0.9,
              CLS_PLASTIC: 0.8, CLS_LIGHT_RED: 1.0, CLS_LIGHT_CLEAR: 1.1, CLS_INTERIOR: 0.4,
              CLS_CARBON: 0.8, CLS_GENERIC: 0.8}
# decimation importance per class (quadric error scales with the square of this)
CLS_IMPORTANCE = {CLS_PAINT: 2.6, CLS_GLASS: 1.1, CLS_CHROME: 1.0, CLS_METAL: 0.6, CLS_TIRE: 0.55,
                  CLS_PLASTIC: 0.55, CLS_LIGHT_RED: 1.1, CLS_LIGHT_CLEAR: 1.2, CLS_INTERIOR: 0.3,
                  CLS_CARBON: 0.8, CLS_GENERIC: 0.9}
# matcap resolution per class (128 for large glossy surfaces)
CLS_MATCAP = {CLS_PAINT: 128, CLS_GLASS: 128, CLS_CHROME: 128}

# classes whose texture brightness detail is kept per vertex
WHEEL_RX = re.compile(r"wheel|(?<![a-z])rims?(?![a-z])|tyre|tire|felg")
BODYCOL_RX = re.compile(r"(?<![a-z])colou?red(?![a-z])|body.?colou?r")
LIVERY_RX = re.compile(r"paint|(?<![a-z])lack|primary")          # a textured paint material = livery
TEX_DETAIL_CLS = (CLS_GENERIC, CLS_PLASTIC, CLS_INTERIOR, CLS_PAINT, CLS_LIGHT_RED, CLS_LIGHT_CLEAR)

MAGIC = b"R3DM"
VERSION = 2


def log(*a):
    print(*a, flush=True)


# --------------------------------------------------------------------------------------
# Loading
# --------------------------------------------------------------------------------------
def srgb_to_linear(c):
    c = np.asarray(c, dtype=np.float64)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def linear_to_srgb(c):
    c = np.clip(c, 0.0, 1.0)
    return np.where(c <= 0.0031308, c * 12.92, 1.055 * np.power(c, 1 / 2.4) - 0.055)


def material_info(mesh):
    """Returns dict(name, color_lin(3), alpha, metallic, roughness)."""
    info = dict(name="default", color=np.array([0.5, 0.5, 0.5]), alpha=1.0,
                metallic=0.0, roughness=0.5, double_sided=False)
    vis = mesh.visual
    mat = getattr(vis, "material", None)
    if mat is not None:
        info["name"] = str(getattr(mat, "name", None) or "default")
        bcf = getattr(mat, "baseColorFactor", None)
        if bcf is not None:
            bcf = np.asarray(bcf, dtype=np.float64)
            if bcf.max() > 1.0:
                bcf = bcf / 255.0
            info["color"] = bcf[:3].copy()          # glTF factor is linear
            info["alpha"] = float(bcf[3]) if len(bcf) > 3 else 1.0
        else:
            dif = getattr(mat, "diffuse", None)     # OBJ SimpleMaterial (sRGB)
            if dif is not None:
                dif = np.asarray(dif, dtype=np.float64)
                if dif.max() > 1.0:
                    dif = dif / 255.0
                info["color"] = srgb_to_linear(dif[:3])
                if len(dif) > 3:
                    info["alpha"] = float(dif[3])
        tex = getattr(mat, "baseColorTexture", None) or getattr(mat, "image", None)
        if tex is not None:
            try:
                arr = np.asarray(tex.convert("RGBA").resize((64, 64)), dtype=np.float64) / 255.0
                tex_lin = srgb_to_linear(arr[..., :3]).reshape(-1, 3).mean(0)
                info["color"] = info["color"] * tex_lin
            except Exception:
                pass
        info["double_sided"] = bool(getattr(mat, "doubleSided", False) or False)
        m = getattr(mat, "metallicFactor", None)
        r = getattr(mat, "roughnessFactor", None)
        if m is not None:
            info["metallic"] = float(m)
        if r is not None:
            info["roughness"] = float(r)
        if str(getattr(mat, "alphaMode", "") or "").upper() == "BLEND" and info["alpha"] > 0.95:
            info["alpha"] = 0.5
    elif hasattr(vis, "vertex_colors") and vis.kind == "vertex":
        vc = np.asarray(vis.vertex_colors, dtype=np.float64)[:, :3] / 255.0
        info["color"] = srgb_to_linear(vc).mean(0)
    return info


def _srgb_to_oklab(c):
    """c: (...,3) sRGB 0..1 -> OKLab (perceptual distances for clustering)."""
    l = srgb_to_linear(c)
    M1 = np.array([[0.4122214708, 0.5363325363, 0.0514459929],
                   [0.2119034982, 0.6806995451, 0.1073969566],
                   [0.0883024619, 0.2817188376, 0.6299787005]])
    M2 = np.array([[0.2104542553, 0.7936177850, -0.0040720468],
                   [1.9779984951, -2.4285922050, 0.4505937099],
                   [0.0259040371, 0.7827717662, -0.8086757660]])
    return np.cbrt(np.maximum(l @ M1.T, 0)) @ M2.T


def face_texture_colors(geom):
    """Per-face sRGB colour sampled from the base colour texture (None if no texture/UVs)."""
    vis = geom.visual
    mat = getattr(vis, "material", None)
    uv = getattr(vis, "uv", None)
    if mat is None or uv is None or len(uv) != len(geom.vertices):
        return None
    tex = getattr(mat, "baseColorTexture", None) or getattr(mat, "image", None)
    if tex is None:
        return None
    img = np.asarray(tex.convert("RGB"), dtype=np.float64) / 255.0
    h, w = img.shape[:2]
    F = np.asarray(geom.faces)
    uv = np.asarray(uv, dtype=np.float64)
    acc = np.zeros((len(F), 3))
    # 4 samples per face: centroid + points half way to each corner (avoids seam bleeding)
    cen = uv[F].mean(1)
    for k in range(-1, 3):
        p = cen if k < 0 else 0.5 * (cen + uv[F[:, k]])
        x = np.clip((np.mod(p[:, 0], 1.0) * w).astype(np.int64), 0, w - 1)
        y = np.clip(((1.0 - np.mod(p[:, 1], 1.0)) * h).astype(np.int64), 0, h - 1)
        acc += img[y, x]
    col = acc / 4.0
    bcf = getattr(mat, "baseColorFactor", None)
    if bcf is not None:
        bcf = np.asarray(bcf, dtype=np.float64)[:3]
        if bcf.max() > 1.0:
            bcf = bcf / 255.0
        col = linear_to_srgb(srgb_to_linear(col) * bcf)
    return col


def split_by_texture(V, F, col, max_colors, l_weight=1.0):
    """Cluster per-face texture colours into regions. Returns per-face labels and region sRGB
    colours. l_weight < 1: regions follow hue more than brightness (objects, where the
    brightness detail is kept per vertex)."""
    from scipy.cluster.vq import kmeans2
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    if col is None:
        return None, None
    F = np.asarray(F)
    V = np.asarray(V)
    area = np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1) * 0.5
    lab = _srgb_to_oklab(col)
    lab_w = lab * np.array([l_weight, 1.6, 1.6])     # hue matters more than shading in the bake
    rng = np.random.default_rng(1)
    n_fit = min(len(F), 40000)
    sel = rng.choice(len(F), n_fit, p=area / area.sum())
    k = max(2, max_colors)
    cent, _ = kmeans2(lab_w[sel], k, minit="++", seed=1)
    # merge near-identical clusters
    keep = []
    for c in cent:
        if all(np.linalg.norm(c - q) > 0.06 for q in keep):
            keep.append(c)
    cent = np.array(keep)
    lbl = np.argmin(((lab_w[:, None, :] - cent[None]) ** 2).sum(-1), axis=1)
    # smooth labels over the surface: majority of edge neighbours (removes texture noise);
    # adjacency over position-welded vertices so UV seams do not break it
    _, inv = np.unique(np.round(V / 1e-6).astype(np.int64), axis=0, return_inverse=True)
    adj = trimesh.graph.face_adjacency(inv.reshape(-1)[F])
    for _ in range(4):
        cnt = np.zeros((len(F), len(cent)))
        np.add.at(cnt, (adj[:, 0], lbl[adj[:, 1]]), 1)
        np.add.at(cnt, (adj[:, 1], lbl[adj[:, 0]]), 1)
        cnt[np.arange(len(F)), lbl] += 1.5
        lbl = cnt.argmax(1)
    # absorb tiny islands into their neighbours (fewer borders -> cleaner decimation)
    min_area = area.sum() * 0.0015
    for _ in range(3):
        same = lbl[adj[:, 0]] == lbl[adj[:, 1]]
        g = coo_matrix((np.ones(same.sum()), (adj[same, 0], adj[same, 1])), shape=(len(F), len(F)))
        ncomp, comp = connected_components(g, directed=False)
        comp_area = np.bincount(comp, weights=area, minlength=ncomp)
        small = comp_area[comp] < min_area
        if not small.any():
            break
        cnt = np.zeros((len(F), len(cent)))
        e = adj[~same]
        np.add.at(cnt, (e[:, 0], lbl[e[:, 1]]), area[e[:, 1]])
        np.add.at(cnt, (e[:, 1], lbl[e[:, 0]]), area[e[:, 0]])
        tgt = cnt.argmax(1)
        upd = small & (cnt.max(1) > 0)
        # propagate per component: majority target of its faces
        for c in np.unique(comp[upd]):
            fm = comp == c
            lbl[fm] = np.bincount(tgt[fm & upd], minlength=len(cent)).argmax()
    used = np.unique(lbl)
    remap = -np.ones(len(cent), np.int64); remap[used] = np.arange(len(used))
    lbl = remap[lbl]
    # region colour = area weighted mean of its faces (in linear light)
    colors = []
    for r in range(len(used)):
        m = lbl == r
        colors.append(linear_to_srgb(np.average(srgb_to_linear(col[m]), axis=0, weights=area[m] + 1e-12)))
    return lbl, np.array(colors)


def _unlit_material_names(path):
    """names of glTF materials with KHR_materials_unlit (light baked into the texture)"""
    try:
        with open(path, "rb") as f:
            data = f.read()
        if data[:4] == b"glTF":
            n = struct.unpack_from("<I", data, 12)[0]
            g = json.loads(data[20:20 + n])
        elif path.lower().endswith(".gltf"):
            g = json.loads(data)
        else:
            return set()
        return {m.get("name") or "default" for m in g.get("materials", [])
                if "KHR_materials_unlit" in (m.get("extensions") or {})}
    except Exception:
        return set()


def load_parts(path):
    unlit = _unlit_material_names(path)
    scene = trimesh.load(path, force="scene")
    parts = []
    for node in scene.graph.nodes_geometry:
        T, gname = scene.graph[node]
        geom = scene.geometry[gname]
        if not isinstance(geom, trimesh.Trimesh) or len(geom.faces) == 0:
            continue
        m = geom.copy()
        m.apply_transform(T)
        info = material_info(geom)
        info["unlit"] = info["name"] in unlit
        parts.append(dict(name=str(node), mesh=m, mat=info, tex_geom=geom))
    if not parts:
        sys.exit("No triangle meshes found in " + path)
    return parts


def _is_reddish(srgb):
    r, g, b = srgb[:3]
    return r > 2.2 * max(g, b) and r > 0.08


def drop_lamp_covers(parts, dist=0.05):
    """Transparent lenses over head/tail lights: the renderer has no transparency, so a
    lens would hide the lamp behind it as a dark pane. Removed: glass triangles lying on top
    of light geometry (closer than `dist`, car length = 2) and glass at the front / rear end
    below the window line (lamp lenses). Windows stay."""
    lights = [p for p in parts if p["cls"] in (CLS_LIGHT_RED, CLS_LIGHT_CLEAR) and len(p["F"])]
    car_h = max(float(p["V"][:, 1].max()) for p in parts if len(p["V"]))
    tree = None
    if lights:
        pts = np.concatenate([np.concatenate([p["V"], p["V"][p["F"]].mean(1)]) for p in lights])
        tree = cKDTree(pts)
    out, dropped = [], 0
    for p in parts:
        if p["cls"] != CLS_GLASS or not len(p["F"]):
            out.append(p)
            continue
        cen = p["V"][p["F"]].mean(1)
        keep = ~((np.abs(cen[:, 0]) > 0.5) & (cen[:, 1] < 0.6 * car_h))
        if tree is not None:
            d, _ = tree.query(cen)
            near = d <= dist
            # glass next to a lamp is its lens - except a window passing a lamp at window
            # height (third brake light at the rear window): only a glass part that is mostly
            # lamp lens loses its faces up there
            lens = near.sum() > 0.5 * len(near)
            keep &= ~(near & ((cen[:, 1] < 0.6 * car_h) | lens))
        dropped += int((~keep).sum())
        if keep.all():
            out.append(p)
            continue
        if keep.any():
            Fp = p["F"][keep]
            used = np.unique(Fp)
            remap = -np.ones(len(p["V"]), np.int64); remap[used] = np.arange(len(used))
            q = dict(p); q["V"], q["F"] = p["V"][used], remap[Fp]
            q.pop("tex_geom", None); q["fcol"] = None; q["tex_detail"] = False
            out.append(q)
    if dropped:
        log(f"  removed {dropped} lamp-lens glass triangles (lamps visible)")
    return out


def drop_overlay_shells(parts, tol=0.012):
    """Transparent shells lying on an opaque surface (clear coat over the paint, decal films):
    without transparency both layers would fight for the same pixels."""
    cand = [p for p in parts if p["mat"]["alpha"] < 0.6 and len(p["F"]) and
            p["cls"] not in (CLS_GLASS, CLS_LIGHT_RED, CLS_LIGHT_CLEAR)]
    opaque = [p for p in parts if p["mat"]["alpha"] >= 0.6 and p["cls"] != CLS_GLASS and len(p["F"])]
    if not cand or not opaque:
        return parts
    tree = cKDTree(np.concatenate([np.concatenate([p["V"], p["V"][p["F"]].mean(1)]) for p in opaque]))
    drop = set()
    for p in cand:
        V, F = p["V"], p["F"]
        area = np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)
        d, _ = tree.query(V[F].mean(1), distance_upper_bound=tol * 1.0001)
        if area.sum() > 0 and area[d <= tol].sum() >= 0.6 * area.sum():
            drop.add(id(p))
    if drop:
        log("  removed transparent overlay layer(s): " +
            ", ".join(sorted({p["mat"]["name"] for p in parts if id(p) in drop})))
    return [p for p in parts if id(p) not in drop]


def lamps_by_position(parts):
    """Cars whose lamps have no telling names: small parts at the very front / rear, bright
    neutral (or amber) at the front = headlights, red at the rear = tail lights. Only for an
    end of the car where no lamp was found by name."""
    car_h = max(float(p["V"][:, 1].max()) for p in parts if len(p["V"]))
    def has_light(front):
        for p in parts:
            if p["cls"] in (CLS_LIGHT_CLEAR, CLS_LIGHT_RED) and not p.get("from_split") and len(p["V"]):
                if (p["V"][:, 0].mean() > 0) == front:
                    return True
        return False
    do_front, do_rear = not has_light(True), not has_light(False)
    if not do_front and not do_rear:
        return parts
    out, nf_, nr_ = [], 0, 0
    for p in parts:
        if p["cls"] not in (CLS_GENERIC, CLS_PLASTIC) or not len(p["F"]):
            out.append(p)
            continue
        V, F = p["V"], p["F"]
        front = do_front and V[:, 0].min() > 0.62
        rear = do_rear and V[:, 0].max() < -0.62
        y0, y1 = V[:, 1].min(), V[:, 1].max()
        if (not front and not rear) or y1 - y0 > 0.35 * car_h or y0 < 0.2 * car_h or y1 > 0.8 * car_h:
            out.append(p)
            continue
        mat_col = linear_to_srgb(p["mat"]["color"])
        col = p.get("fcol")
        if col is None or len(col) != len(F):
            col = np.tile(mat_col, (len(F), 1))
        lum = col @ np.array([0.2126, 0.7152, 0.0722])
        if front:
            lamp = ((lum > 0.45) & (np.ptp(col, axis=1) < 0.15)) | np.array([_is_amber(c) for c in col])
        else:
            lamp = np.array([_is_red(c) or _is_reddish(c) for c in col])
        if not lamp.any():
            out.append(p)
            continue
        # textured lamp: the whole part becomes a lamp, the texture split ('light' mode) then
        # sorts its colour regions (red lens, reverse light, black housing, amber indicator)
        if p.get("fcol") is not None and lamp.sum() >= 0.2 * len(F):
            q = dict(p); q["cls"] = CLS_LIGHT_CLEAR if front else CLS_LIGHT_RED
            out.append(q)
            if front:
                nf_ += len(F)
            else:
                nr_ += len(F)
            continue
        masks = [(np.ones(len(F), bool), True)] if lamp.sum() >= 0.5 * len(F) else [(lamp, True), (~lamp, False)]
        for m, is_lamp in masks:
            if not m.any():
                continue
            Fp = F[m]
            used = np.unique(Fp)
            remap = -np.ones(len(V), np.int64); remap[used] = np.arange(len(used))
            q = dict(p); q["V"], q["F"] = V[used], remap[Fp]
            if not m.all():
                q.pop("tex_geom", None)
                if p.get("fcol") is not None:
                    q["fcol"] = p["fcol"][m]
            if is_lamp:
                c = np.ones(3) if front else mat_col
                if front and _is_amber(mat_col):
                    c = np.array([1.0, 0.55, 0.08])
                if not front and not _is_red(mat_col):
                    c = np.array([0.8, 0.05, 0.04])
                q["cls"] = CLS_LIGHT_CLEAR if front else CLS_LIGHT_RED
                q["mat"] = dict(p["mat"], color=srgb_to_linear(c), name=p["mat"]["name"] + ("_head" if front else "_tail"))
                if front:
                    nf_ += len(Fp)
                else:
                    nr_ += len(Fp)
            out.append(q)
    if nf_ or nr_:
        log(f"  lamps found by position: {nf_} head light, {nr_} tail light triangles")
    return out


def _is_amber(srgb):
    r, g, b = srgb[:3]
    return r > 0.15 and g > 0.3 * r and g < 0.85 * r and b < 0.45 * g


def light_region_class(srgb, cx):
    """Class + colour of one colour region of a car light texture. Unlit lamp textures are
    dark/smoky, so the position decides: rear lamps are red (unless very bright = reverse
    light), front lamps are clear, orange regions are amber indicators."""
    lum = float(np.dot(srgb, [0.2126, 0.7152, 0.0722]))
    sat = float(np.ptp(srgb))
    rear, front = cx < -0.25, cx > 0.25
    if _is_amber(srgb) and not rear:
        return CLS_LIGHT_CLEAR, np.array([1.0, 0.55, 0.08])        # amber indicator
    if rear:
        if lum < 0.12 and sat < 0.08:
            return CLS_PLASTIC, srgb                                 # black housing
        if lum > 0.5 and sat < 0.12:
            return CLS_LIGHT_CLEAR, np.array([1.0, 1.0, 1.0])       # reverse light
        return CLS_LIGHT_RED, srgb
    if _is_reddish(srgb):
        return CLS_LIGHT_RED, srgb
    if lum < 0.06:
        return CLS_PLASTIC, srgb
    return CLS_LIGHT_CLEAR, np.array([1.0, 1.0, 1.0])


def _is_red(srgb):
    r, g, b = srgb[:3]
    return r > 0.15 and r > 1.8 * g and (b > 0.4 * g or g < 0.04)        # red, not amber


def atlas_region_class(srgb, cx):
    """Class + colour of a colour region of a car texture atlas (trim, lights and windows
    painted into one texture). Returns None for 'dark' = glass or plastic, decided per
    triangle by height (see apply_texture_split)."""
    lum = float(np.dot(srgb, [0.2126, 0.7152, 0.0722]))
    sat = float(np.ptp(srgb))
    rear, front = cx < -0.55, cx > 0.55
    if _is_red(srgb):
        return (CLS_LIGHT_RED if rear else CLS_GENERIC), srgb
    if _is_amber(srgb):
        return ((CLS_LIGHT_CLEAR, np.array([1.0, 0.55, 0.08])) if (front or rear) else (CLS_GENERIC, srgb))
    if sat < 0.12:
        if lum > 0.6:
            return (CLS_LIGHT_CLEAR, np.array([1.0, 1.0, 1.0])) if front else (CLS_CHROME, srgb)
        if lum > 0.3:
            return CLS_METAL, srgb
        return None, srgb
    return CLS_GENERIC, srgb


def _is_atlas(col, n_faces, min_faces=1500):
    """A textured car material whose texture holds very different surfaces (trim + lights +
    windows) - its average colour would be meaningless."""
    if n_faces < min_faces:
        return False
    if col is None:
        return False
    lum = col @ np.array([0.2126, 0.7152, 0.0722])
    return float(lum.std()) > 0.12 or float((np.ptp(col, axis=1) > 0.25).mean()) > 0.02


def part_face_colors(p):
    """per-face sRGB texture colours of a part (cached), None without texture"""
    if "fcol" not in p:
        col = None
        if p.get("tex_geom") is not None:
            try:
                col = face_texture_colors(p["tex_geom"])
            except Exception:
                col = None
        p["fcol"] = col if col is not None and len(col) == len(p["F"]) else None
        p["tex_detail"] = p["fcol"] is not None
    return p["fcol"]


def merge_by_material(parts):
    """objects: textured parts of one material are clustered together (consistent regions)"""
    groups, order = {}, []
    for p in parts:
        if part_face_colors(p) is None:
            order.append(p)
            continue
        k = (p["mat"]["name"], p["cls"])
        if k not in groups:
            groups[k] = []
            order.append(k)
        groups[k].append(p)
    out = []
    for e in order:
        if isinstance(e, dict):
            out.append(e)
            continue
        pl = groups[e]
        if len(pl) == 1:
            out.append(pl[0])
            continue
        offs = np.cumsum([0] + [len(q["V"]) for q in pl])[:-1]
        q = dict(pl[0])
        q["V"] = np.concatenate([x["V"] for x in pl])
        q["F"] = np.concatenate([x["F"] + o for x, o in zip(pl, offs)])
        q["fcol"] = np.concatenate([x["fcol"] for x in pl])
        q["tex_detail"] = True
        q.pop("tex_geom", None)
        out.append(q)
    return out


_SPLIT_ID = 0


def apply_texture_split(parts, is_car, max_colors):
    """Textured parts -> one part per colour region (see split_by_texture).
    object mode: every textured part (figures, toys: all colour comes from the texture)
    car mode   : light materials (one texture often holds head AND tail lights) and big
                 texture atlases (trim + lights + windows in one texture). Each region is
                 classified by colour and position on the car. Other car materials keep their
                 averaged colour: they are classified by name."""
    for p in parts:
        part_face_colors(p)
    if max_colors <= 1:
        return parts
    if not is_car:
        parts = merge_by_material(parts)
    # the biggest textured "body" part is the paint even without "paint" in its name
    biggest_paint, bpa = None, 0.0
    for p in parts:
        if p["cls"] != CLS_PAINT or p.get("fcol") is None:
            continue
        Vq, Fq = p["V"], p["F"]
        a = float(np.linalg.norm(np.cross(Vq[Fq[:, 1]] - Vq[Fq[:, 0]], Vq[Fq[:, 2]] - Vq[Fq[:, 0]]), axis=1).sum())
        if a > bpa:
            bpa, biggest_paint = a, p
    car_h = max(float(p["V"][:, 1].max()) for p in parts if len(p["V"])) if is_car else 1.0
    out = []
    for p in parts:
        fcol = p.get("fcol")
        mode = None
        if fcol is not None and is_car and BODYCOL_RX.search((p["mat"]["name"] + " " + p["name"]).lower()):
            pass                              # body-colour tint mask: painted later, never split
        elif fcol is not None:
            if not is_car:
                mode = "object"
            elif p["cls"] in (CLS_LIGHT_RED, CLS_LIGHT_CLEAR):
                mode = "light"
            elif p["cls"] == CLS_PAINT and (p.get("paint_by_name") or
                                             (biggest_paint is not None and p["mat"]["name"] == biggest_paint["mat"]["name"])):
                mode = "paint" if _is_atlas(fcol, len(p["F"]), 300) else None      # livery
            elif ((p["cls"] == CLS_METAL and WHEEL_RX.search((p["mat"]["name"] + " " + p["name"]).lower()))
                  or (p.get("wheel_geom") and p["cls"] in (CLS_METAL, CLS_GENERIC, CLS_PLASTIC))):
                mode = "wheel" if _is_atlas(fcol, len(p["F"]), 300) else None   # rim + tyre texture
            elif p["cls"] in (CLS_GENERIC, CLS_PLASTIC, CLS_PAINT, CLS_INTERIOR):
                mode = "atlas" if _is_atlas(fcol, len(p["F"])) else None
        lbl = colors = None
        if mode:
            try:
                k = max_colors if mode == "object" else (min(max_colors, 6) if mode in ("light", "paint") else
                                                         (min(max_colors, 5) if mode == "wheel" else min(max_colors, 10)))
                lw = 0.8 if mode == "object" else (0.4 if mode == "paint" else 1.0)
                lbl, colors = split_by_texture(p["V"], p["F"], fcol, k, lw)
            except Exception as e:                     # never fail the conversion because of it
                log(f"  texture split skipped for '{p['name']}': {e}")
        if lbl is None or colors is None or len(colors) < 2 or len(lbl) != len(p["F"]):
            out.append(p)
            continue
        cen = p["V"][p["F"]].mean(1)
        summary = []
        global _SPLIT_ID
        _SPLIT_ID += 1
        split_id = _SPLIT_ID
        main_region = 0
        if mode == "paint":                  # livery: the largest region is the body colour
            Vq, Fq = p["V"], p["F"]
            ar = np.linalg.norm(np.cross(Vq[Fq[:, 1]] - Vq[Fq[:, 0]], Vq[Fq[:, 2]] - Vq[Fq[:, 0]]), axis=1)
            main_region = int(np.argmax(np.bincount(lbl, weights=ar, minlength=len(colors))))
        for r, c in enumerate(colors):
            fm = lbl == r
            if not fm.any():
                continue
            c = np.asarray(c)
            cx = float(cen[fm, 0].mean())                 # car length = x in [-1, 1], front +x
            if mode == "object":
                lum = float(np.dot(srgb_to_linear(c), [0.2126, 0.7152, 0.0722]))
                pieces = [(fm, CLS_PLASTIC if lum < 0.02 else CLS_GENERIC, c)]
            elif mode == "light":
                cls, col = light_region_class(c, cx)
                if is_car and float(np.dot(c, [0.2126, 0.7152, 0.0722])) < 0.3 and float(np.ptp(c)) < 0.15:
                    # a window in the lamp material ("lights and glass" texture): high faces = glass
                    high = cen[:, 1] > 0.6 * car_h
                    pieces = [(fm & high, CLS_GLASS, c), (fm & ~high, cls, col)]
                else:
                    pieces = [(fm, cls, col)]
            elif mode == "paint":
                pieces = [(fm, CLS_PAINT, c)]
            elif mode == "wheel":
                lum = float(np.dot(c, [0.2126, 0.7152, 0.0722])); sat = float(np.ptp(c))
                pieces = [(fm, CLS_TIRE if lum < 0.22 and sat < 0.12 else
                               (CLS_CHROME if lum > 0.55 and sat < 0.15 else CLS_METAL), c)]
            else:
                cls, col = atlas_region_class(c, cx)
                if cls is None:                           # dark: windows above the beltline
                    high = cen[:, 1] > 0.58 * car_h
                    pieces = [(fm & high, CLS_GLASS, col), (fm & ~high, CLS_PLASTIC, col)]
                else:
                    pieces = [(fm, cls, col)]
            for j, (m, cls, col) in enumerate(pieces):
                if not m.any():
                    continue
                Fp = p["F"][m]
                used = np.unique(Fp)
                remap = -np.ones(len(p["V"]), np.int64); remap[used] = np.arange(len(used))
                mi = dict(p["mat"])
                mi["color"] = srgb_to_linear(np.asarray(col))
                mi["name"] = f"{p['mat']['name']}_c{r}{'abcd'[j] if len(pieces) > 1 else ''}"
                out.append(dict(name=f"{p['name']}_c{r}_{j}", mat=mi, cls=cls, V=p["V"][used],
                                F=remap[Fp], from_split=True, fcol=fcol[m], tex_detail=True, split_group=split_id))
                if mode == "paint":
                    out[-1]["paint_src"] = p["mat"]["name"]
                    out[-1]["paint_key"] = "main" if r == main_region else f"l{r}"
                summary.append(f"{CLS_NAMES[cls]}:{int(m.sum())}")
        log(f"  '{p['name'][:40]}' ({mode}): {len(colors)} colour regions -> {' '.join(summary)}")
    # livery over several parts of one material: the body colour is the colour with the
    # largest area over all of them
    by_src = {}
    for q in out:
        if q.get("paint_src"):
            by_src.setdefault(q["paint_src"], []).append(q)
    ck = lambda q: tuple(int(round(v * 6)) for v in linear_to_srgb(q["mat"]["color"]))
    for lst in by_src.values():
        area = {}
        for q in lst:
            Vq, Fq = q["V"], q["F"]
            a = float(np.linalg.norm(np.cross(Vq[Fq[:, 1]] - Vq[Fq[:, 0]], Vq[Fq[:, 2]] - Vq[Fq[:, 0]]), axis=1).sum())
            area[ck(q)] = area.get(ck(q), 0.0) + a
        best = max(area, key=area.get)
        for q in lst:
            q["paint_key"] = "main" if ck(q) == best else "l" + ",".join(map(str, ck(q)))
    return out


# --------------------------------------------------------------------------------------
# Classification
# --------------------------------------------------------------------------------------
RULES = [
    # wheel hardware first: "Tire_Hub", "Tire_Disk", "Tire_Brake" are metal, not rubber
    (CLS_METAL, r"hub|disk|disc|caliper|felge|(?<![a-z])rims?(?![a-z])|brake(?!.*(light|lamp|(?<![a-z])leds?(?![a-z])|lens))"),
    (CLS_INTERIOR, r"interior|leather|seat|carpet|dashboard|dash|steering|cockpit|fabric|alcantara|inner|innen"),
    (CLS_LIGHT_RED, r"(tail|brake|stop).*(light|lamp|(?<![a-z])leds?(?![a-z])|lens)|(rear|back).*(light|lamp|(?<![a-z])leds?(?![a-z]))|(light|lamp|(?<![a-z])leds?(?![a-z])|lens)[ _-]?red|red[ _-]?(light|lamp|(?<![a-z])leds?(?![a-z])|lens)|rueckleucht|tylne|taillight"),
    (CLS_LIGHT_CLEAR, r"head.?light|headlamp|projector|front.?light|lamp|lens|light|(?<![a-z])leds?(?![a-z])|indicator|turn.?signal|reflector|scheinwerfer|reflektor"),
    (CLS_GLASS, r"glass|window|windshield|windscreen|scheibe|szyb|glas|cristal|crystal|vidrio|verre|vitre|vetro|szkl"),
    (CLS_CHROME, r"chrome|chrom|mirror_refl|polished"),
    (CLS_TIRE, r"tire|tyre|rubber|reifen|opon|guma|llanta"),
    (CLS_CARBON, r"carbon"),
    (CLS_METAL, r"(?<![a-z])rims?(?![a-z])|wheel|alloy|felge|disc|disk|brake|caliper|metal|steel|(?<![a-z])alu|exhaust|felg"),
    (CLS_PAINT, r"paint|body|carpaint|car_paint|(?<![a-z])lack|karoser|exterior|coat|primary"),
    (CLS_PLASTIC, r"plastic|trim|black|grill|grille|bumper_low|matte|kunststoff"),
]


def classify(part):
    name = (part["mat"]["name"] + " " + part["name"]).lower()
    mi = part["mat"]
    # the part's own name (without the material name it often carries) says window: glass,
    # even when the material is a shared "lights and glass" one
    mn = part["mat"]["name"].lower()
    own = " ".join(part["name"].lower().split(mn)) if len(mn) >= 3 else part["name"].lower()
    if re.search(r"window|windshield|windscreen|glass|scheibe|szyb", own) and not re.search(r"light|lamp|(?<![a-z])leds?(?![a-z])|lens|mirror", own):
        return CLS_GLASS
    for cls, rx in RULES:
        if re.search(rx, name):
            # coloured "lens" materials: decide by hue
            if cls in (CLS_LIGHT_CLEAR, CLS_GLASS) and _is_reddish(linear_to_srgb(mi["color"])):
                return CLS_LIGHT_RED          # red lens / red "glass" = tail light
            return cls
    if mi["alpha"] < 0.7:
        return CLS_GLASS
    if mi["metallic"] > 0.8 and mi["roughness"] < 0.25:
        return CLS_CHROME
    if mi["metallic"] > 0.6:
        return CLS_METAL
    lum = float(np.dot(mi["color"], [0.2126, 0.7152, 0.0722]))
    if lum < 0.02:
        return CLS_PLASTIC
    return CLS_GENERIC


# --------------------------------------------------------------------------------------
# Software depth-splat visibility (pure numpy, no GPU)
# --------------------------------------------------------------------------------------
def fib_sphere(n, min_elev=-1.0):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = math.pi * (1 + 5 ** 0.5) * i
    d = np.stack([np.cos(th) * np.sin(phi), np.cos(phi), np.sin(th) * np.sin(phi)], 1)
    return d[d[:, 1] >= min_elev]


def basis_from_dir(d):
    d = d / np.linalg.norm(d)
    up = np.array([0.0, 1.0, 0.0]) if abs(d[1]) < 0.95 else np.array([1.0, 0.0, 0.0])
    x = np.cross(up, d); x /= np.linalg.norm(x)
    y = np.cross(d, x)
    return x, y, d


def sample_centroids(V, F):
    return (V[F[:, 0]] + V[F[:, 1]] + V[F[:, 2]]) / 3.0


class Raster:
    """Exact orthographic ID/depth-buffer rasterizer in pure numpy (bucketed by bbox size)."""

    def __init__(self, res, radius):
        self.res = res
        self.radius = radius
        self.pix = 2 * radius / res

    def to_screen(self, P, d):
        x, y, z = basis_from_dir(d)
        sx = (P @ x) / self.pix + self.res / 2
        sy = (P @ y) / self.pix + self.res / 2
        depth = -(P @ z)                    # smaller = closer to a viewer looking along -d
        return sx, sy, depth

    def render(self, V, F, d):
        """Returns (zbuf[res*res], idbuf[res*res]) for direction d."""
        res = self.res
        sx, sy, sz = self.to_screen(V, d)
        x0, x1, x2 = sx[F[:, 0]], sx[F[:, 1]], sx[F[:, 2]]
        y0, y1, y2 = sy[F[:, 0]], sy[F[:, 1]], sy[F[:, 2]]
        z0, z1, z2 = sz[F[:, 0]], sz[F[:, 1]], sz[F[:, 2]]
        area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        bx0 = np.clip(np.floor(np.minimum(np.minimum(x0, x1), x2) - 0.5), 0, res - 1).astype(np.int64)
        by0 = np.clip(np.floor(np.minimum(np.minimum(y0, y1), y2) - 0.5), 0, res - 1).astype(np.int64)
        bx1 = np.clip(np.ceil(np.maximum(np.maximum(x0, x1), x2) - 0.5), 0, res - 1).astype(np.int64)
        by1 = np.clip(np.ceil(np.maximum(np.maximum(y0, y1), y2) - 0.5), 0, res - 1).astype(np.int64)
        bw = bx1 - bx0 + 1
        bhh = by1 - by0 + 1
        good = np.abs(area) > 1e-12
        # rectangular power-of-two buckets: slivers stay cheap
        kw = np.ceil(np.log2(np.maximum(bw, 1))).astype(np.int64)
        kh = np.ceil(np.log2(np.maximum(bhh, 1))).astype(np.int64)
        key = kw * 64 + kh
        frag_pix, frag_z, frag_id = [], [], []
        for kk in np.unique(key[good]):
            Sw, Sh = 1 << int(kk // 64), 1 << int(kk % 64)
            sel = np.nonzero(good & (key == kk))[0]
            oy, ox = np.divmod(np.arange(Sw * Sh), Sw)
            for chunk in np.array_split(sel, max(1, len(sel) * Sw * Sh // 3_000_000 + 1)):
                px = bx0[chunk, None] + ox[None, :]
                py = by0[chunk, None] + oy[None, :]
                cx = px + 0.5; cy = py + 0.5
                a = area[chunk, None]
                X0, X1, X2 = x0[chunk, None], x1[chunk, None], x2[chunk, None]
                Y0, Y1, Y2 = y0[chunk, None], y1[chunk, None], y2[chunk, None]
                w0 = ((X1 - cx) * (Y2 - cy) - (X2 - cx) * (Y1 - cy)) / a
                w1 = ((X2 - cx) * (Y0 - cy) - (X0 - cx) * (Y2 - cy)) / a
                w2 = 1.0 - w0 - w1
                inside = (w0 >= -1e-9) & (w1 >= -1e-9) & (w2 >= -1e-9) & (px < res) & (py < res)
                if not inside.any():
                    continue
                z = w0 * z0[chunk, None] + w1 * z1[chunk, None] + w2 * z2[chunk, None]
                frag_pix.append((py * res + px)[inside])
                frag_z.append(z[inside])
                frag_id.append(np.broadcast_to(chunk[:, None], inside.shape)[inside])
        zb = np.full(res * res, np.inf)
        ib = np.full(res * res, -1, np.int64)
        if frag_pix:
            pix = np.concatenate(frag_pix); z = np.concatenate(frag_z); ids = np.concatenate(frag_id)
            np.minimum.at(zb, pix, z)
            win = z <= zb[pix]
            ib[pix[win]] = ids[win]
        return zb, ib

    def depth_at(self, zb, P, d):
        sx, sy, sz = self.to_screen(P, d)
        px = np.clip(sx.astype(np.int64), 0, self.res - 1)
        py = np.clip(sy.astype(np.int64), 0, self.res - 1)
        return zb[py * self.res + px], sz


def visible_triangles(V, F, radius, n_views=64, res=640, min_elev=-0.35):
    """Exact visibility from many directions. Returns (visible[nf], front_views[nf],
    back_views[nf]) - the side counts let us repair inconsistent triangle winding."""
    ras = Raster(res, radius)
    log(f"  visibility: {len(F)} tris, {n_views} views @ {res}px")
    vis = np.zeros(len(F), dtype=bool)
    front = np.zeros(len(F), np.int32)
    back = np.zeros(len(F), np.int32)
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    C = sample_centroids(V, F)
    for d in fib_sphere(n_views, min_elev):
        zb, ib = ras.render(V, F, d)
        seen = np.zeros(len(F), bool)
        seen[ib[ib >= 0]] = True
        # sub-pixel triangles that miss every pixel centre: centroid depth test
        zc, dc = ras.depth_at(zb, C, d)
        seen |= dc <= zc + ras.pix * 0.5
        vis |= seen
        facing = fn @ d
        front += seen & (facing > 0)
        back += seen & (facing < 0)
    return vis, front, back


# --------------------------------------------------------------------------------------
# Mesh processing
# --------------------------------------------------------------------------------------
def weld(V, F, tol, return_inv=False):
    q = np.round(V / tol).astype(np.int64)
    keys, first, inv = np.unique(q, axis=0, return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    V2 = V[first]
    F2 = inv[F]
    good = (F2[:, 0] != F2[:, 1]) & (F2[:, 1] != F2[:, 2]) & (F2[:, 0] != F2[:, 2])
    if return_inv:
        return V2, F2[good], inv
    return V2, F2[good]


def vertex_colors(V, F, fcol):
    """linear per-vertex colour from per-face sRGB colours (area weighted)"""
    area = np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1) + 1e-12
    lin = srgb_to_linear(fcol) * area[:, None]
    acc = np.zeros((len(V), 3)); w = np.zeros(len(V))
    for k in range(3):
        np.add.at(acc, F[:, k], lin)
        np.add.at(w, F[:, k], area)
    return acc / np.maximum(w, 1e-12)[:, None]


def meshopt_simplify(V, F, target, lock_border=False):
    """Edge-collapse simplification (meshoptimizer). Returns faces indexing the ORIGINAL V."""
    idx = np.ascontiguousarray(F.reshape(-1).astype(np.uint32))
    pos = np.ascontiguousarray(V.astype(np.float32))
    dst = np.zeros_like(idx)
    opts = meshopt.SIMPLIFY_LOCK_BORDER if lock_border else 0
    n = meshopt.simplify(dst, idx, pos, target_index_count=int(target) * 3, target_error=1.0, options=opts)
    return dst[:n].reshape(-1, 3).astype(np.int64)


def split_groups(parts):
    """part indices grouped by the surface they were split from (colour regions of one texture)"""
    by, out = {}, []
    for i, p in enumerate(parts):
        g = p.get("split_group")
        if g is None:
            out.append([i])
            continue
        if g not in by:
            by[g] = []
            out.append(by[g])
        by[g].append(i)
    return out


def merge_parts(plist):
    """several parts -> one welded mesh (shared border vertices), owner per face, face centroids"""
    offs = np.cumsum([0] + [len(p["V"]) for p in plist])[:-1]
    V = np.concatenate([p["V"] for p in plist])
    F = np.concatenate([p["F"] + o for p, o in zip(plist, offs)]).astype(np.int64)
    own = np.concatenate([np.full(len(p["F"]), k) for k, p in enumerate(plist)])
    V2, _, inv = weld(V, F, 1e-5, return_inv=True)
    F2 = inv[F]
    good = (F2[:, 0] != F2[:, 1]) & (F2[:, 1] != F2[:, 2]) & (F2[:, 0] != F2[:, 2])
    F2, own = F2[good], own[good]
    return V2, F2, own, V2[F2].mean(1)


def face_owners(V, own, cen, D):
    """owner of each simplified triangle = owner of the nearest original face (by centroid)"""
    if len(D) == 0:
        return np.zeros(0, np.int64)
    _, q = cKDTree(cen).query(V[D].mean(1))
    return own[q]


def dedupe_faces(F):
    """Remove triangles that use the same 3 vertices (double-sided duplicates, copies)."""
    key = np.sort(F, axis=1)
    _, first = np.unique(key, axis=0, return_index=True)
    return F[np.sort(first)]


def decimate(V, F, target):
    if len(F) <= target:
        return V, F
    if meshopt is not None:
        F2 = meshopt_simplify(V, F, target)
        used = np.unique(F2)
        remap = -np.ones(len(V), np.int64); remap[used] = np.arange(len(used))
        return V[used], remap[F2]
    if fast_simplification is None:
        return V, F
    red = 1.0 - target / len(F)
    try:
        V2, F2 = fast_simplification.simplify(V.astype(np.float32), F.astype(np.int32),
                                              target_reduction=float(red), agg=3, preserve_border=True)
        if len(F2) >= 4:
            return V2.astype(np.float64), F2.astype(np.int64)
    except Exception as e:
        log("   decimation failed:", e)
    return V, F


def crease_normals(V, F, angle_deg):
    """Split vertices across edges sharper than angle; returns (V', N', F')."""
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    area = np.linalg.norm(fn, axis=1)
    fnu = fn / np.maximum(area, 1e-20)[:, None]
    cos_t = math.cos(math.radians(angle_deg))
    nv = len(V)
    # incident faces per vertex
    corner_v = F.reshape(-1)
    corner_f = np.repeat(np.arange(len(F)), 3)
    order = np.argsort(corner_v, kind="stable")
    cv, cf = corner_v[order], corner_f[order]
    starts = np.searchsorted(cv, np.arange(nv + 1))
    corner_normal = np.zeros((len(F) * 3, 3))
    for v in range(nv):
        s, e = starts[v], starts[v + 1]
        if s == e:
            continue
        faces = cf[s:e]
        n = fnu[faces]
        dots = n @ n.T
        w = (dots >= cos_t) * area[faces][None, :]
        acc = w @ fn[faces] / np.maximum(area[faces], 1e-20)[:, None] * 1.0
        acc = w @ fnu[faces]
        acc /= np.maximum(np.linalg.norm(acc, axis=1), 1e-20)[:, None]
        # write back for each corner of this vertex
        for k, f in enumerate(faces):
            slot = np.nonzero(F[f] == v)[0][0]
            corner_normal[f * 3 + slot] = acc[k]
    # dedupe (vertex, quantised normal)
    qn = np.round(corner_normal * 127).astype(np.int64)
    key = np.concatenate([corner_v[:, None], qn], 1)
    uniq, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    V2 = V[corner_v[first]]
    N2 = corner_normal[first]
    F2 = inv.reshape(-1, 3)
    return V2, N2, F2


def transfer_normals(Vn, Nn, tree, hiN, max_angle_deg=28.0):
    """Replace decimated normals by the original smooth normals where they agree."""
    d, idx = tree.query(Vn, k=1)
    cand = hiN[idx]
    ok = (d < 1e-5) & (np.sum(cand * Nn, axis=1) > math.cos(math.radians(max_angle_deg)))
    out = Nn.copy()
    out[ok] = cand[ok]
    return out, int(ok.sum())


def bake_ao(V, N, F, radius, n_dirs=128, res=512, ground=True):
    ras = Raster(res, radius)
    Pv = V + N * ras.pix * 1.5
    acc = np.zeros(len(V))
    wsum = np.zeros(len(V))
    for d in fib_sphere(n_dirs):
        w = np.maximum(N @ d, 0.0)
        if ground and d[1] < 0.02:
            wsum += w           # the floor blocks light from below the horizon
            continue
        zb, _ = ras.render(V, F, d)
        zv, dv = ras.depth_at(zb, Pv, d)
        lit = dv <= zv + ras.pix * 1.0
        acc += w * lit
        wsum += w
    ao = acc / np.maximum(wsum, 1e-9)
    ao = np.clip(ao * 1.35, 0, 1) ** 0.7          # gentle curve, avoids crushed blacks
    return 0.22 + 0.78 * ao


# --------------------------------------------------------------------------------------
# Matcap rendering (studio environment, view space: +Z towards viewer, +Y up)
# --------------------------------------------------------------------------------------
def env_radiance(d):
    """Procedural photo studio (view space): dark room, long overhead softbox, two side
    strip lights, a key light and a very dark floor.  Moves with the camera, which is
    exactly what makes the highlights 'flow' over the body while it rotates."""
    x, y, z = d[..., 0], d[..., 1], d[..., 2]
    L = np.zeros(d.shape[:-1] + (3,))
    L += (0.006 + 0.02 * np.clip(y, 0, 1))[..., None]
    # dark floor + soft horizon line
    L = np.where((y < 0)[..., None], 0.004, L)
    L += (np.exp(-((y - 0.03) / 0.05) ** 2) * 0.12)[..., None] * np.array([0.85, 0.9, 1.0])
    # long overhead softbox (stretched along x)
    sb = np.exp(-(((y - 0.92) / 0.10) ** 6)) * np.exp(-((x / 0.75) ** 6)) * (z > -0.5)
    L += (sb * 3.2)[..., None]
    # two strip lights left/right, slightly above the horizon -> white lines on the flanks
    for sx in (-1.0, 1.0):
        strip = np.exp(-(((y - 0.30) / 0.045) ** 4)) * np.exp(-(((x * sx - 0.80) / 0.25) ** 4)) * (z > -0.3)
        L += (strip * 3.0)[..., None]
    # key light front-top-right (hot spot)
    key = np.exp(-np.sum((d - np.array([0.42, 0.52, 0.74])) ** 2, -1) / 0.010)
    L += (key * 8.0)[..., None]
    # cool rim light from behind-left
    rim = np.exp(-np.sum((d - np.array([-0.35, 0.25, -0.90])) ** 2, -1) / 0.06)
    L += (rim * 1.2)[..., None] * np.array([0.8, 0.9, 1.0])
    return L


def _fib_dirs(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)


def env_blurred(dirs, rough, nsamp=192):
    """Prefiltered environment: same deterministic cone kernel for every texel, so the
    result is smooth (no Monte-Carlo noise)."""
    if rough <= 0.01:
        return env_radiance(dirs)
    sigma = rough * 0.9
    i = np.arange(nsamp) + 0.5
    r = 2.2 * sigma * np.sqrt(i / nsamp)            # Vogel spiral in the tangent plane
    th = i * math.pi * (3 - 5 ** 0.5)
    k = np.stack([r * np.cos(th), r * np.sin(th), np.ones_like(r)], 1)
    w = np.exp(-(r / sigma) ** 2)
    # rotate kernel (around +Z) onto each direction
    up = np.where((np.abs(dirs[..., 1]) < 0.99)[..., None], np.array([0, 1.0, 0]), np.array([1.0, 0, 0]))
    tx = np.cross(up, dirs); tx /= np.linalg.norm(tx, axis=-1, keepdims=True)
    ty = np.cross(dirs, tx)
    acc = np.zeros(dirs.shape)
    for kk, ww in zip(k, w):
        dd = tx * kk[0] + ty * kk[1] + dirs * kk[2]
        dd /= np.linalg.norm(dd, axis=-1, keepdims=True)
        acc += env_radiance(dd) * ww
    return acc / w.sum()


def env_irradiance(normals, nsamp=600):
    dirs = _fib_dirs(nsamp)
    L = env_radiance(dirs)                         # (n,3)
    cosw = np.clip(normals @ dirs.T, 0, None)     # (..., n)
    return (cosw @ L) / nsamp * 4.0               # cosine-weighted average * pi-ish


def tonemap(c):
    # filmic (ACES approx) then sRGB encoding
    a, b, cc, d, e = 2.51, 0.03, 2.43, 0.59, 0.14
    c = np.clip((c * (a * c + b)) / (c * (cc * c + d) + e), 0, 1)
    return linear_to_srgb(c)


def render_matcap(cls, color, size, rough_hint=0.5, unlit=False, obj=False):
    out_size = size
    size = size * 2                                 # 2x supersampling, box-filtered below
    t = (np.arange(size) + 0.5) / size * 2 - 1
    X, Y = np.meshgrid(t, -t)                     # row 0 = top (+Y)
    r2 = X * X + Y * Y
    inside = r2 < 1.0
    Z = np.sqrt(np.clip(1 - r2, 0, 1))
    # clamp outside texels to the rim so bilinear/nearest at edge stays sane
    s = np.sqrt(np.maximum(r2, 1e-12))
    Xc = np.where(inside, X, X / s * 0.999)
    Yc = np.where(inside, Y, Y / s * 0.999)
    Zc = np.sqrt(np.clip(1 - Xc * Xc - Yc * Yc, 0, 1))
    n = np.stack([Xc, Yc, Zc], -1)
    v = np.array([0, 0, 1.0])
    ndv = np.clip(n[..., 2], 0, 1)
    refl = 2 * ndv[..., None] * n - v
    refl /= np.linalg.norm(refl, axis=-1, keepdims=True)
    fres = lambda f0: f0 + (1 - f0) * (1 - ndv) ** 5
    c = np.asarray(color, dtype=np.float64)
    irr = env_irradiance(n)

    if unlit:                           # light baked into the texture: only a hint of form
        sh = (0.86 + 0.14 * ndv)[..., None]
        srgb = linear_to_srgb(c * sh).reshape(out_size, 2, out_size, 2, 3)[:, 0, :, 0, :]
        return _quantize565(srgb, out_size), srgb
    if obj and cls in (CLS_GENERIC, CLS_PLASTIC):
        # figures and objects: brighter, softer light than car trim so the colours read true
        rough = float(np.clip(rough_hint, 0.25, 0.8))
        base = np.clip(c, 0.0, 0.35) if cls == CLS_PLASTIC else c
        out = base * (irr * 1.25 + 0.10) + fres(0.04)[..., None] * env_blurred(refl, rough) * (1.0 - rough * 0.7) * 0.7
    elif cls == CLS_PAINT:
        diffuse = c * irr * 0.9
        flake = c * env_blurred(refl, 0.30) * 0.45            # metallic base layer
        coat = fres(0.04)[..., None] * env_blurred(refl, 0.035) * 0.9
        out = diffuse + flake + coat
    elif cls == CLS_GLASS:
        tint = np.array([0.010, 0.012, 0.016])
        out = tint + fres(0.04)[..., None] * env_blurred(refl, 0.02) * 1.0
    elif cls == CLS_CHROME:
        out = env_blurred(refl, 0.03) * 0.95 + 0.03
    elif cls == CLS_METAL:
        base = np.clip(c, 0.35, 0.9)
        out = base * env_blurred(refl, 0.22) * 0.85 + base * irr * 0.08
    elif cls == CLS_TIRE:
        out = np.array([0.018, 0.018, 0.02]) * irr * 3 + fres(0.04)[..., None] * env_blurred(refl, 0.6) * 0.25
    elif cls == CLS_PLASTIC:
        out = np.clip(c, 0.01, 0.2) * irr * 0.6 + fres(0.04)[..., None] * env_blurred(refl, 0.35) * 0.5
    elif cls == CLS_LIGHT_RED:
        out = np.array([0.45, 0.01, 0.01]) * (0.35 + irr * 0.3) + fres(0.06)[..., None] * env_blurred(refl, 0.03) * 1.3
    elif cls == CLS_LIGHT_CLEAR:
        tint = c / max(float(c.max()), 1e-6) if float(np.ptp(c)) > 0.2 * float(c.max()) else np.ones(3)
        out = env_blurred(refl, 0.06) * 0.85 * tint + np.array([0.08, 0.085, 0.09]) * tint
    elif cls == CLS_INTERIOR:
        out = np.clip(c, 0.01, 0.3) * irr * 0.45
    elif cls == CLS_CARBON:
        out = np.array([0.02, 0.02, 0.022]) * irr * 2 + fres(0.05)[..., None] * env_blurred(refl, 0.05) * 0.9
    else:
        rough = float(np.clip(rough_hint, 0.05, 0.8))
        out = c * irr * 0.6 + fres(0.04)[..., None] * env_blurred(refl, rough) * (1.0 - rough * 0.7)

    out = out.reshape(out_size, 2, out_size, 2, 3).mean((1, 3))
    size = out_size
    srgb = tonemap(out)
    return _quantize565(srgb, size), srgb


def _quantize565(srgb, size):
    """quantise to RGB565 (no dither: the renderer samples nearest-neighbour, a dither pattern
    shows as patches on large smooth panels)"""
    bayer = np.array([[0, 8, 2, 10], [12, 4, 14, 6], [3, 11, 1, 9], [15, 7, 13, 5]]) / 16.0 - 0.5
    dith = np.tile(bayer, (size // 4 + 1, size // 4 + 1))[:size, :size] * 0.0
    r = np.clip(np.floor(srgb[..., 0] * 31 + 0.5 + dith * 0.9), 0, 31).astype(np.uint16)
    g = np.clip(np.floor(srgb[..., 1] * 63 + 0.5 + dith * 0.9), 0, 63).astype(np.uint16)
    b = np.clip(np.floor(srgb[..., 2] * 31 + 0.5 + dith * 0.9), 0, 31).astype(np.uint16)
    return (r << 11) | (g << 5) | b


# --------------------------------------------------------------------------------------
# Orientation
# --------------------------------------------------------------------------------------
def rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def lamp_front_sign(parts, R, x0=0.0):
    """+1 if the front of the car (white lamps) is at +x, -1 if at -x, None if unknown.
    Uses the real lamp colours (texture or material) of light/glass parts: strongly red
    surfaces are tail lights, bright neutral surfaces are headlights."""
    xs_red, w_red, xs_wht, w_wht = [], [], [], []
    total_area = 0.0
    for p in parts:
        if p["cls"] == CLS_DROP:
            continue
        Vt = p["mesh"].vertices @ R.T
        Ft = np.asarray(p["mesh"].faces)
        total_area += float(np.linalg.norm(np.cross(Vt[Ft[:, 1]] - Vt[Ft[:, 0]], Vt[Ft[:, 2]] - Vt[Ft[:, 0]]), axis=1).sum())
    for p in parts:
        if p["cls"] == CLS_DROP:
            continue
        atlas = p["cls"] in (CLS_GENERIC, CLS_PLASTIC, CLS_PAINT) and p.get("tex_geom") is not None
        if p["cls"] not in (CLS_LIGHT_RED, CLS_LIGHT_CLEAR, CLS_GLASS) and not atlas:
            continue
        V = p["mesh"].vertices @ R.T
        V[:, 0] -= x0                   # relative to the middle of the model
        F = np.asarray(p["mesh"].faces)
        cx = V[F].mean(1)[:, 0]
        area = np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)
        col = None
        if p.get("tex_geom") is not None:
            try:
                col = face_texture_colors(p["tex_geom"])
            except Exception:
                col = None
        if col is None or len(col) != len(F):
            col = np.tile(linear_to_srgb(p["mat"]["color"]), (len(F), 1))
        r, g, b = col[:, 0], col[:, 1], col[:, 2]
        red = (r > 0.08) & (r > 1.8 * g) & ((b > 0.4 * g) | (g < 0.04))       # red, not amber
        lum = col @ np.array([0.2126, 0.7152, 0.0722])
        wht = (lum > 0.5) & (np.ptp(col, axis=1) < 0.12)
        if p["cls"] == CLS_GLASS or atlas:
            wht[:] = False              # windows / trim atlas (chrome is white too): no vote
        if atlas:
            red &= np.abs(cx) > 0.6 * np.abs(V[:, 0]).max()                   # lamps sit at the ends
        xs_red.append(cx[red]); w_red.append(area[red])
        xs_wht.append(cx[wht]); w_wht.append(area[wht])
    if not xs_red:
        return None
    xr, wr = np.concatenate(xs_red), np.concatenate(w_red)
    xw, ww = np.concatenate(xs_wht), np.concatenate(w_wht)
    # a few stray red triangles are no evidence (relative to the whole model surface)
    if wr.sum() <= 3e-4 * total_area:
        return None
    if ww.sum() <= 3e-4 * total_area:
        ww = np.zeros(0); xw = np.zeros(0)
    mr = np.average(xr, weights=wr)
    if ww.sum() <= 0:                   # only tail lights known: front is the other end
        span = max(float(np.abs(xr).max()), 1e-9)
        return None if abs(mr) < 0.3 * span else (1 if mr < 0 else -1)
    mw = np.average(xw, weights=ww)
    if abs(mr - mw) < 1e-6:
        return None
    return 1 if mw > mr else -1


def normalise_orientation(parts, cfg):
    allV = np.concatenate([p["mesh"].vertices for p in parts])
    up = cfg.get("up_axis", "y").lower()
    R = np.eye(3)
    if up == "z":
        R = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)
    elif up == "-z":
        R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
    elif up == "x":
        R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=np.float64)
    V = allV @ R.T
    # ground / shadow planes: flat parts at the very bottom covering most of the footprint
    mn, mx = V.min(0), V.max(0)
    H, A = mx[1] - mn[1], (mx[0] - mn[0]) * (mx[2] - mn[2])
    ground = []
    for p in parts:
        if p["cls"] == CLS_DROP:
            continue
        Pv = p["mesh"].vertices @ R.T
        a, b = Pv.min(0), Pv.max(0)
        e = b - a
        flat = e.min() <= 0.02 * e.max()
        nm = (p["name"] + " " + p["mat"]["name"]).lower()
        if b[1] - a[1] <= 0.02 * H and b[1] <= mn[1] + 0.03 * H and (b[0] - a[0]) * (b[2] - a[2]) >= 0.25 * A:
            ground.append(p)
        # shadow planes (any orientation) and big flat card / backdrop quads
        elif flat and re.search(r"shadow|sombra|schatten", nm):
            ground.append(p)
        elif flat and len(p["mesh"].faces) <= 12 and e.max() > 0.5 * (mx - mn).max():
            ground.append(p)
    live = [p for p in parts if p["cls"] != CLS_DROP and all(p is not g for g in ground)]
    if ground and live:
        V2 = np.concatenate([p["mesh"].vertices for p in live]) @ R.T
        e2 = V2.max(0) - V2.min(0)
        if e2[1] >= 0.05 * max(e2[0], e2[2]):
            for g in ground:
                g["cls"] = CLS_DROP
            log("  removed ground / shadow planes: " + ", ".join(g["name"] for g in ground))
    live = [p for p in parts if p["cls"] != CLS_DROP]
    allV = np.concatenate([p["mesh"].vertices for p in live])
    V = allV @ R.T
    ext = V.max(0) - V.min(0)
    Lh, Wh = max(ext[0], ext[2]), min(ext[0], ext[2])
    car_shape = Wh > 0 and ext[1] > 0 and 1.5 <= Lh / Wh <= 3.4 and 1.7 <= Lh / ext[1] <= 4.5
    # wheels: round-ish parts standing on the ground, two at each end
    wheels, ends = 0, set()
    if car_shape:
        mn_r, mx_r = V.min(0), V.max(0)
        along_x = ext[0] >= ext[2]
        for p in live:
            Pv = p["mesh"].vertices @ R.T
            a, b = Pv.min(0), Pv.max(0)
            h = b[1] - a[1]
            ln, th = ((b[0] - a[0]), (b[2] - a[2])) if along_x else ((b[2] - a[2]), (b[0] - a[0]))
            if a[1] > mn_r[1] + 0.06 * ext[1] or not (0.18 * ext[1] <= h <= 0.6 * ext[1]):
                continue
            if not (0.7 * h <= ln <= 1.4 * h) or th > 0.75 * h:
                continue
            k = 0 if along_x else 2
            c = (a[k] + b[k]) / 2 - (mn_r[k] + mx_r[k]) / 2
            if abs(c) < 0.25 * Lh:
                continue
            wheels += 1
            ends.add(c > 0)
            p["wheel_geom"] = True
        if len(ends) < 2:
            wheels = 0
    is_car = cfg.get("kind", "auto") == "car" or (
        cfg.get("kind", "auto") == "auto" and
        (any(p["cls"] in (CLS_TIRE, CLS_LIGHT_RED, CLS_LIGHT_CLEAR) for p in live) or
         (car_shape and (wheels >= 3 or any(p["cls"] == CLS_PAINT for p in live)))))
    cfg["_is_car"] = is_car
    # glTF convention: the front faces +Z -> becomes +X (lamps or part names may turn it later)
    if not is_car or ext[2] > ext[0]:
        R = rot_y(90) @ R
    V = allV @ R.T
    # front detection: headlights (light_clear) vs taillights (light_red)
    if "yaw_deg" in cfg:
        R = rot_y(float(cfg["yaw_deg"])) @ R
    elif is_car:
        Vr = np.concatenate([p["mesh"].vertices for p in parts if p["cls"] != CLS_DROP]) @ R.T
        side = lamp_front_sign(parts, R, float(Vr[:, 0].min() + Vr[:, 0].max()) / 2)
        if side is None:                   # fallback: headlight vs taillight parts by class
            cx = lambda cls: [p["mesh"].vertices @ R.T for p in parts if p["cls"] == cls]
            fr, rr = cx(CLS_LIGHT_CLEAR), cx(CLS_LIGHT_RED)
            if fr and rr:
                side = 1 if np.concatenate(fr)[:, 0].mean() > np.concatenate(rr)[:, 0].mean() else -1
        if side is None:                   # part names: "front", "rear", wheel_FL / WheelBR ...
            FRONT = re.compile(r"front|frnt|(?<![a-z])f[lr](?![a-z])|wheel_?f[lr]|f[lr]_?wheel")
            REAR = re.compile(r"rear|back|(?<![a-z])[rb][lr](?![a-z])|wheel_?[rb][lr]|[rb][lr]_?wheel")
            xf, xr2 = [], []
            for p in parts:
                if p["cls"] == CLS_DROP:
                    continue
                nm = p["name"].lower()
                isf, isr = bool(FRONT.search(nm)), bool(REAR.search(nm))
                if isf == isr:
                    continue
                sx = float((p["mesh"].vertices @ R.T)[:, 0].mean())
                (xf if isf else xr2).append(sx)
            if xf and xr2 and abs(np.mean(xf) - np.mean(xr2)) > 1e-6:
                side = 1 if np.mean(xf) > np.mean(xr2) else -1
                log("  front: from part names (front / rear)")
        if side is not None and side < 0:
            R = rot_y(180) @ R
    if cfg.get("flip_front", False):
        R = rot_y(180) @ R
    V = allV @ R.T
    mn, mx = V.min(0), V.max(0)
    center = np.array([(mn[0] + mx[0]) / 2, mn[1], (mn[2] + mx[2]) / 2])
    # car: length -> 2 units (x in [-1, 1]); object: largest dimension -> 2 units
    scale = 2.0 / max((mx[0] - mn[0]) if is_car else (mx - mn).max(), 1e-9)
    for p in parts:
        p["V"] = (p["mesh"].vertices @ R.T - center) * scale
        p["F"] = np.asarray(p["mesh"].faces, dtype=np.int64)


# --------------------------------------------------------------------------------------
# Main pipeline
# --------------------------------------------------------------------------------------
NORMAL_FILTER_R = 0.03
GLASS_LIFT = 0.004


def filtered_normals(Vn, Nn, hf, ok, sheet=False):
    """Normal of a smooth surface patch around each vertex: area weighted normals of the
    visible source faces of the same material group within NORMAL_FILTER_R that face the same
    way (< 50 deg). Panel gaps, rivets and other detail the simplified mesh can not show no
    longer bend the vertex normals, so doors and bonnets reflect evenly. Thin sheets (glass)
    take faces of either orientation."""
    cos_t = math.cos(math.radians(50))
    out = Nn.copy()
    lists = hf["tree"].query_ball_point(Vn, NORMAL_FILTER_R)
    for i, idx in enumerate(lists):
        if len(idx) < 8:
            continue
        idx = np.asarray(idx)
        idx = idx[ok[idx]]
        if len(idx) < 8:
            continue
        fn = hf["N"][idx]
        d = fn @ Nn[i]
        if sheet:
            fn = np.where((d < 0)[:, None], -fn, fn); d = np.abs(d)
        m = d >= cos_t
        if m.sum() < 8:
            continue
        a = (fn[m] * hf["A"][idx[m], None]).sum(0)
        ln = np.linalg.norm(a)
        if ln > 1e-12:
            out[i] = a / ln
    return out


def turn_flipped(Vg, Fg, hf, ok, hi_flip):
    """Faces lying over source patches that are seen from behind get turned around."""
    Fg = Fg.copy()
    cen = Vg[Fg].mean(1)
    d, q = hf["tree"].query(cen, k=16, distance_upper_bound=NORMAL_FILTER_R)
    n = len(hf["C"])
    for f in range(len(Fg)):
        for j in range(16):
            k = q[f, j]
            if k >= n:
                break
            if ok[k]:
                if hi_flip[k]:
                    Fg[f] = Fg[f, [0, 2, 1]]
                break
    return Fg


def parse_color(s):
    s = s.strip().lstrip("#")
    return srgb_to_linear(np.array([int(s[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]))


def convert(args):
    cfg = {}
    if args.config:
        with open(args.config, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    budget = int(args.budget or cfg.get("budget", 10000))
    import os
    if not os.path.isfile(args.input):
        sys.exit(f"File not found: {args.input}  (path is relative to the folder you run the command in: "
                 f"{os.getcwd()})")
    log(f"Loading {args.input}")
    parts = load_parts(args.input)
    log(f"  {len(parts)} parts, {sum(len(p['mesh'].faces) for p in parts)} triangles")
    # fully transparent materials (alpha 0: hidden helpers, texture blend cards) are invisible
    n_inv = len(parts)
    parts = [p for p in parts if p["mat"]["alpha"] >= 0.03]
    if not parts:
        sys.exit("No visible triangle meshes found")
    if len(parts) < n_inv:
        log(f"  skipped {n_inv - len(parts)} invisible parts (alpha 0)")

    # ---- classify
    overrides = {k.lower(): v for k, v in cfg.get("materials", {}).items()}
    for p in parts:
        key_m, key_p = p["mat"]["name"].lower(), p["name"].lower()
        ov = overrides.get(key_p, overrides.get(key_m))
        if ov is not None:
            p["cls"] = CLS_DROP if ov == "drop" else CLS_NAMES.index(ov)
        else:
            p["cls"] = classify(p)
            p["paint_by_name"] = p["cls"] == CLS_PAINT and bool(LIVERY_RX.search((p["mat"]["name"] + " " + p["name"]).lower()))

    normalise_orientation(parts, cfg)
    parts = [p for p in parts if p["cls"] != CLS_DROP]
    log(f"  mode: {'car' if cfg['_is_car'] else 'object'}")
    if not cfg["_is_car"]:               # "interior" (dim, inside a car) means nothing for an object
        for p in parts:
            if p["cls"] == CLS_INTERIOR:
                p["cls"] = CLS_GENERIC
    parts = drop_overlay_shells(parts)
    if cfg["_is_car"]:
        for p in parts:
            part_face_colors(p)
        parts = lamps_by_position(parts)
    parts = apply_texture_split(parts, cfg["_is_car"], args.tex_colors)
    if cfg["_is_car"]:
        parts = drop_lamp_covers(parts)
    ndup = 0
    for p in parts:
        vc = vertex_colors(p["V"], p["F"], p["fcol"]) if p.get("fcol") is not None and p.get("tex_detail") else None
        Vw, Fw, winv = weld(p["V"], p["F"], 1e-6, return_inv=True)
        p["vcol"] = None
        if vc is not None:                  # average the colours of welded vertices
            acc = np.zeros((len(Vw), 3)); cnt = np.zeros(len(Vw))
            np.add.at(acc, winv, vc); np.add.at(cnt, winv, 1)
            p["vcol"] = acc / np.maximum(cnt, 1)[:, None]
        p["fcol"] = None
        Fd = dedupe_faces(Fw)
        ndup += len(Fw) - len(Fd)
        # make winding consistent inside every connected patch (orientation decided later)
        tm = trimesh.Trimesh(Vw, Fd, process=False)
        try:
            trimesh.repair.fix_winding(tm)
        except Exception:
            pass
        p["V"], p["F"] = Vw, np.asarray(tm.faces, np.int64)
    if ndup:
        log(f"  removed {ndup} duplicate (double-sided) triangles")

    # ---- global arrays
    offs, Vs, Fs, owner = 0, [], [], []
    for i, p in enumerate(parts):
        Vs.append(p["V"]); Fs.append(p["F"] + offs); owner.append(np.full(len(p["F"]), i))
        offs += len(p["V"])
    V = np.concatenate(Vs); F = np.concatenate(Fs); owner = np.concatenate(owner)
    radius = float(np.linalg.norm(V, axis=1).max()) * 1.02
    # linear texture colour per vertex (-1 = no texture) for the texture detail further down
    hiCol = -np.ones_like(V)
    o = 0
    for p in parts:
        if p.get("vcol") is not None:
            hiCol[o:o + len(p["V"])] = p["vcol"]
        p["hi_range"] = (o, len(p["V"]))
        p["vcol"] = None
        o += len(p["V"])
    hiV = V

    # ---- hidden surface removal
    hi_flip = np.zeros(len(F), bool)
    if not args.keep_hidden:
        vis, fcnt, bcnt = visible_triangles(V, F, radius, n_views=args.views)
        # orientation vote per connected patch: flip whole patches seen mostly from behind
        flip = np.zeros(len(F), bool)
        tm = trimesh.Trimesh(V, F, process=False)
        try:
            comps = trimesh.graph.connected_components(tm.face_adjacency, nodes=np.arange(len(F)), min_len=1)
        except Exception:
            comps = [np.arange(len(F))]
        for comp in comps:
            comp = np.asarray(comp)
            if (bcnt[comp] - fcnt[comp]).sum() > 0:
                flip[comp] = True
        F[flip] = F[flip][:, [0, 2, 1]]
        # inside a piece whose winding could not be made consistent (a mirrored panel welded to
        # its neighbour), a larger patch seen clearly from behind is wound the wrong way: the
        # device would cull it and show what lies behind. Turned only in the final mesh
        # (hi_flip) - turning it here leaves edges the simplifier treats as borders.
        fr_ = np.where(flip, bcnt, fcnt); bk_ = np.where(flip, fcnt, bcnt)
        cand = (bk_ >= 4) & (bk_ > 6 * fr_)
        if cand.any():
            adj = tm.face_adjacency
            adj = adj[cand[adj[:, 0]] & cand[adj[:, 1]]]
            ci = np.nonzero(cand)[0]
            try:
                cl = trimesh.graph.connected_components(adj, nodes=ci, min_len=16)
            except Exception:
                cl = []
            for c in cl:
                hi_flip[np.asarray(c)] = True
            if hi_flip.any():
                log(f"  {int(hi_flip.sum())} triangles seen from behind: turned in the final mesh")
        twosided = vis & (np.minimum(fcnt, bcnt) >= 0.3 * np.maximum(fcnt, bcnt)) & (np.maximum(fcnt, bcnt) >= 3)
        for i, p in enumerate(parts):
            m = owner == i
            p["twosided"] = bool(twosided[m].sum() > 0.25 * max(vis[m].sum(), 1))
        log(f"  visible: {vis.sum()} / {len(F)} triangles, winding fixed on {flip.sum()}")
    else:
        vis = np.ones(len(F), bool)
        for p in parts:
            p["twosided"] = p["mat"].get("double_sided", False)

    # high-resolution smooth vertex normals (area weighted) - transferred to the final mesh
    fnr = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    fnr[hi_flip] *= -1
    # high resolution faces (centroid / unit normal / area / part / visible) for the
    # filtered normals and the final winding fix
    fl = np.maximum(np.linalg.norm(fnr, axis=1), 1e-20)
    hi_face = dict(C=V[F].mean(1), N=fnr / fl[:, None], A=fl, own=owner.copy(), vis=vis.copy(),
                   start=[p["hi_range"][0] for p in parts])
    hi_face["tree"] = cKDTree(hi_face["C"])
    hiN = np.zeros_like(V)
    for k in range(3):
        np.add.at(hiN, F[:, k], fnr)
    hiN /= np.maximum(np.linalg.norm(hiN, axis=1), 1e-20)[:, None]
    hi_tree = cKDTree(V)

    # paint auto-detection: if nothing is classified as paint, take the most visible
    # coloured exterior material
    areas = np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1) * 0.5
    # ---- car paint: chosen per MATERIAL, the biggest visible saturated surface wins.
    # A "paint/body" name only gives a bonus (part names like "D_Body_35" are often trim).
    if cfg.get("_is_car", True) and not any(k.lower() in overrides for k in
                                            {p["mat"]["name"] for p in parts if p["cls"] == CLS_PAINT}):
        score = {}
        high_f = V[F].mean(1)[:, 1] > 0.3 * V[:, 1].max()
        for i, p in enumerate(parts):
            if p["cls"] not in (CLS_GENERIC, CLS_PLASTIC, CLS_PAINT) or (p.get("from_split") and not p.get("paint_src")):
                continue
            # only surface above the sills counts (the underbody is big but never the paint)
            a = areas[(owner == i) & vis & high_f].sum()
            srgb = linear_to_srgb(p["mat"]["color"])
            sat = float(np.ptp(srgb))
            sc = a * (0.25 + 1.5 * sat) * 0.5
            if p["cls"] == CLS_PAINT:
                sc *= 1.5
            if LIVERY_RX.search((p["mat"]["name"] + " " + p["name"]).lower()):
                sc *= 2.5                                   # "paint", "primary"
            k = p.get("paint_src") or p["mat"]["name"]       # a livery competes with all its regions
            score[k] = score.get(k, 0.0) + sc
        if score:
            mname = max(score, key=score.get)
            for p in parts:
                if (p.get("paint_src") or p["mat"]["name"]) == mname:
                    p["cls"] = CLS_PAINT
                elif p["cls"] == CLS_PAINT:            # lost: back to a neutral class
                    lum = float(np.dot(p["mat"]["color"], [0.2126, 0.7152, 0.0722]))
                    p["cls"] = CLS_PLASTIC if lum < 0.02 else CLS_GENERIC
                    p["paint_key"] = None
            log(f"  paint: material '{mname}'" + (" (with livery)" if any(p.get("paint_src") == mname for p in parts) else ""))
            # "Coloured" / "body colour" materials are tinted with the paint colour at run time
            # (game asset convention); their own base colour is a neutral grey
            ref, ra = None, 0.0
            for i, p in enumerate(parts):
                if p["cls"] == CLS_PAINT and (p.get("paint_src") or p["mat"]["name"]) == mname and \
                        p.get("paint_key") in (None, "main"):
                    a = areas[(owner == i) & vis].sum()
                    if a > ra:
                        ra, ref = a, p
            nbc = 0
            if ref is not None:
                for p in parts:
                    if p.get("from_split") or p is ref or p["cls"] not in (CLS_METAL, CLS_GENERIC, CLS_PLASTIC, CLS_PAINT):
                        continue
                    if not BODYCOL_RX.search((p["mat"]["name"] + " " + p["name"]).lower()):
                        continue
                    p["cls"] = CLS_PAINT; p["paint_key"] = "main"
                    p["mat"] = dict(p["mat"]); p["mat"]["color"] = np.array(ref["mat"]["color"], dtype=np.float64)
                    p["tex_detail"] = False             # its texture is the tint mask, not shading
                    o0, n0 = p["hi_range"]
                    hiCol[o0:o0 + n0] = -1
                    nbc += 1
            if nbc:
                log(f"  body-colour parts painted: {nbc}")

    tot_area = np.array([areas[owner == i].sum() for i in range(len(parts))])
    vis_area0 = np.array([areas[(owner == i) & vis].sum() for i in range(len(parts))])
    # ---- drop hidden triangles now (full resolution), then coarse stage-1 decimation
    for i, p in enumerate(parts):
        m = (owner == i)
        keepm = vis[m]
        Fp = F[m][keepm] - 0
        if len(Fp) == 0:
            p["V"], p["F"] = np.zeros((0, 3)), np.zeros((0, 3), np.int64)
            continue
        used = np.unique(Fp)
        remap = -np.ones(len(V), np.int64); remap[used] = np.arange(len(used))
        p["V"], p["F"] = V[used], remap[Fp]
    total = sum(len(p["F"]) for p in parts)
    keep = min(1.0, 5.0 * budget / max(total, 1))
    if keep < 1.0:
        # colour regions of one surface are simplified together (shared borders, no cracks)
        for idxs in split_groups(parts):
            live = [i for i in idxs if len(parts[i]["F"])]
            if not live:
                continue
            Vm, Fm, own, cen = merge_parts([parts[i] for i in live])
            if not len(Fm):
                continue
            target = max(64, int(len(Fm) * keep))
            D = meshopt_simplify(Vm, Fm, target) if (meshopt is not None and len(Fm) > target) else Fm
            fo = face_owners(Vm, own, cen, D) if len(live) > 1 else np.zeros(len(D), np.int64)
            for k, i in enumerate(live):
                Fk = D[fo == k]
                used = np.unique(Fk)
                remap = -np.ones(len(Vm), np.int64); remap[used] = np.arange(len(used))
                parts[i]["V"], parts[i]["F"] = Vm[used], remap[Fk]
        log(f"  stage-1 decimation: {total} -> {sum(len(p['F']) for p in parts)} triangles")
    offs, Vs, Fs, owner = 0, [], [], []
    for i, p in enumerate(parts):
        Vs.append(p["V"]); Fs.append(p["F"] + offs); owner.append(np.full(len(p["F"]), i))
        offs += len(p["V"])
    V = np.concatenate(Vs); F = np.concatenate(Fs).astype(np.int64); owner = np.concatenate(owner)
    vis = np.ones(len(F), bool)
    areas_new = np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1) * 0.5
    vis_area = np.array([areas_new[owner == i].sum() for i in range(len(parts))])

    # ---- global error-driven decimation ------------------------------------------------
    # All parts are simplified together so the quadric error decides where triangles are
    # needed. Each part is scaled about its centroid by an importance factor (error ~ s^2);
    # meshoptimizer keeps original vertices, so faces map straight back to their part.
    pieces = []
    voff = 0
    for idxs in split_groups(parts):
        live = [i for i in idxs if len(parts[i]["F"])]
        if not live:
            continue
        Vp, Fp, own, cen = merge_parts([parts[i] for i in live])
        if len(Fp) == 0:
            continue
        w = np.array([max(tot_area[i], 1e-12) for i in live])
        s_all = np.array([CLS_IMPORTANCE[parts[i]["cls"]] * (0.35 + 0.65 * math.sqrt(vis_area0[i] / max(tot_area[i], 1e-12)))
                          for i in live])
        s_i = float((s_all * w).sum() / w.sum())            # colour regions: area weighted importance
        c_i = Vp.mean(0)
        pieces.append(dict(live=live, OV=Vp, SV=(Vp - c_i) * s_i + c_i, F=Fp + voff, own=own, cen=cen, voff=voff))
        voff += len(Vp)
    OV = np.concatenate([q["OV"] for q in pieces])          # original positions
    SV = np.concatenate([q["SV"] for q in pieces])          # importance-scaled positions
    GF = np.concatenate([q["F"] for q in pieces])
    vowner = np.concatenate([np.full(len(q["OV"]), k) for k, q in enumerate(pieces)])
    log(f"  global decimation: {len(GF)} -> {budget} triangles")
    if len(GF) > budget and meshopt is not None:
        DF = meshopt_simplify(SV, GF, budget)
    else:
        DF = GF
    DV = OV
    fk = vowner[DF[:, 0]]
    out_parts = []
    for k, q in enumerate(pieces):
        Fk_all = DF[fk == k]
        if not len(Fk_all):
            continue
        Floc = Fk_all - q["voff"]
        fo = face_owners(q["OV"], q["own"], q["cen"], Floc) if len(q["live"]) > 1 else np.zeros(len(Floc), np.int64)
        for kk, i in enumerate(q["live"]):
            Fk = Floc[fo == kk]
            if not len(Fk):
                continue
            used = np.unique(Fk)
            remap = -np.ones(len(q["OV"]), np.int64); remap[used] = np.arange(len(used))
            p = parts[i]
            out_parts.append(dict(name=p["name"], cls=p["cls"], mat=p["mat"], V=q["OV"][used], F=remap[Fk],
                                  tex_detail=bool(p.get("tex_detail")), hi_range=p.get("hi_range"),
                                  twosided=p.get("twosided", False), paint_key=p.get("paint_key")))
    log(f"  after decimation: {sum(len(p['F']) for p in out_parts)} triangles")

    # ---- group by material (class + colour) -> one draw range each
    groups = {}
    for p in out_parts:
        c = p["mat"]["color"]
        srgb6 = tuple(np.round(linear_to_srgb(c) * 6).astype(int))
        key = ("u", p["cls"], srgb6) if p["mat"].get("unlit") else \
            (p["cls"], "main" if p.get("paint_key") in (None, "main") else srgb6) if p["cls"] == CLS_PAINT else \
            (p["cls"], _is_amber(linear_to_srgb(c))) if p["cls"] == CLS_LIGHT_CLEAR else \
            (p["cls"],) if p["cls"] in (CLS_GLASS, CLS_CHROME, CLS_TIRE, CLS_LIGHT_RED,
                                        CLS_CARBON) else \
            (p["cls"], tuple(np.round(linear_to_srgb(c) * 6).astype(int)))
        groups.setdefault(key, []).append(p)
    maxm = max(1, min(16, args.max_materials))
    # too many materials: merge the two closest colours of the SAME class (never glass
    # into lights etc.); only if every class is down to one group, merge the smallest
    # "neutral" group (plastic/generic/interior/metal/tire/carbon) into the closest neutral one
    NEUTRAL = (CLS_PLASTIC, CLS_GENERIC, CLS_INTERIOR, CLS_METAL, CLS_TIRE, CLS_CARBON)
    def gcol(k):
        pl = groups[k]
        w = np.array([len(p["F"]) for p in pl], np.float64) + 1e-9
        return linear_to_srgb(np.average([p["mat"]["color"] for p in pl], axis=0, weights=w))
    def gsize(k):
        return sum(len(p["F"]) for p in groups[k])
    while len(groups) > maxm:
        keys = list(groups)
        best = None
        for i in range(len(keys)):
            for j in range(i + 1, len(keys)):
                if keys[i][0] != keys[j][0]:
                    continue
                d = float(np.linalg.norm(gcol(keys[i]) - gcol(keys[j])))
                if best is None or d < best[0]:
                    best = (d, keys[i], keys[j])
        if best is None:
            neutral = [k for k in keys if k[0] in NEUTRAL]
            if len(neutral) < 2:
                log(f"  warning: {len(groups)} materials, renderer limit is {maxm}; dropping the smallest")
                groups.pop(min(keys, key=gsize))
                continue
            small = min(neutral, key=gsize)
            tgt = min((k for k in neutral if k != small),
                      key=lambda k: float(np.linalg.norm(gcol(k) - gcol(small))))
            best = (0, tgt, small)
        _, a, b = best
        if gsize(b) > gsize(a):
            a, b = b, a
        groups[a].extend(groups.pop(b))

    paint_lin = parse_color(args.paint) if args.paint else (parse_color(cfg["paint"]) if "paint" in cfg else None)

    y_all = np.concatenate([p["V"][:, 1] for p in out_parts])
    y_mid = float(y_all.min() + y_all.max()) / 2
    allV, allN, allF, mats, details = [], [], [], [], []
    lod1_parts = []
    voff, toff = 0, 0
    for key, plist in groups.items():
        unlit = key[0] == "u"
        cls = key[1] if unlit else key[0]
        main = cls == CLS_PAINT and key[1] == "main"
        Vg = np.concatenate([p["V"] for p in plist])
        Fg = np.concatenate([p["F"] + o for p, o in zip(plist, np.cumsum([0] + [len(q["V"]) for q in plist])[:-1])])
        crease = args.crease if cls != CLS_TIRE else 60
        starts = {p["hi_range"][0] for p in plist if p.get("hi_range")}
        ok = np.array([st in starts for st in hi_face["start"]])[hi_face["own"]] & hi_face["vis"]
        if hi_flip.any() and cls != CLS_GLASS and not any(p.get("twosided") for p in plist):
            Fg = turn_flipped(Vg, Fg, hi_face, ok, hi_flip)
        Vn, Nn, Fn = crease_normals(Vg, Fg, crease)
        Nn, _ = transfer_normals(Vn, Nn, hi_tree, hiN)
        if cls == CLS_GLASS:
            # windows are thin sheets (double sided or two layers): normals can end up pointing
            # into the car (dark AO, wrong reflection). Glass is outside: point them outwards.
            rel = Vn - np.array([0.0, y_mid, 0.0])
            Nn = np.where((np.sum(rel * Nn, axis=1) < 0)[:, None], -Nn, Nn)
        if cls != CLS_TIRE:
            Nn = filtered_normals(Vn, Nn, hi_face, ok, sheet=cls == CLS_GLASS)
        if cls == CLS_GLASS and cfg.get("_is_car"):
            # windows sit a hair proud of the frame: flanges and seals just behind the glass
            # edge would otherwise poke through after simplification
            Vn = Vn + Nn * GLASS_LIFT * radius
        if args.reflection_lod > 0 and meshopt is not None and len(Fg) > 8:
            F1g = meshopt_simplify(Vg, Fg, max(4, int(len(Fg) * args.reflection_lod)))
            u1 = np.unique(F1g)
            rm = -np.ones(len(Vg), np.int64); rm[u1] = np.arange(len(u1))
            V1, N1, F1 = crease_normals(Vg[u1], rm[F1g], crease)
            N1, _ = transfer_normals(V1, N1, hi_tree, hiN)
        else:
            V1, N1, F1 = Vn, Nn, Fn
        lod1_parts.append((V1, N1, F1))
        a = np.array([np.linalg.norm(np.cross(p["V"][p["F"][:, 1]] - p["V"][p["F"][:, 0]],
                                              p["V"][p["F"][:, 2]] - p["V"][p["F"][:, 0]]), axis=1).sum() for p in plist])
        color = np.average([p["mat"]["color"] for p in plist], axis=0, weights=a + 1e-12)
        rough = float(np.average([p["mat"]["roughness"] for p in plist], weights=a + 1e-12))
        if cls == CLS_PAINT and paint_lin is not None and main:
            color = paint_lin
        # texture detail: brightness changes inside a colour region go into the per-vertex
        # light factor (the region has one colour), so a textured surface keeps its shading
        detail = None
        if (cls in TEX_DETAIL_CLS or unlit) and any(p.get("tex_detail") for p in plist) and not (cls == CLS_PAINT and paint_lin is not None and main):
            idx = np.concatenate([np.arange(p["hi_range"][0], p["hi_range"][0] + p["hi_range"][1])
                                  for p in plist if p.get("hi_range")] or [np.zeros(0, np.int64)])
            if len(idx):
                d, q = cKDTree(hiV[idx]).query(Vn)
                k = idx[np.minimum(q, len(idx) - 1)]
                lum = np.where((d < 1e-5) & (hiCol[k, 0] >= 0), hiCol[k] @ np.array([0.2126, 0.7152, 0.0722]), -1.0)
                valid = np.sort(lum[lum >= 0])
                if len(valid) > 8:
                    ref = valid[int(len(valid) * 0.8)]; mean = valid.mean()
                    if ref > 1e-4 and mean > 1e-4:
                        detail = np.where(lum < 0, 1.0, np.clip(lum / ref, 0.3, 1.0))
                        color = color * min(ref / mean, 1.0 / max(float(np.max(color)), 1e-6))
        details.append((len(Vn), detail, unlit))
        allV.append(Vn); allN.append(Nn); allF.append(Fn + voff)
        dsided = any(p.get("twosided", False) for p in plist) or cls == CLS_GLASS
        mats.append(dict(cls=cls, color=color, rough=rough, unlit=unlit, tri_start=toff, tri_count=len(Fn), dsided=dsided,
                         names=sorted({p["mat"]["name"] for p in plist})))
        voff += len(Vn); toff += len(Fn)
    V = np.concatenate(allV); N = np.concatenate(allN); F = np.concatenate(allF)
    if len(V) > 65535:
        sys.exit(f"Too many vertices ({len(V)}); lower --budget")
    log(f"  final: {len(V)} vertices, {len(F)} triangles, {len(mats)} materials")

    # ---- AO
    if args.no_ao:
        ao = np.ones(len(V))
    else:
        log("  baking ambient occlusion ...")
        ao = bake_ao(V, N, F, radius)
    o = 0
    for n, d, unl in details:
        if unl:                         # unlit material: light is baked into the texture, no AO
            ao[o:o + n] = 1.0
        if d is not None:
            ao[o:o + n] = ao[o:o + n] * d
        o += n

    # ---- matcaps
    log("  rendering matcaps ...")
    matcap_imgs = []
    for m in mats:
        size = CLS_MATCAP.get(m["cls"], 64) if args.matcap_size == 0 else args.matcap_size
        rgb565, srgb = render_matcap(m["cls"], m["color"], size, m["rough"], unlit=m.get("unlit", False),
                                     obj=not cfg.get("_is_car", True))
        m["matcap_size"] = size
        matcap_imgs.append((rgb565, srgb))

    tm0 = np.zeros(len(F), np.uint8)
    for i, m in enumerate(mats):
        tm0[m["tri_start"]:m["tri_start"] + m["tri_count"]] = i
    lods = [(V, N, F, ao, tm0)]
    # ---- LOD1: light mesh used for the floor reflection (dim + faded -> detail not visible)
    if args.reflection_lod > 0:
        V1 = np.concatenate([q[0] for q in lod1_parts]); N1 = np.concatenate([q[1] for q in lod1_parts])
        offs1 = np.cumsum([0] + [len(q[0]) for q in lod1_parts])[:-1]
        F1 = np.concatenate([q[2] + o for q, o in zip(lod1_parts, offs1)])
        T1 = np.concatenate([np.full(len(q[2]), i, np.uint8) for i, q in enumerate(lod1_parts)])
        _, nn = cKDTree(V).query(V1)
        ao1 = ao[nn]
        lods.append((V1, N1, F1, ao1, T1))
        log(f"  reflection LOD: {len(V1)} vertices, {len(F1)} triangles")
    write_blob(args.output, lods, mats, matcap_imgs, radius, cfg.get("name", args.name))
    if args.preview_matcaps:
        save_matcap_sheet(args.preview_matcaps, mats, matcap_imgs)
    report = dict(input=args.input, vertices=int(len(V)), triangles=int(len(F)), radius=radius,
                  materials=[dict(cls=CLS_NAMES[m["cls"]], tris=m["tri_count"], src=m["names"],
                                  matcap=m["matcap_size"]) for m in mats])
    with open(args.output + ".json", "w", encoding="utf-8") as f:
        json.dump(report, f, indent=1)
    log(f"Wrote {args.output}")


def save_matcap_sheet(path, mats, imgs):
    from PIL import Image
    tiles = []
    for m, (_, srgb) in zip(mats, imgs):
        im = Image.fromarray((np.clip(srgb, 0, 1) * 255).astype(np.uint8)).resize((128, 128))
        tiles.append(im)
    sheet = Image.new("RGB", (128 * len(tiles), 128))
    for i, t in enumerate(tiles):
        sheet.paste(t, (128 * i, 0))
    sheet.save(path)


# --------------------------------------------------------------------------------------
# Binary writer  (layout mirrored in components/r3d/include/r3d_model.h)
# --------------------------------------------------------------------------------------
HDR_FMT = "<4sHHII" + "HH" + "3f" + "ff" + "I" + "IIIHHI" * 2 + "32s" + "12x"
MAT_FMT = "<BBBBIHH"


def write_blob(path, lods, mats, imgs, radius, name):
    allV = np.concatenate([l[0] for l in lods])
    mx = np.abs(allV).max(0) + 1e-9
    scale = mx / 32767.0                       # symmetric int16 quantisation, no offset
    ymax = float(allV[:, 1].max())

    hdr_size = struct.calcsize(HDR_FMT)
    mat_size = struct.calcsize(MAT_FMT)
    al = lambda x: (x + 15) & ~15
    off = al(hdr_size + mat_size * len(mats))
    sections, lod_entries = [], []
    for (V, N, F, ao, tm) in lods:
        if len(V) > 65535:
            sys.exit(f"Too many vertices ({len(V)}); lower --budget")
        vrec = np.zeros(len(V), dtype=[("p", "<i2", 3), ("n", "i1", 3), ("ao", "u1")])
        vrec["p"] = np.round(V / scale).astype(np.int16)
        vrec["n"] = np.clip(np.round(N * 127), -127, 127).astype(np.int8)
        vrec["ao"] = np.clip(np.round(ao * 255), 0, 255).astype(np.uint8)
        vb, ib, tb = vrec.tobytes(), F.astype("<u2").tobytes(), tm.astype(np.uint8).tobytes()
        ov = off; off = al(off + len(vb))
        oi = off; off = al(off + len(ib))
        ot = off; off = al(off + len(tb))
        sections += [(ov, vb), (oi, ib), (ot, tb)]
        lod_entries.append((ov, oi, ot, len(V), 0, len(F)))
    while len(lod_entries) < 2:
        lod_entries.append(lod_entries[0])
    mc_offsets = []
    for (rgb565, _) in imgs:
        b = rgb565.astype("<u2").tobytes()
        mc_offsets.append(off)
        sections.append((off, b))
        off = al(off + len(b))
    total = off
    body = bytearray(total)
    mbuf = bytearray()
    for m, mo in zip(mats, mc_offsets):
        mbuf += struct.pack(MAT_FMT, m["cls"], 1 if m.get("dsided") else 0,
                            int(math.log2(m["matcap_size"])), 0, mo, m["matcap_size"], 0)
    body[hdr_size:hdr_size + len(mbuf)] = mbuf
    for o, b in sections:
        body[o:o + len(b)] = b
    crc = zlib.crc32(bytes(body[hdr_size:])) & 0xFFFFFFFF
    flat = [x for e in lod_entries[:2] for x in e]
    hdr = struct.pack(HDR_FMT, MAGIC, VERSION, hdr_size, total, crc,
                      len(mats), len(lods),
                      *scale.astype(np.float32), radius, ymax, hdr_size,
                      *flat, name.encode("utf-8")[:31])
    assert len(hdr) == 128, len(hdr)
    body[:hdr_size] = hdr
    with open(path, "wb") as f:
        f.write(body)
    log(f"  blob: {total / 1024:.1f} KiB")


def main():
    ap = argparse.ArgumentParser(description="3D model -> R3DM converter for ESP32-S3 renderer")
    ap.add_argument("input")
    ap.add_argument("-o", "--output", default="model.bin")
    ap.add_argument("--config", help="JSON with overrides (materials, paint, budget, yaw_deg, up_axis)")
    ap.add_argument("--budget", type=int, help="target triangle count (default 10000)")
    ap.add_argument("--paint", help="override body colour, e.g. '#B0101A'")
    ap.add_argument("--name", default="model")
    ap.add_argument("--agg", type=float, default=3.0, help="decimation aggressiveness 0..10 (lower = better shape)")
    ap.add_argument("--crease", type=float, default=42.0, help="crease angle for hard edges [deg]")
    ap.add_argument("--views", type=int, default=64, help="views for hidden-surface removal")
    ap.add_argument("--keep-hidden", action="store_true", help="do not remove invisible geometry")
    ap.add_argument("--no-ao", action="store_true")
    ap.add_argument("--reflection-lod", type=float, default=0.0,
                    help="triangle ratio of the reflection mesh (0 = reuse the full mesh)")
    ap.add_argument("--max-materials", type=int, default=15, help="max distinct materials (<=16)")
    ap.add_argument("--matcap-size", type=int, default=0, choices=[0, 32, 64, 128],
                    help="force one matcap size (0 = auto: 128 for paint/glass/chrome, 64 otherwise)")
    ap.add_argument("--preview-matcaps", help="write PNG sheet of generated matcaps")
    ap.add_argument("--tex-colors", type=int, default=12,
                    help="textured models: split the texture into up to N colour materials (0 = off)")
    convert(ap.parse_args())


if __name__ == "__main__":
    main()
