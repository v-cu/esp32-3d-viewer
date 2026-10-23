#!/usr/bin/env python3
"""
preview.py - turn the simulator's PPM frames into an animated GIF / PNG contact sheet.

  python tools/preview.py out/frame_ preview.gif [--fps 25] [--rot 1]
  (--rot = the rotation you passed to the simulator; frames are turned back upright)
"""
import argparse
import glob
from PIL import Image

ap = argparse.ArgumentParser()
ap.add_argument("prefix")
ap.add_argument("output")
ap.add_argument("--fps", type=int, default=25)
ap.add_argument("--rot", type=int, default=0)
ap.add_argument("--scale", type=int, default=1)
a = ap.parse_args()
files = sorted(glob.glob(a.prefix + "*.ppm"))
fix = {0: None, 1: Image.ROTATE_90, 2: Image.ROTATE_180, 3: Image.ROTATE_270}[a.rot & 3]
frames = []
for f in files:
    im = Image.open(f).convert("RGB")
    if fix is not None:
        im = im.transpose(fix)
    if a.scale > 1:
        im = im.resize((im.width * a.scale, im.height * a.scale), Image.NEAREST)
    frames.append(im)
if a.output.lower().endswith(".gif"):
    frames[0].save(a.output, save_all=True, append_images=frames[1:], duration=int(1000 / a.fps), loop=0, optimize=True)
else:
    frames[0].save(a.output)
print(f"{len(frames)} frames -> {a.output}")
