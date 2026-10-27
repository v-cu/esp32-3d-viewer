#!/usr/bin/env python3
"""
build_site.py - assemble the GitHub Pages site (installer + model studio).

  python tools/build_site.py                     # uses firmware/build, writes _site/
  python tools/build_site.py --build path/to/build --out _site --version v1.2.0

Copies web/ and adds:
  firmware/bootloader.bin, partition-table.bin, esp32-3d-viewer.bin, model.bin
  models/NN-name.bin       the demo models (taken out of firmware/models/model.bin)
  firmware/manifest.json   what the installer writes where

Test locally:  python -m http.server -d _site 8000   ->  http://localhost:8000
(Web Serial works on localhost and https only.)
"""
import argparse
import datetime
import json
import os
import re
import shutil
import struct
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = "esp32-3d-viewer.bin"


def git_version():
    try:
        return subprocess.check_output(["git", "describe", "--tags", "--always", "--dirty"], cwd=ROOT,
                                       stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return ""


def read_offsets(build):
    """bootloader / partition table / app offsets from flasher_args.json (fallback: defaults)."""
    off = {"bootloader": 0x0, "partition-table": 0x8000, "app": 0x10000}
    try:
        fa = json.load(open(os.path.join(build, "flasher_args.json")))
        off["bootloader"] = int(fa["bootloader"]["offset"], 16)
        off["partition-table"] = int(fa["partition-table"]["offset"], 16)
        off["app"] = int(fa["app"]["offset"], 16)
    except Exception:
        pass
    return off


def model_partition(csv_path):
    for line in open(csv_path):
        cols = [c.strip() for c in line.split("#")[0].split(",")]
        if len(cols) >= 5 and cols[0] == "model":
            return int(cols[3], 0), int(cols[4], 0)
    sys.exit("no 'model' partition in " + csv_path)


def split_pack(data):
    if data[:4] == b"R3DM":
        return [(data[84:116].split(b"\0")[0].decode("utf-8", "replace"), data)]
    assert data[:4] == b"R3DP", "model.bin is neither a pack nor a model"
    n = struct.unpack_from("<H", data, 6)[0]
    out = []
    for i in range(n):
        off, size, name = struct.unpack_from("<II24s", data, 8 + 32 * i)
        out.append((name.split(b"\0")[0].decode("utf-8", "replace"), data[off:off + size]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", default=os.path.join(ROOT, "firmware", "build"))
    ap.add_argument("--out", default=os.path.join(ROOT, "_site"))
    ap.add_argument("--version", default=os.environ.get("FW_VERSION") or git_version())
    ap.add_argument("--no-firmware", action="store_true", help="site without installer binaries")
    a = ap.parse_args()

    if os.path.isdir(a.out):
        shutil.rmtree(a.out)
    shutil.copytree(os.path.join(ROOT, "web"), a.out)
    fw_dir = os.path.join(a.out, "firmware")
    md_dir = os.path.join(a.out, "models")
    os.makedirs(fw_dir, exist_ok=True)
    os.makedirs(md_dir, exist_ok=True)
    open(os.path.join(a.out, ".nojekyll"), "w").close()

    pack_path = os.path.join(ROOT, "firmware", "models", "model.bin")
    pack = open(pack_path, "rb").read()
    m_off, m_size = model_partition(os.path.join(ROOT, "firmware", "partitions.csv"))
    if len(pack) > m_size:
        sys.exit("model.bin is larger than the model partition")
    shutil.copy(pack_path, os.path.join(fw_dir, "model.bin"))
    models = []
    for i, (name, blob) in enumerate(split_pack(pack)):
        fn = "%02d-%s.bin" % (i + 1, re.sub(r"[^A-Za-z0-9]+", "-", name).strip("-").lower() or "model")
        open(os.path.join(md_dir, fn), "wb").write(blob)
        models.append({"name": name, "file": "models/" + fn, "size": len(blob)})

    manifest = {
        "name": "esp32-3d-viewer",
        "chip": "ESP32-S3",
        "version": a.version,
        "date": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "models": models,
        "pack": {"file": "firmware/model.bin", "offset": m_off, "size": len(pack)},
        "modelPartition": {"offset": m_off, "size": m_size},
    }
    if not a.no_firmware:
        b = a.build
        files = [("Bootloader", os.path.join(b, "bootloader", "bootloader.bin"), "bootloader"),
                 ("Partition table", os.path.join(b, "partition_table", "partition-table.bin"), "partition-table"),
                 ("Application", os.path.join(b, APP), "app")]
        offs = read_offsets(b)
        parts = []
        for label, src, key in files:
            if not os.path.isfile(src):
                sys.exit(f"missing {src} - build the firmware first (idf.py build) or pass --no-firmware")
            dst = os.path.basename(src)
            shutil.copy(src, os.path.join(fw_dir, dst))
            parts.append({"name": label, "file": "firmware/" + dst, "offset": offs[key], "size": os.path.getsize(src)})
        manifest["parts"] = parts
    json.dump(manifest, open(os.path.join(fw_dir, "manifest.json"), "w"), indent=1)
    print(f"site written to {a.out}: {len(models)} demo models, firmware {'yes' if not a.no_firmware else 'no'}"
          f" ({a.version or 'no version'})")


if __name__ == "__main__":
    main()
