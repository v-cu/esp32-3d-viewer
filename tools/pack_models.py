#!/usr/bin/env python3
"""
pack_models.py - combine several converted models into one pack for the "model" partition.
On the device a triple tap cycles through them (order = order on the command line).

  python tools/pack_models.py audi.bin ferrari.bin duck.bin
  python tools/pack_models.py models/*.bin -o firmware/models/model.bin

Then:  idf.py -p COM5 model-flash      (or python tools/flash_model.py COM5)

Pack layout: "R3DP" u16 version=1 u16 count, count x {u32 offset, u32 size, char name[24]},
models follow, each 4 KiB aligned. A single plain model.bin (no pack) also works.
"""
import argparse
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ENTRY = "<II24s"
PART_SIZE_DEFAULT = 0xCF0000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("models", nargs="+", help="R3DM files from model_convert.py")
    ap.add_argument("-o", "--output", default=os.path.join(ROOT, "firmware", "models", "model.bin"))
    ap.add_argument("--names", help="comma separated display names (default: name stored in each model)")
    ap.add_argument("--partition-size", type=lambda x: int(x, 0), default=PART_SIZE_DEFAULT)
    a = ap.parse_args()

    blobs = []
    for path in a.models:
        data = open(path, "rb").read()
        if data[:4] != b"R3DM":
            sys.exit(f"{path}: not an R3DM model (pack files can not be nested)")
        name = data[0x54:0x54 + 32].split(b"\0")[0]       # name field of the R3DM header
        blobs.append((path, data, name))
    if len(blobs) > 32:
        sys.exit("max 32 models")
    names = a.names.split(",") if a.names else None

    hdr_size = 8 + struct.calcsize(ENTRY) * len(blobs)
    off = (hdr_size + 4095) & ~4095
    table, body = bytearray(), bytearray()
    for i, (path, data, name) in enumerate(blobs):
        nm = (names[i].strip().encode("utf-8") if names and i < len(names) else name)[:23]
        table += struct.pack(ENTRY, off, len(data), nm)
        body += data
        body += b"\xFF" * (((len(data) + 4095) & ~4095) - len(data))
        print(f"  #{i + 1} {nm.decode('utf-8', 'replace'):24s} {len(data) / 1024:7.0f} KiB  @0x{off:06X}  ({path})")
        off += (len(data) + 4095) & ~4095
    head = b"R3DP" + struct.pack("<HH", 1, len(blobs)) + table
    head += b"\xFF" * (((hdr_size + 4095) & ~4095) - len(head))
    out = head + body
    if len(out) > a.partition_size:
        sys.exit(f"pack is {len(out) / 1024:.0f} KiB, partition holds {a.partition_size / 1024:.0f} KiB "
                 f"- use fewer models or a lower --budget")
    with open(a.output, "wb") as f:
        f.write(out)
    print(f"wrote {a.output}: {len(blobs)} models, {len(out) / 1024:.0f} KiB "
          f"({100 * len(out) / a.partition_size:.0f}% of the partition)")


if __name__ == "__main__":
    main()
