#!/usr/bin/env python3
"""
flash_model.py - write a converted model (model.bin) into the "model" flash partition
without rebuilding or reflashing the firmware.

  python tools/flash_model.py COM5                      # firmware/models/model.bin
  python tools/flash_model.py COM5 my_audi.bin
  python tools/flash_model.py /dev/ttyACM0 my_audi.bin --baud 921600

Alternative from the firmware folder (ESP-IDF shell):  idf.py -p COM5 model-flash
"""
import argparse
import csv
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def model_partition(csv_path):
    with open(csv_path, newline="") as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith("#"):
                continue
            cols = [c.strip() for c in row]
            if cols[0] == "model":
                return int(cols[3], 0), int(cols[4], 0)
    sys.exit(f"no 'model' partition in {csv_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("model", nargs="?", default=os.path.join(ROOT, "firmware", "models", "model.bin"))
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--partitions", default=os.path.join(ROOT, "firmware", "partitions.csv"))
    a = ap.parse_args()

    offset, size = model_partition(a.partitions)
    data = open(a.model, "rb").read(16)
    if data[:4] not in (b"R3DM", b"R3DP"):
        sys.exit(f"{a.model} is not a model or model pack (run tools/model_convert.py / pack_models.py)")
    n = os.path.getsize(a.model)
    if n > size:
        sys.exit(f"model is {n} bytes but the partition holds only {size}")
    print(f"writing {a.model} ({n / 1024:.0f} KiB) to 0x{offset:X} on {a.port}")
    cmd = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "-p", a.port, "-b", str(a.baud),
           "write_flash", hex(offset), a.model]
    sys.exit(subprocess.call(cmd))


if __name__ == "__main__":
    main()
