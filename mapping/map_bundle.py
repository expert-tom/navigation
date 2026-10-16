"""Read map_saver's PGM/YAML pair and bind the mission to its exact content."""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess

import numpy as np
from premap_core import file_digest, map_digest


def read_pgm(path):
    raw = Path(path).read_bytes()
    # Tokenize only the header: binary pixels may contain '#' or whitespace.
    tokens, offset = [], 0
    while len(tokens) < 4:
        match = re.match(rb"(?:\s+|#[^\r\n]*(?:\r?\n|$))*([^\s#]+)", raw[offset:])
        if not match:
            raise ValueError("invalid PGM header")
        tokens.append(match.group(1))
        offset += match.end()
    if raw[offset:offset+2] == b"\r\n":
        offset += 2
    elif raw[offset:offset+1] in (b" ", b"\n", b"\r", b"\t"):
        offset += 1
    else:
        raise ValueError("PGM lacks raster delimiter")
    magic, width, height, maximum = tokens[0], *map(int, tokens[1:])
    if magic != b"P5" or maximum != 255 or width <= 0 or height <= 0 or width*height > 16000000:
        raise ValueError("require 8-bit binary P5 PGM, <=16M pixels")
    if len(raw)-offset != width*height:
        raise ValueError("PGM raster length mismatch")
    return np.frombuffer(raw, dtype=np.uint8, offset=offset).reshape(height, width)


def load_map(yaml_path):
    import yaml  # ROS Humble dependency; no arbitrary YAML constructors.
    path = Path(yaml_path).resolve()
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    image = (path.parent/config["image"]).resolve()
    if image.suffix.lower() != ".pgm" or config.get("mode", "trinary") != "trinary":
        raise ValueError("save with --fmt pgm --mode trinary")
    pixels = read_pgm(image)
    probability = pixels.astype(float)/255
    if not int(config.get("negate", 0)):
        probability = 1-probability
    occupied, free = float(config["occupied_thresh"]), float(config["free_thresh"])
    origin, resolution = config["origin"], float(config["resolution"])
    if not 0 < free < occupied < 1 or len(origin) != 3 or resolution <= 0 or not np.isfinite([*origin,resolution]).all():
        raise ValueError("invalid map geometry or thresholds")
    if abs(float(origin[2])) > 1e-8:
        raise ValueError("Humble AMCL requires this workflow's axis-aligned map (origin yaw=0); re-export, do not silently ignore rotation")
    data = np.full(pixels.shape, -1, dtype=np.int8)
    data[probability > occupied] = 100
    data[probability < free] = 0
    data = np.flipud(data).reshape(-1)
    height, width = pixels.shape
    return dict(yaml_path=str(path), image_path=str(image), width=width, height=height,
                resolution=resolution, origin=origin, data=data,
                yaml_sha256=file_digest(path), image_sha256=file_digest(image),
                map_digest=map_digest(data,width,height,resolution,origin))


def validate_bundle(path):
    path = Path(path).resolve()
    manifest = json.loads(path.read_text(encoding="utf-8"))
    grid = load_map(path.parent/manifest["map_yaml"])
    for name in ("yaml_sha256", "image_sha256", "map_digest"):
        if grid[name] != manifest[name]:
            raise ValueError("map bundle changed: "+name+"; recapture field coordinates and regenerate manifest")
    return manifest, grid


def main():
    parser = argparse.ArgumentParser(description="Save 2D navigation map + content manifest; never opens serial")
    parser.add_argument("--output", type=Path, required=True, help="new map prefix, e.g. maps/classroom")
    args = parser.parse_args()
    prefix = args.output.resolve()
    if prefix.suffix:
        parser.error("--output is a map prefix without extension")
    prefix.parent.mkdir(parents=True, exist_ok=True)
    for suffix in (".yaml", ".pgm", ".manifest.json"):
        if prefix.with_suffix(suffix).exists():
            parser.error("refuse to overwrite existing map: "+str(prefix.with_suffix(suffix)))
    subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(prefix), "-t", "/race/map",
                    "--fmt", "pgm", "--mode", "trinary", "--occ", "0.65", "--free", "0.196",
                    "--ros-args", "-p", "save_map_timeout:=10.0"], check=True)
    grid = load_map(prefix.with_suffix(".yaml"))
    manifest = {key:grid[key] for key in ("yaml_sha256","image_sha256","map_digest")}
    manifest.update(schema=1, map_yaml=prefix.with_suffix(".yaml").name, map_frame="race_map",
                    description="2D occupancy map derived from LiDAR. Not a PCD localization map.")
    out = prefix.with_suffix(".manifest.json")
    with out.open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
    print("Saved map bundle:", out)


if __name__ == "__main__":
    main()
