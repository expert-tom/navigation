"""Reproducible file selection; no private logs, caches, desktop wheels or maps."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parent


def files():
    selected=list(ROOT.glob("*.py"))
    selected += [ROOT/name for name in ("README.md","AGENTS.md","requirements.txt","config.json","viewer_template.html")]
    for directory in ("assets","config","references","reports"):
        selected.extend(p for p in (ROOT/directory).rglob("*") if p.is_file() and "__pycache__" not in p.parts)
    selected.extend((ROOT/"docs").glob("07_*.md"))
    selected.extend((ROOT/"docs").glob("08_*.md"))
    selected.extend([ROOT/"vendor/pyserial-3.5-py2.py3-none-any.whl",ROOT/"vendor/checksums.json"])
    selected.extend((ROOT/"vendor/linux_py310").glob("*.whl"))
    return sorted(set(selected))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    selected=files()
    inventory={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in selected}
    manifest=dict(version="premapped-race-20261015",source_files_sha256=inventory,
        hardware_verified=False,ros2_runtime_verified=False,
        notes="New strict entry premap_vehicle.py. No real map, measured field or confirmed calibration is bundled.")
    with zipfile.ZipFile(args.output,"x",compression=zipfile.ZIP_DEFLATED) as archive:
        for path in selected:
            archive.write(path,"navigation_starter/"+path.relative_to(ROOT).as_posix())
        archive.writestr("navigation_starter/release_manifest.json",json.dumps(manifest,ensure_ascii=False,indent=2))
    print(args.output.resolve())
    print("files:",len(selected)+1,"bytes:",args.output.stat().st_size)


if __name__=="__main__":
    main()
