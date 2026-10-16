"""Offline, project-local wheel extraction. No sudo, pip, venv or .bashrc edits."""
import hashlib
import json
from pathlib import Path
import platform
import sys
import zipfile

ROOT=Path(__file__).resolve().parent


def main():
    if platform.system()!="Linux" or platform.machine() not in ("x86_64","AMD64") or sys.version_info[:2]!=(3,10):
        raise SystemExit("Bundled native wheels are ONLY for Ubuntu22.04 x86_64 CPython3.10. Other environments: install requirements.txt with your own interpreter.")
    target=ROOT/".deps/python310"
    sums=json.loads((ROOT/"vendor/checksums.json").read_text())
    wheels=[ROOT/"vendor"/name for name in sums]
    for wheel in wheels:
        if hashlib.sha256(wheel.read_bytes()).hexdigest()!=sums[str(wheel.relative_to(ROOT/"vendor")).replace("\\","/")]:
            raise SystemExit("Wheel checksum mismatch: "+str(wheel))
    target.mkdir(parents=True,exist_ok=True)
    for wheel in wheels:
        with zipfile.ZipFile(wheel) as archive:
            for name in archive.namelist():
                resolved=(target/name).resolve()
                if not resolved.is_relative_to(target.resolve()):
                    raise SystemExit("Unsafe archive entry")
            archive.extractall(target)
    sys.path.insert(0,str(target))
    import numpy,serial,yaml
    print("Offline local dependencies ready:",numpy.__version__,serial.__version__,yaml.__version__)
    print("Always launch with python3 run.py SCRIPT.py ...; no temporary PYTHONPATH needed.")


if __name__=="__main__":
    main()
