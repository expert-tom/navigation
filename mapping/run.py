"""Local dependency wrapper; works after terminal close/reboot without export."""
from pathlib import Path
import os
import runpy
import sys

ROOT=Path(__file__).resolve().parent
os.environ.setdefault("OPENBLAS_NUM_THREADS","1")
sys.path.insert(0,str(ROOT/".deps/python310"))
if len(sys.argv)<2:
    raise SystemExit("Usage: python3 run.py SCRIPT.py [arguments]")
script=(ROOT/sys.argv[1]).resolve()
if script.parent!=ROOT or script.suffix!=".py" or not script.is_file():
    raise SystemExit("Choose a Python entry point in this project")
sys.argv=[str(script)]+sys.argv[2:]
runpy.run_path(str(script),run_name="__main__")
