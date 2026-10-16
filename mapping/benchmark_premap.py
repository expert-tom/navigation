"""Offline cloud benchmark + test report. Not a vehicle performance certificate."""
import argparse
import ast
import contextlib
import io
import json
from pathlib import Path
import platform
import sys
import time
import unittest

import numpy as np
from premap_core import vector_obstacles

ROOT=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    output=io.StringIO()
    started=time.monotonic()
    result=unittest.TextTestRunner(stream=output,verbosity=2).run(unittest.defaultTestLoader.discover(str(ROOT)))
    report=dict(date="2026-10-15",platform=platform.platform(),python=sys.version,numpy=np.__version__,
        tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        test_seconds=time.monotonic()-started,ros2_runtime_tested=False,hardware_tested=False,
        note="Desktop synthetic benchmarks only. Target NUC timings / TF / sensor / stop response require field verification.",cloud_benchmarks=[])
    sources=list(ROOT.glob("*.py"))
    for path in sources:
        ast.parse(path.read_text(encoding="utf-8"),filename=path.name,feature_version=(3,10))
    report["python310_syntax_checked_files"]=len(sources)
    import hashlib
    sums=json.loads((ROOT/"vendor/checksums.json").read_text())
    report["bundled_dependency_hashes_verified"]=all(
        hashlib.sha256((ROOT/"vendor"/name).read_bytes()).hexdigest()==value for name,value in sums.items())
    settings=json.loads((ROOT/"config/premapped.json").read_text())["cloud"]
    rng=np.random.default_rng(22)
    for size in (20000,50000,300000):
        points=rng.uniform([-3,-3,-.1],[3,3,1.3],(size,3))
        durations=[]
        for _ in range(5):
            stamp=time.perf_counter()
            obstacles=vector_obstacles(points,(0,0,0),np.eye(3),(0,0,0),np.eye(3),(0,0,0),settings,[-3,3,-3,3])
            durations.append(time.perf_counter()-stamp)
        report["cloud_benchmarks"].append(dict(points=size,obstacles=len(obstacles),
            median_seconds=float(np.median(durations)),worst_seconds=max(durations)))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    args.output.with_suffix(".tests.txt").write_text(output.getvalue(),encoding="utf-8")
    print(json.dumps(report,ensure_ascii=False,indent=2))
    if not result.wasSuccessful(): raise SystemExit(1)


if __name__=="__main__":
    main()
