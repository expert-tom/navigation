"""Offline least-squares calibration; does not send any motion command.

CSV columns: command_vx,command_vy,body_vx_mps,body_vy_mps.
Measure body velocity from stable localized poses, rotate world deltas by yaw.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def fit(rows):
    samples = np.asarray(rows,dtype=float)
    if samples.ndim != 2 or samples.shape[1] != 4 or len(samples)<6 or not np.isfinite(samples).all():
        raise ValueError("need >=6 finite, independent, positive/negative axis steady-motion samples")
    inputs,outputs = samples[:,:2],samples[:,2:]
    if np.linalg.matrix_rank(inputs)!=2 or np.linalg.cond(inputs)>20:
        raise ValueError("both independent axes must be measured")
    matrix = np.linalg.lstsq(inputs,outputs,rcond=None)[0].T
    if abs(np.linalg.det(matrix))<1e-6 or np.linalg.cond(matrix)>20:
        raise ValueError("response matrix singular/ill-conditioned; check control mode and dead zone")
    residual = float(np.sqrt(np.mean((inputs@matrix.T-outputs)**2)))
    scale = float(np.sqrt(np.mean(outputs**2)))
    if residual>max(.01,.2*scale):
        raise ValueError("linear model residual too large: characterize dead zone / speed range before racing")
    return dict(body_from_command=matrix.tolist(),rms_error_mps=residual,
        tested_command_peak=float(np.max(np.abs(inputs))),calibration_confirmed=False,
        note="Review signed axes, latency, low-command dead zone and stopping distance on vehicle before confirming. Do not extrapolate beyond tested commands.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv",type=Path,required=True)
    args = parser.parse_args()
    with args.csv.open(encoding="utf-8-sig",newline="") as stream:
        rows = [[float(row[key]) for key in ("command_vx","command_vy","body_vx_mps","body_vy_mps")]
                for row in csv.DictReader(stream)]
    print(json.dumps(fit(rows),ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
