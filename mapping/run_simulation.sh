#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")"
python3 main.py test
python3 simulate.py
printf '%s\n' '仿真成功，请用浏览器打开 simulation_results/playback.html'
