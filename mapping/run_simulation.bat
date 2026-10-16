@echo off
chcp 65001 >nul
cd /d "%~dp0"
python -X utf8 main.py test
if errorlevel 1 goto finish
python -X utf8 simulate.py
if errorlevel 1 goto finish
echo 仿真成功，请用浏览器打开 simulation_results\playback.html
:finish
pause
