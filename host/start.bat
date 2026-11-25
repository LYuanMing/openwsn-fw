@echo off
rem Launch the topology / voltage monitor with the project venv
set PY=E:/Project/6TiSCH/OpenWSN_serial_parser/venv/Scripts/python.exe
if not exist "%PY%" set PY=python
cd /d %~dp0
"%PY%" wsn_gui.py %*
if errorlevel 1 pause