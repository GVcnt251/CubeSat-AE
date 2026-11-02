@echo off
cd /d "%~dp0"
python iniciar_simulacao.py %*
if errorlevel 1 pause
