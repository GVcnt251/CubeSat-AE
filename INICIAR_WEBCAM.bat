@echo off
cd /d "%~dp0"
python -c "import cv2" >nul 2>&1
if errorlevel 1 (
    echo A instalar o suporte de webcam na primeira utilizacao...
    python -m pip install -r requirements-webcam.txt
    if errorlevel 1 (
        pause
        exit /b 1
    )
)
python iniciar_simulacao.py --webcam
if errorlevel 1 pause
