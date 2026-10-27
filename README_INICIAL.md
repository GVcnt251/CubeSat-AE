# CubeSat-AE — Sistema de Controlo de Atitude e Apontamento

Projeto de Aviónica e Espaciónica para o desenvolvimento e validação do sistema de controlo de atitude de um Cubesat com roda de inércia e deteção visual de alvos.

## 🚀 Arquitetura do Sistema
* **Core (C99):** Filtro de Kalman, Máquina de Estados e Controlador em Cascata sem dependências de hardware.
* **Firmware (ESP-IDF / FreeRTOS):** Implementação multitarefa para Seeed XIAO ESP32-S3 Sense.
* **Ground Station (PyQt5):** Telemetria em tempo real, envio de telecomandos e streaming de vídeo via UDP.
* **Simulador SIL (Python/ctypes):** Ambiente de simulação física realista para validação em PC.

## 🛠️ Como Executar a Simulação (SIL)
1. Compilar o núcleo C no MSYS2 MINGW64: `make`
2. Iniciar o servidor de simulação: `python sim/sim_server.py`
3. Iniciar a Estação de Terra: `python groundstation/gs_app.py`