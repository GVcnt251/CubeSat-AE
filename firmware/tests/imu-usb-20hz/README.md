# Ensaio USB de atitude, vídeo e rodas

Este projeto ESP-IDF guarda o firmware de diagnóstico ensaiado com a Ground Station desta branch. A versão do ensaio envia atitude a 20 Hz e JPEG de 160 × 120 por USB. Para as rodas, faz pedidos de telemetria e regista respostas, falhas e contadores dos controladores CAN.

| Eixo | Barramento | GPIO do chip select | ID moteus |
| --- | --- | --- | --- |
| X | CAN0 | 2 | 1 |
| Y | CAN1 | 3 | 1 |
| Z | CAN2 | 4 | 1 |

Os três MCP2518FD usam CAN-FD a 1 Mbit/s na arbitragem e 5 Mbit/s nos dados. No ensaio registado, os controladores inicializaram e nenhum dos três eixos devolveu uma resposta válida. O firmware envia apenas pedidos de leitura às rodas.

## Compilar a partir da raiz do repositório

Com o ambiente ESP-IDF v6.1 ativo, executar:

```powershell
idf.py -C firmware/tests/imu-usb-20hz -D IDF_TARGET=esp32s3 -D BENCH_CAN_TELEMETRY=ON -D BENCH_CAN_DIAGNOSTICS=ON -D BENCH_USB_CAMERA=ON build
```

Na Ground Station, selecionar **CubeSat · USB** em **Operação → Sessão** e escolher a porta série. Os dados USB adicionais requerem o firmware de diagnóstico em execução.
