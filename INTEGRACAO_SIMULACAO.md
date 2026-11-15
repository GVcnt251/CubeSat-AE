# Integração da simulação — 2026-11-02

Pasta de trabalho: C:\Users\joaoh\CubeSat-AE-simulacao
Branch: feature/sil-sim (acompanha origin/feature/sil-sim).
Base atualizada com main b0f1284 antes de integrar a versão 9 da simulação.

Abrir INICIAR_SIMULACAO.bat; para webcam, INICIAR_WEBCAM.bat.
O terminal ESP-IDF não é necessário para a simulação no computador.

As alterações recentes dos colegas ao core (validação, IMU, telemetria,
detetor e protocolos) foram preservadas através de integração a três versões.
As extensões de câmara, três eixos e descarga de momento da simulação foram
adicionadas ao mesmo core partilhado; a descarga fica desativada por omissão
e o simulador ativa-a para o gimbal com atrito. Firmware preservado da main.
O controlador continua a ser de um eixo, instanciado três vezes no simulador.

As alterações estão locais nesta branch. Não foram publicadas no GitHub.
Não executar git pull descartando alterações locais; rever/guardar as alterações
antes de incorporar atualizações futuras dos colegas.
