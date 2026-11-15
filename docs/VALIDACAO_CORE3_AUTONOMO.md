# Validação da consolidação — 15/11/2026

O objetivo desta alteração é retirar a dependência do controlador de uma roda sem mudar o comportamento do controlador de três rodas.

## Resultados

| Verificação | Resultado |
|---|---|
| Compilação com apenas cinco fontes e sem cube_core.c/.h | Passou |
| test_common.c: protocolo, CRC, payloads, imagens, detetor e moteus | Passou |
| test_core3.c: IMU, calibração, recuperação, modos, motores, câmara e protocolo 3D | Zero falhas |
| test_core3_more.c: configuração, ganhos, magnetómetro tardio, compatibilidade de heading, tilt e Ki=0 | Passou |
| test_standalone.py: API nova presente e funções do controlador antigo ausentes | Passou |
| test_core3_sil.py: dinâmica acoplada, três eixos, impulso externo, IMU/motor, câmara com atraso, procura e roda livre | 8 testes passaram |
| test_proto3_python.py: payloads C/Python e entradas inválidas | 2 testes passaram |
| Reconstrução e testes pelo Makefile | Passou com make -B CC=gcc test |
| Headers/exemplo em C++11 | Passou |
| Cinco módulos e exemplo com compilador ESP32-S3 | Passou |

Compilação C com C99, -O2, -Wall, -Wextra, -Wpedantic e -Werror. A verificação ESP32-S3 compila objetos; não é um link do firmware completo nem um ensaio na placa.

## Comparação com a versão anterior

Foi compilada uma cópia intacta da implementação 3D anterior, que ainda ligava o controlador escalar por causa das dependências comuns. Essa cópia foi usada somente na validação, fora da pasta entregue.

Ao fornecer as mesmas entradas às duas versões durante 4 000 ciclos, foram comparados byte a byte:

- Estado completo do core, incluindo configurações e histórico.
- Códigos de estado e saídas das três rodas.
- Resultados das chamadas de deteção.
- Payload TM_STATE3.

As sequências incluem calibração, RATE, HEADING, TRACK, WHEEL, alterações de referências/ganhos, Ki=0, falhas curtas, interrupção longa, RECAL, indisponibilidade de motor e entradas NaN. Todos os resultados comparados foram idênticos.

Também foram comparadas 135 combinações de identificador/valor para a validação de ganhos, incluindo zero, negativos, NaN, infinito e magnitudes extremas. Os resultados foram iguais.

Os tamanhos das estruturas do GCC do PC foram preservados: cc3_t 2560, cc3_meas_t 68, cc3_out_t 48, cc3_tlm_t 160 bytes. O relatório JSON acompanha esta entrega. Não é uma garantia universal de equivalência para todas as entradas possíveis, mas é uma verificação direta de que o refactor preserva os cenários exercitados.

## Alcance

Nenhum ficheiro do repositório original foi editado. O snapshot dos fontes relevantes foi comparado por SHA-256 antes de empacotar. Não foram publicados commits, releases ou mensagens a colegas.

Esta versão não migra os drivers, o firmware, o GUI nem o wrapper de uma roda. Não acrescenta descarga de momento angular, busca esférica ou adaptação do bias durante funcionamento. A validação mecânica, os sinais/IDs reais, a afinação dos ganhos e a aprovação da extensão TMTC continuam pendentes como na versão anterior.
