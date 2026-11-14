# Validação — 13/11/2026

Validação executada numa cópia separada do projeto, no PC. Fontes novos compilados com C99, `-Wall -Wextra -Wpedantic -Werror`.

## Resultados

| Verificação | Resultado |
|---|---|
| `test_core.c`, `test_imu.c`, `test_review.c` existentes | Passaram, sem alterar os testes/fontes antigos |
| `test_core3.c` | Zero falhas: calibração, IMU, motores, referências, câmara, histórico, payloads e CRC |
| `test_core3_more.c` | Passou: configuração, magnetómetro tardio, ganhos, Ki=0, heading antigo com referencial inclinado, tilt e feedback não finito |
| `test_core3_sil.py` | 8 testes passaram com dinâmica acoplada de três rodas |
| `test_proto3_python.py` | 2 testes passaram: bytes C/Python, unidades e rejeição de entradas inválidas |
| `sim/test_protocol.py` existente | 20 verificações passaram |
| `core/tests/test_aim_sil.py` existente | AIM +10° e −10° passaram |
| `sim/test_three_axis.py` existente | 2 testes passaram, incluindo os casos de pontaria da aplicação anterior |
| Compilação cruzada ESP32-S3 | Os 6 módulos C e o exemplo `core3_adapter.c` compilaram sem avisos/erros |

A compilação cruzada gera objetos; não é uma compilação/link completo do firmware com três drivers nem um teste na placa. O Makefile foi atualizado; a execução completa acima utilizou o construtor Python com GCC. Não foram corridas análises de tempo de execução/stack na placa nem validada a temporização do CAN com três motores.

## Cobertura do contrato IMU

Os ensaios C verificam reinício da janela de calibração, movimento durante CALIB, falhas em RATE/HEADING/TRACK, congelamento de orientação/integral, primeira amostra de recuperação sem integração, fronteira de 50000/50001 µs, bloqueio persistente, WHEEL durante bloqueio, RECAL que só limpa o bloqueio ao concluir, timestamps repetidos e gyro não finito. Também verificam independência entre a atualização do estimador e a disponibilidade do motor.

Os testes de câmara verificam aceitação/rejeição, duplicados, capturas futuras, transformação com orientação da captura e conservação da idade da última deteção aceite através de RECAL. O ensaio físico inclui atraso de imagem, oclusão, procura após perda prolongada e reaquisição.

## Planta 3D dos testes novos

Implementação Python independente da matemática de rotação C. Passo de 5 ms, inércia diagonal do conjunto com rotores bloqueados `[0,004; 0,005; 0,006]` kg·m² e inércias axiais das rodas `[0,00004; 0,00005; 0,00006]` kg·m². Modelo:

```text
J * omega_dot = -Jw * wheel_dot - omega × (J*omega + Jw*wheel)
torque_motor = Jw * (wheel_dot + omega_dot)
```

As velocidades das rodas são relativas ao corpo. O modelo do servo calcula binário pelo erro de velocidade, com constante de tempo nominal de 60 ms e saturação pelo binário/aceleração configurados no core. As acelerações do corpo e das rodas são resolvidas em conjunto. Quando desativado, o motor tem binário zero: a velocidade relativa da roda pode mudar se o corpo acelerar. Um teste específico verifica a roda livre e a conservação aproximada do momento angular inercial. Não se reiniciam artificialmente as rodas para remover momento angular. A imagem usa a orientação física do corpo; a câmara entrega medições a 20 Hz com 80 ms de atraso.

Resultados representativos após 18 s, com sensores ideais e bias constante conhecido apenas através da calibração:

| Orientação inicial, graus | Referência relativa, graus | Erro angular final | Pico das rodas X/Y/Z, rpm |
|---|---|---|---|
| 0, 0, 0 | 20, −15, 35 | 0,0004° | 580 / 238 / 634 |
| 10, 15, −20 | −25, 20, −40 | 0,0005° | 468 / 607 / 623 |
| 0, 0, 0 | 30, 70, 100 | 0,0012° | 647 / 663 / 667 |

O critério do teste é erro inferior a 1°, não atingir estes valores minúsculos. São resultados de uma planta ideal, não precisão prevista do hardware. Após um impulso angular de `[0,6; −0,7; 0,9]` rad/s, o erro de pontaria foi cerca de 0,079° após 4 s. A referência AIM de horizontal −7°/vertical +5° convergiu dentro de 1° de cada valor. A procura após oclusão de 6,5 s entrou em SEARCH e voltou a LOCKED após a imagem regressar.

O teste de propagação inclui orientação inicial com pitch 80° e velocidade angular simultânea XYZ. Há ainda teste da falha de um motor e da recuperação curta da IMU em movimento. Os tamanhos das interfaces de medições/saídas são comparados entre C e ctypes; o estado do novo core ocupa 2560 bytes neste build, sem workspace do detector.

## O que estes ensaios não demonstram

Não validam motores reais, sinais/IDs CAN, inércias reais, folgas, fricção, vibração, ruído e deriva dos sensores, distorção ótica, limites do gimbal ou descarregamento de momento angular. Os testes físicos novos usam gravidade/magnetómetro ideais e não modelam aceleração de translação no acelerómetro. Não constituem prova de estabilidade para todas as combinações possíveis de ganhos e orientações.

O simulador e a interface anteriores continuaram a executar o núcleo antigo. Os testes novos exercitam o núcleo 3D à parte. A integração gráfica, do firmware e da estação de terra ainda tem de ser realizada e validada pelos respetivos responsáveis.

## Referências de matemática

As convenções foram explicitadas no header e testadas com cálculos Python independentes. Para rever a propagação de orientação, ver [AHRS — Angular Rate](https://ahrs.readthedocs.io/en/stable/filters/angular.html); para o princípio de combinar gyro com referências de gravidade/campo magnético, ver [AHRS — Complementary Filter](https://ahrs.readthedocs.io/en/stable/filters/complementary.html). O código entregue é uma implementação própria; não integra essas bibliotecas nem reivindica equivalência a um filtro de Kalman.
