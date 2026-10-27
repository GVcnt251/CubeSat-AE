# Cubo com rodas de reacção — projecto completo (simulador → firmware)

Implementação do guião *"A cube that learns to point on its own"* (Avionics and Spacionics, DETI-UA 2026/27):
controlo de atitude de 1 eixo (guinada) com roda de reacção, guiado por visão.

**Regra do guião, cumprida à letra:** o código de controlo é **um só** (`core/`, C portável, sem hardware). O simulador
corre-o por *ctypes* (SIL) e o firmware ESP-IDF compila exactamente os mesmos ficheiros. Só muda quem fornece os dados.

## Estado real (lê isto primeiro)

| Parte | Estado |
|---|---|
| `core/` — estimador, controlador em cascata, detector do ponto, protocolo, codec moteus | ✅ escrito e **verificado em simulação** (tabela abaixo) |
| `sim/` — simulador (física, atrito, IMU, magnetómetro, câmara sintética) + testes dos níveis 1–6 | ✅ a correr |
| `groundstation/` — GUI PyQt5 + CLI + protocolo | ✅ testada contra o cubo virtual (UDP) |
| `docs/TMTC_ICD.md` | ✅ |
| `firmware/` — tarefas FreeRTOS, drivers MPU-6050 / LIS3MDL / câmara / Wi-Fi / moteus | ⚠️ **escrito mas nunca compilado nem corrido em hardware** (só verificação sintática em PC com cabeçalhos *stub*) |
| Driver **MCP2518FD** (SPI→CAN-FD) | ❌ **não implementado** (esqueleto em `firmware/main/can_mcp2518fd.c`); `CONFIG_CUBE_WHEEL_FAKE=y` permite testar o resto sem motor |
| PIL (firmware num ESP32 real ligado ao simulador) | ❌ não feito |
| Hardware (nível 6 real: atrito, luz, vibração) | ❌ nada foi medido; os números abaixo são do **simulador** |

## Resultados em SIL (`python sim/run_levels.py`)

Medidos contra a **verdade** do simulador, não contra a estimativa do cubo. O simulador inclui: atrito de Coulomb+viscoso nos
rolamentos do gimbal, binário/aceleração limitados, ruído e bias do giroscópio com **deriva térmica**, magnetómetro com
*hard-iron* residual, câmara a 20 fps com ~80 ms de latência, luz variável, sombras em movimento, ruído de imagem, barra
distractora no poster e empurrões de mão.

| Nível | Teste | Critério | Medido (SIL) | |
|---|---|---|---|---|
| 1 | Make it spin | 3000 rpm em <0.5 s, erro <2% | entra em ±2% em 0.40 s; erro estacionário 0.00% | ✅ |
| 2 | Know where I am | erro <2° e deriva <2°/min | erro máx 0.98°, deriva -0.17°/min (só gyro: erro máx 8.5°, deriva +2.95°/min) | ✅ |
| 3 | Turn and stop | degrau 45°, assenta em ±1° (3 sementes) | assenta em 1.7s, 1.8s, 1.7s; sobrelevação máx 0.53°; erro final máx 0.48° | ✅ |
| 4 | See the point | acerta em >98% das frames (erro <2 px) | 100.0% de acertos, 0 falsos positivos, 0/50 com câmara tapada; erro p95 0.07 px (0.02°) | ✅ |
| 4 | See the point | >=15 frames/s (a confirmar no ESP32-S3 - PIL) | PC: 0.08 ms/frame (máx 0.18 ms); câmara a 20 fps | ✅ |
| 5 | Point on its own | de 30° a centrar (±2°) em <6 s (4 sementes) | 1.14s, 1.15s, 1.14s, 1.15s | ✅ |
| 6 | 1 Acquisition | centra (±2°) em <6 s | 1.15 s | ✅ |
| 6 | 2 Hold | nunca sai de ±2° em 1 min | 65 s a partir do 1.º cruzamento: erro máx 1.97°; regime estacionário (após +2 s) 0.19° | ✅ |
| 6 | 3 Disturbance | empurrão ~20° -> recupera em <4 s | 1.08 s | ✅ |
| 6 | 4 Moving target | segue-o sem o perder (3°/s, 24°) | erro máx 2.14°, 0 amostras em SEARCH, erro final 0.13° | ✅ |
| 6 | 5 Occlusion | 2 s tapada: não entra em pânico e retoma | erro máx 0.04°, 0 amostras em SEARCH, re-lock em 0.09 s | ✅ |
| 6 | 5b (extra) Occlusion in recovery | empurrão de 20° e câmara tapada 2 s logo a seguir | 0 amostras em SEARCH; volta a ±2° (parcialmente às cegas) em 1.11 s e, 1 s depois, fica com erro máx 0.54° | ✅ |

Gráficos em `results/level1.png … level6.png`. Captura da estação de terra: `results/gs_screenshot.png`.

## Arranque rápido

```bash
pip install -r requirements.txt
make -C core                      # MSYS2 MINGW64: gera libcubecore.dll;  Linux/macOS: libcubecore.so
python sim/test_protocol.py       # C <-> Python: 20 verificações
python sim/run_levels.py          # critérios dos níveis 1-6  (python sim/run_levels.py 3 6 para só alguns)

# estação de terra com o cubo virtual
python sim/sim_server.py --port 5000 --psi0 -30        # terminal 1
python groundstation/gs_app.py --port 5000             # terminal 2: IDLE -> TRACK, empurrões, tapar câmara...
python groundstation/gs_cli.py --port 5000 --mode track --seconds 10   # versão sem Qt
```

## Estrutura

```
core/            C portável (SIL e ESP32)
  include/cube_core.h     API: cc_step / cc_on_mag / cc_on_detection / cc_command / cc_get_tlm
  src/cube_core.c         estimador (KF 3 estados), cascata de controlo, máquina de seguimento
  src/target_detect.c     detector do ponto (histograma, componentes ligados, forma, contraste, sub-pixel)
  src/cc_proto.c          tramas TMTC, CRC, parser; src/moteus_proto.c  tramas CAN-FD do moteus
sim/             cube_sim.py (mundo), sil.py (ctypes + runner), run_levels.py, test_protocol.py, sim_server.py
groundstation/   gs_app.py (GUI), gs_cli.py, gs_link.py (UDP/série), gs_protocol.py
firmware/        projecto ESP-IDF (main/ + components/cube_core -> ../core)
docs/TMTC_ICD.md
```

## Como funciona

**Cascata** (slide "shopping list"): `atitude → velocidade angular → velocidade da roda`.

1. `ω_ref = sat(k_θ · erro)`, com limite de aceleração da referência.
2. Malha de velocidade angular PI: `Ω_cmd = −(Kp·e_ω + Ki·∫e_ω)`. Como a roda e o corpo trocam momento angular, um corpo
   mais pesado que a roda (`J_b/J_w ≈ 16` no simulador) precisa de ΔΩ ≈ 16·Δω; o `Ki` define a largura de banda
   (`≈ a·Ki/(1+a·Kp)`, `a = J_w/(J_b+J_w)`). Integração condicional (anti-*windup*) e transição sem salto entre modos.
3. O **moteus** fecha a malha interior de velocidade da roda; o núcleo só envia `Ω_cmd`, limite de binário e de aceleração.

**Percepção** ("a câmara corrige a deriva do IMU, o IMU cobre os intervalos entre frames"): um filtro de Kalman de 3 estados
`[ψ, bias do giroscópio, ψ_alvo]`. O giroscópio propaga a 200 Hz; o magnetómetro (compensado da inclinação) mede `ψ`; a
câmara mede `ψ_alvo − ψ`. **A imagem chega ~80 ms atrasada:** o filtro guarda o histórico das últimas 320 ms e calcula a
inovação contra o estado *no instante da captura*; a correcção é propagada retroactivamente ao histórico para a frame
seguinte não a contar duas vezes. Medidas fora de ±17° são rejeitadas (5 seguidas ⇒ reinicializa).

**Seguimento** (`CC_MODE_TRACK`): `LOCKED` (medida recente) → `COAST` (alvo perdido <5 s: continua a apontar com o IMU) →
`SEARCH` (varre a 20°/s). É isto que faz a oclusão de 2 s passar sem "pânico".

**Detector**: fundo = percentil 90 do histograma; limiar = 0.3·fundo; componentes ligados (pilha explícita, sem *malloc*);
critérios de área, preenchimento (≈π/4), razão de aspecto, anel envolvente limpo e contraste absoluto e relativo;
centroide sub-pixel. Rejeita a barra distractora e sombras. Nenhuma alocação dinâmica; ~33 kB de *workspace*.

## Pressupostos que tive de assumir (o guião não os define)

* "Centrado" = dentro de ±2° (é o critério do teste *Hold*). O degrau do nível 3 tem de assentar em <5 s (o guião só dá ±1°).
* O critério do nível 1 está marcado *TBC* no guião; usei-o tal e qual. Para chegar a 3000 rpm em <0.5 s o limite de binário
  por omissão é 0.20 N·m (8 A com Kt = 8.27/330).
* Parâmetros do simulador (`SimParams`): J_corpo = 4·10⁻³, J_roda = 2.5·10⁻⁴ kg·m², atrito de Coulomb dos rolamentos
  1.5·10⁻³ N·m, ponto de 20 px de diâmetro, f = 249 px (deduzido do slide: 83.9 px ↔ 18.6°). **Têm de ser medidos no
  vosso cubo** — o desempenho real vai depender muito do atrito e do balanceamento.
* Eixos dos sensores: `IMU_MAP`/`MAG_MAP` em `firmware/main/board.h` assumem o chip a plano com z para cima.

## O que falta fazer / riscos conhecidos

1. **Compilar o firmware** (`idf.py set-target esp32s3 && idf.py menuconfig` (Cubo → Wi-Fi) `&& idf.py build flash monitor`).
   Verificar versões das APIs do ESP-IDF 5.x e do `esp32-camera` (o *stub* só apanha erros de escrita, não de API).
2. **Driver MCP2518FD** (SPI→CAN-FD): ver `can_mcp2518fd.c`. Depois pôr `CONFIG_CUBE_WHEEL_FAKE=n` e afinar o PID de
   velocidade do próprio moteus (`moteus_tool`), `servo.max_position_slip` e o *watchdog*.
3. **Sinal da roda e dos eixos** (`CONFIG_CUBE_WHEEL_SIGN`, `IMU_MAP`): errado = realimentação positiva. Testar primeiro com o
   cubo seguro na mão e a roda a baixa velocidade.
4. **Latência da câmara**: `CONFIG_CUBE_CAM_LATENCY_MS = 80` é um palpite; medir (LED a piscar) e ajustar.
5. **Tempo do detector no ESP32-S3**: no PC são ~0.1 ms/frame; o critério ≥15 fps tem de ser confirmado no alvo (PIL).
6. **Magnetómetro**: não há calibração *hard-/soft-iron* (precisa de rodar o cubo 360°), e o motor brushless de 20 A perto
   do sensor pode inutilizá-lo. Com `use_mag=0` o rumo fica só com giroscópio (+ câmara, no modo TRACK).
7. **Saturação de momento**: a roda acumula momento (ver nível 6: ±1000 rpm após empurrões). Não há *desaturation*;
   o atrito do gimbal vai dissipando, mas várias perturbações seguidas podem encostar a roda ao limite (flag `RODA SAT`).
8. Roll/pitch: só são medidos (compensação do magnetómetro e *flag* de inclinação); **não há amortecimento activo**,
   como o guião prevê com uma só roda.
9. Windows/MSYS2: o `Makefile` gera a `.dll` mas só testei em Linux.

## Mapeamento com a "How to start – Sequence" do guião

| Passo do guião | Onde |
|---|---|
| Datasheets, I²C, CAN-FD | `firmware/main/imu.c`, `core/src/moteus_proto.c` |
| TMTC ICD | `docs/TMTC_ICD.md` |
| Firmware: drivers/handlers/aplicações, descrição | `firmware/main/` (tarefas em `app_main.c`) |
| Mockup + estação de terra com dados simulados | `groundstation/` + `sim/sim_server.py` |
| Fases 1–3 (IMU/mag → motor → controlo autónomo) | níveis 1–3 em `sim/run_levels.py` |
| Fases 4–6 (imagem → detecção → centrar o ponto) | `target_detect.c`, níveis 4–5 |
| Fase 7 (ponto fora do centro por setpoint) | `TC_SET_AIM` / campo "Pontaria" da GUI |
