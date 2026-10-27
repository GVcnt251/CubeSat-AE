# TMTC ICD — Estação de terra ↔ Cubo

Versão 1.0 · Interface Control Document do protocolo de telemetria e telecomando.
Implementação de referência: `core/src/cc_proto.c` (cubo/simulador, C) e `groundstation/gs_protocol.py` (estação de terra, Python).
As duas são verificadas uma contra a outra em `sim/test_protocol.py` (20 verificações).

## 1. Transporte

| | |
|---|---|
| Wi-Fi (por omissão) | UDP; o cubo escuta na porta `CONFIG_CUBE_GS_PORT` (5000). **1 trama por datagrama.** A estação de terra envia o primeiro TC (ex.: `PING`); o cubo aprende o endereço de origem e envia a telemetria para lá. |
| USB-série | Fluxo de bytes (921600 8N1). O *parser* ressincroniza pelo `SYNC`. |
| Ordem de bytes | *Little-endian*. `float` = IEEE-754 binário32. |
| Unidades na ligação | **graus, graus/s, rpm, N·m, V** (legíveis por humanos). O núcleo trabalha em SI/radianos; a conversão é feita em `cc_proto.c`. |

## 2. Trama

```
 0     1     2     3      4     5 .. 5+LEN-1   5+LEN  6+LEN
+-----+-----+-----+------+-----+--------------+------+------+
| A5  | 5A  | LEN | TYPE | SEQ |   PAYLOAD    | CRC_L| CRC_H|
+-----+-----+-----+------+-----+--------------+------+------+
```

* `LEN` — nº de bytes do payload (0..240).
* `SEQ` — contador de 8 bits do emissor (incrementa a cada trama enviada; o ACK devolve o `SEQ` do TC).
* `CRC` — CRC-16/CCITT-FALSE (poli 0x1021, init 0xFFFF, sem reflexão, sem XOR final) sobre `LEN, TYPE, SEQ, PAYLOAD`.
  Valor de verificação: `CRC("123456789") = 0x29B1`. Tramas com CRC errado são descartadas e contadas.
* Tamanho total = `LEN + 7`.

Exemplos reais (gerados pelo código):

| Trama | Bytes |
|---|---|
| `TC_PING`, seq 1 | `a5 5a 00 80 01 25 c7` |
| `TC_SET_MODE` = TRACK, seq 2 | `a5 5a 01 81 02 05 d9 c8` |
| `TC_SET_HEADING` = 45.0°, seq 3 | `a5 5a 04 84 03 00 00 34 42 8c ad` |
| `TC_SET_GAIN` k_theta = 4.5, seq 4 | `a5 5a 05 86 04 00 00 00 90 40 d2 25` |
| `TC_STREAM_CFG` 20/20/10, seq 5 | `a5 5a 03 87 05 14 14 0a ef c8` |

## 3. Telemetria (cubo → terra)

### 3.1 `TM_STATE` (0x01) — 52 bytes, `state_hz` (por omissão 20 Hz)

| Off. | Tipo | Campo | Unidade | Notas |
|---|---|---|---|---|
| 0 | u32 | `t_ms` | ms | relógio do cubo desde o arranque |
| 4 | u8 | `mode` | — | 0 CALIB, 1 IDLE, 2 WHEEL, 3 RATE, 4 HEADING, 5 TRACK |
| 5 | u8 | `trk` | — | 0 —, 1 LOCKED, 2 COAST (alvo perdido há pouco), 3 SEARCH |
| 6 | u16 | `flags` | bits | b0 alvo válido · b1 roda saturada · b2 magnetómetro OK · b3 calibrado · b4 inclinação > 10° |
| 8 | f32 | `heading` | ° | rumo estimado, relativo ao rumo no fim da calibração; positivo = para a direita |
| 12 | f32 | `rate` | °/s | velocidade de guinada (giroscópio − bias) |
| 16 | f32 | `bias` | °/s | bias do giroscópio estimado |
| 20 | f32 | `err` | ° | erro que o controlador está a usar (rumo: `sp − heading`; TRACK: alvo − pontaria) |
| 24 | f32 | `rate_ref` | °/s | referência da malha de velocidade angular |
| 28 | f32 | `wheel_rpm` | rpm | velocidade medida da roda (relativa ao corpo) |
| 32 | f32 | `wheel_cmd_rpm` | rpm | velocidade pedida ao moteus |
| 36 | f32 | `torque` | N·m | binário medido pelo driver |
| 40 | f32 | `vbat` | V | bateria |
| 44 | f32 | `target_err` | ° | estimativa do erro do alvo (sem desvio de pontaria) |
| 48 | f32 | `cam_age` | s | tempo desde a última medida de câmara aceite (−1 = nunca) |

### 3.2 `TM_VISION` (0x02) — 36 bytes, uma por imagem processada

| Off. | Tipo | Campo | Notas |
|---|---|---|---|
| 0 | u32 | `t_capture_ms` | instante de captura (já descontada a latência) |
| 4 | u8 | `found` | 1 se o detector encontrou o ponto |
| 5 | u8 | `accepted` | 1 se o estimador aceitou a medida (passou o *gate*) |
| 6 | u8 | `n_cand` | candidatos avaliados |
| 7 | u8 | — | 0 |
| 8 | f32 | `x` | px (0 = centro do 1.º pixel; centro óptico ≈ 159.5) |
| 12 | f32 | `y` | px |
| 16 | f32 | `err_x` | ° — bearing horizontal do ponto (+ = à direita do eixo óptico) |
| 20 | f32 | `err_y` | ° |
| 24 | f32 | `contrast` | níveis de cinzento (anel − interior) |
| 28 | u32 | `area` | px |
| 32 | u32 | `proc_us` | tempo de `cc_detect` no cubo |

### 3.3 `TM_IMG` (0x03) — até 205 bytes, 24 pacotes por imagem (`img_every` imagens)

`u8 img_id · u8 chunk (0..n−1) · u8 n_chunks (=24) · u8 w (=80) · u8 h (=60) · u8 data[≤200]`.
A imagem é a de 320×240 reduzida 4×4 por média (80×60, 4800 B), em ordem de linhas. A terra junta os *chunks* com o mesmo `img_id`.

### 3.4 `TM_ACK` (0x7F) — 3 bytes

`u8 type_acked · u8 seq_acked · u8 status` — `0` OK · `1` comprimento inválido · `2` comando desconhecido · `3` argumento inválido.

## 4. Telecomandos (terra → cubo)

| Tipo | Nome | Payload | Efeito |
|---|---|---|---|
| 0x80 | `TC_PING` | — | só ACK (e dá ao cubo o endereço da terra) |
| 0x81 | `TC_SET_MODE` | `u8 mode` (1..5) | muda de modo (ignorado durante CALIB) |
| 0x82 | `TC_SET_WHEEL` | `f32 rpm` | velocidade da roda (modo WHEEL) |
| 0x83 | `TC_SET_RATE` | `f32 °/s` | velocidade angular (modo RATE) |
| 0x84 | `TC_SET_HEADING` | `f32 °` | rumo relativo ao rumo de arranque (modo HEADING) |
| 0x85 | `TC_SET_AIM` | `f32 °` | desvio pretendido do alvo em relação ao centro (modo TRACK; passo 7 da sequência) |
| 0x86 | `TC_SET_GAIN` | `u8 id · f32 valor` | ver tabela 4.1 |
| 0x87 | `TC_STREAM_CFG` | `u8 state_hz · u8 vision_hz · u8 img_every` | 0 desliga; `img_every` = enviar 1 imagem a cada N |
| 0x88 | `TC_RECAL` | — | volta a CALIB (**cubo parado**: ~1.5 s) |

### 4.1 Ganhos (`TC_SET_GAIN`)

| id | Nome | Unidade no ICD | Por omissão | Significado |
|---|---|---|---|---|
| 0 | `k_theta` | 1/s | 3.0 | atitude → velocidade angular |
| 1 | `rate_kp` | — | 5.0 | malha de velocidade angular, P |
| 2 | `rate_ki` | — | 300 | malha de velocidade angular, I (define a largura de banda) |
| 3 | `w_ref_max` | °/s | 40.1 | limite da velocidade angular pedida |
| 4 | `a_ref_max` | °/s² | 229 | limite da variação da referência |
| 5 | `wheel_lim` | rpm | 3000 | limite da roda |
| 6 | `torque_max` | N·m | 0.20 | binário máximo enviado ao moteus |
| 7 | `r_cam` | ° (σ) | 0.4 | ruído de medida da câmara no filtro |
| 8 | `r_mag` | ° (σ) | 2.9 | ruído de medida do magnetómetro no filtro |
| 9 | `use_mag` | 0/1 | 1 | usar o magnetómetro |
| 10 | `t_search` | s | 5.0 | alvo perdido há mais do que isto → SEARCH |

## 5. Só no simulador (`sim/sim_server.py`; não existem no firmware)

| Tipo | Nome | Payload |
|---|---|---|
| 0xF0 | `SIM_SHOVE` | `f32 °` — empurrão de mão (0.25 s) |
| 0xF1 | `SIM_COVER` | `u8` 1 tapa / 0 destapa a câmara |
| 0xF2 | `SIM_TARGET` | `f32 °` — rumo do alvo |
| 0xF3 | `SIM_LIGHT` | `f32` — nível de luz (1.0 nominal) |

## 6. Convenções de sinal

* **Guinada positiva = rodar para a direita** (sentido horário visto de cima; eixos NED: x frente, y direita, z baixo).
* **Imagem:** x cresce para a direita; alvo à direita do centro ⇒ `err_x > 0` ⇒ o cubo tem de rodar com guinada positiva.
* **Roda:** velocidade positiva ⇒ o corpo reage com guinada **negativa**. `CONFIG_CUBE_WHEEL_SIGN` corrige a montagem do motor — se estiver trocado a realimentação é positiva e o cubo foge do alvo.

## 7. Interface CAN-FD com o moteus (cubo ↔ driver)

Gerada por `core/src/moteus_proto.c`, conforme a documentação oficial (`protocol/can.md`, `protocol/registers.md`); validada em PC com a resposta de exemplo da documentação.

* ID CAN: `(origem | 0x80 se pede resposta) << 8 | destino`. Cubo = 0, moteus = `CONFIG_CUBE_MOTEUS_ID`. A resposta vem com o ID `moteus << 8 | 0`.
* Comando (48 bytes, preenchido com NOP `0x50`): `01 00 0a` (modo = posição) · `0c 06 20` + 6 *floats* nos registos 0x020..0x025 (posição = **NaN** ⇒ controlo de velocidade, velocidade em rev/s, *feed-forward* 0, kp/kd = 1, binário máximo) · `0d 29` + *float* (limite de aceleração, rev/s²) · pedido de estado `11 00 · 1f 01 · 13 0d`.
* Conforme o guia *Control Modes* do moteus, em modo velocidade convém configurar `servo.max_position_slip` com um valor finito e ligar um *watchdog* no moteus (`servo.default_timeout_s`) para que a roda pare se o firmware bloquear.
