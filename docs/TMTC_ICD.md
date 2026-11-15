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


## 8. Extensão de simulação de três rodas (v5)

Esta extensão só existe no servidor de PC; não altera TM_STATE/TM_VISION nem os comandos do firmware.

- `SIM_SHOVE_XY` (0xF4): dois f32 little-endian, X horizontal em graus e Y vertical em graus. O servidor aceita X até +/-180° e Y até +/-45°; a interface limita a +/-90° e +/-30°. Empurrão de 0,25 s. X aplica yaw, Y aplica -pitch. ACK_BAD_LEN para comprimento diferente de 8; ACK_BAD_ARG para valores não finitos ou fora do intervalo.
- `SIM_SHOVE` (0xF0) mantém o empurrão horizontal legado.
- `TM_SIM3` (0x04): 89 bytes, formato Python `<IB21f`. u32 t_ms; u8 máscara de saturação (bit0 roda X/roll, bit1 roda Y/pitch, bit2 roda Z/yaw); sete grupos de três f32, sempre por ordem roll/pitch/yaw: orientação estimada relativa à calibração [graus], velocidade angular medida [graus/s], velocidades das rodas [rpm], comandos das rodas [rpm], binários medidos [N m], erros de controlo [graus], orientações estimadas dos alvos/referências [graus]. O pacote acompanha TM_STATE à frequência state_hz.
- As referências roll/pitch são obtidas de medições simuladas da gravidade; yaw usa magnetómetro. Os ganhos/limites enviados são aplicados às três instâncias do núcleo. TRACK no canal roll é mapeado para HEADING=0 para estabilizar a referência de roll.
- O modelo é desacoplado por eixo. O firmware existente não implementa esta extensão nem a integração das três rodas.


## 9. Extensão do gimbal (v6, apenas simulador)

- `SIM_GIMBAL_CFG` (0xF5): `<4f`, 16 bytes: massa [kg], CM x/y/z [m] no corpo desde o pivô. Exige IDLE nas três instâncias. Massa 0,1 a 10 kg; |CM_i| até 0,05 m. Valores não finitos/fora do intervalo ou modo errado dão ACK_BAD_ARG; comprimento errado dá ACK_BAD_LEN. Atualiza carga gravítica e inércia efetiva. Gravidade terrestre fixa em 9,80665 m/s².
- `SIM_REST` (0xF6): sem payload. Captura a orientação física atual como referência. Exige IDLE e ausência de empurrão, com velocidades inferiores a 0,5°/s durante pelo menos 0,5 s. Não altera o alvo, sensores ou equilíbrio gravítico.
- `TM_GIMBAL` (0x05): 110 bytes, `<IH26f`: u32 t_ms, u16 reference_id e 26 floats. Sete grupos de três floats: orientação física absoluta roll/pitch/yaw [graus]; orientação de repouso absoluta R/P/Y [graus]; Euler relativo ZYX [graus]; ângulos dos planos XY/XZ/YZ [0 a 90 graus]; binários gravíticos generalizados R/P/Y [N m]; vetor gravidade no corpo [m/s²]; referência física do alvo R/P/Y [graus]. Depois: massa [kg], g [m/s²], CM x/y/z [m]. Emitido à frequência state_hz. O vetor gravidade é aceleração gravítica, não o binário nem uma aceleração de queda do pivô.
- A orientação de repouso inicial é a orientação física no arranque. `TC_RECAL` não a altera. `TM_STATE` e `TM_SIM3` continuam a representar estimativas/controlo; `TM_GIMBAL` é metadado físico do simulador e não alimenta os controladores.


## 10. Webcam fixa com simulação óptica (v7, apenas simulador)

- `SIM_CAMERA` 0xF7: `<BBff`, 10 B: modo (0 sintética/1 webcam), índice (0..9), FoV horizontal da webcam [graus], FoV virtual [graus]. Campo real 40..120, virtual 20..70 e virtual < real-5. Comprimento inválido: ACK_BAD_LEN; argumentos/importação inválidos: ACK_BAD_ARG. Abrir o dispositivo é assíncrono; um ACK OK não confirma a abertura: ler TM_CAMERA.
- `TM_CAMERA` 0x06: `<BBBB8f` (36 B) seguido de mensagem UTF-8 até 180 B. Modo, estado, ponto detetado no vídeo completo, ponto dentro da vista virtual; FoV real/virtual, x/y do ponto na imagem normalizada 320x240, yaw/pitch da direção observada [graus], cobertura da vista virtual [0..1], idade do frame [s, -1 se ausente]. Estados: 0 sintética, 1 a abrir, 2 deteção, 3 sem ponto, 4 erro do dispositivo/dependência, 5 sem frame recente. Campos sem medida podem ser NaN.
- `TM_VIDEO` 0x07: cabeçalho `<HBHH` (7 B): frame_id u16, vista u8 (0 vídeo completo/1 vista virtual), chunk u16, n_chunks u16; até 230 B de JPEG por chunk. IDs partilhados entre vistas, contador de 16 bits. Imagem em tons de cinzento 320x240. A estação limita a reconstrução a quatro frames pendentes e elimina frames incompletos após 1,5 s.
- TM_IMG mantém o formato antigo para a cena sintética. A GUI representa ambas as resoluções nas coordenadas 320x240 para que o cursor corresponda aos pixels de TM_VISION.
- No modo webcam, TM_VISION descreve a deteção no vídeo completo transformada para o referencial do corpo virtual e para a distância focal virtual configurada. x/y podem ficar fora de 320x240: o alvo continua no vídeo completo, mas fora da vista virtual. A área/contraste são do detector na imagem completa.
- A observação usa a pose virtual guardada no instante de captura estimado. A história de poses evita subtrair a orientação atual a uma imagem atrasada. O campo óptico do núcleo é configurado por cc_set_camera_model; mudar de fonte limpa a aquisição anterior, sem recalibrar sensores.
- A direção medida do ponto alimenta o controlo; o alvo do gimbal no modo webcam é uma direção observada, não uma verdade física conhecida. O modo não controla motores reais.


## 11. Correções de modo webcam (v8)

- `SIM_CAMERA_CENTER` 0xF8, sem payload: centra no eixo óptico da webcam, por controlo de atitude absoluta, sem alterar pose/integradores por imposição gráfica. Exige calibração concluída e modo webcam ativo ou sessão webcam-only. ACK_BAD_LEN para payload não vazio; ACK_BAD_ARG para modo errado/calibração pendente. Mantém HEADING nos três eixos até novo comando. FIXAR ALVO / TC_SET_MODE=TRACK retoma seguimento.
- `TM_CAMERA` passa a `<BBBBB8f` (37 B) + mensagem UTF-8 até 180 B. Ordem: modo, estado, found, inside, intenção de controlo; depois os oito floats anteriores. Modo 0 = sintético; 1 = webcam ativa; 2 = sessão exclusiva de webcam sem fonte ligada. Intenção 0 = normal, 1 = a centrar/manter centro, 2 = aguardar primeira deteção para retomar TRACK.
- TM_VIDEO agora transporta JPEG a cores BGR na codificação OpenCV e RGB após descodificação Qt. Cabeçalho de fragmentos e dimensões mantêm-se. Não interpretar o JPEG como matriz de cinzentos.
- `--webcam-only` impede renderização/alvo sintético antes da abertura, durante erros e após desligar. Direções de alvo em TM_GIMBAL ficam NaN enquanto não há deteção real atual. A intenção de seguimento inicial mantém o centro até existir uma deteção válida.
- `cc_point_absolute` converte a orientação absoluta pedida para o setpoint relativo à calibração e ativa HEADING; a massa/dinâmica/rodas não são reposicionadas.
- Cor e região de papel são pré-processadas para o detector C; área/contraste continuam relativos à imagem de deteção normalizada. A imagem apresentada permanece a cores e sem as máscaras internas.


## 12. Gestão de momento no gimbal (v9)

- Formatos e limites mantidos. TM_STATE flags b5 = rotina de alívio de momento armada/ativa (pode ser suspensa enquanto recupera de perturbação grande). O erro transmitido permanece o erro de apontamento, sem subtrair a excursão interna de alívio. b1 continua a indicar velocidade medida acima de 95% do limite, independentemente de b5.
- No byte de máscara de TM_SIM3, b0..b2 continuam a ser saturação roll/pitch/yaw; b3..b5 passam a indicar alívio nesses três eixos. A propriedade Python Sim3.unloading extrai os três bits superiores.
- cc_set_gimbal_unload ativa explicitamente a rotina no adaptador de simulação do gimbal. A configuração por defeito do núcleo tem a rotina desligada: um sistema isolado sem apoio/atrito externo não descarrega momento desta forma. A rotina usa deslocamento limitado a 0,021817 rad, avanço lento de 0,01 rad/s e retorno limitado da referência, sem reposição de estados físicos. Ativa acima de 65% de wheel_lim e termina abaixo de 45%, após retornar a referência.
- cc_set_search_limits permite ao adaptador limitar a procura de pitch a 15°/20° (alvo conhecido/desconhecido). Guinada usa 20°/160°. A direção inverte ao atingir o limite e o filtro de referência mantém o limite de aceleração.
- A integração da malha de velocidade é suspensa durante movimento imposto fora da gama controlável ou movimento rápido oposto à referência. A proteção condicional de saturação do comando mantém-se.
