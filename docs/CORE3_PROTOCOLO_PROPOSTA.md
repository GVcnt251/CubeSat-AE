# Proposta TMTC para três rodas — 13/11/2026

**DRAFT para revisão pela Pessoa 5. Identificadores não reservados/aprovados.** O ICD atual não é substituído por este documento. Base consultada: `docs/TMTC_ICD.md` da cópia local do projeto em 13/11/2026, incluindo as mensagens SIM_ONLY e a descrição do bit de descarga.

## Transporte e compatibilidade

Mantêm-se `A5 5A | LEN | TYPE | SEQ | PAYLOAD | CRC16`, CRC-16/CCITT-FALSE sobre LEN até ao fim do payload, little-endian, float IEEE-754 de 32 bits, máximo 240 bytes de payload, regras atuais de UDP/série e ACK. Os novos payloads são passados ao `cc_frame_pack` existente.

Não se alteram `cc_proto.h/c`, `cube_core.h/c`, mensagens `0x01..0x03`, `0x7F`, `0x80..0x88`, nem mensagens SIM_ONLY `0x04..0x07`, `0xF0..0xF8`. Não se reutilizam mensagens de simulação como comandos físicos.

O núcleo antigo conserva exatamente a implementação anterior. O adaptador **do novo núcleo** aceita os comandos antigos com estas regras:

| Comando antigo | Aplicação no núcleo 3D |
|---|---|
| MODE | Um modo comum às três rodas; valores 0 CALIB, 1 IDLE, 2 WHEEL, 3 RATE, 4 HEADING, 5 TRACK. Não permite entrar/sair de CALIB por MODE; usar RECAL. |
| WHEEL, RATE | Altera apenas Z, conservando as referências X/Y. |
| HEADING | Rumo mundial relativo ao yaw no fim da calibração; mantém roll/pitch mundiais da orientação desejada. Se o referencial inicial estiver inclinado, a conversão pode alterar vários ângulos relativos internos. |
| AIM | Altera só o desvio horizontal da câmara. |
| GAIN | Ganhos do controlador/limites aplicados aos três eixos; parâmetros dos sensores/tempos são comuns. |
| RECAL | Recalibra os três eixos; coloca referências a zero e desativa atuação durante CALIB. |
| PING, STREAM_CFG | Continuam na camada de comunicações, fora de `cc3_apply_tc`. |

O adaptador devolve BAD_ARG para MODE durante CALIB e para referências/ganhos fora dos limites aceites pelo núcleo 3D. Não promete resposta dinâmica idêntica à do controlador de uma roda.

## TC_SET_VECTOR3 — proposta `0x89`, 14 bytes

| Offset | Tipo | Campo |
|---|---|---|
| 0 | u8 | Versão do payload: 1 |
| 1 | u8 | Kind: 2 WHEEL, 3 RATE, 4 HEADING, 5 AIM |
| 2 | f32 | Primeiro valor |
| 6 | f32 | Segundo valor |
| 10 | f32 | Terceiro valor |

| Kind | Valores no fio | Significado e limites |
|---|---|---|
| 2 WHEEL | `[X,Y,Z]`, rpm | Rodas, relativas ao corpo. Cada valor limitado pela configuração da respetiva roda, inicialmente cerca de ±3000 rpm. |
| 3 RATE | `[X,Y,Z]`, graus/s | Componentes da velocidade angular no corpo; não derivadas dos ângulos de Euler. Limite inicial de cada componente 0,70 rad/s, cerca de ±40,1 graus/s. |
| 4 HEADING | `[roll,pitch,yaw]`, graus | Euler ZYX relativos à orientação inicial: `q_d = q_zero * q_euler(roll,pitch,yaw)`. Roll/yaw ±180°, pitch ±90°. |
| 5 AIM | `[roll_pontaria,vertical,horizontal]`, graus | Roll do referencial de pontaria ±180°; desvios vertical/horizontal ±60°, positivo para baixo/direita na imagem. |

Em AIM, o vetor de câmara desejado é proporcional a `[1,tan(horizontal),tan(vertical)]`. Constrói-se `q_look` apontado ao alvo mundial, com o roll pedido, e `q_aim` que aponta +X para esse vetor sem roll. A orientação desejada é `q_look * conjugate(q_aim)`. Assim, com desvios não nulos, `roll_pontaria` não é necessariamente o ângulo de Euler roll final do corpo. Um único alvo pontual não fornece observação visual do roll.

As referências HEADING não são três rotações desacopladas: a ordem Euler é ZYX. Perto de pitch ±90° os ângulos não são uma representação única, embora a orientação interna continue representada por quaternion.

O comando atualiza os três valores **atomicamente** e não muda de modo. Todos têm de ser finitos e válidos. Referências podem ser preparadas durante CALIB, mas a conclusão da calibração redefine HEADING para zero; a integração deve enviar as referências operacionais após CALIB terminar.

## TC_SET_GAIN3 — proposta `0x8A`, 7 bytes

| Offset | Tipo | Campo |
|---|---|---|
| 0 | u8 | Versão: 1 |
| 1 | u8 | Eixo: 0 X, 1 Y, 2 Z |
| 2 | u8 | Gain ID existente, 0..10 |
| 3 | f32 | Valor nas unidades do ICD atual |

IDs 0 k_theta, 1 rate_kp, 2 rate_ki, 3 w_ref_max, 4 a_ref_max, 5 wheel_lim e 6 torque_max são independentes por eixo. IDs 7 R_CAM, 8 R_MAG, 9 USE_MAG e 10 T_SEARCH são comuns e só aceitam eixo 2. R_CAM/R_MAG continuam a receber um desvio padrão em graus e são guardados como variância em rad².

Neste estimador complementar, R_CAM pondera a atualização da direção do alvo e R_MAG a correção magnética; não representam uma covariância calculada por um filtro de Kalman. Os pesos usados estão explícitos no código. Aumentar estes parâmetros reduz a confiança nas respetivas medições.

Limites numéricos do núcleo, em SI: k_theta 0..100; kp/ki 0..10000; w_ref_max (0,20] rad/s; a_ref_max (0,100] rad/s²; wheel_lim (0,10000] rad/s; torque_max (0,100] N·m; variâncias R_CAM/R_MAG (0,100] rad²; T_SEARCH entre T_LOCK e 300 s; USE_MAG apenas 0/1. Estes são guardas de software, **não especificações dos motores reais**. Os limites de operação devem ser configurados conforme o hardware.

## TM_STATE3 — proposta `0x08`, 159 bytes

| Offset | Tamanho | Conteúdo |
|---|---|---|
| 0 | 1 | Versão 1 |
| 1 | 1 | IMU: 0 OK, 1 UNAVAILABLE, 2 RECAL_REQUIRED |
| 2 | 1 | Máscara de motores aceites: bit0 X, bit1 Y, bit2 Z; restante zero |
| 3 | 52 | Bloco X |
| 55 | 52 | Bloco Y |
| 107 | 52 | Bloco Z |

Cada bloco tem o layout e conversões do TM_STATE de 52 bytes. Campos comuns, como tempo, modo, bateria e idade da câmara, repetem-se. A máscara combina a disponibilidade fornecida pelo firmware com a validação da velocidade finita de cada roda; não diagnostica eletricamente o motor.

| Campo em cada bloco | Interpretação 3D |
|---|---|
| heading | Componente roll/pitch/yaw de `conjugate(q_zero) * q`, em graus |
| rate, bias, rate_ref | Componentes X/Y/Z no corpo, graus/s |
| err | Componente do vetor de rotação de erro no corpo, graus; zero em RATE |
| wheel_rpm, wheel_cmd_rpm, torque | Roda desse eixo |
| target_err | X: zero (o ponto não observa roll); Y: elevação positiva para cima; Z: desvio horizontal positivo para a direita. Sem alvo válido: zero. |
| flags | Mesmos bits: alvo válido, saturação dessa roda, magnetómetro aceite no ciclo, calibrado, inclinação mundial >10°. Bit de descarga permanece zero. |

O aviso de inclinação é informativo, não uma ordem de paragem num sistema 3D. MAG_OK é falso quando não há medição fresca/correção aceite nesse ciclo. CALIBRATED indica que se terminou a calibração; uma falha longa posterior é indicada por imu_status e pode coexistir com esse bit. Os valores de estado congelados durante falha não devem ser apresentados como novas medições.

Pode continuar a emitir-se `TM_STATE` antigo com `cc3_get_legacy_tlm` e `cc_pack_tm_state`, para observar yaw/roda Z. Nesse caso heading conserva a diferença entre yaw mundial atual e inicial, normalizada para ±180°; pode diferir do ângulo relativo Z de TM_STATE3 quando há inclinação. Não conta voltas completas acumuladas. O campo err reporta a componente Z do erro efetivamente usado pelo controlador 3D. Esse painel antigo não mostra a orientação completa.

TM_VISION e imagens usam os packers antigos. Em particular err_y de TM_VISION permanece positivo para baixo. Não enviar as estruturas C por memcpy: o tamanho/alinhamento em memória não define o payload.

## ACK e ativação

Mantêm-se OK=0, BAD_LEN=1, UNKNOWN=2, BAD_ARG=3 e o ACK com TYPE/SEQ do TC original. `cc3_apply_tc` recebe um payload já validado pelo parser e devolve o estado; quem trata a ligação envia o ACK existente. Em caso de erro não aplica parcialmente o comando.

O firmware deve ter uma opção explícita para ativar esta proposta, desativada até acordo da equipa. Negociar com a Pessoa 5 os IDs, a forma de anunciar suporte e a frequência/ativação de TM_STATE3; o pacote não implementa descoberta de capacidades. A capacidade do payload existente é suficiente: 159 < 240 bytes.

O codec `core/tests/proto3_reference.py` serve de referência à estação de terra. Não atualiza automaticamente o seu GUI nem o ICD oficial.
