# Integração de três rodas — 13/11/2026

## Modelo comum

Usar **uma instância `cc3_t`**, não três instâncias de `cc_t`. Os giroscópios medem as três componentes da rotação do mesmo corpo; o quaternion representa uma única orientação. O tipo e ABI antigos não foram aumentados, pelo que os wrappers antigos não precisam de novos campos para continuar a funcionar.

Eixos do corpo: X para a frente (eixo ótico), Y para a direita, Z para baixo; rotações positivas pela regra da mão direita. Rodas ortogonais e fixas nestes eixos. Quaternion `[w,x,y,z]` transforma corpo → mundo. A referência mundial usa a vertical local e, se disponível na calibração, o norte magnético; não aplica declinação magnética para obter norte verdadeiro.

## Firmware / Pessoa 1

1. Acrescentar `cube_core3.c` e `cc_proto3.c` à lista SRCS de `firmware/components/cube_core/CMakeLists.txt`, mantendo os quatro fontes atuais. Exemplo para essa lista:

```cmake
SRCS ${CORE_DIR}/src/cube_core.c ${CORE_DIR}/src/target_detect.c
     ${CORE_DIR}/src/cc_proto.c ${CORE_DIR}/src/moteus_proto.c
     ${CORE_DIR}/src/cube_core3.c ${CORE_DIR}/src/cc_proto3.c
```

2. Incluir `cube_core3.h`, declarar `static cc3_t` e iniciar com `cc3_init(&state, NULL)`, verificando retorno 1. Para configurar, obter primeiro `cc3_config_default`, ajustar os campos e passar a configuração a init. Usar memória estática/caller-owned, não uma grande variável automática de uma tarefa com pouca stack. O núcleo não contém o workspace do detector: reservar **um** `cc_det_ws_t` fora dele.

3. Em cada ciclo preencher `cc3_meas_t`: gyro rad/s, accel m/s², mag µT, velocidades relativas das rodas em rad/s, binários N·m e bateria V. Transformar eixos e sinais dos sensores para o corpo. Por convenção desta API, parado e nivelado o acelerómetro deve dar `[0,0,+9.80665]`; confirmar no driver se é necessária inversão do sinal da força específica. Não adivinhar este sinal no core.

4. `imu_valid=1` apenas para uma leitura nova de gyro/accel bem-sucedida no ciclo. `mag_valid=1` apenas num ciclo com amostra magnética nova. O magnetómetro passa agora no mesmo conjunto de medições; não chamar `cc_on_mag` do núcleo antigo sobre o estado novo.

5. Preencher `motor_valid_mask` a partir de feedback recente e sem falha por roda. Não é suficiente a transmissão CAN ter sido bem-sucedida. Validar idade, fault e ID da resposta. Cada velocidade deve ter o mesmo sinal positivo que o comando correspondente.

6. Chamar `cc3_step_with_imu(&state, esp_timer_get_time(), &meas, imu_valid, &out)`. O relógio é monotónico em microssegundos. O núcleo não aceita timestamps repetidos/recuados como uma amostra nova. Guardar o estado IMU devolvido e enviar cada `out.axis[i]` ao driver certo. `enable=0` significa stop/roda livre, não uma ordem para travar até velocidade zero.

7. São necessários três estados de driver, três IDs moteus distintos e sinais configuráveis por eixo. O `wheel.c` atual tem estado estático de uma roda; não basta chamá-lo três vezes. `moteus_proto.c` não precisa de um novo formato de telegrama; o firmware tem de despachar as respostas pelo ID. A proteção e verificação final do motor continuam no firmware.

8. Manter um mutex comum para comandos, step, atualização da deteção e cópia de telemetria. Deteção de imagem pode decorrer fora do mutex; entregar depois o `cc_det_t` e timestamp de captura a `cc3_on_detection`. O timestamp da câmara partilha o relógio da IMU. Histórico de 64 amostras: a 200 Hz cobre aproximadamente 315 ms; capturas mais antigas ou futuras são rejeitadas. A câmara está alinhada com o corpo; outro alinhamento exige transformar o raio.

9. Adaptar `comms.c` para chamar `cc3_apply_tc` e gerar o ACK. PING/STREAM continuam tratados na camada de comunicações. Guardar a ativação dos IDs draft e a emissão de TM_STATE3 até ao acordo com a Pessoa 5. A telemetria antiga pode continuar a mostrar yaw/roda Z.

O ficheiro `core/examples/core3_adapter.c` demonstra chamadas e foi compilado para ESP32-S3. É um exemplo de API, não uma substituição de `app_main.c` nem um firmware pronto a fazer flash.

## IMU e recuperação

| Situação | Comportamento |
|---|---|
| CALIB | Saídas desativadas. Janela contínua, por omissão 1,5 s e pelo menos 20 amostras; qualquer falha/movimento incompatível reinicia a janela. |
| IMU inválida | Não atualiza estimador/histórico nem integra controlador. RATE/HEADING/TRACK desativados. |
| Primeira leitura válida após falha curta | Atualiza referência temporal, limpa histórico/alvo e acumuladores; não integra o intervalo perdido e não ativa o controlo fechado nesse ciclo. Volta a permitir no ciclo seguinte. |
| Tempo desde última leitura válida >50 ms | Latch RECAL_REQUIRED, incluindo uma pausa entre chamadas sem ciclos intermédios. Exatamente 50 ms ainda não ativa o latch. |
| RECAL recebido | Entra em CALIB mas não limpa imediatamente o latch. Só a conclusão da nova janela contínua o limpa. Termina em IDLE; é necessário pedir o modo operacional novamente. |
| WHEEL após calibração inicial | Permite comando direto por roda mesmo com IMU ausente ou latch. Só ativa rodas cuja disponibilidade e velocidade sejam válidas. |
| Motor indisponível em RATE/HEADING/TRACK | Desativa atuação nos três eixos e limpa o integral. O estimador pode continuar a usar uma IMU válida. Não implementa controlo degradado com duas rodas. |

`axis[2].imu_timeout_us` é o parâmetro comum configurável, inicialmente 50000. Não foi acrescentado ao TMTC antigo. Mesmo depois de uma falha longa durante a calibração inicial, o latch exige RECAL explícito e uma janela concluída. O firmware continua a tentar ler a IMU.

Valores não finitos de gyro/accel também invalidam a leitura. A magnitude do gyro tem um limite configurável. Acelerómetro finito mas com magnitude incompatível com g é ignorado como referência de vertical durante movimento; durante CALIB reinicia a janela. O magnetómetro tem verificação de módulo e de disponibilidade. A calibração exige o cubo parado e não consegue distinguir perfeitamente bias de rotação física lenta.

Durante recuperação mantém-se a última orientação estimada: não se inventa a rotação perdida. Gravidade, magnetómetro e nova câmara podem corrigir posteriormente; a interrupção não garante erro de orientação zero.

## Estimador, configuração e limites

`cfg.axis[0..2]` fornece ganhos/limites independentes. Tempos, ótica, magnetómetro e procura usam `axis[2]`. `accel_kp` e `mag_kp` são ganhos do filtro complementar; o ganho efetivo magnético também depende de R_MAG e da frequência de amostras frescas. O core herda os campos q_psi/q_bias/q_target do tipo antigo para reutilização da configuração, mas **não os usa** no estimador 3D. O detector usa a configuração det que o chamador lhe fornecer.

A correção de yaw magnético é suspensa perto de pitch vertical (|cos(pitch)| <= 0,15). O estado quaternion continua normalizado; as referências/telemetria Euler têm a ambiguidade habitual em ±90°. O bias é fixo entre calibrações. Se o magnetómetro estiver ausente ou desativado na calibração, uma leitura posterior não muda subitamente o referencial: fazer RECAL com medições magnéticas válidas para estabelecer esse referencial.

Limites, anti-windup e rampa de referência não eliminam a necessidade de ganhos adequados à inércia/roda real. O controlo é uma cascata de erro de orientação → velocidade angular → velocidade das rodas, sem compensação explícita dos termos giroscópicos. Os testes incluem esses termos na planta simulada. A descarga de momento angular não está implementada; config `gimbal_unload != 0` é rejeitada.

## Simulação / Pessoa 3 e estação de terra

Manter o wrapper antigo para o modo de uma roda. Para o novo wrapper, usar `cc3_sizeof_state()` e `cc3_init`, declarando os tipos ctypes de medições e saída exatamente como no header. Há verificações de tamanho em `test_core3_sil.py`. Não reinterpretar `cc_t` como `cc3_t` e não usar tamanhos codificados à mão. A biblioteca tem de ser recompilada com os seis fontes.

O teste novo inclui uma implementação independente da dinâmica 3D e exemplos de ctypes. A orientação da câmara é a do mesmo corpo: gerar a imagem/posição do alvo a partir do quaternion físico, e não diretamente da referência de comando ou do quaternion estimado. Isto evita que o ponteiro pareça mover-se antes do corpo.

A estação de terra necessita de entradas XYZ, seletor de kind, descodificação de TM_STATE3 e distinção clara entre Euler relativos, velocidades no corpo e desvios da imagem. O codec Python incluído apenas cria/lê payloads; a trama e ACK continuam a usar as funções atuais. Os detalhes dos IDs e descoberta de capacidades são uma proposta para a Pessoa 5.
