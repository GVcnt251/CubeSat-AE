# Migração para um único núcleo 3D

Data: 15/11/2026. Este documento descreve a integração a realizar nos módulos dos colegas; não afirma que o firmware ou o GUI foram migrados neste pacote.

## Pessoa 2: resultado entregue

O componente entregue tem só um controlador. `cube_core3.h` inclui apenas headers de C e `target_detect.h`. Não inclui `cube_core.h`. `cube_core3.c` tem a sua configuração inicial e validador de ganhos. `cc_proto.h` passa a incluir o header autónomo; `cc_proto.c` usa `cc3_gain_valid`. Os outros algoritmos/packers existentes foram preservados.

O conjunto necessário à compilação é:

```text
cube_core3.c
target_detect.c
cc_proto.c
cc_proto3.c
moteus_proto.c
```

## Pessoa 1: firmware

O firmware consultado ainda utiliza `cc_t`, `cc_meas_t` e funções `cc_*` do controlador de uma roda. É necessária a migração já prevista para três rodas, e não apenas uma troca de nomes.

| Uso anterior | Uso no núcleo autónomo |
|---|---|
| `#include "cube_core.h"` | `#include "cube_core3.h"` |
| `cc_t` | Uma instância `cc3_t` comum aos três eixos |
| `cc_meas_t` | `cc3_meas_t`, incluindo três velocidades/binários, magnetómetro e disponibilidades |
| `cc_out_t` para todo o cubo | `cc3_out_t` para todo o cubo; cada roda recebe `out.axis[i]`, do tipo `cc_out_t` |
| `cc_init()` | `cc3_init()`, verificar retorno 1 |
| `cc_step_with_imu()` | `cc3_step_with_imu()` |
| `cc_on_mag()` | Medição magnética e indicador fresco em `cc3_meas_t` |
| `cc_on_detection()` | `cc3_on_detection()` |
| `cc_command()` | `cc3_command_legacy()` ou `cc3_command_vector()`; conferir retorno |
| `cc_get_tlm()` | `cc3_get_tlm()` para XYZ, ou `cc3_get_legacy_tlm()` para a vista de yaw |

Uma saída escalar `cc_out_t` continua a ser a interface de um motor, por isso o código do driver pode reutilizar esse tipo. É o estado de cada driver e o despacho CAN que têm de existir por roda.

Atualizar em conjunto `app.h`, `app_main.c`, a tarefa da câmara, `comms.h/c` e os includes do driver. Todos os chamadores do mesmo firmware devem usar a definição nova dos tipos; não incluir simultaneamente o header antigo e o autónomo na mesma unidade de compilação, porque ambos definem nomes comuns.

No componente ESP-IDF, depois de migrar os chamadores, a lista de fontes passa a ser:

```cmake
set(CORE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../core)
idf_component_register(
    SRCS
        ${CORE_DIR}/src/cube_core3.c
        ${CORE_DIR}/src/target_detect.c
        ${CORE_DIR}/src/cc_proto.c
        ${CORE_DIR}/src/cc_proto3.c
        ${CORE_DIR}/src/moteus_proto.c
    INCLUDE_DIRS ${CORE_DIR}/include)
```

Manter um mutex comum nas chamadas ao core. Os eixos do sensor, sinais dos motores, três IDs CAN, validade e idade do feedback são responsabilidade do firmware. O exemplo `core/examples/core3_adapter.c` foi compilado; não é um `app_main.c` completo.

Continuam as convenções: X frente, Y direita, Z baixo; gyro e rodas em rad/s; aceleração em m/s²; relógio comum monotónico em microssegundos; acelerómetro parado/nivelado `[0,0,+g]`. `imu_valid` identifica uma leitura nova bem-sucedida, `mag_valid` uma leitura magnética nova e `motor_valid_mask` contém os bits XYZ de feedback utilizável.

## Pessoa 3: wrapper e simulação

O wrapper de uma roda não pode interpretar uma instância `cc3_t`. A biblioteca nova não exporta `cc_create`, `cc_step`, `cc_on_image` nem outros métodos do controlador anterior. Não há uma adaptação automática de um vetor XYZ para uma chamada escalar.

O wrapper 3D deve carregar `libcubecore3.dll/.so`, reservar o estado por `cc3_sizeof_state()`, chamar `cc3_init` e declarar as estruturas de entrada/saída conforme o header. `core/tests/test_core3_sil.py` demonstra as ligações ctypes e verifica os tamanhos. Os testes já carregam a biblioteca nova.

O layout 3D não mudou neste refactor: no GCC do PC, estado 2560 bytes, medições 68, saídas 48, telemetria 160. Usar as funções sizeof na integração; o payload TM_STATE3 tem 159 bytes e não é uma cópia da estrutura C em memória.

O `compilar_core.py` antigo não é o construtor deste componente. O ponto de entrada é `python core/build_core3.py --test`; os scripts de lançamento da simulação devem chamá-lo e carregar a biblioteca com o nome novo.

A imagem da câmara deve derivar da orientação física do corpo, e a estimativa vem de `cc3_get_attitude`. As três rodas pertencem à mesma dinâmica acoplada. A interface gráfica anterior não foi modificada nesta entrega.

## Pessoa 5 / estação de terra

A mudança é de organização do código. Não alterou campos, unidades, CRC, comprimentos ou identificadores do protocolo anterior, incluindo a extensão proposta. `cc3_apply_tc` devolve o estado ACK; a camada de comunicações continua responsável por o enviar.

`cc3_command_legacy` significa adaptação de comandos escalares ao controlador 3D. Não significa que ainda exista um controlador escalar. O mesmo se aplica a `cc3_get_legacy_tlm`, que disponibiliza apenas a vista yaw/roda Z.

Os IDs propostos continuam provisórios: TC_SET_VECTOR3 0x89 (14 bytes), TC_SET_GAIN3 0x8A (7 bytes), TM_STATE3 0x08 (159 bytes), versão de payload 1. O exemplo mantém o parâmetro `draft_enabled` para não executar telecomandos propostos antes do acordo.

## Quando retirar os ficheiros antigos do repositório comum

A pasta entregue demonstra que o núcleo 3D já funciona sem eles. No projeto partilhado, fazer a remoção definitiva numa migração coordenada, quando os chamadores deixarem de depender da API antiga.

Nessa migração, deixam de ser necessários `core/src/cube_core.c` e `core/include/cube_core.h` no sistema de três eixos. Os testes exclusivos do controlador escalar (`test_imu.c`, `test_review.c`, a parte escalar de `test_core.c` e os testes SIL da aplicação antiga) não fazem parte do build autónomo. Os testes comuns de protocolo, deteção e moteus foram preservados em `test_common.c`; os testes de IMU/controlo 3D continuam em `test_core3.c` e `test_core3_more.c`.

O ZIP não apaga ficheiros do projeto partilhado. Extrair por cima de uma pasta existente também não remove automaticamente ficheiros antigos. Para validar esta entrega, extrair primeiro numa pasta vazia e compilar aí.
