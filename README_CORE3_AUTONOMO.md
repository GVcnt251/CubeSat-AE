# Núcleo autónomo de três eixos — 15/11/2026

O alvo **`libcubecore3` contém apenas o controlador `cube_core3.c/.h`**, para três rodas ortogonais X/Y/Z. Compila e executa sem `cube_core.c` e sem `cube_core.h`.

**Transição do firmware:** os ficheiros escalares foram repostos temporariamente para os chamadores ainda não migrados. O firmware atual compila o núcleo escalar; `python core/build_core3.py --test` continua a compilar apenas o 3D. Ver [compatibilidade temporária](docs/COMPATIBILIDADE_CORE_TEMPORARIA.md).

## O que foi consolidado

Os tipos, modos, comandos, telemetria e configurações comuns estão em `cc_types.h`, incluído por ambos os headers. As configurações iniciais 3D estão na função interna `axis_config_default()`. `cc3_gain_valid()` mantém a API pública e usa as mesmas regras de `cc_gain_validation.h` que o descodificador TMTC, sem obrigar o protocolo a ligar um controlador específico.

O estimador, calibração, controlo, câmara, proteção IMU e regras dos motores mantêm o comportamento da versão de três eixos anterior. Não se executam três cópias do controlador escalar, e a biblioteca não contém as funções antigas `cc_init`, `cc_step` ou `cc_step_with_imu`.

Os nomes públicos `cc3_*` foram mantidos. Os tipos `cc_cmd_t`, `cc_out_t`, `cc_tlm_t` e `cc_config_t` continuam a existir, definidos em `cc_types.h`. Representam comandos, uma saída por roda, um bloco de telemetria e configurações; o controlador antigo `cc_t` fica exclusivamente em `cube_core.h`.

Para não alterar o layout da API 3D já entregue, os campos `q_psi/q_bias/q_target` ficam reservados e não são usados pelo filtro complementar. `gimbal_unload` permanece reservado e tem de ser zero. Conservar estes campos não inclui nem executa o estimador antigo.

## Organização

```text
core/
  include/cc_types.h        Tipos comuns, sem implementação de controlador
  include/cc_gain_validation.h  Validação numérica comum
  include/cube_core3.h      Tipos e API do controlador 3D
  src/cube_core3.c           Estimador e controlo de três eixos
  include/cc_proto.h        Protocolo base e formatos existentes
  src/cc_proto.c
  include/cc_proto3.h       Extensão proposta para referências XYZ
  src/cc_proto3.c
  include/target_detect.h   Deteção do alvo na imagem
  src/target_detect.c
  include/moteus_proto.h    Codificação dos comandos CAN das rodas
  src/moteus_proto.c
  include/wire_le.h         Conversão explícita little-endian
  examples/core3_adapter.c  Exemplo de integração C
  tests/                    Testes C e Python
  build_core3.py            Compilação no PC
  Makefile                  Alternativa com GNU Make
```

Continuam a existir vários módulos porque deteção de imagem, protocolo e controlo têm funções diferentes. A biblioteca 3D contém apenas o núcleo 3D. `cc_proto` e `cc_proto3` são codecs, não controladores de um e de três eixos.

## Experimentar este ZIP

**Este pacote é autónomo, não é um conjunto de ficheiros para copiar imediatamente por cima da aplicação antiga.** Extrai-o para uma pasta nova. Essa pasta já contém todos os fontes C necessários e não precisa de ir buscar o core anterior.

Na pasta extraída, executar:

```powershell
python core/build_core3.py --test
python core/tests/test_core3_sil.py
python core/tests/test_proto3_python.py
```

É necessário GCC do PC e Python; os testes de simulação usam NumPy. Não são instaladas dependências automaticamente. O compilador Xtensa do ESP-IDF não produz DLLs para Windows.

O primeiro comando compila **cinco fontes C** e executa três programas de teste C mais a verificação da API autónoma. Os outros comandos executam oito testes de dinâmica 3D e dois de interoperabilidade Python/C.

Alternativa com GNU Make:

```powershell
make -C core CC=gcc test
make -C core CC=gcc test-sil
```

A biblioteca chama-se **`libcubecore3.dll`** no Windows ou `libcubecore3.so` no Linux. Tem um nome diferente para não substituir uma DLL antiga que a interface gráfica ainda esteja a usar. Os testes incluídos já carregam esse nome. A DLL não vai no ZIP: é gerada no teu PC.

## Compatibilidade e integração

O **protocolo no fio mantém-se igual** ao da versão 3D anterior: trama, CRC, mensagens escalares e proposta de mensagens XYZ. As funções `cc3_command_legacy()` e `cc3_get_legacy_tlm()` continuam a adaptar os comandos escalares e a vista de yaw, mas não chamam o controlador escalar.

Isto não mantém a API de C do controlador de uma roda. O `app_main.c` e o `sim/sil.py` que foram consultados ainda usam essa API antiga. Trocar apenas os headers ou apagar os fontes antigos nesse projeto sem migrar os chamadores provocaria erros de compilação/carregamento.

A aplicação de três rodas deve usar `cc3_t`, `cc3_meas_t`, `cc3_out_t` e `cc3_step_with_imu`. A Pessoa 1 continua responsável pelos três drivers/mapeamento de eixos e a Pessoa 3 pelo wrapper e dinâmica gráfica. Ver `docs/MIGRACAO_CORE3_AUTONOMO.md`.

Os IDs da extensão TMTC (`0x89`, `0x8A`, `0x08`) continuam uma proposta a acordar com a Pessoa 5. Esta limpeza não os aprova, não os altera e não acrescenta novas mensagens.

## Verificação da entrega autónoma original

- Compilação e testes sem os ficheiros do controlador antigo, que não existem na pasta entregue.
- Três programas C: protocolo/detetor/moteus; núcleo 3D; casos adicionais de configuração e compatibilidade.
- Verificação dos símbolos da DLL: API 3D presente; controlador escalar ausente.
- Oito testes SIL 3D e dois testes Python/C passaram.
- Comparação com a implementação 3D anterior durante **4 000 ciclos**: estado, comandos das rodas, deteções e bytes de TM_STATE3 idênticos.
- Validação de ganhos comparada em 135 casos; tamanhos das estruturas 3D preservados no GCC do PC.
- Cinco módulos e exemplo compilados para ESP32-S3 com `-Wall -Wextra -Wpedantic -Werror`.
- Headers/exemplo também verificados em C++11. Makefile testado com reconstrução completa.

A comparação usa uma cópia da versão anterior apenas como referência de validação, fora da pasta entregue. Essa cópia não é uma dependência do produto. Resultados em `docs/VALIDACAO_CORE3_AUTONOMO.md`.

Não houve validação no hardware nem compilação do firmware completo com três drivers. Continuam as limitações anteriores: procura horizontal do alvo, ausência de descarga de momento angular e necessidade de ajustar ganhos e convenções aos sensores/motores reais.

## E a explicação/PDF anterior?

A explicação do algoritmo continua aplicável. O núcleo 3D usa a função interna `axis_config_default()` e a API `cc3_gain_valid()`. Durante a transição, os tipos comuns estão em `cc_types.h`; isto não cria dependência do controlador escalar.
