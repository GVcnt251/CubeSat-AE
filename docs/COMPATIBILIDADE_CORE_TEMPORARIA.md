# Compatibilidade temporária com o firmware escalar

O PR #12 retirou `cube_core.c/.h` antes da migração dos chamadores no firmware.
O CMake ainda compila `cube_core.c`; `app_main.c`, a câmara e as comunicações ainda
usam `cc_t`, `cc_step_with_imu`, `cc_on_mag`, `cc_on_detection` e `cc_command`.

Esta correção repõe o controlador e os testes escalares a partir de `fe12964^`,
incluindo o contrato de indisponibilidade IMU e as correções anteriores. O algoritmo
escalar é preservado; a sua validação de ganhos apenas passa a chamar a auxiliar
comum equivalente. O novo núcleo 3D também é preservado.

## Separação das duas compilações

- Firmware atual: `cube_core.c`, `target_detect.c`, `cc_proto.c`, `moteus_proto.c`,
  conforme o CMake existente. Não precisa de `cube_core3.c` para resolver símbolos.
- Biblioteca 3D: `cube_core3.c`, `target_detect.c`, `cc_proto.c`, `cc_proto3.c`,
  `moteus_proto.c`, conforme `build_core3.py` e o Makefile. Não liga o escalar.
- `cc_types.h` contém os tipos comuns sem alterar ordem dos campos, valores dos
  enums ou unidades. Ambos os headers podem ser incluídos, em qualquer ordem.
- `cc_gain_validation.h` contém as mesmas verificações numéricas usadas pelas
  duas APIs públicas de ganhos e pelo protocolo. O codec não força um controlador
  específico a entrar na compilação.

Não há alteração a `wheel.c`, ao escalonamento das tarefas, aos IDs CAN ou ao TMTC.
Os IDs propostos para XYZ continuam dependentes de acordo com a Pessoa 5.

## Verificações reproduzíveis

Na raiz do repositório:

```powershell
python core/tests/test_legacy_compat.py
python core/build_core3.py --test
python core/tests/test_core3_sil.py
python core/tests/test_proto3_python.py
```

O primeiro comando compila e executa os três testes escalares usando apenas os
quatro fontes do firmware e verifica as duas ordens de inclusão dos headers.
O build 3D verifica também que os símbolos do controlador escalar continuam
ausentes da DLL. Os restantes testes cobrem dinâmica 3D e protocolo Python/C.

Na pasta `firmware`, num terminal ESP-IDF configurado para ESP32-S3:

```powershell
idf.py build
```

Validação desta correção em 15/11/2026: os testes acima passaram e o build completo
ESP-IDF 6.1 para ESP32-S3 gerou `cube.bin` com sucesso (0xde320 bytes). Foi usada a
configuração padrão, `CONFIG_CUBE_WHEEL_FAKE=y`, sem atuação em hardware. O build
foi executado numa cópia temporária com caminho ASCII, porque o compilador Xtensa
instalado rejeitou os acentos no caminho da pasta de trabalho original.

## Retirar a compatibilidade

Manter estes ficheiros até a Pessoa 1 concluir o estado/comando de cada roda e
migrar os chamadores para uma única instância `cc3_t`. Coordenar a câmara e as
comunicações com a Pessoa 5. Confirmar também os chamadores SIL antes de retirar
a API escalar. A migração definitiva deve incluir o CMake e passar o build
completo e os testes 3D; só então remover o controlador e os testes escalares.

Este PR não demonstra atuação física das três rodas. A validação do colega na
sua branch e os ensaios de hardware continuam a fazer parte da integração.
