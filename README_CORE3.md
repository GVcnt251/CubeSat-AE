# Núcleo para três reaction wheels — 13/11/2026

Implementação opcional da Pessoa 2, para uma roda em cada eixo X/Y/Z e referências independentes. Preparada numa cópia do projeto. A pasta partilhada `C:\Users\diogo\CubeSat-AE` não foi editada.

**Estado: núcleo e proposta de interface testados; integração completa no firmware, estação de terra e interface gráfica ainda pendente.** Acrescentar estes ficheiros ao projeto não muda automaticamente a aplicação de uma roda para três.

## O que ficou feito

- Um estimador comum da orientação 3D, com quaternion, giroscópios XYZ, gravidade e magnetómetro quando disponível. Não são três estimadores de yaw independentes.
- Três malhas de velocidade angular, uma por roda, e controlo da orientação pelo erro entre quaternions.
- WHEEL: velocidade independente de cada roda. RATE: velocidades angulares do corpo. HEADING: orientação relativa à calibração em roll/pitch/yaw.
- TRACK: segue um ponto em duas direções da imagem, com referência adicional de rotação em torno da direção de pontaria. Um ponto isolado não mede essa rotação; é necessária a referência inercial/magnética.
- Histórico de orientação para compensar o atraso da câmara, limites de comando e tratamento das falhas da IMU e dos motores.
- Adaptador de telecomandos/telemetria, exemplo C e codec Python para a extensão proposta.

## O que significa «manter o protocolo»

O formato da trama, CRC, comandos antigos e estruturas públicas de `cube_core.h` continuam intactos. O núcleo antigo `cube_core.c` também continua intacto e disponível na mesma biblioteca.

Contudo, os comandos atuais transportam apenas um valor. **Referências independentes para três eixos exigem mensagens adicionais.** Foram implementados três identificadores **provisórios**, `0x89`, `0x8A` e `0x08`, numa extensão separada. A Pessoa 5 tem de confirmar os identificadores e o contrato antes de os ativar no sistema. Não são uma nova versão aprovada do ICD.

A aplicação antiga continua a usar as funções `cc_*`. A aplicação de três rodas terá de usar explicitamente as funções `cc3_*`. O exemplo deixa os novos telecomandos sujeitos a uma opção de ativação.

## Ficheiros entregues

| Ficheiros | Função |
|---|---|
| `core/include/cube_core3.h`, `core/src/cube_core3.c` | Estado 3D, estimador, calibração, controlo e falhas |
| `core/include/cc_proto3.h`, `core/src/cc_proto3.c` | Extensão TMTC proposta e adaptação dos comandos antigos |
| `core/examples/core3_adapter.c` | Exemplo compilável de chamadas para a Pessoa 1 |
| `core/tests/test_core3.c`, `test_core3_more.c` | Regressões C do novo núcleo |
| `core/tests/test_core3_sil.py` | Ensaio com dinâmica acoplada de um corpo e três rodas |
| `core/tests/proto3_reference.py`, `test_proto3_python.py` | Codec de referência e verificação C/Python |
| `core/build_core3.py`, `core/Makefile` | Compilação conjunta dos módulos antigos e novos |
| `docs/CORE3_*.md` | Contrato proposto, integração e validação |

O ZIP é incremental: **copiar o conteúdo de `core/` para a pasta existente, mantendo os restantes ficheiros**. Não apagar a pasta `core` anterior. Dos ficheiros preexistentes, só o `core/Makefile` tem uma versão nova neste pacote. Se o colega entretanto o tiver alterado, comparar as alterações antes de o substituir.

## Como experimentar no PC

Numa cópia atualizada do projeto, juntar os ficheiros do pacote e, na raiz dessa cópia, executar:

```powershell
python core/build_core3.py --test
python core/tests/test_core3_sil.py
python core/tests/test_proto3_python.py
python sim/test_protocol.py
python core/tests/test_aim_sil.py
python sim/test_three_axis.py
```

É necessário o GCC do PC e o Python com as dependências já usadas pelo simulador, incluindo NumPy. O compilador Xtensa do ESP-IDF não serve para produzir a DLL do Windows. Fechar primeiro qualquer simulação que tenha a DLL aberta. O script não instala dependências nem usa a rede.

O primeiro comando recompila a biblioteca com os seis módulos e executa cinco programas de teste C. Em ambientes com GNU Make configurado também se pode usar `make -C core all test`.

O `compilar_core.py` preexistente só inclui os módulos antigos. Para esta versão usar `core/build_core3.py`; executar o construtor antigo posteriormente pode produzir uma DLL sem os símbolos `cc3_*`.

Os testes novos usam a DLL, mas não alteram a interface gráfica. Para visualizar o sistema novo, a Pessoa 3 terá de adaptar o simulador à nova API comum.

## Integração necessária

- **Pessoa 1:** ligar uma instância `cc3_t` à IMU e aos três drivers, com sinais, IDs CAN e disponibilidade por roda; ver `docs/CORE3_INTEGRACAO.md`.
- **Pessoa 5 / comunicações:** validar a proposta em `docs/CORE3_PROTOCOLO_PROPOSTA.md`, acrescentar despacho e telemetria estendida.
- **Pessoa responsável pela estação de terra:** criar controlos XYZ e descodificação da nova telemetria.
- **Pessoa 3 / simulação:** usar o quaternion e os três comandos do novo núcleo na dinâmica e na imagem da câmara.

Os ganhos iniciais são valores de partida. As inércias reais, os sinais dos motores, o equilíbrio do gimbal, o atrito e os limites mecânicos ainda precisam de ser medidos e testados.

## Limitações explícitas

A procura automática faz uma varredura horizontal limitada, mantendo a inclinação de referência; não procura em toda a esfera. O núcleo limita a velocidade das rodas mas não descarrega o momento angular acumulado: três rodas também saturam perante perturbações externas persistentes. A opção antiga de descarga por atrito do gimbal não foi transportada; a configuração é rejeitada se a tentar ativar.

O estimador usa um filtro complementar, não o Kalman escalar antigo. O bias é estimado na calibração e não adaptado durante a execução. Sem referência magnética inicial, o yaw é relativo e pode derivar; um magnetómetro que apareça mais tarde só estabelece a referência após RECAL. Perto da vertical a correção de rumo pelo magnetómetro é suspensa; a propagação por quaternion continua.

O sucesso dos testes e da compilação dos módulos para ESP32-S3 não equivale a um firmware integrado nem a uma validação em hardware. Ver resultados e condições em `docs/CORE3_VALIDACAO.md`.
