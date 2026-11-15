# Verificação da versão corrigida

Executado em Windows com Python 3.11 de 64 bits e GCC MSYS2 MINGW64 em 29/10/2026.

- Núcleo C recompilado a partir dos quatro ficheiros-fonte: passou.
- `python sim/test_protocol.py`: 20 verificações passaram.
- `python sim/run_levels.py`: todos os 12 critérios dos níveis 1 a 6 passaram; ver results/summary.md e os seis gráficos.
- `python sim/test_startup.py`: passou a ligação com a GUI aberta antes do servidor, receção de imagem e entrada em TRACK; vision_hz=5 produziu 11 medidas em 2 s, incluindo pacotes em trânsito da configuração anterior; CLI aguardou a calibração e entrou em TRACK; PING/STREAM_CFG com comprimento inválido receberam ACK de erro.
- Abertura e fecho conjunto: confirmado que o lançador termina o servidor que criou quando a interface termina.
- Verificação sintática dos ficheiros Python: passou.

Os testes UDP foram executados com acesso à rede local permitido. O ambiente restrito de execução bloqueava a receção local e não servia para validar a ligação.
A interface foi testada com Qt em modo offscreen. O teste produz uma captura para diagnóstico; algumas instalações Qt offscreen não desenham as fontes do Windows.
Não foi ensaiado o CubeSat real nem a captura de webcam. Os testes e o núcleo continuam a representar apenas um eixo de guinada.

## Atualização v2 — apontamento e animação

- Confirmado alinhamento exato da ponta da seta e do alvo com erro zero em seis orientações e nove vistas de câmara.
- Interpolação a 60 Hz testada com um empurrão do simulador físico; produz mais posições intermédias e saltos menores que a apresentação direta dos pacotes a 20 Hz.
- Testada passagem de +179° a -179° pelo trajeto curto, interpolação da velocidade da roda, perda do alvo, reinício do relógio e reconexão.
- `python sim/test_visual.py`: quatro testes passaram.
- `python sim/test_startup.py`: voltou a passar após a atualização da interface.
- Revisto o desenho em duas situações: desvio de 30° e alinhamento; captura em results/alinhamento_3d.png.
- Alterada apenas a interface e adicionados os testes/documentação; não foi alterada a física nem o núcleo de controlo.

## Atualização v3 — controlo ativo e recuperação

- Abertura pelo lançador ativa TRACK automaticamente depois de CALIB, com aim=0. A GUI indica Cubo v3, estado de controlo, erro, velocidade pedida à roda e binário medido.
- Núcleo C e DLL atualizados: erro de TRACK, inovação da câmara e target_err tratados como ângulos periódicos; a câmara sintética trata voltas completas e oculta alvos atrás da câmara.
- `python sim/test_protocol.py`: 20 verificações passaram com a nova DLL.
- `python sim/run_levels.py`: todos os 12 critérios voltaram a passar; recuperação do empurrão padrão de 20° em 1,08 s no teste do nível 6.
- `python sim/test_visual.py`: quatro testes passaram.
- `python sim/test_startup.py`: a GUI arrancou antes do servidor, ativou TRACK automaticamente após calibração e recuperou após empurrões de +20° e -20°, com comando de motor observado. Passaram ainda imagem, frequência de visão, CLI e ACK de comprimento inválido.
- `python sim/test_tracking.py`: três testes passaram. Nove casos de empurrão de 0,25 s (+/-20°, +/-60°, +/-120°, +180°, +/-360°), alvo físico fixo, recuperação para menos de 2° e LOCKED ao fim de 16 s de simulação. Maior erro nos últimos 2 s entre esses casos: 0,043°.
- Uma sequência com semente 2 (+20°, -60°, +120°, -180°, +360°) recuperou entre os eventos sem alterar o alvo físico.
- Não foi imposta a posição da seta nem a orientação do simulador. A recuperação usa o controlador C, a dinâmica do corpo e roda e os sensores simulados. Resultados limitados aos parâmetros e cenários ensaiados.

## Atualização v4 — interface simplificada

Retirados os botões WHEEL, RATE, HEADING e TRACK e os campos de Roda, Taxa e Rumo. A interface mantém FIXAR ALVO, IDLE, RECALIBRAR e a Pontaria do alvo. O estado de seguimento aparece como FIXAR ALVO. O protocolo e o núcleo continuam compatíveis com os comandos existentes.

Verificação local da janela em Qt offscreen: abertura sem erro; ausência dos quatro botões e dos três campos; FIXAR ALVO agenda o seguimento e repõe Pontaria=0; IDLE cancela o arranque pendente e envia o modo IDLE; RECALIBRAR cancela o arranque pendente e envia o comando de calibração. A física, o controlador C e a DLL são os da versão v3, já validados.

## Atualização v5 — três rodas, empurrões X/Y e dashboard

- `sim/three_axis.py`: três modelos corpo/roda desacoplados e três instâncias do mesmo controlador C. Controlo de yaw e pitch pelo alvo e manutenção da referência de roll. Sensores simulados com ruído/bias, referências magnética/gravítica e projeção 3D da câmara. Não é o firmware de três rodas nem uma dinâmica completa acoplada.
- `python sim/test_three_axis.py`: dois testes passaram; cinco cenários com sementes diferentes: X/Y de (+20,+15), (-20,-15), (+45,-25) com roll+10°, (+90,+30), (-90,-30). Todos recuperaram para erro físico de pontaria inferior a 0,05° ao fim dos ensaios de 12 s. Terceira roda respondeu à perturbação de roll. IDLE desativou todos os motores. A antiga barra não existe na câmara da demonstração.
- `python sim/test_startup.py`: passou arranque/reconexão, ativação automática de seguimento, empurrões +/-20°, botão de ângulos personalizados X+Y, comandos da roda vertical e recuperação de ambos os eixos, receção de imagem, frequência de visão estabilizada em 5 Hz, CLI e ACKs.
- `python sim/test_dashboard.py`: dois testes passaram: quatro botões de empurrão enviam os valores/eixos corretos, personalizado envia ambos os ângulos, definições avançadas e oclusão enviam os comandos corretos, botões de deslocação do alvo/modos retirados ausentes, CSV tem os três eixos e o aviso identifica a roda saturada.
- `python sim/test_visual.py`: quatro testes passaram.
- `python sim/test_protocol.py`: 20 verificações passaram; os formatos antigos permanecem compatíveis. Extensões SIM_SHOVE_XY/TM_SIM3 documentadas no ICD.
- `python sim/run_levels.py`: todos os 12 critérios do banco de um eixo passaram. O teste do detector mantém explicitamente a barra distratora, enquanto a demonstração a omite. Estes resultados antigos não são apresentados como teste de três eixos.
- Verificação sintática dos módulos Python: passou. Capturas do dashboard e das definições avançadas revistas em results/dashboard_v5.png e results/advanced_v5.png.
- A medição de frequência de visão descarta os pacotes em trânsito durante 350 ms após a nova configuração; não conta a fila do fluxo anterior como frequência do fluxo novo.

Hipótese de montagem: rodas ortogonais nos eixos X/roll, Y/pitch e Z/yaw. Os X/Y dos empurrões são coordenadas horizontal/vertical da imagem. Parâmetros físicos iguais por eixo são provisórios e precisam de medição no equipamento real.

## Atualização v6 — gimbal e gravidade

Requisitos: pivô fixo no centro geométrico; CM inicialmente coincidente com esse ponto conforme indicação do utilizador; massa inicial provisória de 1 kg. Massa e CM são editáveis nas definições avançadas.

Implementado `sim/gimbal.py`: R = Rz Ry Rx; gravidade no corpo; momento r_CM x m g; projeção por trabalho virtual nas coordenadas de rotação. A massa escala a inércia efetiva e o braço do CM acrescenta inércia pelos eixos paralelos. A dinâmica inercial continua aproximada por eixo. Não se introduz uma mola fictícia quando o CM está no pivô.

A referência inicial é física, fixa e independente de RECALIBRAR. Nova referência exige IDLE e pelo menos 0,5 s com velocidades inferiores a 0,5 graus/s. Planos XY/XZ/YZ usam o ângulo entre normais não orientadas, 0 a 90 graus. Roll/pitch/yaw relativos usam a matriz R0^T R e convenção ZYX.

- `python sim/test_gimbal.py`: seis testes passaram: CM centrado gera binário zero; gravidade projetada conserva módulo; binário é o negativo do gradiente numérico da energia potencial; CM abaixo do pivô gera movimento pendular passivo; geometria de planos/rotações e caso 180 graus; referência conserva-se durante calibração; alteração de massa/inércia e recuperação com gravidade ativa e CM 5 mm abaixo, massa 2 kg.
- `python sim/test_dashboard.py`: três testes passaram, incluindo envio de massa/CM em unidades corretas, comando de referência, concordância dos valores visuais com a geometria e CSV com 43 colunas.
- `python sim/test_startup.py`: passou a ligação UDP, empurrões, receção de metadados do gimbal, referência inicial de yaw -30 graus, parâmetros de massa/CM, captura válida de repouso e rejeição de payload/valores inválidos. Frequência de visão estabilizada, imagem e CLI continuam a passar.
- `python sim/test_three_axis.py`: os cinco cenários voltaram a passar com o gimbal centrado; pior erro físico final 0,047 graus.
- `python sim/test_visual.py`: quatro testes passaram.
- `python sim/test_protocol.py`: 20 verificações passaram.
- Capturas revistas: results/gimbal_v6.png e results/gimbal_advanced_v6.png. A animação usa a orientação física e o alvo físico em referencial fixo; os controladores mantêm os seus sensores e estimativas separados da verdade do simulador.

O CSV mantém o tempo de cada mensagem do gimbal em gimbal_t_ms para distinguir a sua amostra da telemetria do controlador. Os valores apresentados sob a animação são calculados sobre a mesma pose interpolada desenhada, com 100 ms de atraso visual.

Não foram medidos massa, inércias, atrito, montagem ou batentes do equipamento real. O gimbal é idealizado e a implementação do firmware de três rodas continua fora da validação deste simulador.

## Atualização v7 — webcam fixa e papel com ponto preto

Implementado modo de vídeo real opcional. A webcam é aberta apenas ao ativar esse modo; o simulador sintético não depende de OpenCV. O lançador INICIAR_WEBCAM.bat instala requirements-webcam.txt se cv2 estiver ausente. Testado com OpenCV 5.0.0 (wheel 5.0.0.93), Python 3.11.15, NumPy 2.4.6, Windows de 64 bits.

O detector C processa a imagem completa normalizada e preserva a razão de aspeto por letterbox. A direção medida é transformada para o corpo virtual no instante estimado da captura, usando a história de poses. As medidas ópticas entram nas duas instâncias C de seguimento; as rodas/dinâmica continuam a produzir a orientação física. A imagem virtual é uma reprojeção do vídeo e a cobertura real é indicada. Não se desenha um ponto artificial a substituir o ponto real. A vista completa pode detetar o ponto mesmo quando este está fora da vista virtual, conforme documentado no ICD.

- `python sim/test_webcam.py`: cinco testes passaram, sem abrir um dispositivo real. Verifica deteção de pixels, projeção para a direção do ponto, rejeição de frames repetidos/velhos, movimento de papel nos dois eixos e comandos de motor. O maior erro observado no percurso móvel foi 2,43 graus; ao parar o papel na nova direção, o erro ficou abaixo de 0,3 graus no ensaio. Também verifica JPEG/fragmentos fora de ordem, apresentação Qt e falha de abertura com backend substituto.
- `python sim/test_webcam_udp.py`: passou o percurso completo do vídeo gerado até ao simulador e à GUI por UDP/JPEG: imagem completa e virtual, deteção, telemetria, ativação de seguimento e resposta à deslocação do papel. Verificado fecho do servidor e da fonte.
- `python sim/test_startup.py`: modo sintético anterior passou novamente, incluindo empurrões X+Y, imagem, parâmetros/ref do gimbal, CLI e validação de comandos.
- `python sim/test_dashboard.py`: três testes passaram.
- `python sim/test_visual.py`: quatro testes passaram.
- `python sim/test_protocol.py`: 20 verificações passaram com a DLL recompilada.
- Verificação sintática dos módulos Python: passou. Capturas de interface com vídeo gerado revistas em results/webcam_v7_demo.png e results/webcam_advanced_v7.png; não são capturas de uma webcam física.

Não foi aberta uma webcam física nem capturada imagem pessoal durante a implementação. A abertura do dispositivo, permissões do Windows, foco/exposição e desempenho com o papel real precisam de ser confirmados no computador do utilizador. Os campos ópticos por defeito (60/40 graus) são aproximados; a precisão angular real exige calibração da lente.

Frames são processados em memória. O código de produção não guarda vídeo nem usa serviços externos. O modo usa a ligação local do lançador. O CSV continua a guardar apenas os dados numéricos do simulador. Roll é estabilizado por sensores simulados; um ponto isolado não fornece distância ou atitude completa do papel.

## Atualização v8 — alvo exclusivamente real, centragem e vídeo a cores

- INICIAR_WEBCAM passa --webcam-only ao servidor desde o arranque. Sem fonte real há apenas uma imagem uniforme e direções de alvo NaN. Não se renderiza o ponto sintético antes de abrir a webcam, durante erros ou após desligar. O seguimento inicial aguarda uma deteção mantendo o centro em vez de procurar um ponto inexistente. Ao perder o ponto, o marcador físico deixa de ser mostrado.
- CENTRAR NA WEBCAM usa cc_point_absolute para comandar HEADING absoluto zero nos três eixos, respeitando a calibração dos sensores. Mantém a orientação central até FIXAR ALVO retomar TRACK. Não altera pose por imposição, referência do gimbal ou alvo real.
- As duas vistas são transmitidas/apresentadas a cores. A deteção usa uma imagem separada pré-processada: cor/baixa saturação para marcas pretas, regiões claras com área suficiente para o papel e detector C de forma/contraste. Círculos de objetos muito coloridos são excluídos. Ambiguidades entre marcas pretas semelhantes continuam possíveis.
- OpenCV limitado a uma thread para os frames pequenos; binário gravítico de braço exatamente nulo evita cálculos matriciais repetidos na física, preservando o resultado zero.

Verificação:

- `python sim/test_webcam.py`: sete testes passaram. Inclui ausência de qualquer ponto sintético durante oito segundos sem fonte, espera inicial sem deriva/procura, fidelidade de cor, rejeição de círculo azul saturado como falso alvo, movimento do papel, frames velhos/duplicados, vídeo JPEG/Qt, erro de dispositivo com backend substituto e o ciclo seguir-parar-centrar-manter-retomar. No ensaio de centragem, ângulos físicos ficaram abaixo de 0,6 graus; ao retomar o ponto fixo, erro inferior a 0,3 graus. Maior erro do percurso móvel: 2,43 graus.
- `python sim/test_webcam_udp.py`: passou vídeo RGB e controlo pela ligação local, botão CENTRAR NA WEBCAM, confirmação do estado HEADING/centragem, regresso físico ao centro e retoma de FIXAR ALVO pela GUI.
- `python sim/test_dashboard.py`: quatro testes passaram, incluindo botão de centrar/espera, comandos e cancelamento de intenções concorrentes.
- `python sim/test_visual.py`: quatro testes passaram.
- `python sim/test_protocol.py`: 20 verificações passaram com a DLL recompilada.
- Módulos Python compilados sintaticamente sem erros; captura de layout revista em results/webcam_v8_teste.png, com vídeo gerado e aviso explícito de que não foi usada webcam física.

Não foi aberta uma webcam física nestes testes. As melhorias de cor/segmentação foram verificadas com cenas geradas; o desempenho no papel real continua dependente de iluminação, foco, tamanho da marca e fundo. A massa/gimbal/rodas continuam simulados e os campos ópticos são aproximados até calibração.

## Atualização v9 — saturação no simulador sintético

Reproduzido na v8 com valores por defeito: empurrões de +20 graus a cada 5 s atingiam a zona de saturação no terceiro empurrão, por volta de 15 s; a roda permanecia em ~3000 rpm e o corpo deixava de recuperar. Com oclusão de 50 s, a procura contínua acumulava velocidade, atingia 3000 rpm e perdia a recuperação. Um ensaio em repouso de 90 s não mostrou saturação; empurrões alternados também não reproduziram a acumulação.

Corrigido o integrador durante movimento rápido imposto no sentido contrário à referência, mantida a proteção de saturação do comando e substituída a procura contínua por varrimento alternado limitado. A procura vertical tem limites próprios para não fazer o pitch atravessar rotações incompatíveis com o modelo de sensores. A primeira versão da proteção apenas adiava a saturação; essa variante não foi entregue.

Acrescentado alívio de momento específico do gimbal: pequenas excursões de referência até 1,25 graus por eixo aproveitam o atrito externo para descarregar as rodas; não se impõe velocidade zero, não se aumenta wheel_lim nem se aplica um binário externo fictício. Histerese 65%/45%. Desativado por defeito no núcleo C, ativado pelo adaptador de três eixos do gimbal. Telemetria mostra a rotina e preserva o aviso de saturação real.

Ensaios finais:

- `test_saturation.py`, repetição: 32 empurrões no mesmo sentido ao longo de 180 s, pico **2196 rpm**, nenhuma amostra saturada e erro horizontal inferior a 2 graus nas janelas de recuperação avaliadas a partir de 3 s após o arranque/empurrão. Gráfico em results/saturation_v9.png.
- `test_saturation.py`, oclusão: 50 s tapada, pico de todas as rodas **318 rpm**, procura dentro dos limites e readquisição ao destapar; orientação física final a menos de 0,5 graus.
- `test_saturation.py`, descarga: condições iniciais com a roda em +/-2400 rpm; nos dois sentidos desceu abaixo de 1700 rpm mantendo erro inferior a 2 graus. Repetido sem atrito externo: o momento angular conservou-se com desvio inferior a 1e-8 e a roda não descarregou. Isto verifica que não existe reposição oculta de estados.
- A janela do primeiro teste foi alinhada com o início do controlo, para não tratar a aquisição inicial de -30 graus como estado já recuperado; esse teste foi reexecutado isoladamente e passou. Os outros dois cenários passaram na execução conjunta.
- `python sim/run_levels.py`: todos os 12 critérios do banco original de um eixo passaram. Recuperação do empurrão padrão em 1,05 s; demais resultados em results/summary.md.
- `python sim/test_startup.py`: passou a interface do modo sintético por UDP, empurrões +/-20 graus, personalizado X+Y, recuperação, imagem, CLI e parâmetros/ref do gimbal.
- `python sim/test_webcam.py`: os sete testes passaram; preservados alvo exclusivamente real, cor, centragem e retoma.
- `python sim/test_protocol.py`: 20 verificações passaram com a DLL final recompilada.
- `python sim/test_dashboard.py`: quatro testes passaram, incluindo a distinção visual entre alívio e saturação e a prioridade do aviso vermelho quando existe saturação verdadeira.
- `python sim/test_visual.py`: quatro testes passaram.
- Teste direto da API: RECALIBRAR cancela a rotina e o indicador de alívio. Verificação sintática dos módulos Python passou.

Condições dos ensaios: massa 1 kg, CM no pivô e atrito do gimbal modelado originalmente. Esta correção não garante controlo contra perturbações arbitrárias, ganhos alterados ou binários externos que excedam a capacidade das rodas. Num satélite isolado, o momento não se elimina por estas excursões; o mecanismo depende do suporte externo. O hardware real continua por validar.
