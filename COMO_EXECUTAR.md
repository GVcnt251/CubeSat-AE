# CubeSat v9 — estação de terra com três rodas





## Correção v9 — saturação no simulador inicial

Para este ensaio abre **INICIAR_SIMULACAO.bat**. A aplicação usa a cena sintética do gimbal, com as três rodas e os limites anteriores: **3000 rpm por roda**. A correção também preserva o modo Webcam.

Foram tratados três pontos:

1. **Empurrões rápidos**: o integrador deixa de acumular uma correção excessiva quando o corpo é forçado a rodar depressa no sentido contrário ao pedido, ou muito acima da velocidade de referência admissível. A ação proporcional mantém-se e a integração retoma na gama controlável.
2. **Procura do alvo**: deixa de rodar continuamente no mesmo sentido. Depois de conhecer o alvo, procura à volta da última direção: cerca de +/-20° em guinada e +/-15° em inclinação. Sem aquisição anterior, a guinada pode varrer até +/-160° e a inclinação até +/-20°, invertendo o sentido nos limites.
3. **Alívio de momento no gimbal**: se uma roda ultrapassar cerca de 65% do limite (1950 rpm por defeito), pode iniciar pequenas correções de orientação, até **1,25° por eixo**, para transferir momento angular para o atrito externo dos rolamentos do gimbal. O alívio termina depois de baixar para cerca de 45% (1350 rpm) e concluir o retorno da pequena correção. Durante um empurrão forte, a prioridade é recuperar o alvo e o ciclo é interrompido temporariamente.

Aparece **Alívio de momento · X/Y/Z** em amarelo quando essa gestão está ativa. A indicação **SATURAÇÃO** continua vermelha se a velocidade realmente entrar na zona de 95% do limite. Não se esconde o aviso nem se aumentam os limites.

O erro mostrado continua a ser o erro real relativamente ao alvo, incluindo as pequenas correções de alívio. As velocidades, integradores e orientação não são repostos à força: o efeito resulta das rodas, corpo e atrito do suporte. Num CubeSat livre, sem binário externo, este procedimento não elimina momento angular; por isso só é ativado pelo adaptador do gimbal e fica desligado por defeito no núcleo C.

Validação no modelo com valores por defeito (massa 1 kg, CM no pivô e atrito já modelado): 32 empurrões de +20° no mesmo sentido, a cada 5 s, pico de 2196 rpm e erro horizontal inferior a 2° nos períodos de recuperação entre empurrões. Com 50 s de câmara tapada, a procura ficou limitada, o pico das rodas foi cerca de 318 rpm e o alvo foi readquirido ao destapar. Gráfico em `results/saturation_v9.png`.

Continuam a existir limites físicos: perturbações demasiado grandes/frequentes, ganhos inadequados, atrito real muito diferente ou CM descentrado com binário gravítico persistente podem exceder a capacidade das rodas. Os testes não substituem a medição do conjunto real.

## Correções v8 — só alvo real, centro da webcam e cores

**INICIAR_WEBCAM.bat** agora inicia o servidor num modo exclusivo de webcam desde o primeiro instante. Não há ponto sintético inicial, nem regresso à cena sintética se a câmara estiver desligada, bloqueada ou sem permissões. Enquanto não houver um ponto real detetado, o seguimento fica à espera, mantendo a direção central. A cena sintética continua disponível separadamente em INICIAR_SIMULACAO.bat.

Se o ponto real desaparecer, o marcador do alvo é retirado do modelo 3D. Os frames antigos não contam como novas medidas. O controlador pode manter a última referência temporariamente, mas isso não cria um ponto fictício na imagem ou na animação.

### Voltar à direção inicial da webcam

Carrega em **CENTRAR NA WEBCAM**. O controlador comanda as três rodas para levar roll/pitch/yaw absolutos a zero, isto é, ao eixo óptico central da webcam fixa, e mantém essa orientação. Não há salto gráfico nem reposição forçada da posição. Este comando funciona mesmo sem o ponto visível e espera pelo fim da calibração se for premido no arranque.

Enquanto está a centrar, o papel pode continuar noutro sítio: o CubeSat não tenta segui-lo. Quando quiseres recomeçar, carrega em **FIXAR ALVO**. O seguimento do papel é retomado; se não houver ponto, espera que ele apareça. O comando de centrar não redefine a referência de repouso do gimbal nem recalibra os sensores.

### Vídeo a cores e deteção

As duas vistas — Webcam completa e Câmara virtual — apresentam vídeo a cores. A deteção trabalha separadamente: usa a cor para excluir objetos muito coloridos que pareçam escuros quando convertidos para cinzento, procura uma superfície clara com área suficiente e usa o detector C para avaliar a marca escura/circular dentro dela. O pré-processamento não altera as cores mostradas no vídeo.

Continua a ser necessário um círculo preto preenchido sobre papel claro e bem iluminado. A aplicação não escolhe simplesmente o pixel mais escuro de toda a divisão: sombras, cabelo ou roupa escura não devem ser confundidos com o alvo. Círculos pretos semelhantes sobre outras superfícies claras ainda podem ser ambíguos; mantém o papel como objeto dominante na imagem.

## Usar a webcam com o papel e o ponto preto

1. Fecha a versão anterior e extrai este ZIP inteiro para uma pasta nova.
2. Abre **INICIAR_WEBCAM.bat**. Na primeira utilização, instala o suporte de vídeo OpenCV; é necessária internet para essa instalação. Depois abre a aplicação.
3. Aguarda a abertura da webcam e o fim da calibração. O seguimento é ativado automaticamente.
4. Mostra um papel branco com um círculo preto preenchido, de frente para a webcam, bem iluminado e ocupando boa parte da imagem. Um círculo de cerca de 2–3 cm é um ponto de partida; aproxima o papel se o ponto aparecer demasiado pequeno.
5. Move o papel devagar para a esquerda/direita e para cima/baixo. O CubeSat 3D acompanha a direção observada. Quando paras o papel, o controlador tenta estabilizar na nova direção.

A aplicação mostra duas vistas em **Vídeo e deteção**:

- **Webcam completa**: o vídeo a cores da câmara fixa, com um anel verde no círculo detetado.
- **Câmara virtual**: reprojeção desse vídeo segundo a orientação física do CubeSat simulado. A cruz representa o centro da vista e o anel amarelo marca o alvo quando está dentro dela. As zonas sem cobertura da webcam são preenchidas de cinzento claro. A percentagem de cobertura aparece no rodapé.

No modelo 3D, o ponto preto representa a direção medida do ponto real. As rodas, o gimbal, a gravidade e os sensores de atitude continuam simulados. Um ponto permite obter as direções horizontal/vertical; não mede a distância ao papel nem a rotação do papel em torno da linha de visão. O roll do CubeSat é estabilizado pelos sensores simulados.

O vídeo é processado localmente e transmitido apenas pela ligação UDP local entre os dois processos da aplicação. Não é enviado para serviços externos nem gravado em ficheiro. O CSV regista telemetria numérica; não grava vídeo.

### Se a webcam não aparecer

- Fecha outras aplicações que estejam a usar a câmara, abre a proteção física da webcam e verifica as permissões de câmara do Windows.
- Em **Definições avançadas > Webcam do computador**, experimenta o número 1 ou 2, em vez de 0, e carrega em **Ligar / reaplicar webcam**.
- Usa **Desligar webcam** para libertar a câmara. No lançador Webcam, o ecrã fica sem vídeo/alvo até voltares a ligá-la; não regressa à cena sintética. Podes voltar a ligá-la pelo mesmo botão.
- Se faltar suporte de vídeo, executa no terminal da pasta:

```powershell
python -m pip install -r requirements-webcam.txt
python iniciar_simulacao.py --webcam
```

Também podes escolher o dispositivo ao abrir: `python iniciar_simulacao.py --webcam --camera-index 1`.

### Modelo óptico e limites

O campo horizontal da webcam começa em **60° como aproximação**; a vista virtual começa em 40°. Podem ser alterados nas definições avançadas. O campo virtual deve ser pelo menos 5° menor que o campo real. Para comparar ângulos com medidas reais, é necessário calibrar o campo óptico e, idealmente, a distorção da lente.

A webcam está parada. O simulador deteta o círculo no vídeo completo usando o mesmo detector C, converte a sua posição numa direção observada e calcula o erro relativamente à orientação virtual. Esse erro entra no controlador C; a resposta dos motores roda o corpo virtual. A reprojeção do vídeo serve para mostrar o que a câmara virtual observaria. A deteção pode existir no vídeo completo enquanto o alvo está fora da vista virtual.

O vídeo real só contém o campo da webcam. Mover o papel para fora desse campo causa perda do alvo; não se inventa imagem além dos limites. Frames repetidos não criam medidas novas e frames com mais de 350 ms são rejeitados. Quando há perda temporária, o controlador mantém a última referência e depois pode entrar em procura, tal como na simulação anterior. Se o ponto deixar de ser detetado, volta a colocar o papel no campo visível e deixa tempo para readquirir.

A deteção procura um círculo escuro com fundo claro. Outros objetos circulares escuros ou um ponto demasiado pequeno/inclinado podem dificultá-la. O vídeo não está espelhado: usa a posição do ponto no separador Webcam completa como referência para os movimentos.

Este modo foi validado com vídeo de teste gerado localmente, incluindo movimento nos dois eixos, perdas de frames, erro de abertura do dispositivo e transmissão UDP/JPEG até à interface. A abertura da webcam física do teu computador precisa de ser confirmada ao usares este modo; não foi capturada imagem pessoal durante o desenvolvimento.

## Novidades v6 — gimbal, gravidade e repouso

O CubeSat está preso num gimbal com pivô fixo no centro geométrico: pode rodar, mas não cai nem se desloca. A gravidade terrestre é 9,80665 m/s². O peso é sustentado pela montagem.

Por indicação do utilizador, o centro de massa começa exatamente no centro do cubo, coincidente com o pivô: **CM = (0, 0, 0)**. Neste caso a gravidade está presente, mas não cria binário de rotação. Um centro de massa deslocado cria um binário gravítico e pode produzir movimento pendular. Para baixo do pivô tende a estabilizar a inclinação; acima pode torná-la instável. A gravidade não fixa o rumo horizontal por si só.

Em **Definições avançadas > Gimbal · gravidade e referência** podes configurar:

- Massa entre 0,1 e 10 kg. O valor inicial de **1 kg é um exemplo**, não uma medição do vosso equipamento.
- Centro de massa X/Y/Z em milímetros, relativo ao pivô. +X é frente, +Y direita e +Z baixo quando o cubo está nivelado. Zero nos três campos representa o centro do cubo.
- Usa **IDLE** antes de aplicar os parâmetros. A massa altera a inércia efetiva; o afastamento do CM acrescenta inércia pelo teorema dos eixos paralelos e cria a carga gravítica.
- O painel mostra a massa aplicada, peso e binários gravíticos, para confirmar o efeito das alterações.

A **orientação inicial** é guardada automaticamente quando o simulador arranca. Mantém-se fixa mesmo após RECALIBRAR, que trata apenas os sensores. Para definir outra orientação de repouso, seleciona IDLE, espera que o cubo esteja parado pelo menos 0,5 s e carrega em **Memorizar posição de repouso**. Isto muda apenas a referência de medição; não desloca o alvo nem muda o ponto de equilíbrio mecânico.

Na animação 3D:

- A base e os apoios são fixos; os anéis representam o gimbal idealizado.
- O cubo tracejado e os eixos tracejados mostram a orientação de repouso.
- A seta roxa mostra a vertical da gravidade; o marcador roxo no cubo mostra o CM. A seta vermelha continua a representar a câmara e o ponto preto o alvo fixo.
- O desenho usa a orientação física do simulador no referencial fixo do gimbal; os erros de controlo e as medidas dos sensores continuam nos cartões/gráficos.

Debaixo do modelo 3D aparecem continuamente:

- **Roll, Pitch e Yaw com sinal**, calculados a partir da rotação relativa R_reposo^T R_atual, pela convenção Z-Y-X.
- **XY, XZ e YZ**, os ângulos entre cada plano atual e o mesmo plano no repouso. São ângulos geométricos entre planos não orientados, de **0° a 90°**; não têm sinal. Uma meia-volta pode deixar um plano coincidente, enquanto a rotação com sinal mostra a meia-volta.

Os valores debaixo da animação são calculados sobre a mesma orientação interpolada que está desenhada. O repouso inicial e o alvo são referências diferentes: por exemplo, se o cubo arrancar a -30° de rumo e apontar a um alvo a 0°, o erro do alvo pode ser zero e o Yaw relativo ao repouso ser +30°.

O CSV inclui a orientação física, rotações relativas, ângulos dos planos, referência, massa/CM e binários gravíticos. A telemetria do gimbal tem a sua própria coluna `gimbal_t_ms`, pois pode chegar num pacote diferente da telemetria do controlador.

O modelo continua a usar inércias por eixo e não a dinâmica acoplada completa dos três anéis/rodas. A gravidade é calculada em 3D a partir do braço do CM, projetada nas coordenadas de rotação. O valor base de inércia efetiva é 0,004 kg m² a 1 kg, ainda um parâmetro provisório a ajustar ao conjunto real. Os anéis da animação são uma representação do suporte, não um CAD dimensionado nem uma simulação das colisões/batentes.

## Abrir

Fecha a versão anterior. Extrai o ZIP inteiro para uma pasta nova e abre `INICIAR_SIMULACAO.bat` dentro da pasta CubeSat-AE. As dependências das versões anteriores continuam a servir.

Em alternativa:

```powershell
python -m pip install -r requirements.txt
python iniciar_simulacao.py
```

A janela indica **Cubo v9 — Proteção das rodas no gimbal**. O lançador abre o servidor e a janela, espera pelo fim da calibração e ativa o seguimento. Ao fechar a janela, termina o servidor que abriu.

Se a porta estiver ocupada: `python iniciar_simulacao.py --port 5001`.
Erros do servidor ficam em `simulador.log`.

## Operação

- **FIXAR ALVO**: ativa o seguimento nos dois eixos de pontaria e repõe a pontaria horizontal em zero. A terceira roda mantém o roll na referência de calibração.
- **IDLE**: desliga a atuação dos três motores. As rodas e o corpo podem continuar a mover-se por inércia.
- **RECALIBRAR**: reinicia a calibração dos três canais; o cubo deve estar parado. Termina em IDLE. Depois, carrega em FIXAR ALVO para retomar.
- **X -20° / X +20°**: empurrão horizontal, isto é, guinada.
- **Y -10° / Y +10°**: empurrão vertical, isto é, inclinação da câmara.
- **Ângulo X / Ângulo Y + Aplicar X + Y**: aplica os dois empurrões ao mesmo tempo, durante 0,25 s. X permite -90° a +90° e Y -30° a +30°.
- **Tapar / Destapar câmara**: simula a oclusão, mantendo a dinâmica e os restantes sensores.
- **Registar dados em CSV**: grava tempo, modo, bateria e, para cada eixo, orientação, velocidade angular, erro, velocidade medida/pedida da roda e binário. O ficheiro `cubo_DATA_HORA.csv` é criado na pasta de execução.

Os cartões mostram ligação, seguimento, erros horizontal/vertical e saturação de qualquer roda. Uma roda saturada tem pouca margem para continuar a corrigir. Os gráficos principais mostram os erros e as velocidades medidas/pedidas das três rodas. O modelo 3D e a imagem da câmara permitem confirmar o alinhamento.

O alvo permanece fixo; os botões de deslocação do alvo foram retirados. A barra preta era um objeto distrator do cenário e foi removida da câmara da demonstração. O teste específico do detector continua a incluí-la, para manter essa verificação.

## Definições avançadas

- **Pontaria**: desvio horizontal desejado do alvo na imagem. 0° centra o alvo. FIXAR ALVO repõe 0°.
- **Iluminação**: altera a luz da cena sem deslocar o alvo.
- **Ganhos e limites**: seleciona uma linha, altera o valor e carrega em Enviar linha selecionada. Os ganhos são aplicados às três instâncias do controlador. Passa o rato pelo nome do parâmetro para ver a sua função.
- **Diagnóstico**: estado, deteção, idade da medida da câmara e contagem de pacotes.
- **Gráficos adicionais**: rumo e velocidade angular horizontais.

## Eixos e modelo físico

Há três rodas ortogonais, com inércia, atrito, limites de velocidade, binário e aceleração, e uma instância do mesmo controlador C por eixo:

- Roda X: roll, rotação em torno do eixo longitudinal.
- Roda Y: pitch, inclinação vertical.
- Roda Z: yaw/guinada, direção horizontal.

**Nos botões de empurrão, X e Y são as direções horizontal e vertical da imagem**, não os nomes dos eixos de rotação do corpo. Assim, o empurrão X atua na guinada/roda Z e o empurrão Y atua na inclinação/roda Y. A roda X estabiliza o roll.

O simulador usa uma **aproximação desacoplada por eixo**: três modelos de corpo/roda, sem os termos giroscópicos de acoplamento de um corpo rígido completo. A câmara usa a orientação 3D para projetar o alvo. O controlador recebe medidas com ruído/bias, não a orientação perfeita do modelo: o rumo usa magnetómetro e os outros eixos usam a inclinação medida pela gravidade; o ponto detetado corrige a pontaria horizontal e vertical. Estes canais são adaptados às interfaces do núcleo C originalmente de um eixo.

A animação interpola a telemetria a cerca de 60 imagens/s com 100 ms de atraso visual. O controlo corre a 200 Hz e a dinâmica a 1 kHz. Os motores corrigem o desvio após o empurrão; não se prende graficamente a seta ao alvo.

As inércias/limites são parâmetros de simulação, ainda por medir no equipamento real. Não se garante erro exatamente zero durante um empurrão ou recuperação de perturbações ilimitadas. O firmware ESP32 fornecido continua a precisar de integração e validação para as três rodas; esta entrega atualiza a simulação e a estação de terra.

## Verificar

```powershell
python sim/test_protocol.py
python sim/test_visual.py
python sim/test_dashboard.py
python sim/test_three_axis.py
python sim/test_gimbal.py
python sim/test_saturation.py
python sim/test_webcam.py
python sim/test_webcam_udp.py
python sim/test_startup.py
python sim/run_levels.py
```

Os níveis de `run_levels.py` continuam a ser o banco de ensaios original de um eixo; `test_three_axis.py` verifica o novo modelo de três rodas contra a sua orientação física. `test_startup.py` testa a interface e a ligação UDP local.

A DLL Windows de 64 bits está incluída. Se for necessário recompilar, fecha a aplicação e executa `python compilar_core.py`. Requer GCC, por exemplo o MSYS2 MINGW64. Usa Python de 64 bits.
