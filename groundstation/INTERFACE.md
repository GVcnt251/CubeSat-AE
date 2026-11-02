# Ground Station: modos da interface

Abrir `INICIAR_GROUNDSTATION.bat` na raiz do projeto.

Na aba Operação, a secção Sessão reúne:

- **Ambiente:** Simulação. A opção CubeSat real está preparada visualmente,
  mas desativada até à integração de hardware pela equipa responsável.
- **Origem do vídeo:** Simulação (alvo virtual) ou Webcam (alvo real).
  Trocar de origem fecha a sessão atual e inicia outra, incluindo calibração.
  O cubo e as rodas continuam simulados quando se usa a webcam.
- **Cor do alvo:** botão Ponto preto / Ponto branco. É apenas uma seleção
  visual para futura integração; não envia comandos nem altera a deteção.
  O detetor atual continua a procurar um ponto preto.

Os atalhos anteriores continuam disponíveis e usam o mesmo lançador:
`INICIAR_SIMULACAO.bat` e `INICIAR_WEBCAM.bat`.
Para a webcam são necessárias as dependências de `requirements-webcam.txt`:

```powershell
python -m pip install -r requirements-webcam.txt
```

A troca de sessão é coordenada pelo lançador e pela interface. Não foram
alterados a física, os controladores, a deteção ou o protocolo de comunicação.
A execução direta de `groundstation/gs_app.py` mantém o funcionamento anterior;
o seletor de origem fica desativado porque o lançador não gere esse servidor.
