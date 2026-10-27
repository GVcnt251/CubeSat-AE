| Nível | Teste | Critério | Medido (SIL) | |
|---|---|---|---|---|
| 1 | Make it spin | 3000 rpm em <0.5 s, erro <2% | entra em ±2% em 0.40 s; erro estacionário 0.00% | ✅ |
| 2 | Know where I am | erro <2° e deriva <2°/min | erro máx 0.98°, deriva -0.17°/min (só gyro: erro máx 8.5°, deriva +2.95°/min) | ✅ |
| 3 | Turn and stop | degrau 45°, assenta em ±1° (3 sementes) | assenta em 1.7s, 1.8s, 1.7s; sobrelevação máx 0.53°; erro final máx 0.48° | ✅ |
| 4 | See the point | acerta em >98% das frames (erro <2 px) | 100.0% de acertos, 0 falsos positivos, 0/50 com câmara tapada; erro p95 0.07 px (0.02°) | ✅ |
| 4 | See the point | >=15 frames/s (a confirmar no ESP32-S3 - PIL) | PC: 0.08 ms/frame (máx 0.18 ms); câmara a 20 fps | ✅ |
| 5 | Point on its own | de 30° a centrar (±2°) em <6 s (4 sementes) | 1.14s, 1.15s, 1.14s, 1.15s | ✅ |
| 6 | 1 Acquisition | centra (±2°) em <6 s | 1.15 s | ✅ |
| 6 | 2 Hold | nunca sai de ±2° em 1 min | 65 s a partir do 1.º cruzamento: erro máx 1.97°; regime estacionário (após +2 s) 0.19° | ✅ |
| 6 | 3 Disturbance | empurrão ~20° -> recupera em <4 s | 1.08 s | ✅ |
| 6 | 4 Moving target | segue-o sem o perder (3°/s, 24°) | erro máx 2.14°, 0 amostras em SEARCH, erro final 0.13° | ✅ |
| 6 | 5 Occlusion | 2 s tapada: não entra em pânico e retoma | erro máx 0.04°, 0 amostras em SEARCH, re-lock em 0.09 s | ✅ |
| 6 | 5b (extra) Occlusion in recovery | empurrão de 20° e câmara tapada 2 s logo a seguir | 0 amostras em SEARCH; volta a ±2° (parcialmente às cegas) em 1.11 s e, 1 s depois, fica com erro máx 0.54° | ✅ |
