#pragma once
/* Inicializa a câmara e fica em ciclo: captura -> cc_detect (fora do mutex) -> cc_on_detection (dentro). */
void camera_task(void *arg);
