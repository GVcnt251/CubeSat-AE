#pragma once
int   battery_init(void);
float battery_read_v(void);   /* tensão da bateria (V), pelo divisor 100k/13k em GPIO1 */
