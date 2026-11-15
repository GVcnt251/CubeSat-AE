#include "battery.h"
#include <stdbool.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "board.h"

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static bool s_cali_ok;
static bool s_adc_ok;

int battery_init(void)
{
    s_adc_ok = false;
    adc_oneshot_unit_init_cfg_t u = { .unit_id = ADC_UNIT_1 };
    if (adc_oneshot_new_unit(&u, &s_adc) != ESP_OK) return -1;
    adc_oneshot_chan_cfg_t c = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    if (adc_oneshot_config_channel(s_adc, ADC_CHANNEL_0, &c) != ESP_OK) return -1;   /* GPIO1 = ADC1_CH0 */
    s_adc_ok = true;
    adc_cali_curve_fitting_config_t cc = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_0,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    s_cali_ok =
        adc_cali_create_scheme_curve_fitting(&cc, &s_cali) == ESP_OK;
    return 0;
}

float battery_read_v(void)
{
    if (!s_adc_ok) {
    return 0.0f;
    }
    int raw = 0, mv = 0;
    if (adc_oneshot_read(s_adc, ADC_CHANNEL_0, &raw) != ESP_OK) return 0.0f;
    if (s_cali_ok) {
        if (adc_cali_raw_to_voltage(s_cali, raw, &mv) != ESP_OK) {
            return 0.0f;
        }
    } else {
        mv = raw * 3100 / 4095;  /* aproximação sem calibração */
    }                           /* aproximação grosseira */
    return (mv * 1e-3f) * (BAT_DIV_R_TOP + BAT_DIV_R_BOT) / BAT_DIV_R_BOT;
}
