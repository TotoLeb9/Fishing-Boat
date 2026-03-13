/*
 * battery.h
 *
 *  Created on: Jan 22, 2026
 *      Author: totoleb
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

#define SENSIBILITY 0.066
#define V_ZERO 1.696
#define RATIO 0.6644
#define BAT_CAPACITY_MAH  6000.0f
#define BAT_FULL_VOLTAGE  12.60f
#define BAT_EMPTY_VOLTAGE 10.50f
#define R1 9900.0f
#define R2 2380.0f
#define DIVIDER_RATIO (R2 / (R1 + R2))

static inline float ADC_To_Voltage(uint32_t vMesure){
	return (vMesure * 3.3f) / 4095.0f;
}

static inline float Voltage_To_Current(float v_pin) {
    float v_sensor = v_pin / 0.6644f;
    const float V_OFFSET = 2.536f;
    const float SENSITIVITY = 0.066f;
    return (v_sensor - V_OFFSET) / SENSITIVITY;
}

static inline float ADC_To_BattVoltage(uint32_t adc_raw) {
    float v_pin = (adc_raw / 4095.0f) * 3.3f;
    return v_pin / DIVIDER_RATIO;
}

static inline uint8_t BattVoltage_To_Percent(float v_batt) {
    if(v_batt >= 12.6f) return 100;
    if(v_batt <= 10.5f) return 0;
    const float voltages[] = {12.6f, 12.3f, 12.0f, 11.7f, 11.4f, 11.1f, 10.8f, 10.5f};
    const uint8_t percents[] = {100,   90,    75,    60,    40,    20,    10,     0};
    for(int i = 0; i < 7; i++) {
        if(v_batt >= voltages[i+1]) {
            float ratio = (v_batt - voltages[i+1]) /
                          (voltages[i] - voltages[i+1]);
            return (uint8_t)(percents[i+1] + ratio * (percents[i] - percents[i+1]));
        }
    }
    return 0;
}
#endif /* INC_BATTERY_H_ */
