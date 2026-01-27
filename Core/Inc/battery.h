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

static inline float ADC_To_Voltage(uint32_t vMesure){
	return (vMesure * 3.3f) / 4095.0f;
}

static inline float Voltage_To_Current(float v_pin) {
    float v_sensor = v_pin / 0.6644f;
    const float V_OFFSET = 2.536f;
    const float SENSITIVITY = 0.066f;
    return (v_sensor - V_OFFSET) / SENSITIVITY;
}

#endif /* INC_BATTERY_H_ */
