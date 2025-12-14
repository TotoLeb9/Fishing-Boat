#ifndef __INA219_H
#define __INA219_H

#include "stm32f4xx_hal.h"
#include <stdio.h>

#define INA219_I2C_ADDRESS 0x40 << 1

#define INA219_REG_CONFIG         0x00
#define INA219_REG_SHUNTVOLTAGE   0x01
#define INA219_REG_BUSVOLTAGE     0x02
#define INA219_REG_POWER          0x03
#define INA219_REG_CURRENT        0x04
#define INA219_REG_CALIBRATION    0x05


typedef struct {
    I2C_HandleTypeDef *hi2c;
    float shunt_resistance_ohm;

    float current_lsb;
    float power_lsb;

    uint16_t calibration_value;
} INA219_HandleTypedef;


/**
 * @brief Initialise le capteur INA219 et calcule les LSBs de conversion.
 * @param handle : Pointeur vers la structure du driver INA219.
 * @param hi2c : Pointeur vers le handle I2C de l'STM32 (ex: &hi2c1).
 * @param shunt_ohm : La valeur de la résistance de shunt utilisée (ex: 0.1 Ohm).
 * @param current_max_amp : Le courant maximal attendu pour le calcul de calibration (ex: 3.2A).
 * @retval HAL_StatusTypeDef : HAL_OK si l'initialisation réussit.
 */
HAL_StatusTypeDef INA219_Init(INA219_HandleTypedef *handle,
                              I2C_HandleTypeDef *hi2c,
                              float shunt_ohm,
                              float current_max_amp);

/**
 * @brief Lit le registre de tension du bus (VBUS).
 * @param handle : Pointeur vers la structure du driver INA219.
 * @retval Tension du Bus en Volts (V).
 */
float INA219_GetBusVoltage_V(INA219_HandleTypedef *handle);

/**
 * @brief Lit le registre de tension de Shunt (VSHUNT).
 * @param handle : Pointeur vers la structure du driver INA219.
 * @retval Tension du Shunt en Volts (V).
 */
float INA219_GetShuntVoltage_V(INA219_HandleTypedef *handle);

/**
 * @brief Lit le registre de courant (I).
 * @param handle : Pointeur vers la structure du driver INA219.
 * @retval Courant en Ampères (A).
 */
float INA219_GetCurrent_A(INA219_HandleTypedef *handle);

/**
 * @brief Lit le registre de puissance (P).
 * @param handle : Pointeur vers la structure du driver INA219.
 * @retval Puissance en Watts (W).
 */
float INA219_GetPower_W(INA219_HandleTypedef *handle);
void INA219_DumpRegisters(INA219_HandleTypedef *handle);

#endif // __INA219_H
