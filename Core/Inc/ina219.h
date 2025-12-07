#ifndef __INA219_H
#define __INA219_H

#include "stm32f4xx_hal.h" // Adapter si nécessaire (ex: stm32l4xx_hal.h)
#include <stdio.h>
// --- Constantes de l'INA219 ---

// Adresse I2C de base de l'INA219 (si A0 et A1 sont à GND)
#define INA219_I2C_ADDRESS 0x40 << 1 // L'adresse est décalée à gauche pour HAL

// Registres
#define INA219_REG_CONFIG         0x00
#define INA219_REG_SHUNTVOLTAGE   0x01
#define INA219_REG_BUSVOLTAGE     0x02
#define INA219_REG_POWER          0x03
#define INA219_REG_CURRENT        0x04
#define INA219_REG_CALIBRATION    0x05

// --- Structures de données ---

typedef struct {
    I2C_HandleTypeDef *hi2c; // Pointeur vers le handle I2C de l'STM32
    float shunt_resistance_ohm; // Résistance de Shunt utilisée (en Ohms)

    // Variables de calibration calculées (utilisées pour la conversion)
    float current_lsb; // Least Significant Bit pour le courant (en Ampères/bit)
    float power_lsb; // Least Significant Bit pour la puissance (en Watts/bit)

    uint16_t calibration_value; // Valeur du registre de calibration
} INA219_HandleTypedef;

// --- Prototypes des fonctions ---

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
