#include "ina219.h"
#include <math.h> // Pour les calculs float

// --- Fonctions utilitaires I2C (Privées) ---

static HAL_StatusTypeDef INA219_WriteReg(INA219_HandleTypedef *handle, uint8_t reg, uint16_t data) {
    uint8_t buffer[3];
    buffer[0] = reg;
    // L'INA219 attend les données en Big-Endian (MSB d'abord)
    buffer[1] = (uint8_t)(data >> 8);
    buffer[2] = (uint8_t)(data & 0xFF);
    return HAL_I2C_Master_Transmit(handle->hi2c, INA219_I2C_ADDRESS, buffer, 3, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef INA219_ReadReg(INA219_HandleTypedef *handle, uint8_t reg, uint16_t *data) {
    HAL_StatusTypeDef status;
    uint8_t buffer[2];

    // Étape 1: Écrire l'adresse du registre à lire
    status = HAL_I2C_Master_Transmit(handle->hi2c, INA219_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Étape 2: Lire 2 octets du registre (MSB d'abord)
    status = HAL_I2C_Master_Receive(handle->hi2c, INA219_I2C_ADDRESS, buffer, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Reconstruire l'entier 16 bits (Big-Endian vers Little-Endian)
    *data = (uint16_t)((buffer[0] << 8) | buffer[1]);
    return HAL_OK;
}

// --- Fonctions publiques ---

HAL_StatusTypeDef INA219_Init(INA219_HandleTypedef *handle,
                              I2C_HandleTypeDef *hi2c,
                              float shunt_ohm,
                              float current_max_amp) {
    HAL_StatusTypeDef status;

    // 1. Sauvegarder les handles
    handle->hi2c = hi2c;
    handle->shunt_resistance_ohm = shunt_ohm;


    handle->current_lsb = 0.0001f; // 100 microAmpère/bit

    // 3. Calcul du registre de calibration (Cal)
    // $\text{Cal} = \text{trunc}(0.04096 / (Current_{LSB} \cdot R_{shunt}))$
    float cal_float = 0.04096f / (handle->current_lsb * handle->shunt_resistance_ohm);
    handle->calibration_value = (uint16_t)roundf(cal_float);

    // 4. Calcul du LSB de Puissance (W/bit)
    // $Power_{LSB} = 20 \cdot Current_{LSB}$
    handle->power_lsb = 20.0f * handle->current_lsb;

    // 5. Écrire le registre de Calibration
    status = INA219_WriteReg(handle, INA219_REG_CALIBRATION, handle->calibration_value);
    if (status != HAL_OK) return status;

    // 6. Écrire le registre de Configuration (Exemple : VBUS 32V, PGA 320mV, ADC 12bit, mode Continu Shunt et Bus)
    // Configuration :
    // BRS: 11 = 32V (Bus Voltage Range)
    // PG: 11 = 320mV (PGA Range pour le Shunt)
    // BADC : 1001 (12-bit, 8.5ms)
    // SADC : 1001 (12-bit, 8.5ms)
    // MODE : 111 (Shunt and Bus, Continuous)
    uint16_t config_value =
          (0b0001 << 13) // BRS: 1 = 32V (Plage de Tension Bus, recommandée)
        | (0b0011 << 11) // PG: 3 = +/-320mV (Peut rester, mais n'est pas utilisé)
        | (0b1001 << 7)  // BADC: 12-bit, 8.5ms
        | (0b1001 << 3)  // SADC: 12-bit, 8.5ms (Peut rester, mais est ignoré)
        | (0b0101 << 0); // MODE: Shunt and Bus, Continuous

    status = INA219_WriteReg(handle, INA219_REG_CONFIG, config_value);

    return status;
}

// --- Fonctions de Lecture ---

float INA219_GetBusVoltage_V(INA219_HandleTypedef *handle) {
    uint16_t raw_data;

    if (INA219_ReadReg(handle, INA219_REG_BUSVOLTAGE, &raw_data) != HAL_OK) {
        return 0.0f; // Retourne 0.0f ou NAN en cas d'erreur
    }

    // Le registre de tension (0x02) utilise les bits 3 à 15 (13 bits).
    // On décale de 3 bits à droite pour isoler la valeur.
    raw_data = raw_data >> 3;

    // Conversion : Valeur LSB (0.004V)
    // 2801 (valeur décalée de 0x578A) * 0.004 V/LSB = 11.204 V
    return (float)raw_data * 0.004f;
}

float INA219_GetShuntVoltage_V(INA219_HandleTypedef *handle) {
    uint16_t raw_data_u;
    int16_t raw_data; // Le registre de Shunt est signé

    if (INA219_ReadReg(handle, INA219_REG_SHUNTVOLTAGE, &raw_data_u) != HAL_OK) {
        return NAN;
    }
    raw_data = (int16_t)raw_data_u; // Conversion signée

    // Le registre VSHUNT (0x01) a un LSB de 10µV (0.00001V)
    return (float)raw_data * 0.00001f;
}

float INA219_GetCurrent_A(INA219_HandleTypedef *handle) {
    uint16_t raw_data_u; // Pour la lecture I2C
    int16_t current_signed; // Pour la conversion 2's complement

    // Le registre de courant est calculé automatiquement après la calibration
    if (INA219_ReadReg(handle, INA219_REG_CURRENT, &raw_data_u) != HAL_OK) {
        return NAN;
    }

    // Conversion de l'entier non signé (lu) en entier signé (complément à deux)
    // C'est souvent implicite en C, mais le rendre explicite aide au débogage
    current_signed = (int16_t)raw_data_u;

    // Si vous lisez la valeur maximale (32767), il y a une erreur de calibration
    // Si current_signed == 32767 ou -32768, la plage est dépassée ou il y a un problème.

    // Conversion: Valeur lue * Current_LSB
    return (float)current_signed * handle->current_lsb;
}
float INA219_GetPower_W(INA219_HandleTypedef *handle) {
    uint16_t raw_data_u;
    uint16_t raw_data; // Le registre de Puissance est non signé

    // Le registre de puissance est calculé automatiquement
    if (INA219_ReadReg(handle, INA219_REG_POWER, &raw_data_u) != HAL_OK) {
        return NAN;
    }
    raw_data = raw_data_u;

    // Conversion: Valeur lue * Power_LSB
    return (float)raw_data * handle->power_lsb;
}

// Fonction de Debug pour voir les registres bruts
void INA219_DumpRegisters(INA219_HandleTypedef *handle) {
    uint16_t reg_config, reg_shunt, reg_bus, reg_power, reg_current, reg_cal;

    INA219_ReadReg(handle, 0x00, &reg_config);      // Configuration
    INA219_ReadReg(handle, 0x01, &reg_shunt);       // Shunt Voltage
    INA219_ReadReg(handle, 0x02, &reg_bus);         // Bus Voltage
    INA219_ReadReg(handle, 0x03, &reg_power);       // Power
    INA219_ReadReg(handle, 0x04, &reg_current);     // Current
    INA219_ReadReg(handle, 0x05, &reg_cal);         // Calibration

    printf("\r\n--- INA219 DUMP ---\r\n");
    printf("00 Config : 0x%04X\r\n", reg_config);
    printf("01 Shunt  : 0x%04X (Brut)\r\n", reg_shunt);
    printf("02 Bus    : 0x%04X (Brut)\r\n", reg_bus);
    printf("03 Power  : 0x%04X\r\n", reg_power);
    printf("04 Current: 0x%04X\r\n", reg_current);
    printf("05 Calib  : 0x%04X\r\n", reg_cal);
    printf("-------------------\r\n");
}
