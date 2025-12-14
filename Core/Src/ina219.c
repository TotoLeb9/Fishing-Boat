#include "ina219.h"
#include <math.h> // Pour les calculs float

// --- Fonctions utilitaires I2C (Privées) ---

static HAL_StatusTypeDef INA219_WriteReg(INA219_HandleTypedef *handle, uint8_t reg, uint16_t data) {
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = (uint8_t)(data >> 8);
    buffer[2] = (uint8_t)(data & 0xFF);
    return HAL_I2C_Master_Transmit(handle->hi2c, INA219_I2C_ADDRESS, buffer, 3, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef INA219_ReadReg(INA219_HandleTypedef *handle, uint8_t reg, uint16_t *data) {
    HAL_StatusTypeDef status;
    uint8_t buffer[2];

    status = HAL_I2C_Master_Transmit(handle->hi2c, INA219_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    status = HAL_I2C_Master_Receive(handle->hi2c, INA219_I2C_ADDRESS, buffer, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    *data = (uint16_t)((buffer[0] << 8) | buffer[1]);
    return HAL_OK;
}


HAL_StatusTypeDef INA219_Init(INA219_HandleTypedef *handle,
                              I2C_HandleTypeDef *hi2c,
                              float shunt_ohm,
                              float current_max_amp) {
    HAL_StatusTypeDef status;

    handle->hi2c = hi2c;
    handle->shunt_resistance_ohm = shunt_ohm;


    handle->current_lsb = 0.0001f; // 100 microAmpère/bit

    float cal_float = 0.04096f / (handle->current_lsb * handle->shunt_resistance_ohm);
    handle->calibration_value = (uint16_t)roundf(cal_float);

    handle->power_lsb = 20.0f * handle->current_lsb;

    status = INA219_WriteReg(handle, INA219_REG_CALIBRATION, handle->calibration_value);
    if (status != HAL_OK) return status;

    uint16_t config_value =
          (0b0001 << 13)
        | (0b0011 << 11)
        | (0b1001 << 7)
        | (0b1001 << 3)
        | (0b0101 << 0);

    status = INA219_WriteReg(handle, INA219_REG_CONFIG, config_value);

    return status;
}


float INA219_GetBusVoltage_V(INA219_HandleTypedef *handle) {
    uint16_t raw_data;

    if (INA219_ReadReg(handle, INA219_REG_BUSVOLTAGE, &raw_data) != HAL_OK) {
        return 0.0f; // Retourne 0.0f ou NAN en cas d'erreur
    }
    raw_data = raw_data >> 3;

    return (float)raw_data * 0.004f;
}

float INA219_GetShuntVoltage_V(INA219_HandleTypedef *handle) {
    uint16_t raw_data_u;
    int16_t raw_data;

    if (INA219_ReadReg(handle, INA219_REG_SHUNTVOLTAGE, &raw_data_u) != HAL_OK) {
        return NAN;
    }
    raw_data = (int16_t)raw_data_u;

    return (float)raw_data * 0.00001f;
}

float INA219_GetCurrent_A(INA219_HandleTypedef *handle) {
    uint16_t raw_data_u;
    int16_t current_signed;

    if (INA219_ReadReg(handle, INA219_REG_CURRENT, &raw_data_u) != HAL_OK) {
        return NAN;
    }
    current_signed = (int16_t)raw_data_u;

    return (float)current_signed * handle->current_lsb;
}
float INA219_GetPower_W(INA219_HandleTypedef *handle) {
    uint16_t raw_data_u;
    uint16_t raw_data;

    if (INA219_ReadReg(handle, INA219_REG_POWER, &raw_data_u) != HAL_OK) {
        return NAN;
    }
    raw_data = raw_data_u;

    return (float)raw_data * handle->power_lsb;
}

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
