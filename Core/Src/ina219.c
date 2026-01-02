#include "ina219.h"
#include <math.h> // Pour les calculs float

// --- Fonctions utilitaires I2C (Privées) ---

 HAL_StatusTypeDef INA219_WriteReg(INA219_HandleTypedef *handle, uint8_t reg, uint16_t data) {
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = (uint8_t)(data >> 8);
    buffer[2] = (uint8_t)(data & 0xFF);
    return HAL_I2C_Master_Transmit(handle->hi2c, INA219_I2C_ADDRESS, buffer, 3, HAL_MAX_DELAY);
}

HAL_StatusTypeDef INA219_ReadReg(INA219_HandleTypedef *handle, uint8_t reg, uint16_t *data) {
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
                              float current_max_amp)
{
	handle->hi2c = hi2c;

	    // 0x2192 configure :
	    // - Range 32V
	    // - ADC 12-bit (très précis)
	    // - Mode : Bus Voltage Continuous (le capteur mesure tout seul sans s'arrêter)
	    uint16_t config = 0x2192;

	    return INA219_WriteReg(handle, INA219_REG_CONFIG, config);
}

void INA219_DiagnosticTest(INA219_HandleTypedef *handle) {
    uint16_t config, bus_raw;

    printf("\r\n=== INA219 DIAGNOSTIC ===\r\n");

    // Lire la configuration actuelle
    if (INA219_ReadReg(handle, INA219_REG_CONFIG, &config) == HAL_OK) {
        printf("Config Register: 0x%04X\r\n", config);
    } else {
        printf("ERREUR: Impossible de lire CONFIG\r\n");
    }

    // Lire le registre bus voltage
    if (INA219_ReadReg(handle, INA219_REG_BUSVOLTAGE, &bus_raw) == HAL_OK) {
        printf("Bus Voltage RAW: 0x%04X\r\n", bus_raw);

        // Vérifier le bit CNVR (Conversion Ready) - bit 1
        if (bus_raw & 0x0002) {
            printf("  -> Conversion Ready: OUI\r\n");
        } else {
            printf("  -> Conversion Ready: NON (pas de mesure valide!)\r\n");
        }

        // Vérifier le bit OVF (Overflow) - bit 0
        if (bus_raw & 0x0001) {
            printf("  -> Math Overflow: OUI (dépassement!)\r\n");
        }

        uint16_t voltage_value = bus_raw >> 3;
        float voltage = voltage_value * 0.004f;
        printf("  -> Voltage calculé: %.3f V\r\n", voltage);

    } else {
        printf("ERREUR: Impossible de lire BUS VOLTAGE\r\n");
    }

    printf("========================\r\n\r\n");
}

float INA219_GetBusVoltage_V(INA219_HandleTypedef *handle) {
    uint16_t raw;
    // On lit le registre 0x02
    if (INA219_ReadReg(handle, INA219_REG_BUSVOLTAGE, &raw) != HAL_OK) return 0.0f;

    // 1. On décale de 3 bits vers la droite
    uint16_t data = raw >> 3;

    // 2. On multiplie par 4mV (0.004)
    float voltage = (float)data * 0.004f;

    return voltage;
}

float INA219_GetVoltage(INA219_HandleTypedef *handle) {
    uint16_t raw;
    if (INA219_ReadReg(handle, INA219_REG_BUSVOLTAGE, &raw) != HAL_OK) return 0.0f;

    // On décale de 3 bits vers la droite pour obtenir la valeur réelle
    // Le LSB (le cran minimum) est toujours de 4mV
    uint16_t value = raw >> 3;

    return (float)value * 0.004f;
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
