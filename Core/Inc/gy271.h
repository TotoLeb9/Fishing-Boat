/*
 * qmc5883p.h
 *
 *  Created on: Jan 17, 2026
 *      Author: totoleb
 */

#ifndef INC_QMC5883P_H_
#define INC_QMC5883P_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "log.h"
/* Adresse I2C du QMC5883P */
#define QMC5883P_ADDR      (0x2C << 1)

/* Registres */
#define QMC_REG_CHIP_ID        0x00
#define QMC_REG_DATA_OUT_X_LSB 0x01
#define QMC_REG_DATA_OUT_X_MSB 0x02
#define QMC_REG_DATA_OUT_Y_LSB 0x03
#define QMC_REG_DATA_OUT_Y_MSB 0x04
#define QMC_REG_DATA_OUT_Z_LSB 0x05
#define QMC_REG_DATA_OUT_Z_MSB 0x06
#define QMC_REG_STATUS         0x09
#define QMC_REG_CTL1           0x0A
#define QMC_REG_CTL2           0x0B

/* Chip ID attendu */
#define QMC5883P_CHIP_ID       0x80

/* Codes de retour */
#define QMC_OK    0
#define QMC_ERROR 1

/* Structure de données */
typedef struct {
    float x;
    float y;
    float z;
    float heading;
} QMC5883P_Data_t;

/* Structure de calibration */
typedef struct {
    float offsetX;
    float offsetY;
    float offsetZ;
    float scaleX;
    float scaleY;
    float scaleZ;
} QMC5883P_Calib_t;

/* Structure interne du capteur */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    QMC5883P_Calib_t calib;
    int16_t lastRawX;
    int16_t lastRawY;
    int16_t lastRawZ;
    uint32_t lastReadTime;
    uint32_t minInterval;  // ms
} QMC5883P_t;

extern QMC5883P_t mag;
extern QMC5883P_Data_t magData;

/* Prototypes */
uint8_t QMC5883P_Init(QMC5883P_t *dev, I2C_HandleTypeDef *hi2c);
uint8_t QMC5883P_ReadRaw(QMC5883P_t *dev);
uint8_t QMC5883P_ReadXYZ(QMC5883P_t *dev, QMC5883P_Data_t *data);
float QMC5883P_GetHeadingDeg(QMC5883P_t *dev, float declDeg);
void QMC5883P_SetHardIronOffsets(QMC5883P_t *dev, float xOff, float yOff, float zOff);
void QMC5883P_SetSoftIronScales(QMC5883P_t *dev, float scaleX, float scaleY, float scaleZ);
void CompassCalibration(void);
#endif /* INC_QMC5883P_H_ */
