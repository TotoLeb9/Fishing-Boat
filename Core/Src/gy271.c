/*
 * qmc5883p.c
 *
 *  Created on: Jan 17, 2026
 *      Author: totoleb
 */

#include "gy271.h"/*
 * qmc5883p.c
 *
 *  Created on: Jan 17, 2026
 *      Author: totoleb
 */
/* Fonctions internes */
static uint8_t QMC_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint8_t len);
static uint8_t QMC_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val);

/**
 * @brief Initialise le capteur QMC5883P
 * @param dev Pointeur vers la structure du capteur
 * @param hi2c Pointeur vers le handle I2C
 * @return QMC_OK si succès, QMC_ERROR sinon
 */
uint8_t QMC5883P_Init(QMC5883P_t *dev, I2C_HandleTypeDef *hi2c)
{
    uint8_t chip_id;

    dev->hi2c = hi2c;

    // Initialiser les paramètres de calibration par défaut
    dev->calib.offsetX = 0.0f;
    dev->calib.offsetY = 0.0f;
    dev->calib.offsetZ = 0.0f;
    dev->calib.scaleX = 1.0f;
    dev->calib.scaleY = 1.0f;
    dev->calib.scaleZ = 1.0f;

    dev->lastRawX = 0;
    dev->lastRawY = 0;
    dev->lastRawZ = 0;
    dev->lastReadTime = 0;
    dev->minInterval = 5;  // 5 ms minimum entre les lectures

    // Vérifier le Chip ID
    if (QMC_ReadReg(hi2c, QMC_REG_CHIP_ID, &chip_id, 1) != QMC_OK)
    {
        return QMC_ERROR;
    }

    if (chip_id != QMC5883P_CHIP_ID)
    {
        return QMC_ERROR;  // Chip ID incorrect
    }

    // Configuration standard : 200 Hz, Continuous, ±2G
    // Registre 0x0D = 0x40
    if (QMC_WriteReg(hi2c, 0x0D, 0x40) != QMC_OK) return QMC_ERROR;
    HAL_Delay(10);

    // Registre 0x29 = 0x06
    if (QMC_WriteReg(hi2c, 0x29, 0x06) != QMC_OK) return QMC_ERROR;
    HAL_Delay(10);

    // CTL1 = 0xCF (200Hz, Continuous, ±2G, OSR=512)
    if (QMC_WriteReg(hi2c, QMC_REG_CTL1, 0xCF) != QMC_OK) return QMC_ERROR;
    HAL_Delay(10);

    // CTL2 = 0x00
    if (QMC_WriteReg(hi2c, QMC_REG_CTL2, 0x00) != QMC_OK) return QMC_ERROR;
    HAL_Delay(10);

    return QMC_OK;
}

/**
 * @brief Lit les données brutes du capteur
 * @param dev Pointeur vers la structure du capteur
 * @return QMC_OK si nouvelles données lues, QMC_ERROR sinon
 */
uint8_t QMC5883P_ReadRaw(QMC5883P_t *dev)
{
    uint32_t now = HAL_GetTick();
    uint8_t status;
    uint8_t buf[6];

    // Vérifier l'intervalle minimum
    if ((now - dev->lastReadTime) < dev->minInterval)
    {
        return QMC_ERROR;  // Utiliser le cache
    }

    // Lire le registre de status
    if (QMC_ReadReg(dev->hi2c, QMC_REG_STATUS, &status, 1) != QMC_OK)
    {
        return QMC_ERROR;
    }

    // Vérifier si de nouvelles données sont disponibles (bit 0)
    if (!(status & 0x01))
    {
        return QMC_ERROR;  // Pas de nouvelles données
    }

    // Lire les 6 octets de données (X, Y, Z en LSB-MSB)
    if (QMC_ReadReg(dev->hi2c, QMC_REG_DATA_OUT_X_LSB, buf, 6) != QMC_OK)
    {
        return QMC_ERROR;
    }

    // Reconstruction des valeurs 16 bits signées (LSB d'abord)
    dev->lastRawX = (int16_t)((buf[1] << 8) | buf[0]);
    dev->lastRawY = (int16_t)((buf[3] << 8) | buf[2]);
    dev->lastRawZ = (int16_t)((buf[5] << 8) | buf[4]);

    dev->lastReadTime = now;

    return QMC_OK;
}

/**
 * @brief Lit les données calibrées en µT
 * @param dev Pointeur vers la structure du capteur
 * @param data Pointeur vers la structure de données
 * @return QMC_OK si nouvelles données, QMC_ERROR sinon
 */
uint8_t QMC5883P_ReadXYZ(QMC5883P_t *dev, QMC5883P_Data_t *data)
{
    if (QMC5883P_ReadRaw(dev) != QMC_OK)
    {
        return QMC_ERROR;  // Pas de nouvelles données
    }

    // Conversion des données brutes en µT (diviser par 1000)
    float x = dev->lastRawX / 1000.0f;
    float y = dev->lastRawY / 1000.0f;
    float z = dev->lastRawZ / 1000.0f;

    // Appliquer la calibration (Hard Iron + Soft Iron)
    data->x = (x - dev->calib.offsetX) * dev->calib.scaleX;
    data->y = (y - dev->calib.offsetY) * dev->calib.scaleY;
    data->z = (z - dev->calib.offsetZ) * dev->calib.scaleZ;

    return QMC_OK;
}

/**
 * @brief Calcule le cap magnétique en degrés
 * @param dev Pointeur vers la structure du capteur
 * @param declDeg Déclinaison magnétique en degrés
 * @return Cap en degrés (0-360)
 */
float QMC5883P_GetHeadingDeg(QMC5883P_t *dev, float declDeg)
{
    // Tenter de lire de nouvelles données (utilise le cache si pas disponible)
    QMC5883P_ReadRaw(dev);

    // Conversion et calibration
    float x = (dev->lastRawX / 1000.0f - dev->calib.offsetX) * dev->calib.scaleX;
    float y = (dev->lastRawY / 1000.0f - dev->calib.offsetY) * dev->calib.scaleY;

    // Calcul de l'angle de base (-π à π)
    float heading = atan2f(y, x);

    // Ajouter la déclinaison magnétique (deg → rad)
    heading += declDeg * (M_PI / 180.0f);

    // Normaliser sur 0 à 2π
    if (heading < 0)
    {
        heading += 2.0f * M_PI;
    }
    else if (heading > 2.0f * M_PI)
    {
        heading -= 2.0f * M_PI;
    }

    // Convertir en degrés
    return heading * (180.0f / M_PI);
}

/**
 * @brief Configure les offsets Hard Iron
 * @param dev Pointeur vers la structure du capteur
 * @param xOff Offset X
 * @param yOff Offset Y
 * @param zOff Offset Z
 */
void QMC5883P_SetHardIronOffsets(QMC5883P_t *dev, float xOff, float yOff, float zOff)
{
    dev->calib.offsetX = xOff;
    dev->calib.offsetY = yOff;
    dev->calib.offsetZ = zOff;
}

/**
 * @brief Configure les échelles Soft Iron
 * @param dev Pointeur vers la structure du capteur
 * @param scaleX Échelle X
 * @param scaleY Échelle Y
 * @param scaleZ Échelle Z
 */
void QMC5883P_SetSoftIronScales(QMC5883P_t *dev, float scaleX, float scaleY, float scaleZ)
{
    dev->calib.scaleX = scaleX;
    dev->calib.scaleY = scaleY;
    dev->calib.scaleZ = scaleZ;
}

/**
 * @brief Lit un ou plusieurs registres
 */
static uint8_t QMC_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (HAL_I2C_Mem_Read(hi2c, QMC5883P_ADDR, reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len, 100) != HAL_OK)
    {
        return QMC_ERROR;
    }
    return QMC_OK;
}

/**
 * @brief Écrit dans un registre
 */
static uint8_t QMC_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    if (HAL_I2C_Mem_Write(hi2c, QMC5883P_ADDR, reg,
                          I2C_MEMADD_SIZE_8BIT, &val, 1, 100) != HAL_OK)
    {
        return QMC_ERROR;
    }
    return QMC_OK;
}

/**
 * @brief Calibration automatique de la boussole
 * @details Fait tourner lentement sur 360° pendant la calibration
 */
void CompassCalibration(void)
{
    float min_x = 10000.0f, max_x = -10000.0f;
    float min_y = 10000.0f, max_y = -10000.0f;
    float min_z = 10000.0f, max_z = -10000.0f;

    uint32_t samples = 0;

    LOG_INFO("\r\n========================================\r\n");
    LOG_INFO("      CALIBRATION BOUSSOLE QMC5883P\r\n");
    LOG_INFO("========================================\r\n");
    LOG_INFO("ATTENTION: Eloignez tous les aimants!\r\n");
    LOG_INFO("Tournez LENTEMENT le robot sur 360°\r\n");
    LOG_INFO("Duree: 30 secondes\r\n");
    LOG_INFO("Demarrage dans 3 secondes...\r\n");
    HAL_Delay(3000);

    LOG_INFO("DEBUT CALIBRATION!\r\n");

    uint32_t start = HAL_GetTick();
    uint32_t last_info = start;

    while ((HAL_GetTick() - start) < 30000)
    {
        if (QMC5883P_ReadXYZ(&mag, &magData) == QMC_OK)
        {
            // Trouver min/max pour chaque axe
            if (magData.x < min_x) min_x = magData.x;
            if (magData.x > max_x) max_x = magData.x;
            if (magData.y < min_y) min_y = magData.y;
            if (magData.y > max_y) max_y = magData.y;
            if (magData.z < min_z) min_z = magData.z;
            if (magData.z > max_z) max_z = magData.z;

            samples++;

            if ((HAL_GetTick() - last_info) >= 5000)
            {
                LOG_INFO("Progression: %lu/30 sec | Echantillons: %lu\r\n",
                         (HAL_GetTick() - start) / 1000, samples);
                last_info = HAL_GetTick();
            }
        }
        //HAL_Delay(50);
    }

    LOG_INFO("CALIBRATION TERMINEE!\r\n");
    LOG_INFO("Echantillons collectes: %lu\r\n\r\n", samples);
    float offset_x = (max_x + min_x) / 2.0f;
    float offset_y = (max_y + min_y) / 2.0f;
    float offset_z = (max_z + min_z) / 2.0f;
    float range_x = max_x - min_x;
    float range_y = max_y - min_y;
    float range_z = max_z - min_z;
    float avg_range = (range_x + range_y + range_z) / 3.0f;
    float scale_x = avg_range / range_x;
    float scale_y = avg_range / range_y;
    float scale_z = avg_range / range_z;
    LOG_INFO("========================================\r\n");
    LOG_INFO("      RESULTATS CALIBRATION\r\n");
    LOG_INFO("========================================\r\n");
    LOG_INFO("Min/Max collectes:\r\n");
    LOG_INFO("  X: %.2f / %.2f (range: %.2f)\r\n", min_x, max_x, range_x);
    LOG_INFO("  Y: %.2f / %.2f (range: %.2f)\r\n", min_y, max_y, range_y);
    LOG_INFO("  Z: %.2f / %.2f (range: %.2f)\r\n\r\n", min_z, max_z, range_z);
    LOG_INFO("VALEURS A COPIER DANS VOTRE CODE:\r\n");
    LOG_INFO("----------------------------------------\r\n");
    LOG_INFO("QMC5883P_SetHardIronOffsets(&mag, %.2ff, %.2ff, %.2ff);\r\n",
             offset_x, offset_y, offset_z);
    LOG_INFO("QMC5883P_SetSoftIronScales(&mag, %.3ff, %.3ff, %.3ff);\r\n",
             scale_x, scale_y, scale_z);
    LOG_INFO("========================================\r\n\r\n");
    QMC5883P_SetHardIronOffsets(&mag, offset_x, offset_y, offset_z);
    QMC5883P_SetSoftIronScales(&mag, scale_x, scale_y, scale_z);

    LOG_INFO("Calibration appliquee!\r\n");
}
