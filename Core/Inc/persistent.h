#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <string.h>
#define FLASH_STORAGE_SECTOR     FLASH_SECTOR_11       // Dernier secteur (~0x080E0000)
#define FLASH_USER_START_ADDR   0x08060000
#define FLASH_MAGIC              0xDEADBEEF            // Signature pour valider les données

typedef struct {
    uint32_t magic;                // Vérification d'intégrité
    double   home_latitude;        // Point de départ
    double   home_longitude;
    uint8_t  trappe_gauche_max;    // Angle maxi trappe gauche
    uint8_t  trappe_droite_max;    // Angle maxi trappe droite
    uint8_t  trappe_arriere_max;   // Angle maxi trappe arrière
    uint8_t  _pad[1];              // Alignement 32 bits
} PersistentData_t;

bool  Flash_Load(PersistentData_t *out);
bool  Flash_Save(const PersistentData_t *data);
void  Flash_Erase(void);

#endif
