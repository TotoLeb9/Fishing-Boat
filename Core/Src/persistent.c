/*
 * persistent.c
 *
 *  Created on: Apr 24, 2026
 *      Author: totoleb
 */
/*#include "persistent.h"

bool Flash_Load(PersistentData_t *out) {
    memcpy(out, (void*)FLASH_STORAGE_ADDR, sizeof(PersistentData_t));
    return (out->magic == FLASH_MAGIC);
}

void Flash_Erase(void) {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = FLASH_STORAGE_SECTOR,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t error;
    HAL_FLASHEx_Erase(&erase, &error);
    HAL_FLASH_Lock();
}
*/
