#include <font.h>
#include "display.h"
#include <string.h>
/*

HAL_StatusTypeDef SSD1315_SendCommand(uint8_t cmd)
{
    uint8_t buf[2] = {I2C_COMMAND, cmd};
    return HAL_I2C_Master_Transmit(&hi2c1, I2C_DEVICE_ADDR, buf, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef SSD1315_SendData(uint8_t *data, uint16_t size)
{
    // Alloue un buffer temporaire avec 1 octet de contrôle + données
    uint8_t buf[size + 1];
    buf[0] = I2C_DATA;          // indique données
    for(uint16_t i = 0; i < size; i++)
        buf[i + 1] = data[i];   // copie les données

    return HAL_I2C_Master_Transmit(&hi2c1, I2C_DEVICE_ADDR, buf, size + 1, HAL_MAX_DELAY);
}



void SSD1315_Init(void)
{
    SSD1315_SendCommand(0xAE); // Display OFF
    SSD1315_SendCommand(0xD5); // Set display clock divide ratio
    SSD1315_SendCommand(0x80); // Default

    SSD1315_SendCommand(0xA8); // Set multiplex ratio
    SSD1315_SendCommand(0x3F); // 64MUX

    SSD1315_SendCommand(0xD3); // Set display offset
    SSD1315_SendCommand(0x00); // No offset

    SSD1315_SendCommand(0x40); // Set start line = 0

    SSD1315_SendCommand(0x8D); // Charge pump
    SSD1315_SendCommand(0x14); // Enable

    SSD1315_SendCommand(0x20); // Memory addressing mode
    SSD1315_SendCommand(0x00); // Horizontal addressing

    SSD1315_SendCommand(0xA1); // Segment remap
    SSD1315_SendCommand(0xC8); // COM scan direction

    SSD1315_SendCommand(0xDA); // Set COM pins
    SSD1315_SendCommand(0x12);

    SSD1315_SendCommand(0x81); // Contrast
    SSD1315_SendCommand(0xCF);

    SSD1315_SendCommand(0xD9); // Pre-charge period
    SSD1315_SendCommand(0xF1);

    SSD1315_SendCommand(0xDB); // VCOMH deselect level
    SSD1315_SendCommand(0x40);

    SSD1315_SendCommand(0xA4); // Entire display ON
    SSD1315_SendCommand(0xA6); // Normal display

    SSD1315_SendCommand(0xAF); // Display ON
}

void display_clear()
{
    uint8_t pixels[128];
    for(int i=0;i<128;i++) pixels[i] = 0x00; // Éteint tous les pixels

    for(uint8_t page=0; page<8; page++) // 8 pages pour 64 pixels vertical
    {
        SSD1315_SendCommand(0xB0 | page); // Sélection page
        SSD1315_SendCommand(0x00);       // Colonne basse
        SSD1315_SendCommand(0x10);       // Colonne haute
        SSD1315_SendData(pixels, 128);
    }
}

void SSD1315_SetCursor(uint8_t page, uint8_t column)
{
    SSD1315_SendCommand(0xB0 | page);             // page address
    SSD1315_SendCommand(0x00 | (column & 0x0F)); // colonne basse
    SSD1315_SendCommand(0x10 | ((column>>4)&0x0F)); // colonne haute
}

void SSD1315_DrawChar(char c)
{
    uint8_t index;

    if(c == ' ') index = 0;
    else if(c >= 'A' && c <= 'Z') index = c - 'A' + 1;
    else if(c >= '0' && c <= '9') index = c - '0' + 27;
    else return; // caractère non supporté

    for(uint8_t i=0; i<5; i++)
    {
        SSD1315_SendData(&FONT_5X8[index][i], 1);
    }

    uint8_t spacer = 0x00;
    SSD1315_SendData(&spacer, 1); // colonne vide entre caractères
}



void SSD1315_DrawCharAt(uint8_t page, uint8_t column, char c, uint8_t v_offset)
{
    uint8_t index;

    if(c == ' ') index = 0;
    else if(c >= 'A' && c <= 'Z') index = c - 'A' + 1;
    else if(c >= '0' && c <= '9') index = c - '0' + 27;
    else return; // caractère non supporté

    // Si v_offset > 0, on écrit sur deux pages
    for(uint8_t col = 0; col < 5; col++)
    {
        uint8_t byte = FONT_5X8[index][col];

        if(v_offset == 0)
        {
            SSD1315_SetCursor(page, column + col);
            SSD1315_SendData(&byte, 1);
        }
        else
        {
            uint8_t upper = byte << v_offset;           // partie haute sur page actuelle
            uint8_t lower = byte >> (8 - v_offset);    // partie basse sur page suivante

            SSD1315_SetCursor(page, column + col);
            SSD1315_SendData(&upper, 1);

            SSD1315_SetCursor(page + 1, column + col);
            SSD1315_SendData(&lower, 1);
        }
    }

    // Colonne vide entre caractères
    uint8_t spacer = 0x00;
    SSD1315_SetCursor(page, column + 5);
    SSD1315_SendData(&spacer, 1);

    if(v_offset != 0)
    {
        SSD1315_SetCursor(page + 1, column + 5);
        SSD1315_SendData(&spacer, 1);
    }
}

void SSD1315_DrawString_At(const char *str,
                          uint8_t page,
                          uint8_t column,
                          uint8_t offset,
                          int align)
{
    if (str == NULL) return;

    uint8_t string_width = strlen(str) * 6; // 5px glyph + 1px spacing

    // Adjust column if the string would exceed display width
    if (string_width + column > 128) {
        column = 128 - string_width;
    }

    // Center alignment
    if (align == 1) {
        column = (128 - string_width) / 2;
    }

    // Draw characters sequentially
    while (*str) {
        SSD1315_DrawCharAt(page, column, *str, offset);
        column += 6;  // Advance to next character position
        str++;
    }
}
*/

