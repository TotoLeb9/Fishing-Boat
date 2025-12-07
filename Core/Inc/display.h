/**
 ******************************************************************************
 * @file    display.h
 * @author  TotoLeb
 * @date    27-Sept-2025
 * @brief   Driver for the SSD1315 module
 *
 * This header provides all functions required to communicate with the display module via I2C.
 * Those function allow you to put your text on the display wherever you want
 *
 * @attention
 *  Each character is 5 columns wide and 8 pixels tall.
 *  An empty column should be added between two characters for spacing.
 ******************************************************************************
 */

#include <stdint.h>
#include "main.h"
#define I2C_DEVICE_ADDR (0x3C << 1) // HAL I2C need 8 bytes addresses, so we just shift it to the left
#define I2C_COMMAND 0x00 // The next byte is a command
#define I2C_DATA 0x40 // The next byte is data
#define I2C_DISPLAY_OFF 0xAE // Command to turn off the display
#define I2C_DISPLAY_ON 0xAF // Command to turn on the display
extern const uint8_t Font5x7[][5];

/***********
 * @brief   Send data to put on display to the SSD1315
 * @param uint8_t *data : Data to send
 * 		  uint16_t size : Size of data
 * @return HAL status (HAL_OK etc...)
 ***********/
HAL_StatusTypeDef SSD1315_SendData(uint8_t *data, uint16_t size);

/***********
 * @brief   Send command to the SSD1315, Display_on , display_off etc...
 * @param uint8_t cmd : Command to send
 * @return HAL status (HAL_OK etc...)
 ***********/
HAL_StatusTypeDef SSD1315_SendCommand(uint8_t cmd);

/***********
 * @brief   Erase the display
 * @return void
 ***********/
void display_clear();

/***********
 * @brief Init the SSD1315 module
 * @return void
 ***********/
void SSD1315_Init(void);

/***********
 * @brief Write a single character at a specific position
 * @param page      Page index where the character starts (0–7).
 *                  Each page corresponds to 8 vertical pixels.
 * @param column    Column index where the character starts (0–127).
 * @param c         ASCII character to draw ('A'–'Z', '0'–'9', space).
 *                  Unsupported characters are ignored.
 * @param v_offset  Vertical offset in pixels (0–7).
 *                  - 0: normal (aligned to the page)
 *                  - >0: shifts the character down, splitting it across two pages
 *
 * @note A 1-pixel wide empty column is automatically added after the character
 *       to provide spacing between characters.
 *
 * @return void
 ***********/
void SSD1315_DrawCharAt(uint8_t page, uint8_t column, char c, uint8_t v_offset);

/***********
 * @brief Write a string at a specific position
  * @param str       Pointer to a null-terminated ASCII string.
 *                  Supported characters: 'A'–'Z', '0'–'9', and space.
 * @param page      Starting page index (0–7).
 *                  Each page corresponds to 8 vertical pixels.
 * @param column    Starting column index (0–127).
 * @param offset    Vertical offset in pixels (0–7).
 *                  - 0: normal alignment (characters fit in one page)
 *                  - >0: shifts characters down, split across two pages
 *
 * @note A 1-pixel wide empty column is automatically added after the character
 *       to provide spacing between characters.
 *
 * @return void
 ***********/
void SSD1315_DrawString_At(const char *str, uint8_t page, uint8_t column,uint8_t offset, int align);
