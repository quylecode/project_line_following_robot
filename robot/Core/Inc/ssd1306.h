/**
  ******************************************************************************

  OLED For STM32F446RE
  Author:   LVQ
  Updated:  1st July 2022

  ******************************************************************************
*/

#ifndef __SSD1306_H
#define __SSD1306_H 

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

/**
 * This LVQ_SSD1306 LCD uses I2C for communication
 *
 * Library features functions for drawing lines, rectangles and circles.
 *
 * It also allows you to draw texts and characters using appropriate functions provided in library.
 *
 * Default pinout
 *
LVQ_SSD1306    |STM32F10x    |DESCRIPTION

VCC        |3.3V         |
GND        |GND          |
SCL        |PB6          |Serial clock line
SDA        |PB7          |Serial data line
 */

#include "main.h"
#include "fonts.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f1xx_hal_i2c.h"

extern I2C_HandleTypeDef hi2c1;

/* I2C address */
#ifndef LVQ_SSD1306_I2C_ADDR
#define LVQ_SSD1306_I2C_ADDR         0x78
//#define LVQ_SSD1306_I2C_ADDR       0x7A
#endif

/* I2Cx */
#ifndef LVQ_SSD1306_I2Cx
#define LVQ_SSD1306_I2Cx             hi2c1
#endif

/* I2C Pinspack */
#ifndef LVQ_SSD1306_PINSPACK
#define LVQ_SSD1306_PINSPACK         LVQ_I2C_PinsPack_2
#endif

/* I2C Clock */
#ifndef LVQ_SSD1306_CLOCK
#define LVQ_SSD1306_CLOCK            LVQ_I2C_CLOCK_FAST_MODE
#endif

/* LVQ_SSD1306 settings */
/* LVQ_SSD1306 width in pixels */
#ifndef LVQ_SSD1306_WIDTH
#define LVQ_SSD1306_WIDTH            128
#endif
/* LVQ_SSD1306 LCD height in pixels */
#ifndef LVQ_SSD1306_HEIGHT
#define LVQ_SSD1306_HEIGHT           64
#endif

/**
 * @brief  LVQ_SSD1306 color enumeration
 */
typedef enum {
	LVQ_SSD1306_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
	LVQ_SSD1306_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
} LVQ_SSD1306_COLOR_t;



/**
 * @brief  Initializes LVQ_SSD1306 LCD
 * @param  None
 * @retval Initialization status:
 *           - 0: LCD was not detected on I2C port
 *           - > 0: LCD initialized OK and ready to use
 */
uint8_t LVQ_SSD1306_Init(void);

/** 
 * @brief  Updates buffer from internal RAM to LCD
 * @note   This function must be called each time you do some changes to LCD, to update buffer from RAM to LCD
 * @param  None
 * @retval None
 */
void LVQ_SSD1306_UpdateScreen(void);

/**
 * @brief  Toggles pixels invertion inside internal RAM
 * @note   @ref LVQ_SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  None
 * @retval None
 */
void LVQ_SSD1306_ToggleInvert(void);

/** 
 * @brief  Fills entire LCD with desired color
 * @note   @ref LVQ_SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  Color: Color to be used for screen fill. This parameter can be a value of @ref LVQ_SSD1306_COLOR_t enumeration
 * @retval None
 */
void LVQ_SSD1306_Fill(LVQ_SSD1306_COLOR_t Color);

/**
 * @brief  Draws pixel at desired location
 * @note   @ref LVQ_SSD1306_UpdateScreen() must called after that in order to see updated LCD screen
 * @param  x: X location. This parameter can be a value between 0 and LVQ_SSD1306_WIDTH - 1
 * @param  y: Y location. This parameter can be a value between 0 and LVQ_SSD1306_HEIGHT - 1
 * @param  color: Color to be used for screen fill. This parameter can be a value of @ref LVQ_SSD1306_COLOR_t enumeration
 * @retval None
 */
void LVQ_SSD1306_DrawPixel(uint16_t x, uint16_t y, LVQ_SSD1306_COLOR_t color);

/**
 * @brief  Sets cursor pointer to desired location for strings
 * @param  x: X location. This parameter can be a value between 0 and LVQ_SSD1306_WIDTH - 1
 * @param  y: Y location. This parameter can be a value between 0 and LVQ_SSD1306_HEIGHT - 1
 * @retval None
 */
void LVQ_SSD1306_GotoXY(uint16_t x, uint16_t y);

/**
 * @brief  Puts character to internal RAM
 * @note   @ref LVQ_SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  ch: Character to be written
 * @param  *Font: Pointer to @ref FontDef_t structure with used font
 * @param  color: Color used for drawing. This parameter can be a value of @ref LVQ_SSD1306_COLOR_t enumeration
 * @retval Character written
 */
char LVQ_SSD1306_Putc(char ch, FontDef_t* Font, LVQ_SSD1306_COLOR_t color);

/**
 * @brief  Puts string to internal RAM
 * @note   @ref LVQ_SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  *str: String to be written
 * @param  *Font: Pointer to @ref FontDef_t structure with used font
 * @param  color: Color used for drawing. This parameter can be a value of @ref LVQ_SSD1306_COLOR_t enumeration
 * @retval Zero on success or character value when function failed
 */
char LVQ_SSD1306_Puts(char* str, FontDef_t* Font, LVQ_SSD1306_COLOR_t color);

/**
 * @brief  Draws line on LCD
 * @note   @ref LVQ_SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x0: Line X start point. Valid input is 0 to LVQ_SSD1306_WIDTH - 1
 * @param  y0: Line Y start point. Valid input is 0 to LVQ_SSD1306_HEIGHT - 1
 * @param  x1: Line X end point. Valid input is 0 to LVQ_SSD1306_WIDTH - 1
 * @param  y1: Line Y end point. Valid input is 0 to LVQ_SSD1306_HEIGHT - 1
 * @param  c: Color to be used. This parameter can be a value of @ref LVQ_SSD1306_COLOR_t enumeration
 * @retval None
 */
void LVQ_SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, LVQ_SSD1306_COLOR_t c);

/**
 * @brief  Draws rectangle on LCD
 * @note   @ref LVQ_SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: Top left X start point. Valid input is 0 to LVQ_SSD1306_WIDTH - 1
 * @param  y: Top left Y start point. Valid input is 0 to LVQ_SSD1306_HEIGHT - 1
 * @param  w: Rectangle width in units of pixels
 * @param  h: Rectangle height in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref LVQ_SSD1306_COLOR_t enumeration
 * @retval None
 */
void LVQ_SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, LVQ_SSD1306_COLOR_t c);

/**
 * @brief  Draws filled rectangle on LCD
 * @note   @ref LVQ_SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: Top left X start point. Valid input is 0 to LVQ_SSD1306_WIDTH - 1
 * @param  y: Top left Y start point. Valid input is 0 to LVQ_SSD1306_HEIGHT - 1
 * @param  w: Rectangle width in units of pixels
 * @param  h: Rectangle height in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref LVQ_SSD1306_COLOR_t enumeration
 * @retval None
 */
void LVQ_SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, LVQ_SSD1306_COLOR_t c);

/**
 * @brief  Draws triangle on LCD
 * @note   @ref LVQ_SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x1: First coordinate X location. Valid input is 0 to LVQ_SSD1306_WIDTH - 1
 * @param  y1: First coordinate Y location. Valid input is 0 to LVQ_SSD1306_HEIGHT - 1
 * @param  x2: Second coordinate X location. Valid input is 0 to LVQ_SSD1306_WIDTH - 1
 * @param  y2: Second coordinate Y location. Valid input is 0 to LVQ_SSD1306_HEIGHT - 1
 * @param  x3: Third coordinate X location. Valid input is 0 to LVQ_SSD1306_WIDTH - 1
 * @param  y3: Third coordinate Y location. Valid input is 0 to LVQ_SSD1306_HEIGHT - 1
 * @param  c: Color to be used. This parameter can be a value of @ref LVQ_SSD1306_COLOR_t enumeration
 * @retval None
 */
void LVQ_SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, LVQ_SSD1306_COLOR_t color);

/**
 * @brief  Draws circle to STM buffer
 * @note   @ref LVQ_SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: X location for center of circle. Valid input is 0 to LVQ_SSD1306_WIDTH - 1
 * @param  y: Y location for center of circle. Valid input is 0 to LVQ_SSD1306_HEIGHT - 1
 * @param  r: Circle radius in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref LVQ_SSD1306_COLOR_t enumeration
 * @retval None
 */
void LVQ_SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, LVQ_SSD1306_COLOR_t c);

/**
 * @brief  Draws filled circle to STM buffer
 * @note   @ref LVQ_SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: X location for center of circle. Valid input is 0 to LVQ_SSD1306_WIDTH - 1
 * @param  y: Y location for center of circle. Valid input is 0 to LVQ_SSD1306_HEIGHT - 1
 * @param  r: Circle radius in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref LVQ_SSD1306_COLOR_t enumeration
 * @retval None
 */
void LVQ_SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, LVQ_SSD1306_COLOR_t c);



#ifndef LVQ_SSD1306_I2C_TIMEOUT
#define LVQ_SSD1306_I2C_TIMEOUT					20000
#endif

void LVQ_SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);

// scroll the screen for fixed rows

void LVQ_SSD1306_ScrollRight(uint8_t start_row, uint8_t end_row);

void LVQ_SSD1306_ScrollLeft(uint8_t start_row, uint8_t end_row);

void LVQ_SSD1306_Scrolldiagright(uint8_t start_row, uint8_t end_row);

void LVQ_SSD1306_Scrolldiagleft(uint8_t start_row, uint8_t end_row);

void LVQ_SSD1306_Stopscroll(void);

// inverts the display i = 1->inverted, i = 0->normal

void LVQ_SSD1306_InvertDisplay (int i);

// clear the display

void LVQ_SSD1306_Clear (void);

void LVQ_SSD1306_ON(void);

void LVQ_SSD1306_OFF(void);

void Review_member(void);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
