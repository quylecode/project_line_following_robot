/**
  ******************************************************************************

  OLED For STM32F446RE
  Author:   LVQ
  Updated:  1st July 2022

  ******************************************************************************
*/

#include "ssd1306.h"
#include "logobachkhoa.h"

/* Write command */
void LVQ_SSD1306_WRITECOMMAND(uint8_t command)     
{
		HAL_I2C_Mem_Write(&hi2c1, LVQ_SSD1306_I2C_ADDR, 0x00, 1, &command, 1, 1000);  
}

/* Write data */
void LVQ_SSD1306_WRITEDATA(uint8_t data)            
{ 
		HAL_I2C_Mem_Write(&hi2c1, LVQ_SSD1306_I2C_ADDR, 0x40, 1, &data, 1, 1000);  
}

/* Absolute value */
#define ABS(x)   ((x) > 0 ? (x) : -(x))

/* LVQ_SSD1306 data buffer */
static uint8_t LVQ_SSD1306_Buffer[LVQ_SSD1306_WIDTH * LVQ_SSD1306_HEIGHT / 8];

/* Private LVQ_SSD1306 structure */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} LVQ_SSD1306_t;

/* Private variable */
static LVQ_SSD1306_t LVQ_SSD1306;

#define LVQ_SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26
#define LVQ_SSD1306_LEFT_HORIZONTAL_SCROLL               0x27
#define LVQ_SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define LVQ_SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A
#define LVQ_SSD1306_DEACTIVATE_SCROLL                    0x2E // Stop scroll
#define LVQ_SSD1306_ACTIVATE_SCROLL                      0x2F // Start scroll
#define LVQ_SSD1306_SET_VERTICAL_SCROLL_AREA             0xA3 // Set scroll range

#define LVQ_SSD1306_NORMALDISPLAY       0xA6
#define LVQ_SSD1306_INVERTDISPLAY       0xA7

void LVQ_SSD1306_ScrollRight(uint8_t start_row, uint8_t end_row)
{
  LVQ_SSD1306_WRITECOMMAND (LVQ_SSD1306_RIGHT_HORIZONTAL_SCROLL);  // send 0x26
  LVQ_SSD1306_WRITECOMMAND (0x00);  // send dummy
  LVQ_SSD1306_WRITECOMMAND(start_row);  // start page address
  LVQ_SSD1306_WRITECOMMAND(0X00);  // time interval 5 frames
  LVQ_SSD1306_WRITECOMMAND(end_row);  // end page address
  LVQ_SSD1306_WRITECOMMAND(0X00);
  LVQ_SSD1306_WRITECOMMAND(0XFF);
  LVQ_SSD1306_WRITECOMMAND (LVQ_SSD1306_ACTIVATE_SCROLL); // start scroll
}

void LVQ_SSD1306_ScrollLeft(uint8_t start_row, uint8_t end_row)
{
  LVQ_SSD1306_WRITECOMMAND (LVQ_SSD1306_LEFT_HORIZONTAL_SCROLL);  // send 0x26
  LVQ_SSD1306_WRITECOMMAND (0x00);  // send dummy
  LVQ_SSD1306_WRITECOMMAND(start_row);  // start page address
  LVQ_SSD1306_WRITECOMMAND(0X00);  // time interval 5 frames
  LVQ_SSD1306_WRITECOMMAND(end_row);  // end page address
  LVQ_SSD1306_WRITECOMMAND(0X00);
  LVQ_SSD1306_WRITECOMMAND(0XFF);
  LVQ_SSD1306_WRITECOMMAND (LVQ_SSD1306_ACTIVATE_SCROLL); // start scroll
}

void LVQ_SSD1306_Scrolldiagright(uint8_t start_row, uint8_t end_row)
{
  LVQ_SSD1306_WRITECOMMAND(LVQ_SSD1306_SET_VERTICAL_SCROLL_AREA);  // sect the area
  LVQ_SSD1306_WRITECOMMAND (0x00);   // write dummy
  LVQ_SSD1306_WRITECOMMAND(LVQ_SSD1306_HEIGHT);

  LVQ_SSD1306_WRITECOMMAND(LVQ_SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
  LVQ_SSD1306_WRITECOMMAND (0x00);
  LVQ_SSD1306_WRITECOMMAND(start_row);
  LVQ_SSD1306_WRITECOMMAND(0X00);
  LVQ_SSD1306_WRITECOMMAND(end_row);
  LVQ_SSD1306_WRITECOMMAND (0x01);
  LVQ_SSD1306_WRITECOMMAND (LVQ_SSD1306_ACTIVATE_SCROLL);
}

void LVQ_SSD1306_Scrolldiagleft(uint8_t start_row, uint8_t end_row)
{
  LVQ_SSD1306_WRITECOMMAND(LVQ_SSD1306_SET_VERTICAL_SCROLL_AREA);  // sect the area
  LVQ_SSD1306_WRITECOMMAND (0x00);   // write dummy
  LVQ_SSD1306_WRITECOMMAND(LVQ_SSD1306_HEIGHT);

  LVQ_SSD1306_WRITECOMMAND(LVQ_SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
  LVQ_SSD1306_WRITECOMMAND (0x00);
  LVQ_SSD1306_WRITECOMMAND(start_row);
  LVQ_SSD1306_WRITECOMMAND(0X00);
  LVQ_SSD1306_WRITECOMMAND(end_row);
  LVQ_SSD1306_WRITECOMMAND (0x01);
  LVQ_SSD1306_WRITECOMMAND (LVQ_SSD1306_ACTIVATE_SCROLL);
}

void LVQ_SSD1306_Stopscroll(void)
{
	LVQ_SSD1306_WRITECOMMAND(LVQ_SSD1306_DEACTIVATE_SCROLL);
}

void LVQ_SSD1306_InvertDisplay (int i)
{
  if(i) 
	{
		LVQ_SSD1306_WRITECOMMAND (LVQ_SSD1306_INVERTDISPLAY);
	}
  else
	{
		LVQ_SSD1306_WRITECOMMAND (LVQ_SSD1306_NORMALDISPLAY);
	}
}

void LVQ_SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color)
{

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, y++)
    {
        for(int16_t i=0; i<w; i++)
        {
            if(i & 7)
            {
               byte <<= 1;
            }
            else
            {
               byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }
            if(byte & 0x80) { LVQ_SSD1306_DrawPixel(x+i, y, color); }
        }
    }
}

uint8_t LVQ_SSD1306_Init(void) 
{
	/* Init I2C */
	
	/* A little delay */
	uint32_t p = 2500;
	while(p>0)
		p--;
	
	/* Init LCD */
	LVQ_SSD1306_WRITECOMMAND(0xAE); //display off
	LVQ_SSD1306_WRITECOMMAND(0x20); //Set Memory Addressing Mode   
	LVQ_SSD1306_WRITECOMMAND(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	LVQ_SSD1306_WRITECOMMAND(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	LVQ_SSD1306_WRITECOMMAND(0xC8); //Set COM Output Scan Direction
	LVQ_SSD1306_WRITECOMMAND(0x00); //---set low column address
	LVQ_SSD1306_WRITECOMMAND(0x10); //---set high column address
	LVQ_SSD1306_WRITECOMMAND(0x40); //--set start line address
	LVQ_SSD1306_WRITECOMMAND(0x81); //--set contrast control register
	LVQ_SSD1306_WRITECOMMAND(0xFF);
	LVQ_SSD1306_WRITECOMMAND(0xA1); //--set segment re-map 0 to 127
	LVQ_SSD1306_WRITECOMMAND(0xA6); //--set normal display
	LVQ_SSD1306_WRITECOMMAND(0xA8); //--set multiplex ratio(1 to 64)
	LVQ_SSD1306_WRITECOMMAND(0x3F); //
	LVQ_SSD1306_WRITECOMMAND(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	LVQ_SSD1306_WRITECOMMAND(0xD3); //-set display offset
	LVQ_SSD1306_WRITECOMMAND(0x00); //-not offset
	LVQ_SSD1306_WRITECOMMAND(0xD5); //--set display clock divide ratio/oscillator frequency
	LVQ_SSD1306_WRITECOMMAND(0xF0); //--set divide ratio
	LVQ_SSD1306_WRITECOMMAND(0xD9); //--set pre-charge period
	LVQ_SSD1306_WRITECOMMAND(0x22); //
	LVQ_SSD1306_WRITECOMMAND(0xDA); //--set com pins hardware configuration
	LVQ_SSD1306_WRITECOMMAND(0x12);
	LVQ_SSD1306_WRITECOMMAND(0xDB); //--set vcomh
	LVQ_SSD1306_WRITECOMMAND(0x20); //0x20,0.77xVcc
	LVQ_SSD1306_WRITECOMMAND(0x8D); //--set DC-DC enable
	LVQ_SSD1306_WRITECOMMAND(0x14); //
	LVQ_SSD1306_WRITECOMMAND(0xAF); //--turn on LVQ_SSD1306 panel
	

	LVQ_SSD1306_WRITECOMMAND(LVQ_SSD1306_DEACTIVATE_SCROLL);

	/* Clear screen */
	LVQ_SSD1306_Fill(LVQ_SSD1306_COLOR_BLACK);
	
	/* Update screen */
	LVQ_SSD1306_UpdateScreen();
	
	/* Set default values */
	LVQ_SSD1306.CurrentX = 0;
	LVQ_SSD1306.CurrentY = 0;
	
	/* Initialized OK */
	LVQ_SSD1306.Initialized = 1;
	
	/* Return OK */
	return 1;
}

void LVQ_SSD1306_UpdateScreen(void) 
{
	uint8_t m;
	
	for (m = 0; m < 8; m++) 
	{
		LVQ_SSD1306_WRITECOMMAND(0xB0 + m);
		LVQ_SSD1306_WRITECOMMAND(0x00);
		LVQ_SSD1306_WRITECOMMAND(0x10);
		
		/* Write multi data */
		HAL_I2C_Mem_Write(&hi2c1, LVQ_SSD1306_I2C_ADDR, 0x40, 1, &LVQ_SSD1306_Buffer[LVQ_SSD1306_WIDTH * m], LVQ_SSD1306_WIDTH, 1000);
	}
}

void LVQ_SSD1306_ToggleInvert(void) 
{
	uint16_t i;
	
	/* Toggle invert */
	LVQ_SSD1306.Inverted = !LVQ_SSD1306.Inverted;
	
	/* Do memory toggle */
	for (i = 0; i < sizeof(LVQ_SSD1306_Buffer); i++) {
		LVQ_SSD1306_Buffer[i] = ~LVQ_SSD1306_Buffer[i];
	}
}

void LVQ_SSD1306_Fill(LVQ_SSD1306_COLOR_t color) 
{
	/* Set memory */
	memset(LVQ_SSD1306_Buffer, (color == LVQ_SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(LVQ_SSD1306_Buffer));
}

void LVQ_SSD1306_DrawPixel(uint16_t x, uint16_t y, LVQ_SSD1306_COLOR_t color) 
{
	if (
		x >= LVQ_SSD1306_WIDTH ||
		y >= LVQ_SSD1306_HEIGHT
	) {
		/* Error */
		return;
	}
	
	/* Check if pixels are inverted */
	if (LVQ_SSD1306.Inverted) {
		color = (LVQ_SSD1306_COLOR_t)!color;
	}
	
	/* Set color */
	if (color == LVQ_SSD1306_COLOR_WHITE) {
		LVQ_SSD1306_Buffer[x + (y / 8) * LVQ_SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		LVQ_SSD1306_Buffer[x + (y / 8) * LVQ_SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

void LVQ_SSD1306_GotoXY(uint16_t x, uint16_t y) 
{
	/* Set write pointers */
	LVQ_SSD1306.CurrentX = x;
	LVQ_SSD1306.CurrentY = y;
}

char LVQ_SSD1306_Putc(char ch, FontDef_t* Font, LVQ_SSD1306_COLOR_t color) 
{
	uint32_t i, b, j;
	
	/* Check available space in LCD */
	if (
		LVQ_SSD1306_WIDTH <= (LVQ_SSD1306.CurrentX + Font->FontWidth) ||
		LVQ_SSD1306_HEIGHT <= (LVQ_SSD1306.CurrentY + Font->FontHeight)
	) {
		/* Error */
		return 0;
	}
	
	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				LVQ_SSD1306_DrawPixel(LVQ_SSD1306.CurrentX + j, (LVQ_SSD1306.CurrentY + i), (LVQ_SSD1306_COLOR_t) color);
			} else {
				LVQ_SSD1306_DrawPixel(LVQ_SSD1306.CurrentX + j, (LVQ_SSD1306.CurrentY + i), (LVQ_SSD1306_COLOR_t)!color);
			}
		}
	}
	
	/* Increase pointer */
	LVQ_SSD1306.CurrentX += Font->FontWidth;
	
	/* Return character written */
	return ch;
}

char LVQ_SSD1306_Puts(char* str, FontDef_t* Font, LVQ_SSD1306_COLOR_t color) 
{
	/* Write characters */
	while (*str) {
		/* Write character by character */
		if (LVQ_SSD1306_Putc(*str, Font, color) != *str) {
			/* Return error */
			return *str;
		}
		
		/* Increase string pointer */
		str++;
	}
	
	/* Everything OK, zero should be returned */
	return *str;
}
 

void LVQ_SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, LVQ_SSD1306_COLOR_t c) 
{
	int16_t dx, dy, sx, sy, err, e2, i, tmp; 
	
	/* Check for overflow */
	if (x0 >= LVQ_SSD1306_WIDTH) {
		x0 = LVQ_SSD1306_WIDTH - 1;
	}
	if (x1 >= LVQ_SSD1306_WIDTH) {
		x1 = LVQ_SSD1306_WIDTH - 1;
	}
	if (y0 >= LVQ_SSD1306_HEIGHT) {
		y0 = LVQ_SSD1306_HEIGHT - 1;
	}
	if (y1 >= LVQ_SSD1306_HEIGHT) {
		y1 = LVQ_SSD1306_HEIGHT - 1;
	}
	
	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1); 
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1); 
	sx = (x0 < x1) ? 1 : -1; 
	sy = (y0 < y1) ? 1 : -1; 
	err = ((dx > dy) ? dx : -dy) / 2; 

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}
		
		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}
		
		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			LVQ_SSD1306_DrawPixel(x0, i, c);
		}
		
		/* Return from function */
		return;
	}
	
	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}
		
		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}
		
		/* Horizontal line */
		for (i = x0; i <= x1; i++) 
		{
			LVQ_SSD1306_DrawPixel(i, y0, c);
		}
		
		/* Return from function */
		return;
	}
	
	while (1) 
	{
		LVQ_SSD1306_DrawPixel(x0, y0, c);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err; 
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		} 
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		} 
	}
}

void LVQ_SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, LVQ_SSD1306_COLOR_t c) 
{
	/* Check input parameters */
	if (
		x >= LVQ_SSD1306_WIDTH ||
		y >= LVQ_SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}
	
	/* Check width and height */
	if ((x + w) >= LVQ_SSD1306_WIDTH) {
		w = LVQ_SSD1306_WIDTH - x;
	}
	if ((y + h) >= LVQ_SSD1306_HEIGHT) {
		h = LVQ_SSD1306_HEIGHT - y;
	}
	
	/* Draw 4 lines */
	LVQ_SSD1306_DrawLine(x, y, x + w, y, c);         /* Top line */
	LVQ_SSD1306_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
	LVQ_SSD1306_DrawLine(x, y, x, y + h, c);         /* Left line */
	LVQ_SSD1306_DrawLine(x + w, y, x + w, y + h, c); /* Right line */
}

void LVQ_SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, LVQ_SSD1306_COLOR_t c) 
{
	uint8_t i;
	
	/* Check input parameters */
	if (
		x >= LVQ_SSD1306_WIDTH ||
		y >= LVQ_SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}
	
	/* Check width and height */
	if ((x + w) >= LVQ_SSD1306_WIDTH) {
		w = LVQ_SSD1306_WIDTH - x;
	}
	if ((y + h) >= LVQ_SSD1306_HEIGHT) {
		h = LVQ_SSD1306_HEIGHT - y;
	}
	
	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		LVQ_SSD1306_DrawLine(x, y + i, x + w, y + i, c);
	}
}

void LVQ_SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, LVQ_SSD1306_COLOR_t color) 
{
	/* Draw lines */
	LVQ_SSD1306_DrawLine(x1, y1, x2, y2, color);
	LVQ_SSD1306_DrawLine(x2, y2, x3, y3, color);
	LVQ_SSD1306_DrawLine(x3, y3, x1, y1, color);
}


void LVQ_SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, LVQ_SSD1306_COLOR_t color) 
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
	curpixel = 0;
	
	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) 
	{
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) 
	{
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay)
	{
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++)
	{
		LVQ_SSD1306_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

void LVQ_SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, LVQ_SSD1306_COLOR_t c) 
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    LVQ_SSD1306_DrawPixel(x0, y0 + r, c);
    LVQ_SSD1306_DrawPixel(x0, y0 - r, c);
    LVQ_SSD1306_DrawPixel(x0 + r, y0, c);
    LVQ_SSD1306_DrawPixel(x0 - r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        LVQ_SSD1306_DrawPixel(x0 + x, y0 + y, c);
        LVQ_SSD1306_DrawPixel(x0 - x, y0 + y, c);
        LVQ_SSD1306_DrawPixel(x0 + x, y0 - y, c);
        LVQ_SSD1306_DrawPixel(x0 - x, y0 - y, c);

        LVQ_SSD1306_DrawPixel(x0 + y, y0 + x, c);
        LVQ_SSD1306_DrawPixel(x0 - y, y0 + x, c);
        LVQ_SSD1306_DrawPixel(x0 + y, y0 - x, c);
        LVQ_SSD1306_DrawPixel(x0 - y, y0 - x, c);
    }
}

void LVQ_SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, LVQ_SSD1306_COLOR_t c) 
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    LVQ_SSD1306_DrawPixel(x0, y0 + r, c);
    LVQ_SSD1306_DrawPixel(x0, y0 - r, c);
    LVQ_SSD1306_DrawPixel(x0 + r, y0, c);
    LVQ_SSD1306_DrawPixel(x0 - r, y0, c);
    LVQ_SSD1306_DrawLine(x0 - r, y0, x0 + r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        LVQ_SSD1306_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
        LVQ_SSD1306_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, c);

        LVQ_SSD1306_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, c);
        LVQ_SSD1306_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, c);
    }
}
 
void LVQ_SSD1306_Clear(void)
{
	LVQ_SSD1306_Fill(0);
  LVQ_SSD1306_UpdateScreen();
}
void LVQ_SSD1306_ON(void) 
{
	LVQ_SSD1306_WRITECOMMAND(0x8D);  
	LVQ_SSD1306_WRITECOMMAND(0x14);  
	LVQ_SSD1306_WRITECOMMAND(0xAF);  
}
void LVQ_SSD1306_OFF(void) 
{
	LVQ_SSD1306_WRITECOMMAND(0x8D);  
	LVQ_SSD1306_WRITECOMMAND(0x10);
	LVQ_SSD1306_WRITECOMMAND(0xAE);  
}

void Review_member(void)
{
		LVQ_SSD1306_DrawBitmap(30, 0, Logo_BK, 128, 64, 1);			
		LVQ_SSD1306_UpdateScreen();
	
		HAL_Delay(1000);
	
		LVQ_SSD1306_Clear();
	
		LVQ_SSD1306_GotoXY (40, 5);
		LVQ_SSD1306_Puts ("PROJECT", &Font_7x10, 1);
		LVQ_SSD1306_GotoXY (20, 20);
		LVQ_SSD1306_Puts ("MECHANTRONICS", &Font_7x10, 1);
		LVQ_SSD1306_GotoXY (40, 35);
		LVQ_SSD1306_Puts ("XE AGV", &Font_7x10, 1);
	  LVQ_SSD1306_GotoXY (40, 50);
		LVQ_SSD1306_Puts ("ME4071", &Font_7x10, 1);
		LVQ_SSD1306_UpdateScreen(); //display
	
		HAL_Delay(1000);
	
		LVQ_SSD1306_Clear();
	
		LVQ_SSD1306_GotoXY (15, 10);
		LVQ_SSD1306_Puts ("GVHD:", &Font_7x10, 1);
		LVQ_SSD1306_GotoXY (15, 25);
		LVQ_SSD1306_Puts ("PGS.TS", &Font_7x10, 1);
	  LVQ_SSD1306_GotoXY (20, 40);
		LVQ_SSD1306_Puts ("NGUYEN QUOC CHI", &Font_7x10, 1);
		LVQ_SSD1306_UpdateScreen(); //display
	
		HAL_Delay(1000);
	
		LVQ_SSD1306_Clear();
	
		LVQ_SSD1306_GotoXY (30, 15);
		LVQ_SSD1306_Puts ("MECHANCIAL", &Font_7x10, 1);
		LVQ_SSD1306_GotoXY (25, 30);
		LVQ_SSD1306_Puts ("PHAM CAO TAM", &Font_7x10, 1);
	  LVQ_SSD1306_GotoXY (35, 45);
		LVQ_SSD1306_Puts ("CK19CD02", &Font_7x10, 1);
		LVQ_SSD1306_UpdateScreen(); //display
	
		HAL_Delay(1000);
		
		LVQ_SSD1306_Clear();
	
		LVQ_SSD1306_GotoXY (30, 15);
		LVQ_SSD1306_Puts ("ELECTRONIC", &Font_7x10, 1);
		LVQ_SSD1306_GotoXY (30, 30);
		LVQ_SSD1306_Puts ("LE VAN QUY", &Font_7x10, 1);
	  LVQ_SSD1306_GotoXY (35, 45);
		LVQ_SSD1306_Puts ("CK19CD02", &Font_7x10, 1);
		LVQ_SSD1306_UpdateScreen(); //display
		
		HAL_Delay(1000);
	
		LVQ_SSD1306_Clear();
	
		LVQ_SSD1306_GotoXY (40, 15);
		LVQ_SSD1306_Puts ("CONTROL", &Font_7x10, 1);
		LVQ_SSD1306_GotoXY (20, 30);
		LVQ_SSD1306_Puts ("LE MINH QUAN", &Font_7x10, 1);
	  LVQ_SSD1306_GotoXY (35, 45);
		LVQ_SSD1306_Puts ("CK19KSCD", &Font_7x10, 1);
		LVQ_SSD1306_UpdateScreen(); //display
		
		HAL_Delay(1000);
		
		LVQ_SSD1306_Clear();
		
}
