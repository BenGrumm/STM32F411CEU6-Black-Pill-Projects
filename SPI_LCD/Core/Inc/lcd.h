#ifndef _I_LCD_H
#define _I_LCD_H

	
#include "DEV_Config.h"
#include <stdint.h>

#include <stdlib.h>		//itoa()
#include <stdio.h>


#define LCD_1IN28_HEIGHT 240
#define LCD_1IN28_WIDTH 240


#define HORIZONTAL 0
#define VERTICAL   1

#define LCD_1IN28_SetBacklight(Value) DEV_SetBacklight(Value) 

#define LCD_1IN28_CS_0	    DEV_Digital_Write(DEV_CS_PIN, 0) 
#define LCD_1IN28_CS_1	    DEV_Digital_Write(DEV_CS_PIN, 1)
	                        
#define LCD_1IN28_RST_0	  DEV_Digital_Write(DEV_RST_PIN,0)
#define LCD_1IN28_RST_1	  DEV_Digital_Write(DEV_RST_PIN,1)
	                        
#define LCD_1IN28_DC_0	    DEV_Digital_Write(DEV_DC_PIN, 0)
#define LCD_1IN28_DC_1	    DEV_Digital_Write(DEV_DC_PIN, 1)  

typedef struct{
	UWORD WIDTH;
	UWORD HEIGHT;
	UBYTE SCAN_DIR;
}LCD_1IN28_ATTRIBUTES;
extern LCD_1IN28_ATTRIBUTES LCD_1IN28;

/********************************************************************************
function:	
			Macro definition variable name
********************************************************************************/
void LCD_1IN28_Init(UBYTE Scan_dir);
void LCD_1IN28_Clear(UWORD Color);
void LCD_1IN28_Display(UWORD *Image);
void LCD_1IN28_DisplayWindows(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend, UWORD *Image);
void LCD_1IN28_DisplayPoint(UWORD X, UWORD Y, UWORD Color);

void LCD_1IN28_DrawPaint(UWORD x, UWORD y, UWORD Color);
void LCD_1IN28_SetBackLight(UWORD Value);
void LCD_1in28_test();

#endif