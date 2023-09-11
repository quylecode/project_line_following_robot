/**
  ******************************************************************************

  HX711 For STM32F446RE
  Author:   LVQ
  Updated:  11st September 2022

  ******************************************************************************
*/

#include "hx711.h"

void LVQ_HX711_Init(void)
{
		/* Set clock high */
		LVQ_HX711_CLOCK_HIGH;
}

uint32_t LVQ_HX711_Read(void)
{
		uint32_t u32HX711_Value = 0;
		int16_t TimeOut = 1000;
	
		//LVQ_GPIO_SetPinHigh(LVQ_HX711_DATA_GPIO, LVQ_HX711_DATA_PIN);
	
		HAL_GPIO_WritePin(DT_GPIO_Port, DT_Pin, 1);
		LVQ_HX711_CLOCK_LOW;
	
		while((LVQ_HX711_DATA_VALUE == 1) && TimeOut)
		{
				if(TimeOut--)
						return 0;
		}
		
		for(uint8_t i = 0; i < 24; i++)
		{
				LVQ_HX711_CLOCK_HIGH;
				for (volatile uint8_t i=0; i!=0xFF; i++);
				LVQ_HX711_CLOCK_LOW;
				for (volatile uint8_t i=0; i!=0xFF; i++);
				u32HX711_Value <<= 1;
				if(LVQ_HX711_DATA_VALUE) u32HX711_Value++;
		}
		LVQ_HX711_CLOCK_HIGH;
		for (volatile uint8_t i=0; i!=0xFF; i++);
		LVQ_HX711_CLOCK_LOW;
		
		u32HX711_Value ^= 0x800000;
		return u32HX711_Value;
}

uint32_t LVQ_HX711_Read_Ave(uint16_t u16Sample)
{
		uint64_t HX711_Ave = 0;
		uint32_t Check = 0;
		for(int16_t i = 0; i < u16Sample; i++)
		{
			Check = LVQ_HX711_Read();
			if(Check != 0)
			{
					HX711_Ave += Check;
			}
			else
				i--;
			HAL_Delay(1);
		}
		uint32_t HX711_Ans = (uint32_t) ( HX711_Ave / u16Sample );
		return HX711_Ans;
}

void LVQ_HX711_Tare(LVQ_HX711_DaTa_t *HX711, uint16_t u16Sample)
{
		uint64_t HX711_Ave = 0;
		uint32_t Check = 0;
		for(int16_t i = 0; i < u16Sample; i++)
		{
			Check = LVQ_HX711_Read();
			if(Check != 0)
			{
					HX711_Ave += Check;
			}
			else
				i--;
			HAL_Delay(1);
		}
		HX711->offset = (uint32_t) ( HX711_Ave / u16Sample );
}

void LVQ_HX711_Calibration(LVQ_HX711_DaTa_t *HX711, float weight)
{
		HX711->coef =  (LVQ_HX711_Read_Ave(5) - HX711->offset) / weight;  
}

float LVQ_HX711_Weight(LVQ_HX711_DaTa_t *HX711, uint16_t u16Sample)
{
		uint64_t HX711_Ave = 0;
		uint32_t Check = 0;
		for(int16_t i = 0; i < u16Sample; i++)
		{
			Check = LVQ_HX711_Read();
			if(Check != 0)
			{
					HX711_Ave += Check;
			}
			else
				i--;
			HAL_Delay(1);
		}
		uint32_t data = (uint32_t)(HX711_Ave / u16Sample);
		float answer =  (data - HX711->offset) / HX711->coef;
		return answer;
}

void LVQ_HX711_Coef_Set(LVQ_HX711_DaTa_t *HX711, float coef)
{
  HX711->coef = coef;  
}
