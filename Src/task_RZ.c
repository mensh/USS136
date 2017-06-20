#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f2xx_hal.h"
#include "holt.h"


extern uint8_t SPI1_TX[8];
extern uint8_t SPI1_RX[8];
#define data_Mkp   0xc1 
#define data_Ny    0xc0 
#define data_binar 0x57 
uint32_t data_RX;

uint16_t Mkp;
uint16_t Ny;
uint8_t Pvib_Left = 1;
uint8_t Pvib_Right;
uint8_t Pod_nas_vkl;
uint8_t Vint_Na_Torm;
uint8_t Poj_kran_otkr = 1;



void RZ_Task(void const* argument)
{
    Holt_Configuration(1, 0xE429); //[E429] - for 12.5 kHz
    while (1) {
        Holt_Write_TFIFO1(2, ((int16_t)(1 * 128.f)) << 13, 0x01);
				data_RX = Holt_Read_RFIFO(1);
				if ((data_RX & 0x000000ff) == data_Mkp)
				{
					
				}
			
				if ((data_RX & 0x000000ff) == data_Ny)
				{
					
				}
				
				if ((data_RX & 0x000000ff) == data_binar)
				{
					
				}
				
        osDelay(100);
    }
}
