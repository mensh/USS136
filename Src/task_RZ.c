#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f2xx_hal.h"
#include "holt.h"

void RZ_Task(void const* argument)
{
    Holt_Configuration(1, 0xE429); //[E429] - for 12.5 kHz
    while (1) {
        Holt_Write_TFIFO1(2, ((int16_t)(1 * 128.f)) << 13, 0x01);
        osDelay(100);
    }
}
