#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f2xx_hal.h"
#include "adc.h"
#include "math_func.h"
#include "tcc_task.h"

struct _s_STR TCC1;
struct _s_STR TCC2;
struct _s_STR TCC3;
struct _s_STR TCC4;

osMutexId mid_Thread_Mutex;  // mutex id
osMutexDef(SampleMutex);

uint16_t rawADC_1[14] = {0};
float P1 = 0, P2 = 0, P3 = 0, P4 = 0;

uint32_t Timer_100_ms = 0;
uint32_t Timer_1_s = 0;
uint32_t Timer_8_s = 0;
float Temp;
float Uverif;

static float Core_Tempsensor(uint32_t data) {
  float Temp;
  Temp = (data * 3.3 / 4095.f - 0.76) / 0.0025 + 25;
  return Temp;
}

void Algoritm(struct _s_STR *STR, float P) {
  STR->P = P;
  if (STR->P >= P_SIG_POROG) {
    STR->Struhka = 1;
  }

  if (STR->P >= 20 && STR->P < 36) {
    STR->Period_8_sec = 1;
    STR->Period_1_sec = 0;
  }

  if (STR->P > 36) {
    STR->Kz = 1;
  }
	else
	{
		STR->Kz = 0;
	}

  if (STR->P < 20) {
    STR->Period_8_sec = 0;
    STR->Period_1_sec = 1;
  }

  if (STR->P > 0.04 && STR->P < 20 && STR->Kz == 0) {
    STR->Struhka = 0;
  }

  if (STR->P >= 0 && STR->P < 0.04) {
    STR->Otkaz = 1;
  }
	else 
	{
		STR->Otkaz = 0;
	}
}

void Sensor_1_Task(void const *argument) {
  float I_1_IN_M;
  float U_1_IN_M;
  float U_1_OUT_M;
  uint8_t i;
  static uint16_t Median_mass_I_1_IN_M[10];
  static uint16_t Median_mass_U_1_IN_M[10];
  static uint16_t Median_mass_U_1_OUT_M[10];
  static uint8_t Start_Timer = 0;
  osStatus status;
  mid_Thread_Mutex = osMutexCreate(osMutex(SampleMutex));
  while (1) {
    status = osMutexWait(mid_Thread_Mutex, NULL);
    if (status == osOK) {
      if (Start_Timer == 0) {
        S_A_M_LOW
        S_B_M_LOW
        S_C_M_LOW
        Timer_100_ms = 100;
        Start_Timer = 1;
      }

      while (Timer_100_ms > 0) {
        for (i = 9; i > 0; i--) {
          Median_mass_I_1_IN_M[i] = Median_mass_I_1_IN_M[i - 1];
          Median_mass_U_1_IN_M[i] = Median_mass_U_1_IN_M[i - 1];
          Median_mass_U_1_OUT_M[i] = Median_mass_U_1_OUT_M[i - 1];
        }
        Median_mass_I_1_IN_M[0] = rawADC_1[4];
        Median_mass_U_1_IN_M[0] = rawADC_1[5];
        Median_mass_U_1_OUT_M[0] = rawADC_1[6];
        I_1_IN_M =
            (Mediana_filter16(Median_mass_I_1_IN_M, 10)) * Uverif / 4095.f;
        U_1_IN_M =
            (Mediana_filter16(Median_mass_U_1_IN_M, 10)) * Uverif / 4095.f;
        U_1_OUT_M =
            (Mediana_filter16(Median_mass_U_1_OUT_M, 10)) * Uverif / 4095.f;
        // taskYIELD();
      }

      P1 = (U_1_OUT_M - U_1_IN_M) * I_1_IN_M * Kp;
      Algoritm(&TCC1, P1);
      Start_Timer = 0;
			S_A_M_HIGHT
      S_B_M_HIGHT
      S_C_M_HIGHT
      osMutexRelease(mid_Thread_Mutex);
      if (TCC1.Period_1_sec == 1) osDelay(1000);
      if (TCC1.Period_8_sec == 1) osDelay(8000);
    }
    taskYIELD();
  }
}

void Sensor_2_Task(void const *argument) {
  float I_2_IN_M;
  float U_2_IN_M;
  float U_2_OUT_M;
  uint8_t i;
  static uint16_t Median_mass_I_2_IN_M[10];
  static uint16_t Median_mass_U_2_IN_M[10];
  static uint16_t Median_mass_U_2_OUT_M[10];
  static uint8_t Start_Timer = 0;
  osStatus status;
  while (1) {
    status = osMutexWait(mid_Thread_Mutex, NULL);
    if (status == osOK) {
      if (Start_Timer == 0) {
        S_A_M_HIGHT
        S_B_M_LOW
        S_C_M_LOW
        Timer_100_ms = 100;
        Start_Timer = 1;
      }

      while (Timer_100_ms > 0) {
        for (i = 9; i > 0; i--) {
          Median_mass_I_2_IN_M[i] = Median_mass_I_2_IN_M[i - 1];
          Median_mass_U_2_IN_M[i] = Median_mass_U_2_IN_M[i - 1];
          Median_mass_U_2_OUT_M[i] = Median_mass_U_2_OUT_M[i - 1];
        }
        Median_mass_I_2_IN_M[0] = rawADC_1[7];
        Median_mass_U_2_IN_M[0] = rawADC_1[8];
        Median_mass_U_2_OUT_M[0] = rawADC_1[9];
        I_2_IN_M = (Mediana_filter16(Median_mass_I_2_IN_M, 10)) * Uverif / 4095;
        U_2_IN_M = (Mediana_filter16(Median_mass_U_2_IN_M, 10)) * Uverif / 4095;
        U_2_OUT_M =
            (Mediana_filter16(Median_mass_U_2_OUT_M, 10)) * Uverif / 4095;
        // taskYIELD();
      }
      P2 = (U_2_OUT_M - U_2_IN_M) * I_2_IN_M * Kp;
      Start_Timer = 0;
      Algoritm(&TCC2, P2);
			S_A_M_HIGHT
      S_B_M_HIGHT
      S_C_M_HIGHT
      osMutexRelease(mid_Thread_Mutex);
      if (TCC2.Period_1_sec == 1) osDelay(1000);
      if (TCC2.Period_8_sec == 1) osDelay(8000);
    }
    taskYIELD();
  }
}

void Sensor_3_Task(void const *argument) {
  float I_3_IN_M;
  float U_3_IN_M;
  float U_3_OUT_M;
  uint8_t i;
  static uint16_t Median_mass_I_3_IN_M[10];
  static uint16_t Median_mass_U_3_IN_M[10];
  static uint16_t Median_mass_U_3_OUT_M[10];
  static uint8_t Start_Timer = 0;
  osStatus status;
  while (1) {
    status = osMutexWait(mid_Thread_Mutex, NULL);
    if (status == osOK) {
      if (Start_Timer == 0) {
        Timer_100_ms = 100;
        S_A_M_LOW
        S_B_M_HIGHT
        S_C_M_LOW
        Start_Timer = 1;
      }

      while (Timer_100_ms > 0) {
        for (i = 9; i > 0; i--) {
          Median_mass_I_3_IN_M[i] = Median_mass_I_3_IN_M[i - 1];
          Median_mass_U_3_IN_M[i] = Median_mass_U_3_IN_M[i - 1];
          Median_mass_U_3_OUT_M[i] = Median_mass_U_3_OUT_M[i - 1];
        }
        Median_mass_I_3_IN_M[0] = rawADC_1[2];
        Median_mass_U_3_IN_M[0] = rawADC_1[3];
        Median_mass_U_3_OUT_M[0] = rawADC_1[10];

        I_3_IN_M = (Mediana_filter16(Median_mass_I_3_IN_M, 10)) * Uverif / 4095;
        U_3_IN_M = (Mediana_filter16(Median_mass_U_3_IN_M, 10)) * Uverif / 4095;
        U_3_OUT_M =
            (Mediana_filter16(Median_mass_U_3_OUT_M, 10)) * Uverif / 4095;
        // taskYIELD();
      }
      P3 = (U_3_OUT_M - U_3_IN_M) * I_3_IN_M * Kp;
      Start_Timer = 0;
      Algoritm(&TCC3, P3);
			S_A_M_HIGHT
      S_B_M_HIGHT
      S_C_M_HIGHT
      osMutexRelease(mid_Thread_Mutex);
      if (TCC3.Period_1_sec == 1) osDelay(1000);
      if (TCC3.Period_8_sec == 1) osDelay(8000);
    }
    taskYIELD();
  }
}

void Sensor_4_Task(void const *argument) {
  float I_4_IN_M;
  float U_4_IN_M;
  float U_4_OUT_M;
  uint8_t i;
  static uint16_t Median_mass_I_4_IN_M[10];
  static uint16_t Median_mass_U_4_IN_M[10];
  static uint16_t Median_mass_U_4_OUT_M[10];
  static uint8_t Start_Timer = 0;
  osStatus status;

  while (1) {
    status = osMutexWait(mid_Thread_Mutex, NULL);
    if (status == osOK) {
      if (Start_Timer == 0) {
        Timer_100_ms = 100;
        S_A_M_HIGHT
        S_B_M_HIGHT
        S_C_M_LOW
        Start_Timer = 1;
      }

      while (Timer_100_ms > 0) {
        for (i = 9; i > 0; i--) {
          Median_mass_I_4_IN_M[i] = Median_mass_I_4_IN_M[i - 1];
          Median_mass_U_4_IN_M[i] = Median_mass_U_4_IN_M[i - 1];
          Median_mass_U_4_OUT_M[i] = Median_mass_U_4_OUT_M[i - 1];
        }
        Median_mass_I_4_IN_M[0] = rawADC_1[0];
        Median_mass_U_4_IN_M[0] = rawADC_1[1];
        Median_mass_U_4_OUT_M[0] = rawADC_1[11];

        I_4_IN_M = (Mediana_filter16(Median_mass_I_4_IN_M, 10)) * Uverif / 4095;
        U_4_IN_M = (Mediana_filter16(Median_mass_U_4_IN_M, 10)) * Uverif / 4095;
        U_4_OUT_M =
            (Mediana_filter16(Median_mass_U_4_OUT_M, 10)) * Uverif / 4095;
        // taskYIELD();
      }
      P4 = (U_4_OUT_M - U_4_IN_M) * I_4_IN_M * Kp;
      Start_Timer = 0;
      Algoritm(&TCC4, P4);
      S_A_M_HIGHT
      S_B_M_HIGHT
      S_C_M_HIGHT
      osMutexRelease(mid_Thread_Mutex);
      if (TCC4.Period_1_sec == 1) osDelay(1000);
      if (TCC4.Period_8_sec == 1) osDelay(8000);
    }
    taskYIELD();
  }
}

void Task_TCC(void) {
  static uint8_t STATE;
  static uint16_t Median_mass[10];
  static uint16_t Median_mass_temp[10];
  uint8_t i;

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&rawADC_1, 14);
  while (1) {
    /*
switch (STATE) {
case (0): {
Timer_1_s = 1000;
            //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
STATE = 1;
break;
}
case (1): {
Sensor_1_Task();
Algoritm(&TCC1, P1);
STATE = 2;
break;
}
case (2): {
if (Timer_1_s == 0)
            {
STATE = 3;
                    //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6,
GPIO_PIN_RESET);
            }
break;
}
case (3): {
Timer_1_s = 1000;
STATE = 4;
break;
}
case (4): {
Sensor_2_Task();
Algoritm(&TCC2, P2);
STATE = 5;
break;
}
case (5): {
if (Timer_1_s == 0) {
STATE = 6;
Timer_1_s = 1000;
}
break;
}
case (6): {
Sensor_3_Task();
Algoritm(&TCC3, P3);
STATE = 7;
break;
}
case (7): {
if (Timer_1_s == 0) {
STATE = 8;
Timer_1_s = 1000;
}
break;
}
case (8): {
Sensor_4_Task();
Algoritm(&TCC4, P4);
STATE = 0;
break;
}
}
*/
    for (i = 9; i > 0; i--) {
      Median_mass[i] = Median_mass[i - 1];
      Median_mass_temp[i] = Median_mass_temp[i - 1];
    }

    Median_mass[0] = rawADC_1[13];
    Median_mass_temp[0] = rawADC_1[12];

    Temp = Core_Tempsensor(Mediana_filter16(Median_mass_temp, 10));
    Uverif = ((1678.f) * 3.f) / (Mediana_filter16(Median_mass, 10));

    //  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);

    osDelay(100);
  }
}
