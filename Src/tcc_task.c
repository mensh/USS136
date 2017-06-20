#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f2xx_hal.h"
#include "adc.h"
#include "math_func.h"
#include "tcc_task.h"
#include "math.h"

struct _s_STR TCC1;
struct _s_STR TCC2;
struct _s_STR TCC3;
struct _s_STR TCC4;

osMutexId mid_Thread_Mutex; // mutex id
osMutexDef(SampleMutex);

uint16_t rawADC_1[14] = { 0 };
uint32_t Timer_100_ms = 0;
uint32_t Timer_1_s = 0;
uint32_t Timer_8_s = 0;
float Temp;
float Uverif;

static float Core_Tempsensor(uint32_t data)
{
    float Temp;
    Temp = (data * Uverif / 4095.f - 0.76) / 0.0025 + 25;
    return Temp;
}

#define KZ_const 0.003
#define Ku 11.f
#define Strushka_const 0.08 // 0.060
#define Obriv_const 6.1
#define Ki 2.6
#define Rizm 22000.f




void Algoritm_New(struct _s_STR* STR, float U_In, float U_Out, float I,
    uint8_t state)
{
    uint8_t i;
    float temp_mass[5];
    if (state == 0) {
        STR->Ushunt = (U_Out - U_In) * Ku;
        if (STR->Ushunt <= KZ_const) {
            STR->P = 0;
        }

        if (STR->Ushunt > Obriv_const) {
            STR->Obriv = 1;
            STR->P = 0;
        }
        else {
            STR->Obriv = 0;
        }
    }
		
	//	test = STR->Uio - I;
    if (/*STR->Ushunt > KZ_const && STR->Ushunt < Strushka_const &&*/ state == 1) {
        STR->P2 = ((U_Out - U_In) * Ku * I * 1.8) * 1.05 ;//* Ki;
       // STR->Iresult = (I - (STR->Uio * 3.f)) * Ki;
    }

    if (/*STR->Ushunt > Strushka_const && STR->Ushunt < Obriv_const &&*/ state == 0) {
        STR->P1 = (((U_Out * U_Out) * U_In * Ku * Ku) / ((U_Out - U_In) * Rizm)) * 1.5;
    }

    if (STR->P_nf > 5.f) {
        STR->P_nf = STR->P2;
				STR->algo = 0;
    }
    else {
        if (STR->P_nf<4.5 || STR->algo==1) 
				{
					STR->P_nf = STR->P1;
					STR->algo = 1;
				}
				else STR->P_nf = STR->P2;
    }
			
    for (i = 4; i > 0; i--) {
        STR->mass_P[i] = STR->mass_P[i - 1];
    }
    STR->mass_P[0] = STR->P_nf;
    for (i = 0; i < 5; i++) temp_mass[i] = STR->mass_P[i];
		
    STR->P_f = MedianFilter(temp_mass, 4);
		
		if (STR->P_f > 4.5 && STR->P_f < 8 )
		{
			 if ( STR->P_f < STR->P_max)
			 {
				 STR->P_min = STR->P_f;
			 }
			 if (  STR->P_f > STR->P_min )
			 {
				 STR->P_max = STR->P_f;
			 }				 
			 if (STR->counter>5)
			 {
				 STR->P = (STR->P_min + STR->P_max)/2.f;
				 STR->counter =0;
			 }
			 STR->counter++;
		}			
		else
		{
			STR->P = STR->P_f;
		}
		
    if (STR->P >= P_SIG_POROG) {
        STR->Struhka = 1;
				STR->counter++;
    }
    else 
		if (STR->counter >=3){
        STR->Struhka = 0;
				STR->counter = 0;
    }

    if (STR->P >= 20 && STR->P < 36) {
        STR->Period_8_sec = 1;
        STR->Period_1_sec = 0;
			
    }

    if (STR->P < 20 && STR->counter>=3) {
        STR->Period_8_sec = 0;
        STR->Period_1_sec = 1;
				
    }

    if (STR->P > 40) {
        STR->counter_KZ++;
    }
    else {
        if (STR->counter_KZ > 0)
            STR->counter_KZ--;
    }

    if (STR->counter_KZ >= 3)
        STR->Kz = 1;
    else
        STR->Kz = 0;
}

void Sensor_1_Task_Vsk(void const* argument)
{
    float I_1_IN_M;
    float U_1_IN_M;
    float U_1_OUT_M;
    static uint8_t Start_Timer = 0;
    static uint16_t mass_I[19];
    static uint16_t mass_U_In[19];
    static uint16_t mass_U_Out[19];
    uint32_t counter;
    uint16_t i;
    osStatus status;
    // mid_Thread_Mutex = osMutexCreate(osMutex(SampleMutex));
    while (1) {
        if (TCC1.Kz == 0) {
            status = osMutexWait(mid_Thread_Mutex, NULL);
            if (status == osOK) {
                if (Start_Timer == 0) {
                    S_A_M_LOW_plus
                    S_B_M_LOW_plus
                    S_C_M_LOW_plus
                    // HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);
                    Timer_100_ms = 100;
                    Start_Timer = 1;
                    osDelay(1);
                }

                while (Timer_100_ms > 0) {
                    for (i = 18; i > 0; i--) {
                        mass_I[i] = mass_I[i - 1];
                        mass_U_In[i] = mass_U_In[i - 1];
                        mass_U_Out[i] = mass_U_Out[i - 1];
                    }

                    mass_I[0] = rawADC_1[4];
                    mass_U_In[0] = rawADC_1[5];
                    mass_U_Out[0] = rawADC_1[6];

                    I_1_IN_M += Mediana_filter16(mass_I, 19) * Uverif / 4095.f;
                    U_1_IN_M += Mediana_filter16(mass_U_In, 19) * Uverif / 4095.f;
                    U_1_OUT_M += Mediana_filter16(mass_U_Out, 19) * Uverif / 4095.f;
                    counter++;
                }
                I_1_IN_M = I_1_IN_M / counter;
                U_1_IN_M = U_1_IN_M / counter;
                U_1_OUT_M = U_1_OUT_M / counter;
                Algoritm_New(&TCC1, U_1_IN_M, U_1_OUT_M, I_1_IN_M, 0);
                counter = 0;
                I_1_IN_M = 0;
                U_1_IN_M = 0;
                U_1_OUT_M = 0;
                Start_Timer = 0;
                S_A_M_HIGHT_plus
                S_B_M_HIGHT_plus
                S_C_M_HIGHT_plus
                // HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
                osMutexRelease(mid_Thread_Mutex);
                // if (TCC1.Period_1_sec == 1) osDelay(1000);
                // if (TCC1.Period_8_sec == 1) osDelay(8000);
            }
        }
        taskYIELD();
    }
}


void Sensor_1_Task(void const* argument)
{
    float I_1_IN_M;
    float U_1_IN_M;
    float U_1_OUT_M;

    static uint8_t Start_Timer = 0;
    static uint16_t mass_I[19];
		float I_1_IN_M_not;		
	  float minus_I;
		static uint16_t mass_I_not[19];
    uint16_t i;
	
    static uint16_t mass_U_In[19];
    static uint16_t mass_U_Out[19];

    uint32_t counter;

    osStatus status;
    mid_Thread_Mutex = osMutexCreate(osMutex(SampleMutex));
    while (1) {
        if (TCC1.Kz == 0) {
            status = osMutexWait(mid_Thread_Mutex, NULL);
            if (status == osOK) {
							   if (Start_Timer == 0) {
                    Timer_100_ms = 100;
                    Start_Timer = 1;
                    osDelay(1);
                }
								 
								
							 while (Timer_100_ms > 0) {
               for (i = 18; i > 0; i--) {
                        mass_I_not[i] = mass_I_not[i - 1];         
                    }
										mass_I_not[0] = rawADC_1[4];
										I_1_IN_M_not += Mediana_filter16(mass_I_not, 19) * Uverif / 4095.f;
										counter++;
										
							}
								I_1_IN_M_not = I_1_IN_M_not / counter;
							
							  counter = 0;
								Start_Timer = 0;
                if (Start_Timer == 0) {
                    S_A_M_LOW
                    S_B_M_LOW
                    S_C_M_LOW
                   // HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);
									
									  S_A_M_LOW_plus
                    S_B_M_LOW_plus
                    S_C_M_LOW_plus
									
                    Timer_100_ms = 100;
                    Start_Timer = 1;
                    osDelay(1);
                }
                while (Timer_100_ms > 0) {
                    for (i = 18; i > 0; i--) {
                        mass_I[i] = mass_I[i - 1];
                        mass_U_In[i] = mass_U_In[i - 1];
                        mass_U_Out[i] = mass_U_Out[i - 1];
                    }
                    mass_I[0] = rawADC_1[4];
                    mass_U_In[0] = rawADC_1[5];
                    mass_U_Out[0] = rawADC_1[6];								
                    I_1_IN_M += Mediana_filter16(mass_I, 19) * Uverif / 4095.f;
                    U_1_IN_M += Mediana_filter16(mass_U_In, 19) * Uverif / 4095.f;
                    U_1_OUT_M += Mediana_filter16(mass_U_Out, 19) * Uverif / 4095.f;
                    counter++;
                }
                I_1_IN_M = I_1_IN_M / counter;
                U_1_IN_M = U_1_IN_M / counter;
                U_1_OUT_M = U_1_OUT_M / counter;
								minus_I = fabs(I_1_IN_M_not - I_1_IN_M);
								
                Algoritm_New(&TCC1, U_1_IN_M, U_1_OUT_M, minus_I, 1);
                counter = 0;
                I_1_IN_M = 0;
                U_1_IN_M = 0;
                U_1_OUT_M = 0;
                Start_Timer = 0;
                S_A_M_HIGHT
                S_B_M_HIGHT
                S_C_M_HIGHT
								S_A_M_HIGHT_plus
                S_B_M_HIGHT_plus
                S_C_M_HIGHT_plus
								
                //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
                osMutexRelease(mid_Thread_Mutex);
                if (TCC1.Period_1_sec == 1)
                    osDelay(1000);
                if (TCC1.Period_8_sec == 1)
                    osDelay(8000);
            }
        }
        taskYIELD();
    }
}

void Sensor_2_Task_Vsk(void const* argument)
{
    float U_2_IN_M;
    float U_2_OUT_M;
    float I_2_IN_M;
    static uint8_t Start_Timer = 0;
    uint32_t counter;
    osStatus status;
    while (1) {
        if (TCC2.Kz == 0) {
            status = osMutexWait(mid_Thread_Mutex, NULL);
            if (status == osOK) {
                if (Start_Timer == 0) {
                    S_A_M_HIGHT_plus
                    S_B_M_LOW_plus
                    S_C_M_LOW_plus
                    Timer_100_ms = 100;
                    Start_Timer = 1;
                }

                while (Timer_100_ms > 0) {
                    I_2_IN_M += rawADC_1[7] * Uverif / 4095.f;
                    U_2_IN_M += rawADC_1[8] * Uverif / 4095.f;
                    U_2_OUT_M += rawADC_1[9] * Uverif / 4095.f;
                    counter++;
                }
                I_2_IN_M = I_2_IN_M / counter;
                U_2_IN_M = U_2_IN_M / counter;
                U_2_OUT_M = U_2_OUT_M / counter;
                Algoritm_New(&TCC2, U_2_IN_M, U_2_OUT_M, I_2_IN_M, 0);
                counter = 0;
                I_2_IN_M = 0;
                U_2_IN_M = 0;
                U_2_OUT_M = 0;
                Start_Timer = 0;
                S_A_M_HIGHT_plus
                S_B_M_HIGHT_plus
                S_C_M_HIGHT_plus
                osDelay(1);
                osMutexRelease(mid_Thread_Mutex);
                //      if (TCC2.Period_1_sec == 1) osDelay(1000);
                //      if (TCC2.Period_8_sec == 1) osDelay(8000);
            }
        }
        taskYIELD();
    }
}

void Sensor_2_Task(void const* argument)
{
    float U_2_IN_M;
    float U_2_OUT_M;
    float I_2_IN_M;
    static uint8_t Start_Timer = 0;
    uint32_t counter;
    osStatus status;
		float I_1_IN_M_not;		
	  float minus_I;
	  uint16_t i;
		static uint16_t mass_I_not[19];
    while (1) {
        if (TCC2.Kz == 0) {
            status = osMutexWait(mid_Thread_Mutex, NULL);
            if (status == osOK) {
								   if (Start_Timer == 0) {
                    Timer_100_ms = 100;
                    Start_Timer = 1;
                    osDelay(1);
                }
								 
								
							 while (Timer_100_ms > 0) {
               for (i = 18; i > 0; i--) {
                        mass_I_not[i] = mass_I_not[i - 1];         
                    }
										mass_I_not[0] = rawADC_1[7];
										I_1_IN_M_not += Mediana_filter16(mass_I_not, 19) * Uverif / 4095.f;
										counter++;
										
							}
								I_1_IN_M_not = I_1_IN_M_not / counter;
							
							  counter = 0;
								Start_Timer = 0;
                if (Start_Timer == 0) {
                    S_A_M_HIGHT
                    S_B_M_LOW
                    S_C_M_LOW
									  S_A_M_HIGHT_plus
                    S_B_M_LOW_plus
                    S_C_M_LOW_plus
                    Timer_100_ms = 100;
                    Start_Timer = 1;
                    //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
                }

                while (Timer_100_ms > 0) {
                    I_2_IN_M += rawADC_1[7] * Uverif / 4095.f;
                    U_2_IN_M += rawADC_1[8] * Uverif / 4095.f;
                    U_2_OUT_M += rawADC_1[9] * Uverif / 4095.f;
                    counter++;
                }
                I_2_IN_M = I_2_IN_M / counter;
                U_2_IN_M = U_2_IN_M / counter;
                U_2_OUT_M = U_2_OUT_M / counter;
								minus_I = fabs(I_1_IN_M_not - I_2_IN_M);
                Algoritm_New(&TCC2, U_2_IN_M, U_2_OUT_M, minus_I, 1);
                counter = 0;
                I_2_IN_M = 0;
                U_2_IN_M = 0;
                U_2_OUT_M = 0;
                Start_Timer = 0;
                //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
                S_A_M_HIGHT
                S_B_M_HIGHT
                S_C_M_HIGHT
								 S_A_M_HIGHT_plus
                S_B_M_HIGHT_plus
                S_C_M_HIGHT_plus
                osMutexRelease(mid_Thread_Mutex);
                if (TCC2.Period_1_sec == 1)
                    osDelay(1000);
                if (TCC2.Period_8_sec == 1)
                    osDelay(8000);
            }
        }
        taskYIELD();
    }
}

void Sensor_3_Task_Vsk(void const* argument)
{
    float U_3_IN_M;
    float U_3_OUT_M;
    float I_3_IN_M;
    uint32_t counter;
    static uint8_t Start_Timer = 0;
	
    osStatus status;
    while (1) {
        if (TCC3.Kz == 0) {
            status = osMutexWait(mid_Thread_Mutex, NULL);
            if (status == osOK) {
                if (Start_Timer == 0) {
                    Timer_100_ms = 100;
                    S_A_M_LOW_plus
                    S_B_M_HIGHT_plus
                    S_C_M_LOW_plus
                    Start_Timer = 1;
                }

                while (Timer_100_ms > 0) {
                    I_3_IN_M += rawADC_1[2] * Uverif / 4095.f;

                    U_3_IN_M += rawADC_1[3] * Uverif / 4095.f;

                    U_3_OUT_M += rawADC_1[10] * Uverif / 4095.f;

                    counter++;
                }
                I_3_IN_M = I_3_IN_M / counter;
                U_3_IN_M = U_3_IN_M / counter;
                U_3_OUT_M = U_3_OUT_M / counter;
                Algoritm_New(&TCC3, U_3_IN_M, U_3_OUT_M, I_3_IN_M, 0);
                counter = 0;
                I_3_IN_M = 0;
                U_3_IN_M = 0;
                U_3_OUT_M = 0;
                Start_Timer = 0;
                S_A_M_HIGHT_plus
                S_B_M_HIGHT_plus
                S_C_M_HIGHT_plus
                osDelay(1);
                osMutexRelease(mid_Thread_Mutex);
                //      if (TCC3.Period_1_sec == 1) osDelay(1000);
                //      if (TCC3.Period_8_sec == 1) osDelay(8000);
            }
        }
        taskYIELD();
    }
}

void Sensor_3_Task(void const* argument)
{
    float U_3_IN_M;
    float U_3_OUT_M;
    float I_3_IN_M;
    uint32_t counter;
    static uint8_t Start_Timer = 0;
    osStatus status;
		float I_1_IN_M_not;		
	  float minus_I;
	  uint16_t i;
		static uint16_t mass_I_not[19];
    while (1) {
        if (TCC3.Kz == 0) {
            status = osMutexWait(mid_Thread_Mutex, NULL);
            if (status == osOK) {
										if (Start_Timer == 0) {
                    Timer_100_ms = 100;
                    Start_Timer = 1;
                    osDelay(1);
                }
								 
								
							 while (Timer_100_ms > 0) {
               for (i = 18; i > 0; i--) {
                        mass_I_not[i] = mass_I_not[i - 1];         
                    }
										mass_I_not[0] = rawADC_1[2];
										I_1_IN_M_not += Mediana_filter16(mass_I_not, 19) * Uverif / 4095.f;
										counter++;
										
							}
								I_1_IN_M_not = I_1_IN_M_not / counter;
							
							  counter = 0;
								Start_Timer = 0;
							
							
							
                if (Start_Timer == 0) {
                    Timer_100_ms = 100;
                    S_A_M_LOW
                    S_B_M_HIGHT
                    S_C_M_LOW
									  S_A_M_LOW_plus
                    S_B_M_HIGHT_plus
                    S_C_M_LOW_plus
                    Start_Timer = 1;
                    //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);
                }

                while (Timer_100_ms > 0) {
                    I_3_IN_M += rawADC_1[2] * Uverif / 4095.f;
                    U_3_IN_M += rawADC_1[3] * Uverif / 4095.f;
                    U_3_OUT_M += rawADC_1[10] * Uverif / 4095.f;
                    counter++;
                }
                I_3_IN_M = I_3_IN_M / counter;
                U_3_IN_M = U_3_IN_M / counter;
                U_3_OUT_M = U_3_OUT_M / counter;
								minus_I = fabs(I_1_IN_M_not - I_3_IN_M);
                Algoritm_New(&TCC3, U_3_IN_M, U_3_OUT_M, minus_I, 1);
                counter = 0;
                I_3_IN_M = 0;
                U_3_IN_M = 0;
                U_3_OUT_M = 0;
                Start_Timer = 0;
                //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
                S_A_M_HIGHT
                S_B_M_HIGHT
                S_C_M_HIGHT
								S_A_M_HIGHT_plus
                S_B_M_HIGHT_plus
                S_C_M_HIGHT_plus
                osMutexRelease(mid_Thread_Mutex);
                if (TCC3.Period_1_sec == 1)
                    osDelay(1000);
                if (TCC3.Period_8_sec == 1)
                    osDelay(8000);
            }
        }
        taskYIELD();
    }
}

void Sensor_4_Task_Vsk(void const* argument)
{
    float U_4_IN_M;
    float U_4_OUT_M;
    float I_4_IN_M;
    static uint8_t Start_Timer = 0;
    osStatus status;
    uint32_t counter;
    while (1) {
        if (TCC4.Kz == 0) {
            status = osMutexWait(mid_Thread_Mutex, NULL);
            if (status == osOK) {
                if (Start_Timer == 0) {
                    Timer_100_ms = 100;
                    S_A_M_HIGHT_plus
                    S_B_M_HIGHT_plus
                    S_C_M_LOW_plus
                    Start_Timer = 1;
                }

                while (Timer_100_ms > 0) {
                    I_4_IN_M += rawADC_1[0] * Uverif / 4095.f;
                    U_4_IN_M += rawADC_1[1] * Uverif / 4095.f;
                    U_4_OUT_M += rawADC_1[11] * Uverif / 4095.f;
                    counter++;
                }
                I_4_IN_M = I_4_IN_M / counter;
                U_4_IN_M = U_4_IN_M / counter;
                U_4_OUT_M = U_4_OUT_M / counter;
                Algoritm_New(&TCC4, U_4_IN_M, U_4_OUT_M, I_4_IN_M, 0);
                counter = 0;
                I_4_IN_M = 0;
                U_4_IN_M = 0;
                U_4_OUT_M = 0;
                Start_Timer = 0;
                S_A_M_HIGHT_plus
                S_B_M_HIGHT_plus
                S_C_M_HIGHT_plus
                osDelay(1);
                osMutexRelease(mid_Thread_Mutex);
                //      if (TCC4.Period_1_sec == 1) osDelay(1000);
                //      if (TCC4.Period_8_sec == 1) osDelay(8000);
            }
        }
        taskYIELD();
    }
}

void Sensor_4_Task(void const* argument)
{
    float U_4_IN_M;
    float U_4_OUT_M;
    float I_4_IN_M;
    static uint8_t Start_Timer = 0;
    osStatus status;
    uint32_t counter;
		float I_1_IN_M_not;		
	  float minus_I;
	  uint16_t i;
		static uint16_t mass_I_not[19];
    while (1) {
        if (TCC4.Kz == 0) {
            status = osMutexWait(mid_Thread_Mutex, NULL);
            if (status == osOK) {
                if (Start_Timer == 0) {
																		if (Start_Timer == 0) {
                    Timer_100_ms = 100;
                    Start_Timer = 1;
                    osDelay(1);
                }
								 
								
							 while (Timer_100_ms > 0) {
               for (i = 18; i > 0; i--) {
                        mass_I_not[i] = mass_I_not[i - 1];         
                    }
										mass_I_not[0] = rawADC_1[0];
										I_1_IN_M_not += Mediana_filter16(mass_I_not, 19) * Uverif / 4095.f;
										counter++;
										
							}
								I_1_IN_M_not = I_1_IN_M_not / counter;
							
							  counter = 0;
								Start_Timer = 0;
							
                    Timer_100_ms = 100;
                    S_A_M_HIGHT
                    S_B_M_HIGHT
                    S_C_M_LOW
										S_A_M_HIGHT_plus
                    S_B_M_HIGHT_plus
                    S_C_M_LOW_plus
                    Start_Timer = 1;
                   // HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
                }

                while (Timer_100_ms > 0) {
                    I_4_IN_M += rawADC_1[0] * Uverif / 4095.f;
                    U_4_IN_M += rawADC_1[1] * Uverif / 4095.f;
                    U_4_OUT_M += rawADC_1[11] * Uverif / 4095.f;
                    counter++;
                }
                I_4_IN_M = I_4_IN_M / counter;
                U_4_IN_M = U_4_IN_M / counter;
                U_4_OUT_M = U_4_OUT_M / counter;
								minus_I = fabs(I_1_IN_M_not - I_4_IN_M);
                Algoritm_New(&TCC4, U_4_IN_M, U_4_OUT_M, minus_I, 1);
                counter = 0;
                I_4_IN_M = 0;
                U_4_IN_M = 0;
                U_4_OUT_M = 0;
                Start_Timer = 0;
                //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
                S_A_M_HIGHT
                S_B_M_HIGHT
                S_C_M_HIGHT
								S_A_M_HIGHT_plus
                S_B_M_HIGHT_plus
                S_C_M_HIGHT_plus
                osMutexRelease(mid_Thread_Mutex);
                if (TCC4.Period_1_sec == 1)
                    osDelay(1000);
                if (TCC4.Period_8_sec == 1)
                    osDelay(8000);
            }
        }
        taskYIELD();
    }
}

void Task_TCC(void)
{
    static uint16_t Median_mass[10];
    static uint16_t Median_mass_temp[10];
    uint8_t i;

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&rawADC_1, 14);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
    while (1) {
        for (i = 9; i > 0; i--) {
            Median_mass[i] = Median_mass[i - 1];
            Median_mass_temp[i] = Median_mass_temp[i - 1];
        }

        Median_mass[0] = rawADC_1[13];
        Median_mass_temp[0] = rawADC_1[12];

        Temp = Core_Tempsensor(Mediana_filter16(Median_mass_temp, 10));
        Uverif = 3.f; //((1678.f) * 3.f) / (Mediana_filter16(Median_mass, 10));
        osDelay(100);
    }
}
