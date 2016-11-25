struct _s_STR {
  float P;
  uint8_t Struhka;
  uint8_t Period_8_sec;
  uint8_t Kz;
  uint8_t Period_1_sec;
  uint8_t Otkaz;
	uint8_t Obriv;
	float Ushunt;
	float Uio;
	float Iresult;
	uint32_t counter_KZ;
};

#define S_A_M_LOW HAL_GPIO_WritePin(S_A_M_GPIO_Port, S_A_M_Pin, GPIO_PIN_RESET);
#define S_A_M_HIGHT HAL_GPIO_WritePin(S_A_M_GPIO_Port, S_A_M_Pin, GPIO_PIN_SET);

#define S_B_M_LOW HAL_GPIO_WritePin(S_B_M_GPIO_Port, S_B_M_Pin, GPIO_PIN_RESET);
#define S_B_M_HIGHT HAL_GPIO_WritePin(S_B_M_GPIO_Port, S_B_M_Pin, GPIO_PIN_SET);

#define S_C_M_LOW HAL_GPIO_WritePin(S_C_M_GPIO_Port, S_C_M_Pin, GPIO_PIN_RESET);
#define S_C_M_HIGHT HAL_GPIO_WritePin(S_C_M_GPIO_Port, S_C_M_Pin, GPIO_PIN_SET);

#define Kp 19.24
#define P_SIG_POROG 20.f
#define Kp1 10.1

extern void Sensor_4_Task(void const *argument);
extern void Sensor_3_Task(void const *argument);
extern void Sensor_2_Task(void const *argument);
extern void Sensor_1_Task(void const *argument);


extern void Sensor_1_Task_Vsk (void const *argument);
extern void Sensor_2_Task_Vsk (void const *argument);
extern void Sensor_3_Task_Vsk (void const *argument);
extern void Sensor_4_Task_Vsk (void const *argument);

extern struct _s_STR TCC1;
extern struct _s_STR TCC2;
extern struct _s_STR TCC3;
extern struct _s_STR TCC4;

void Task_TCC(void);
