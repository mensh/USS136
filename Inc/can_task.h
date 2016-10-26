void CAN_Transmit(void);

#define Net_Pm_bx_ON HAL_GPIO_WritePin(GPIOG, OUT1_M_Pin, GPIO_PIN_SET);
#define Net_Pm_bx_OFF HAL_GPIO_WritePin(GPIOG, OUT1_M_Pin, GPIO_PIN_RESET);

#define MaxDTtnd_ON HAL_GPIO_WritePin(GPIOG, OUT2_M_Pin, GPIO_PIN_SET);
#define MaxDTtnd_OFF HAL_GPIO_WritePin(GPIOG, OUT2_M_Pin, GPIO_PIN_RESET);

#define MaxDnvd_ON HAL_GPIO_WritePin(GPIOG, OUT3_M_Pin, GPIO_PIN_SET);
#define MaxDnvd_OFF HAL_GPIO_WritePin(GPIOG, OUT3_M_Pin, GPIO_PIN_RESET);

#define BHDisp_ON HAL_GPIO_WritePin(GPIOG, OUT4_M_Pin, GPIO_PIN_SET);
#define BHDisp_OFF HAL_GPIO_WritePin(GPIOG, OUT4_M_Pin, GPIO_PIN_RESET);
