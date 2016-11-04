#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f2xx_hal.h"
#include "can.h"
#include "string.h"
#include "tcc_task.h"
#include "can_task.h"

extern  float I_1_IN_M;
extern	float I_2_IN_M;
extern  float I_3_IN_M;
extern	float I_4_IN_M;


void CAN_Transmit(void) {
  memset(hcan1.pTxMsg, 0, sizeof(CanTxMsgTypeDef));
  hcan1.pTxMsg->Data[0] =
      (uint8_t)(((uint32_t)((TCC1.P * 256.f)) & 0x00ff) >> 0);  //Òåìïåðàòóðà
  hcan1.pTxMsg->Data[1] =
      (uint8_t)(((uint32_t)((TCC1.P * 256.f)) & 0xff00) >> 8);  //Òåìïåðàòóðà

  hcan1.pTxMsg->Data[2] =
      (uint8_t)(((uint32_t)((TCC2.P * 256.f)) & 0x00ff) >> 0);  //Òåìïåðàòóðà
  hcan1.pTxMsg->Data[3] =
      (uint8_t)(((uint32_t)((TCC2.P * 256.f)) & 0xff00) >> 8);  //Òåìïåðàòóðà

  hcan1.pTxMsg->Data[4] =
      (uint8_t)(((uint32_t)((TCC3.P * 256.f)) & 0x00ff) >> 0);  //Òåìïåðàòóðà
  hcan1.pTxMsg->Data[5] =
      (uint8_t)(((uint32_t)((TCC3.P * 256.f)) & 0xff00) >> 8);  //Òåìïåðàòóðà

  hcan1.pTxMsg->Data[6] =
      (uint8_t)(((uint32_t)((TCC4.P * 256.f)) & 0x00ff) >> 0);  //Òåìïåðàòóðà
  hcan1.pTxMsg->Data[7] =
      (uint8_t)(((uint32_t)((TCC4.P * 256.f)) & 0xff00) >> 8);  //Òåìïåðàòóðà

  hcan1.pTxMsg->StdId = 0x02b;
  hcan1.pTxMsg->DLC = 8;
  hcan1.pTxMsg->IDE = CAN_ID_STD;
  hcan1.pTxMsg->RTR = CAN_RTR_DATA;
  HAL_CAN_Transmit(&hcan1, 500);
	
  memset(hcan1.pTxMsg, 0, sizeof(CanTxMsgTypeDef));
  hcan1.pTxMsg->Data[0] = TCC1.Kz << 0 | TCC1.Otkaz << 1 |
                          TCC1.Period_1_sec << 2 | TCC1.Period_8_sec << 3 |
                          TCC1.Struhka << 4;
  hcan1.pTxMsg->Data[1] = TCC2.Kz << 0 | TCC2.Otkaz << 1 |
                          TCC2.Period_1_sec << 2 | TCC2.Period_8_sec << 3 |
                          TCC2.Struhka << 4;
  hcan1.pTxMsg->Data[2] = TCC3.Kz << 0 | TCC3.Otkaz << 1 |
                          TCC3.Period_1_sec << 2 | TCC3.Period_8_sec << 3 |
                          TCC3.Struhka << 4;
  hcan1.pTxMsg->Data[3] = TCC4.Kz << 0 | TCC4.Otkaz << 1 |
                          TCC4.Period_1_sec << 2 | TCC4.Period_8_sec << 3 |
                          TCC4.Struhka << 4;
  hcan1.pTxMsg->StdId = 0x02c;
  hcan1.pTxMsg->DLC = 8;
  hcan1.pTxMsg->IDE = CAN_ID_STD;
  HAL_CAN_Transmit(&hcan1, 500);
	
	
	memset(hcan1.pTxMsg, 0, sizeof(CanTxMsgTypeDef));
  hcan1.pTxMsg->Data[0] =
      (uint8_t)(((uint32_t)((I_1_IN_M * 256.f)) & 0x00ff) >> 0);  //Òåìïåðàòóðà
  hcan1.pTxMsg->Data[1] =
      (uint8_t)(((uint32_t)((I_1_IN_M * 256.f)) & 0xff00) >> 8);  //Òåìïåðàòóðà

  hcan1.pTxMsg->Data[2] =
      (uint8_t)(((uint32_t)((I_2_IN_M * 256.f)) & 0x00ff) >> 0);  //Òåìïåðàòóðà
  hcan1.pTxMsg->Data[3] =
      (uint8_t)(((uint32_t)((I_2_IN_M * 256.f)) & 0xff00) >> 8);  //Òåìïåðàòóðà

  hcan1.pTxMsg->Data[4] =
      (uint8_t)(((uint32_t)((I_3_IN_M* 256.f)) & 0x00ff) >> 0);  //Òåìïåðàòóðà
  hcan1.pTxMsg->Data[5] =
      (uint8_t)(((uint32_t)((I_3_IN_M * 256.f)) & 0xff00) >> 8);  //Òåìïåðàòóðà

  hcan1.pTxMsg->Data[6] =
      (uint8_t)(((uint32_t)((I_4_IN_M * 256.f)) & 0x00ff) >> 0);  //Òåìïåðàòóðà
  hcan1.pTxMsg->Data[7] =
      (uint8_t)(((uint32_t)((I_4_IN_M * 256.f)) & 0xff00) >> 8);  //Òåìïåðàòóðà

  hcan1.pTxMsg->StdId = 0x02d;
  hcan1.pTxMsg->DLC = 8;
  hcan1.pTxMsg->IDE = CAN_ID_STD;
  hcan1.pTxMsg->RTR = CAN_RTR_DATA;
  HAL_CAN_Transmit_IT(&hcan1);
	
	
	
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle) {
  uint32_t _dataH = 0;
  uint32_t _dataL = 0;
  if (hcan1.pRxMsg->StdId == 0x2F) {
    _dataL = hcan1.pRxMsg->Data[3] << 24 | hcan1.pRxMsg->Data[2] << 16 |
             hcan1.pRxMsg->Data[1] << 8 | hcan1.pRxMsg->Data[0] << 0;
    _dataH = hcan1.pRxMsg->Data[7] << 24 | hcan1.pRxMsg->Data[6] << 16 |
             hcan1.pRxMsg->Data[5] << 8 | hcan1.pRxMsg->Data[4] << 0;
  }

  HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0);
  hcan1.Instance->IER = hcan1.Instance->IER | 1 << 1;
}
