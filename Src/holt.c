#include "cmsis_os.h"
#include "stm32f2xx_hal.h"

extern SPI_HandleTypeDef hspi1;
uint8_t SPI1_TX[8];
uint8_t SPI1_RX[8];
uint8_t DATA_R[8];
uint8_t DATA_CR[8];

uint32_t Timer_RZ_1_BusState = 0;
uint32_t Timer_RZ_2_BusState = 0;

#define HOLT_RxE (1 << 0) // Recive   FIFO Empty
#define HOLT_RxF (1 << 2) // Recive   FIFO Full
#define HOLT_TxE (1 << 3) // Transmit FIFO Empty
#define HOLT_TxF (1 << 5) // Transmit FIFO Full

#define false 0
#define true 1

#define CS_UP HAL_GPIO_WritePin(CS_HOLT_GPIO_Port, CS_HOLT_Pin, GPIO_PIN_SET);
#define CS_DOWN HAL_GPIO_WritePin(CS_HOLT_GPIO_Port, CS_HOLT_Pin, GPIO_PIN_RESET);
void SPI2_Start(uint8_t HOLT, uint8_t* SPI_TxBuf, uint8_t* SPI_RxBuf, uint8_t data_len)
{
    CS_DOWN
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)(SPI_TxBuf), (uint8_t*)(SPI_RxBuf), data_len);
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
        taskYIELD();
    }
    CS_UP
}

void Holt_Configuration(uint8_t HOLT, uint16_t DATA)
{
    unsigned char HCFG = 0;
    int lol = 0;
    while (HCFG != 3) {
        if (lol++ > 0xFFF)
            break;

        switch (HCFG) {
        case 0: // Configure Control Register
        {
            SPI1_TX[0] = 0x10; //Write Control Register
            SPI1_TX[1] = ((DATA & 0xff00) >> 8); //[E028] - for 100  kHz
            SPI1_TX[2] = (DATA & 0x00ff); //[E429] - for 12.5 kHz
            SPI2_Start(HOLT, SPI1_TX, SPI1_RX, 3);
            HCFG = 3;
            break;
        }

        case 1: // ACLK Division
        {
            SPI1_TX[0] = 0x07; //Write ACLK Division
            SPI1_TX[1] = 0x01; //Division x1
            SPI2_Start(HOLT, SPI1_TX, SPI1_RX, 2);
            HCFG = 3;
            break;
        }

        case 2: // Reset Holt
        {
            SPI1_TX[0] = 0x01; //Reset HI3585
            SPI2_Start(HOLT, SPI1_TX, SPI1_RX, 1);
            HCFG = 3;
            break;
        }

        default:
            break;
        }
    }
}

//-----------------------------------
void Holt_Reset_TxFIFO(uint8_t HOLT)
{
    while (1) {

        SPI1_TX[0] = 0x11; //Reset Transmit FIFO
        SPI2_Start(HOLT, SPI1_TX, SPI1_RX, 1);
        break;
    }
}

unsigned char Holt_SR = 0;
//-----------------------------------
void Holt_Read_SR(uint8_t HOLT)
{
    unsigned char HRSR = 0;

    while (HRSR != 3) {

        switch (HRSR) {
        case 0: {
            SPI1_TX[0] = 0x0A; //Read Status Registr
            SPI1_TX[1] = 0x00;
            SPI2_Start(HOLT, SPI1_TX, SPI1_RX, 2);
            HRSR = 1;
            break;
        }

        case 1: {
            HRSR = 2;
            break;
        }

        case 2: {
            Holt_SR = SPI1_RX[1];
            HRSR = 3;
            break;
        }

        default:
            break;
        }
    }
}

uint8_t isHoltRXnotEmpty(uint8_t HOLT)
{
    Holt_Read_SR(HOLT);
    return (!(Holt_SR & HOLT_RxE)) ? true : false;
}

//-----------------------------------

uint16_t Holt_Read_CR(uint8_t HOLT)
{

    unsigned short Holt_CR = 0;

    SPI1_TX[0] = 0x0B; //Read Control Register
    SPI1_TX[1] = 0x00;
    SPI1_TX[2] = 0x00;

    SPI2_Start(HOLT, SPI1_TX, SPI1_RX, 3);

    Holt_CR = SPI1_RX[1] << 8 | SPI1_RX[2];

    return Holt_CR;
}

struct HOLT_RX_DATA {
    uint8_t Addres;
    uint16_t DATA;
};

//-----------------------------------
unsigned int Holt_Read_RFIFO(uint8_t HOLT)
{
    unsigned int result = 0;

    SPI1_TX[0] = 0x08; //Read RX FIFO
    SPI1_TX[1] = 0x00;
    SPI1_TX[2] = 0x00;
    SPI1_TX[3] = 0x00;
    SPI1_TX[4] = 0x00;
    SPI1_TX[5] = 0x00;

    SPI2_Start(HOLT, SPI1_TX, SPI1_RX, 5);
    result = SPI1_RX[1] << 24 | SPI1_RX[2] << 16 | SPI1_RX[3] << 8 | SPI1_RX[4];

    return result;
};

void Holt_Write_TFIFO1(uint8_t HOLT, unsigned int _data, uint8_t address)
{
    while (1) {

        SPI1_TX[0] = 0x0E; //Write TX FIFO
        SPI1_TX[1] = _data >> 24;
        SPI1_TX[2] = _data >> 16;
        SPI1_TX[3] = _data >> 8;
        SPI1_TX[4] = address;
        SPI2_Start(HOLT, SPI1_TX, SPI1_RX, 5);
        break;
    }
}
