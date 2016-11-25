extern void Holt_Configuration(uint8_t HOLT,uint16_t DATA);
extern void Holt_Read_SR(uint8_t HOLT);
extern uint16_t Holt_Read_CR(uint8_t HOLT);
extern unsigned int Holt_Read_RFIFO(uint8_t HOLT);
extern void Holt_Write_TFIFO1(uint8_t HOLT,unsigned int _data,uint8_t address);

