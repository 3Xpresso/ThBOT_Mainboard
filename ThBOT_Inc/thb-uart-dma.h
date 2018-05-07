

extern void thb_UART5_Init(void);
extern void thb_UART5Task(void const *argument);
extern void thb_UART5_IRQHandler(void);
extern void thb_DMA1_Stream0_IRQHandler(void);
extern void thb_DMA1_Stream2_IRQHandler(void);

extern void thb_UART5_SendData(char * pu8_Buff, uint32_t DataLen);
extern void thb_UART4_SendData(char * pu8_Buff, uint32_t DataLen);
