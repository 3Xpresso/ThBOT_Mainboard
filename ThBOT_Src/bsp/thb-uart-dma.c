
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "string.h"
#include "stdlib.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "thb-uart-dma.h"
#include "thb-fsm.h"

#define LONG_TIME 0xffff

#define DMA_RX_BUFFER_SIZE          64
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

#define UART_BUFFER_SIZE            64
#define UART_MAX_RX_BUFF            10

uint32_t u32_ReadIndex = 0;
uint32_t u32_WriteIndex = 0;
uint8_t UART_Buffer[UART_MAX_RX_BUFF][UART_BUFFER_SIZE];

volatile size_t Read, Write = 0;

SemaphoreHandle_t xSemCmdReady = NULL;

void thb_UART5Task(void const *argument){
    printf("Start UART5 task...\n");
    uint32_t uint32_ModeId;
    uint32_t uint32_StateId;

    for(;;){
        if( xSemaphoreTake( xSemCmdReady, portMAX_DELAY ) == pdTRUE )
        {
        	while (u32_ReadIndex != u32_WriteIndex)
        	{
        		if (   (UART_Buffer[u32_ReadIndex][0] == 'C')
        		    && (UART_Buffer[u32_ReadIndex][1] == 'M')
				    && (UART_Buffer[u32_ReadIndex][2] == 'D')
				   )
        		{
        			uint32_ModeId = (UART_Buffer[u32_ReadIndex][4] -48) * 10 + (UART_Buffer[u32_ReadIndex][5] - 48);
        			uint32_StateId  = (UART_Buffer[u32_ReadIndex][7] -48) * 10 + (UART_Buffer[u32_ReadIndex][8] - 48);

        			thb_fsm_ChangeModeState(uint32_ModeId, uint32_StateId);
        		}
        		else
        		{
        			printf("Hum : %s\n", &UART_Buffer[u32_ReadIndex][0]);
        		}
        		u32_ReadIndex++;
        		if (u32_ReadIndex >= UART_MAX_RX_BUFF)
        			u32_ReadIndex = 0;
        	}
        }
    }
}

void thb_UART5_SendData(char * pu8_Buff, uint32_t DataLen)
{
	uint32_t Index;

	for(Index=0; Index < DataLen; Index++)
	{
	    LL_USART_TransmitData8(UART5, pu8_Buff[Index]);
	    while (!LL_USART_IsActiveFlag_TXE(UART5)) {}
	}
}

void thb_UART5_Init(void)
{
    LL_USART_InitTypeDef USART_InitStruct;
    LL_DMA_InitTypeDef DMA_InitStruct;

    for (uint32_t Index=0; Index < UART_MAX_RX_BUFF;Index++)
    	memset(UART_Buffer[Index], 0, UART_BUFFER_SIZE);

    vSemaphoreCreateBinary(xSemCmdReady);
    xSemaphoreTake(xSemCmdReady, 0);

    /* Configure USART */
    LL_USART_StructInit(&USART_InitStruct);
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    LL_USART_Init(UART5, &USART_InitStruct);
    
    /* Enable USART and enable interrupt for IDLE line detection */
    LL_USART_Enable(UART5);
    LL_USART_EnableDMAReq_RX(UART5);
    LL_USART_EnableIT_IDLE(UART5);
    
    /* Enable USART global interrupts */
    NVIC_SetPriority(UART5_IRQn, 1);
    NVIC_EnableIRQ(UART5_IRQn);
    
    /* Configure DMA for USART RX */
    LL_DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
    DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)DMA_RX_Buffer;
    DMA_InitStruct.NbData = DMA_RX_BUFFER_SIZE;
    DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&UART5->DR;
    LL_DMA_Init(DMA1, LL_DMA_STREAM_0, &DMA_InitStruct);
    
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
    
    /* Enable global DMA stream interrupts */
    NVIC_SetPriority(DMA1_Stream0_IRQn, 12);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}


void thb_UART5_IRQHandler(void)
{
    //printf("thb_UART5_IRQHandler\n");
    if (LL_USART_IsActiveFlag_IDLE(UART5)) {
        LL_USART_ClearFlag_IDLE(UART5);
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
    }
}

void thb_DMA1_Stream0_IRQHandler(void)
{
    size_t len, tocopy;
    uint8_t* ptr;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    portSET_INTERRUPT_MASK_FROM_ISR();

    //printf("thb_DMA1_Stream0_IRQHandler\n");
    if (LL_DMA_IsActiveFlag_TC0(DMA1)) {
        LL_DMA_ClearFlag_TC0(DMA1);

        len = DMA_RX_BUFFER_SIZE - DMA1_Stream0->NDTR;
        tocopy = UART_BUFFER_SIZE - Write;      /* Get number of bytes we can copy to the end of buffer */

        /* Check how many bytes to copy */
        if (tocopy > len) {
            tocopy = len;
        }

        /* Write received data for UART main buffer for manipulation later */
        ptr = DMA_RX_Buffer;
        //printf("DMA get : {%s} (%d - %d)\n", DMA_RX_Buffer, u32_WriteIndex, u32_ReadIndex);
        //memcpy(&UART_Buffer[u32_WriteIndex][Write], ptr, tocopy);   /* Copy first part */
        memset(&UART_Buffer[u32_WriteIndex][0], 0, UART_BUFFER_SIZE);
        memcpy(&UART_Buffer[u32_WriteIndex][0], ptr, tocopy);

        /* Correct values for remaining data */
        //Write += tocopy;
        //len -= tocopy;
        //ptr += tocopy;

        /* If still data to write for beginning of buffer */
        //if (len) {
        //    memcpy(&UART_Buffer[u32_WriteIndex][0], ptr, len);      /* Don't care if we override Read pointer now */
        //    Write = len;
        //}

        u32_WriteIndex++;
        if (u32_WriteIndex >= UART_MAX_RX_BUFF)
        	u32_WriteIndex = 0;

        /* Prepare DMA for next transfer */
        /* Important! DMA stream won't start if all flags are not cleared first */
        DMA1->HIFCR = DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4 | DMA_FLAG_TEIF0_4;
        DMA1_Stream0->M0AR = (uint32_t)DMA_RX_Buffer;   /* Set memory address for DMA again */
        DMA1_Stream0->NDTR = DMA_RX_BUFFER_SIZE;    /* Set number of bytes to receive */
        DMA1_Stream0->CR |= DMA_SxCR_EN;            /* Start DMA transfer */

        //printf("Release semaphore\n");
        // Unblock the task by releasing the semaphore.
        xSemaphoreGiveFromISR( xSemCmdReady, &xHigherPriorityTaskWoken );

        //if( xHigherPriorityTaskWoken != pdFALSE )
        {
            /* Now inform the OS to check if a task-switch is necessary. */
            portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}
