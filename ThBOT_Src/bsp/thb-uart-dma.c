
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
#include "task.h"
#include "cmsis_os.h"

#include "thb-task.h"
#include "thb-uart-dma.h"
#include "thb-fsm.h"
#include "thb-param.h"

osThreadId uart5ThdId;
TaskHandle_t uart5TaskHandle;
osThreadId uart4ThdId;
TaskHandle_t uart4TaskHandle;

#define LONG_TIME 0xffff

#define DMA_RX_BUFFER_SIZE          64
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
uint8_t DEBUG_UART_DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

#define UART_BUFFER_SIZE            64
#define UART_MAX_RX_BUFF            64

volatile uint32_t u32_WriteIndex = 0;
uint8_t LCD_UART_Buffer[UART_MAX_RX_BUFF];

volatile uint32_t u32_DebugWriteIndex = 0;
uint8_t DEBUG_UART_Buffer[UART_BUFFER_SIZE];

volatile uint32_t u32_DebugRxIndex = 0;
//volatile size_t DebugRead, DebugWrite = 0;

//volatile size_t Read, Write = 0;

SemaphoreHandle_t xSemCmdReady = NULL;
SemaphoreHandle_t xSemLineReady = NULL;

/* Stores the handle of the task that will be notified when the
transmission is complete. */
//static TaskHandle_t xTaskToNotify = NULL;

void thb_UART5Task(void const *argument){
    printf("Start UART5 task...\n");
    uint32_t uint32_ModeId;
    uint32_t uint32_StateId;
    uint32_t uint32_ParamId;
    uint32_t uint32_ParamLen;
    static uint32_t thread_notification;
    uint32_t u32_ReadIndex = 0;

    uart5TaskHandle = xTaskGetHandle("uart5Task");
    for(;;){
#if 0
        if( xSemaphoreTake( xSemCmdReady, portMAX_DELAY ) == pdTRUE )
#else
        /* Sleep until we are notified of a state change by an
         * interrupt handler. Note the first parameter is pdTRUE,
         * which has the effect of clearing the task's notification
         * value back to 0, making the notification value act like
         * a binary (rather than a counting) semaphore.  */

        thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(thread_notification)
#endif
        {
        	while (u32_ReadIndex != u32_WriteIndex)
        	{
        		if (   (LCD_UART_Buffer[u32_ReadIndex] == 'C')
        		    && (LCD_UART_Buffer[u32_ReadIndex+1] == 'M')
				    && (LCD_UART_Buffer[u32_ReadIndex+2] == 'D')
				   )
        		{
        			uint32_ModeId = ((char)LCD_UART_Buffer[u32_ReadIndex+4] -48) * 10 + ((char)LCD_UART_Buffer[u32_ReadIndex+5] - 48);
        			uint32_StateId  = ((char)LCD_UART_Buffer[u32_ReadIndex+7] -48) * 10 + ((char)LCD_UART_Buffer[u32_ReadIndex+8] - 48);

        			thb_fsm_ChangeModeState(uint32_ModeId, uint32_StateId);
        		}
        		else if (   (LCD_UART_Buffer[u32_ReadIndex] == 'P')
        				 && (LCD_UART_Buffer[u32_ReadIndex+1] == 'A')
						 && (LCD_UART_Buffer[u32_ReadIndex+2] == 'R')
				   )
        		{
        			uint32_ParamLen = ((char)LCD_UART_Buffer[u32_ReadIndex+4] -48) * 10 + ((char)LCD_UART_Buffer[u32_ReadIndex+5] - 48);
        			uint32_ParamId  = ((char)LCD_UART_Buffer[u32_ReadIndex+7] -48) * 10 + ((char)LCD_UART_Buffer[u32_ReadIndex+8] - 48);

        			LCD_UART_Buffer[u32_ReadIndex+10+uint32_ParamLen] = 0;

        			thb_param_SetParameter(uint32_ParamId, (char*)&LCD_UART_Buffer[u32_ReadIndex+10], uint32_ParamLen);
        		}
        		else
        		{
        			printf("Hum : %s\n", &LCD_UART_Buffer[u32_ReadIndex]);
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
	uint32_t MoreBytes = 0;

    if (DataLen > 60)
    {
    	MoreBytes = DataLen - 60;
    	DataLen = 60;
    }

	for(Index=0; Index < DataLen; Index++)
	{
	    LL_USART_TransmitData8(UART5, pu8_Buff[Index]);
	    while (!LL_USART_IsActiveFlag_TXE(UART5)) {}
	}

	if (MoreBytes > 0)
	{
		osDelay(300);
		for(Index=0; Index < MoreBytes; Index++)
		{
			LL_USART_TransmitData8(UART5, pu8_Buff[DataLen+Index]);
			while (!LL_USART_IsActiveFlag_TXE(UART5)) {}
		}
	}
}

void thb_UART5_Init(void)
{
    LL_USART_InitTypeDef USART_InitStruct;
    LL_DMA_InitTypeDef DMA_InitStruct;

    osThreadDef(uart5Task, thb_UART5Task, PRIORITY_UART, 0, 128);
    uart5ThdId = osThreadCreate(osThread(uart5Task), NULL);

    //for (uint32_t Index=0; Index < UART_MAX_RX_BUFF;Index++)
    	memset(LCD_UART_Buffer, 0, UART_BUFFER_SIZE);

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
        tocopy = UART_BUFFER_SIZE - u32_WriteIndex;      /* Get number of bytes we can copy to the end of buffer */

        /* Check how many bytes to copy */
        if (tocopy > len) {
            tocopy = len;
        }

        /* Write received data for UART main buffer for manipulation later */
        ptr = DMA_RX_Buffer;

        memset(&LCD_UART_Buffer[u32_WriteIndex], 0, UART_BUFFER_SIZE);
        memcpy(&LCD_UART_Buffer[u32_WriteIndex], ptr, tocopy);

        /* Correct values for remaining data */
        u32_WriteIndex += tocopy;
        len -= tocopy;
        ptr += tocopy;

        /* If still data to write for beginning of buffer */
        if (len) {
            memcpy(&LCD_UART_Buffer[u32_WriteIndex], ptr, len);      /* Don't care if we override Read pointer now */
            u32_WriteIndex = len;
        }

        /* Prepare DMA for next transfer */
        /* Important! DMA stream won't start if all flags are not cleared first */
        DMA1->HIFCR = DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4 | DMA_FLAG_TEIF0_4;
        DMA1_Stream0->M0AR = (uint32_t)DMA_RX_Buffer;   /* Set memory address for DMA again */
        DMA1_Stream0->NDTR = DMA_RX_BUFFER_SIZE;    /* Set number of bytes to receive */
        DMA1_Stream0->CR |= DMA_SxCR_EN;            /* Start DMA transfer */
#if 0
        //printf("Release semaphore\n");
        // Unblock the task by releasing the semaphore.
        xSemaphoreGiveFromISR( xSemCmdReady, &xHigherPriorityTaskWoken );

        //if( xHigherPriorityTaskWoken != pdFALSE )
        {
            /* Now inform the OS to check if a task-switch is necessary. */
            portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
        }
#else
        /* Unblock the handling task so the task can perform any processing
        necessitated by the interrupt.  xHandlingTask is the task's handle, which was
        obtained when the task was created.  vTaskNotifyGiveFromISR() also increments
        the receiving task's notification value. */
        vTaskNotifyGiveFromISR( uart5TaskHandle, &xHigherPriorityTaskWoken );

        /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
        The macro used to do this is dependent on the port and may be called
        portEND_SWITCHING_ISR. */
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void thb_UART4Task(void const *argument){
	static uint32_t thread_notification;
	uint32_t u32_ReadIndex = 0;

    printf("Start UART4 task...\n");

    uart4TaskHandle = xTaskGetHandle("uart4Task");
    for(;;){
#if 0
        if( xSemaphoreTake( xSemLineReady, portMAX_DELAY ) == pdTRUE )
#else
        /* Sleep until we are notified of a state change by an
         * interrupt handler. Note the first parameter is pdTRUE,
         * which has the effect of clearing the task's notification
         * value back to 0, making the notification value act like
         * a binary (rather than a counting) semaphore.  */

        thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(thread_notification)
#endif
        {
        	while (u32_ReadIndex != u32_DebugWriteIndex) {
       			// Echo the character...
        		printf("%c", DEBUG_UART_Buffer[u32_ReadIndex]);

        		u32_ReadIndex++;
        		if (u32_ReadIndex >= UART_BUFFER_SIZE)
        			u32_ReadIndex = 0;
        	}
        }
    }
}

void thb_UART4_SendData(char * pu8_Buff, uint32_t DataLen)
{
	uint32_t Index;

	for(Index=0; Index < DataLen; Index++)
	{
	    LL_USART_TransmitData8(UART4, pu8_Buff[Index]);
	    while (!LL_USART_IsActiveFlag_TXE(UART4)) {}

	    if ( pu8_Buff[Index] == '\n' )
	    {
	    	LL_USART_TransmitData8(UART4, '\r');
	    	while (!LL_USART_IsActiveFlag_TXE(UART4)) {}
	    }
	}
}

uint32_t thb_UART4_ReceivedData(char * pu8_Buff, uint32_t DataLen) {

	if (u32_DebugRxIndex != u32_DebugWriteIndex) {
		pu8_Buff[0] = DEBUG_UART_Buffer[u32_DebugRxIndex];
		//printf("[%i] <%c> \n", u32_DebugRxIndex, pu8_Buff[0]);
		u32_DebugRxIndex++;
		if (u32_DebugRxIndex >= UART_BUFFER_SIZE)
			u32_DebugRxIndex = 0;
		return 1;
	}
	pu8_Buff[0] = '\0';
	return 0;
}

void thb_UART4_Init(void)
{
	osThreadDef(uart4Task, thb_UART4Task, PRIORITY_UART, 0, 128);
	uart4ThdId = osThreadCreate(osThread(uart4Task), NULL);

    LL_USART_InitTypeDef USART_InitStruct;
    LL_DMA_InitTypeDef DMA_InitStruct;

   	memset(DEBUG_UART_Buffer, 0, UART_BUFFER_SIZE);

    vSemaphoreCreateBinary(xSemLineReady);
    xSemaphoreTake(xSemLineReady, 0);

    /* Configure USART */
    LL_USART_StructInit(&USART_InitStruct);
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    LL_USART_Init(UART4, &USART_InitStruct);

    /* Enable USART and enable interrupt for IDLE line detection */
    LL_USART_Enable(UART4);
    LL_USART_EnableDMAReq_RX(UART4);
    LL_USART_EnableIT_IDLE(UART4);

    /* Enable USART global interrupts */
    NVIC_SetPriority(UART4_IRQn, 1);
    NVIC_EnableIRQ(UART4_IRQn);

    /* Configure DMA for USART RX */
    LL_DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
    DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)DEBUG_UART_DMA_RX_Buffer;
    DMA_InitStruct.NbData = DMA_RX_BUFFER_SIZE;
    DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&UART4->DR;
    LL_DMA_Init(DMA1, LL_DMA_STREAM_2, &DMA_InitStruct);

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_2);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);

    /* Enable global DMA stream interrupts */
    NVIC_SetPriority(DMA1_Stream2_IRQn, 12);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}


void thb_UART4_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_IDLE(UART4)) {
        LL_USART_ClearFlag_IDLE(UART4);
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
    }
}

void thb_DMA1_Stream2_IRQHandler(void)
{
    size_t len, tocopy;
    uint8_t* ptr;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    portSET_INTERRUPT_MASK_FROM_ISR();

    if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
        LL_DMA_ClearFlag_TC2(DMA1);

        len = DMA_RX_BUFFER_SIZE - DMA1_Stream2->NDTR;
        tocopy = UART_BUFFER_SIZE - u32_DebugWriteIndex;      /* Get number of bytes we can copy to the end of buffer */

        /* Check how many bytes to copy */
        if (tocopy > len) {
            tocopy = len;
        }

        /* Write received data for UART main buffer for manipulation later */
        ptr = DEBUG_UART_DMA_RX_Buffer;

        //memset(&DEBUG_UART_Buffer[u32_DebugWriteIndex], 0, UART_BUFFER_SIZE-u32_DebugWriteIndex);
        memcpy(&DEBUG_UART_Buffer[u32_DebugWriteIndex], ptr, tocopy);

        /* Correct values for remaining data */
        u32_DebugWriteIndex += tocopy;
        len -= tocopy;
        ptr += tocopy;

        /* If still data to write for beginning of buffer */
        if (len) {
            memcpy(&DEBUG_UART_Buffer[u32_DebugWriteIndex], ptr, len);      /* Don't care if we override Read pointer now */
            u32_DebugWriteIndex = len;
        }
/*
        while (tocopy) {
			u32_DebugWriteIndex++;
			if (u32_DebugWriteIndex >= UART_BUFFER_SIZE)
				u32_DebugWriteIndex = 0;
			tocopy--;
        }
        while (len) {
			u32_DebugWriteIndex++;
			if (u32_DebugWriteIndex >= UART_BUFFER_SIZE)
				u32_DebugWriteIndex = 0;
			len--;
        }*/
        /* Prepare DMA for next transfer */
        /* Important! DMA stream won't start if all flags are not cleared first */
        DMA1->HIFCR = DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4 | DMA_FLAG_TEIF0_4;
        DMA1_Stream2->M0AR = (uint32_t)DEBUG_UART_DMA_RX_Buffer;   /* Set memory address for DMA again */
        DMA1_Stream2->NDTR = DMA_RX_BUFFER_SIZE;    /* Set number of bytes to receive */
        DMA1_Stream2->CR |= DMA_SxCR_EN;            /* Start DMA transfer */
#if 0
        //printf("Release semaphore\n");
        // Unblock the task by releasing the semaphore.
        xSemaphoreGiveFromISR( xSemLineReady, &xHigherPriorityTaskWoken );

        //if( xHigherPriorityTaskWoken != pdFALSE )
        {
            /* Now inform the OS to check if a task-switch is necessary. */
            portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
        }
#else
        /* Unblock the handling task so the task can perform any processing
        necessitated by the interrupt.  xHandlingTask is the task's handle, which was
        obtained when the task was created.  vTaskNotifyGiveFromISR() also increments
        the receiving task's notification value. */
        vTaskNotifyGiveFromISR( uart4TaskHandle, &xHigherPriorityTaskWoken );

        /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
        The macro used to do this is dependent on the port and may be called
        portEND_SWITCHING_ISR. */
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

