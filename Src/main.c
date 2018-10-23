/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
/* http://www.openstm32.org/forumthread339 */
/* #include "cmsis_os.h" */
#include "thb-bsp.h"

#include "RobotCore_C.h"

//#include "core_cm4.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
struct RobotCore * ptr_RobotCore;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void initialise_monitor_handles(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// Stub for release version
__weak void initialise_monitor_handles(void)
{
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART and Loop until the end of transmission */
  thb_UART4_SendData((char *)&ch, 1);

  return ch;
}

GETCHAR_PROTOTYPE {
	char ch = '\0';

	thb_UART4_ReceivedData((char *)&ch, 1);

	return ch;
}
static const char CrashMessage[]="Crash!!!\n";

#if 1
// https://www.segger.com/downloads/application-notes/AN00016

// System Handler Control and State Register
#define SYSHND_CTRL (*(volatile unsigned int*)  (0xE000ED24u))
// Memory Management Fault Status Register
#define NVIC_MFSR   (*(volatile unsigned char*) (0xE000ED28u))
// Bus Fault Status Register
#define NVIC_BFSR   (*(volatile unsigned char*) (0xE000ED29u))
// Usage Fault Status Register
#define NVIC_UFSR   (*(volatile unsigned short*)(0xE000ED2Au))
// Hard Fault Status Register
#define NVIC_HFSR   (*(volatile unsigned int*)  (0xE000ED2Cu))
// Debug Fault Status Register
#define NVIC_DFSR   (*(volatile unsigned int*)  (0xE000ED30u))
// Bus Fault Manage Address Register
#define NVIC_BFAR   (*(volatile unsigned int*)  (0xE000ED38u))
// Auxiliary Fault Status Register
#define NVIC_AFSR   (*(volatile unsigned int*)  (0xE000ED3Cu))

static struct {
  struct {
    volatile unsigned int r0;           // Register R0
    volatile unsigned int r1;// Register R1
    volatile unsigned int r2;// Register R2
    volatile unsigned int r3;// Register R3
    volatile unsigned int r12;// Register R12
    volatile unsigned int lr;// Link register
    volatile unsigned int pc;  // Program counter
    union {
      volatile unsigned int byte;
      struct {
        unsigned int IPSR : 8;          // Interrupt Program Status register (IPSR)
        unsigned int EPSR : 19;         // Execution Program Status register (EPSR)
         unsigned int APSR : 5;           // Application Program Status register (APSR)
      } bits;
    } psr;                              // Program status register.
  } SavedRegs;

  union {
   volatile unsigned int byte;
   struct {
     unsigned int MEMFAULTACT    : 1;// Read as 1 if memory management fault
                                       // is active
     unsigned int BUSFAULTACT     : 1;// Read as 1 if bus fault exception is active
  unsigned int UnusedBits1    : 1;
  unsigned int USGFAULTACT    : 1;// Read as 1 if usage fault exception
                                     // is active
  unsigned int UnusedBits2    : 3;
  unsigned int SVCALLACT      : 1;// Read as 1 if SVC exception is active
  unsigned int MONITORACT     : 1;// Read as 1 if debug monitor exception
                                     // is active
  unsigned int UnusedBits3    : 1;
  unsigned int PENDSVACT      : 1;// Read as 1 if PendSV exception is active
  unsigned int SYSTICKACT     : 1;// Read as 1 if SYSTICK exception is active
  unsigned int USGFAULTPENDED : 1;// Usage fault pended; usage fault started
                                     // but was replaced by a higher-priority
                                     // exception
  unsigned int MEMFAULTPENDED : 1;// Memory management fault pended; memory
                                     // management fault started but was
                                     // replaced by a higher-priority exception
  unsigned int BUSFAULTPENDED : 1;// Bus fault pended; bus fault handler was
                                     // started but was replaced by a
                                     // higher-priority exception
  unsigned int SVCALLPENDED   : 1;// SVC pended; SVC was started but was
                                     // replaced by a higher-priority exception
  unsigned int MEMFAULTENA    : 1;// Memory management fault handler enable
  unsigned int BUSFAULTENA    : 1;// Bus fault handler enable
  unsigned int USGFAULTENA    : 1;// Usage fault handler enable
} bits;
} syshndctrl;

  union {
    volatile unsigned char byte;
    struct {
      unsigned char IACCVIOL    : 1;// Instruction access violation
      unsigned char DACCVIOL    : 1;// Data access violation
      unsigned char UnusedBits  : 1;
      unsigned char MUNSTKERR   : 1;// Unstacking error
      unsigned char MSTKERR     : 1; // Stacking error
      unsigned char UnusedBits2 : 2;
      unsigned char MMARVALID   : 1;// Indicates the MMAR is valid
    } bits;
  } mfsr;
                                // Memory Management Fault Status
                                         // Register (0xE000ED28)
  union {
    volatile unsigned int byte;
    struct {
      unsigned int IBUSERR    : 1; // Instruction access violation
      unsigned int PRECISERR  : 1;// Precise data access violation
      unsigned int IMPREISERR : 1;// Imprecise data access violation
      unsigned int UNSTKERR   : 1;// Unstacking error
      unsigned int STKERR     : 1;// Stacking error
      unsigned int UnusedBits : 2;
      unsigned int BFARVALID  : 1; // Indicates BFAR is valid
    } bits;
  } bfsr; // Bus Fault Status Register (0xE000ED29)

  volatile unsigned int bfar;            // Bus Fault Manage Address Register
                                         // (0xE000ED38)
  union {
    volatile unsigned short byte;
    struct {
      unsigned short UNDEFINSTR : 1;     // Attempts to execute an undefined
                                         // instruction
      unsigned short INVSTATE   : 1;     // Attempts to switch to an invalid state
                                         // (e.g., ARM)
      unsigned short INVPC      : 1;     // Attempts to do an exception with a bad
                                         // value in the EXC_RETURN number
      unsigned short NOCP       : 1;     // Attempts to execute a coprocessor
                                         // instruction
      unsigned short UnusedBits : 4;
      unsigned short UNALIGNED  : 1;     // Indicates that an unaligned access fault
                                         // has taken place
      unsigned short DIVBYZERO  : 1;     // Indicates a divide by zero has taken
                                         // place (can be set only if DIV_0_TRP
                                         // is set)
    } bits;
  } ufsr;

  union {
    volatile unsigned int byte;
    struct {
      unsigned int UnusedBits  : 1;
  unsigned int VECTBL      : 1;		// Indicates hard fault is caused by failed
                                    // vector fetch
  unsigned int UnusedBits2 : 28;
  unsigned int FORCED      : 1;		// Indicates hard fault is taken because of
                                    // bus fault/memory management fault/usage
                                    // fault
  unsigned int DEBUGEVT    : 1;		// Indicates hard fault is triggered by
                                    // debug event
	} bits;
  } hfsr;							// Hard Fault Status Register (0xE000ED2C)

  union {
	  volatile unsigned int byte;
	  struct {
		  unsigned int HALTED   : 1;// Halt requested in NVIC
		  unsigned int BKPT     : 1;// BKPT instruction executed
		  unsigned int DWTTRAP  : 1;// DWT match occurred
		  unsigned int VCATCH   : 1;// Vector fetch occurred
		  unsigned int EXTERNAL : 1;// EDBGRQ signal asserted
    } bits;
  } dfsr;                              // Debug Fault Status Register (0xE000ED30)
  volatile unsigned int afsr;          // Auxiliary Fault Status Register
                                     // (0xE000ED3C) Vendor controlled (optional)
} HardFaultRegs;

__weak void HardFault_Handler(void)
{
	__asm volatile
	(
			" tst lr, #4                                                \n"
	        " ite eq                                                    \n"
	        " mrseq r0, msp                                             \n"
	        " mrsne r0, psp                                             \n"
			" b     HardFaultHandler                                    \n"
	);
}

static volatile uint32_t _Continue = 0u;

void HardFaultHandler(unsigned int* pStack){

	  //
	  // In case we received a hard fault because of a breakpoint instruction, we return.
	  // This may happen when using semihosting for printf outputs and no debugger
	  // is connected, i.e. when running a "Debug" configuration in release mode.
	  //
	if (SCB->HFSR & (1uL << 31)) {
		SCB->HFSR |=  (1uL << 31);
	// Reset Hard Fault status
	  *(pStack + 6u) += 2u;
	        // PC is located on stack at SP + 24 bytes;
	                               // increment PC by 2 to skip break instruction.
	  return;
	}

	//
	// Read NVIC registers
	//
	HardFaultRegs.syshndctrl.byte = SYSHND_CTRL;	// System Handler Control and
	                                                // State Register
	HardFaultRegs.mfsr.byte       = NVIC_MFSR;      // Memory Fault Status Register
	HardFaultRegs.bfsr.byte       = NVIC_BFSR;    	// Bus Fault Status Register
	HardFaultRegs.bfar            = NVIC_BFAR;    	// Bus Fault Manage Address Register
	HardFaultRegs.ufsr.byte       = NVIC_UFSR;    	// Usage Fault Status Register
	HardFaultRegs.hfsr.byte       = NVIC_HFSR;    	// Hard Fault Status Register
	HardFaultRegs.dfsr.byte       = NVIC_DFSR;    	// Debug Fault Status Register
	HardFaultRegs.afsr            = NVIC_AFSR;    	// Auxiliary Fault Status Register
	//
	// Halt execution
	// If NVIC registers indicate readable memory, change the variable value
	// to != 0 to continue execution.
	//

	thb_UART4_SendData(&HardFaultRegs.bfsr.byte, 1);

	thb_UART4_SendData((char *)CrashMessage, 9);
	while (_Continue == 0u);

	//
	// Read saved registers from the stack
	//
	HardFaultRegs.SavedRegs.r0       = pStack[0];  // Register R0
	HardFaultRegs.SavedRegs.r1       = pStack[1];  // Register R1
	HardFaultRegs.SavedRegs.r2       = pStack[2];  // Register R2
	HardFaultRegs.SavedRegs.r3       = pStack[3];  // Register R3
	HardFaultRegs.SavedRegs.r12      = pStack[4];  // Register R12
	HardFaultRegs.SavedRegs.lr       = pStack[5];  // Link register LR
	HardFaultRegs.SavedRegs.pc       = pStack[6];  // Program counter PC
	HardFaultRegs.SavedRegs.psr.byte = pStack[7];  // Program status word PSR
	//
	// Halt execution
	// To step out of the HardFaultHandler, change the variable value to != 0.
	//
	_Continue = 0u;
	while (_Continue == 0u) {
	}
}
#else
/************************************************************************************/
// https://mcuoneclipse.com/2017/07/02/using-freertos-with-newlib-and-newlib-nano/
// configUSE_NEWLIB_REENTRANT
//
/* https://www.freertos.org/Debugging-Hard-Faults-On-Cortex-M-Microcontrollers.html */
/************************************************************************************/
/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
__weak void HardFault_Handler(void)
{
	static volatile uint32_t _Continue = 0u;
	  //
	  // In case we received a hard fault because of a breakpoint instruction, we return.
	  // This may happen when using semihosting for printf outputs and no debugger
	  // is connected, i.e. when running a "Debug" configuration in release mode.
	  //
	if (SCB->HFSR & (1uL << 31)) {
		SCB->HFSR |=  (1uL << 31);
	// Reset Hard Fault status
	  *(pStack + 6u) += 2u;
	        // PC is located on stack at SP + 24 bytes;
	                               // increment PC by 2 to skip break instruction.
	  return;
	}

	while(_Continue == 0u);
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
	static volatile uint32_t _Continue = 0u;
/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr; /* Link register. */
volatile uint32_t pc; /* Program counter. */
volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    /* When the following line is hit, the variables contain the register values. */
    while(_Continue == 0u);
}
/************************************************************************************/
#endif

void MainLoop(void)
{
	RobotCore_C_task(ptr_RobotCore);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	initialise_monitor_handles();

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART5_Init();
  MX_ADC3_Init();
  MX_UART4_Init();

  /* USER CODE BEGIN 2 */
  thb_UART4_Init();
  thb_UART5_Init();

  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

  // Init and Start Encoders Counters
  __HAL_TIM_SET_COUNTER(&htim2, 2147483647);
  __HAL_TIM_SET_COUNTER(&htim5, 2147483647);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  // Start Motors PWMs
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  ptr_RobotCore = RobotCore_C_new();
  RobotCore_C_init(ptr_RobotCore);

  //SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Pos;
	printf("Hello !\n");
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
