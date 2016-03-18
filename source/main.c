
/******************************************************************************
 * This application implements the reading of the UART messages, and writting
 * LSB on the right GPIO.
 *
 *****************************************************************************/
 
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Application include from CMSIS. */
#include "stm32f10x.h"

/* Definition of preambles and trailer of package. */
#define PREAMBLE_1    0xAA
#define PREAMBLE_2    0x55
#define TRAILER       0xEE
/*-----------------------------------------------------------*/

/*
 * Perform any application specific hardware configuration.  The clocks,
 * memory, etc. are configured before main() is called.
 */
static void prvSetupHardware( void );

/* Main function that do the task. */
static void prvTask( void *pvParameters );
/*-----------------------------------------------------------*/

/* Print string on the UART port. */
void print_uart( const char* );
/*
 * CMSIS clock configuration function.
 */
extern void SystemCoreClockUpdate( void );
/*-----------------------------------------------------------*/

int main( void )
{
	/* Prepare the hardware to run this application. */
	prvSetupHardware();
	
	print_uart("Hardware setup.\n");
	
	/* Create the task that reads USART and makes GPIO output. */
	xTaskCreate( 	prvTask, 		            /* Function that implements the task. */
					(const char*)"UART",   	/* Text name of the task. */
					configMINIMAL_STACK_SIZE, 	/* Stack allocated to the task (in words). */
					NULL, 						/* The task parameter is not used. */
					configMAX_PRIORITIES-1,    	/* The priority of the task. */
					NULL );						/* Don't receive a handle back, it is not needed. */


	/* Start the kernel.  From here on, only tasks and interrupts will run. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then there
	was	insufficient FreeRTOS heap memory available for the idle and/or timer
	tasks to be created. */
	for( ;; );

	
	return 0;
}
/*-----------------------------------------------------------*/

void uartConfiguration(void)
 {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	
	/* Enable bus clocks.*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Set USART2 Tx (PA.02) as AF push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Set USART2 Rx (PA.03) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* USART clock initialization */
	USART_ClockStructInit(&USART_ClockInitStructure);
	USART_ClockInit(USART2, &USART_ClockInitStructure);
	
	/* USART initialization */
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	//Write USART2 parameters
	USART_Init(USART2, &USART_InitStructure);
	
	//Enable USART2
	USART_Cmd(USART2, ENABLE);
 }
/*-----------------------------------------------------------*/

void gpioConfiguration(void)
 {
	GPIO_InitTypeDef GPIO_InitStructure;
	 
	/* Enable bus clocks. */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
	
	/* Initialization of GPIOB. */
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
 }
 /*-----------------------------------------------------------*/

void uartSend(uint8_t ch)
{
     USART_SendData(USART2, (uint8_t) ch);
     /* Loop until the end of transmission */
     while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}
/*-----------------------------------------------------------*/

uint8_t uartReceive(void){
	 /* Loop until the end of transmission */
     while ( USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
        return (uint8_t)USART_ReceiveData(USART2);
}
/*-----------------------------------------------------------*/

void print_uart(const char *string)
{
    while(*string != '\0'){
        uartSend(*string);
        string++;
    }
}
/*-----------------------------------------------------------*/

static void prvTask( void *pvParameters )
{
	int8_t preamble1;
	int8_t preamble2;
	int8_t byte_LSB;
	int8_t byte_GPIO;
	int8_t trailer;
	
	/* Just to stop compiler warnings. */
	(void) pvParameters;          

	print_uart("Starting task.\n");	
	
	for( ;; )
	{	preamble1=uartReceive();
		if ( preamble1 == PREAMBLE_1 )
		{
			/* If first preamble is 0xAA continue. */
			preamble2=uartReceive();
			if ( preamble2 == PREAMBLE_2 )
			{	
				/* If second preamble is 0x55 continue. */
				byte_GPIO=uartReceive();
				byte_LSB=uartReceive();
				trailer=uartReceive();
				if ( trailer == TRAILER )
				{
					/* If trailer is 0xEE, USART 
					successfully received package */
					byte_LSB&=1;
					if ( byte_GPIO>=0 && byte_GPIO<=15 )
					{
						GPIO_WriteBit(GPIOB, 1<<byte_GPIO , byte_LSB);   
					}
					
				}
				
			}
		}	
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	SystemInit();
	uartConfiguration();
	gpioConfiguration();
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}




