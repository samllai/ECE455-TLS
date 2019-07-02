/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h"
#include "../Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 	100
#define MAX_POTENTIOMETER	4096

#define DISPLAY_REFRESH			2000
#define TRAFFICLIGHT_REFRESH	4000

#define OFF		0
#define ON		1

#define RED		0b001
#define YELLOW	0b100
#define GREEN	0b010

#define LINEUP		21
#define PASSING		13
#define EXIT		7

#define TRAFFIC_LIGHT_RED 		8
#define TRAFFIC_LIGHT_YELLOW	10
#define TRAFFIC_LIGHT_GREEN		9


GPIO_InitTypeDef GPIO_InitData;
GPIO_InitTypeDef GPIO_InitClock;
GPIO_InitTypeDef GPIO_InitClear;
GPIO_InitTypeDef GPIO_InitPotentiometer;
ADC_InitTypeDef ADC_InitPotentiometer;

void clear_LEDs(void);
void initialize_ADC(void);
void initialize_GPIO(void);
void send_clock_pulse(void);

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void Flow_Task( void *pvParameters );
static void Creator_Task( void *pvParameters );
static void StopLight_Callback( void *pvParameters );
static void Display_Task( void *pvParameters );

xQueueHandle xQueue_newCar = 0;
xQueueHandle xQueue_stopLight = 0;
xQueueHandle xQueue_potentiometer = 0;

TimerHandle_t xTimer_stopLight = 0;

/*-----------------------------------------------------------*/

int main(void) {
	initialize_ADC();
	initialize_GPIO();

	srand(time(NULL));

	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */
	xQueue_newCar = xQueueCreate( 1, sizeof(uint8_t) );
	xQueue_stopLight = xQueueCreate( 1, sizeof(uint32_t) );
	xQueue_potentiometer = xQueueCreate( 1, sizeof(uint32_t) );

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xQueue_newCar, "NewCarQueue" );
	vQueueAddToRegistry( xQueue_stopLight, "StopLightQueue" );
	vQueueAddToRegistry( xQueue_potentiometer, "PotentiometerQueue" );

	xTimer_stopLight = xTimerCreate( "StopLightTimer", pdMS_TO_TICKS(TRAFFICLIGHT_REFRESH), pdTRUE, NULL, StopLight_Callback );

	xTaskCreate ( Flow_Task, 	"Flow",		configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate	( Creator_Task, "Creator",	configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate ( Display_Task, "Display",	configMINIMAL_STACK_SIZE, NULL, 3, NULL);

	if ( xTimerStart( xTimer_stopLight, 100 ) != pdPASS ) {
		printf( "Failed to start Traffic Light timer.\n" );
	}

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

/*-----------------------------------------------------------*/
// helper functions
/*-----------------------------------------------------------*/
void initialize_GPIO() {
	/* Initialize GPIOA */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// Data IO: GPIOA pin 1
	GPIO_StructInit(&GPIO_InitData);
	GPIO_InitData.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitData.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitData.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitData);

	// Clock IO: GPIOA pin 3
	GPIO_StructInit(&GPIO_InitClock);
	GPIO_InitClock.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitClock.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitClock.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitClock);

	// Clear IO: GPIOA pin 5
	GPIO_StructInit(&GPIO_InitClear);
	GPIO_InitClear.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitClear.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitClear.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitClear);

	// Potentiometer IO: GPIOA pin 7
	GPIO_StructInit(&GPIO_InitPotentiometer);
	GPIO_InitPotentiometer.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitPotentiometer.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitPotentiometer.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitPotentiometer);
}

void initialize_ADC() {
	/* Initialize ADC */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// Potentiometer IO: GPIOA pin 7
	ADC_StructInit(&ADC_InitPotentiometer);
	ADC_Init(ADC1, &ADC_InitPotentiometer);
	ADC_Cmd(ADC1, ENABLE);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_84Cycles);
}

void send_clock_pulse() {
	GPIO_WriteBit(GPIOA, GPIO_InitClock.GPIO_Pin, Bit_SET); //clock up
	GPIO_WriteBit(GPIOA, GPIO_InitClock.GPIO_Pin, Bit_RESET); //clock down
}

void clear_LEDs() {
	GPIO_WriteBit(GPIOA, GPIO_InitClear.GPIO_Pin, Bit_RESET); //clear down
	GPIO_WriteBit(GPIOA, GPIO_InitClear.GPIO_Pin, Bit_SET); //clear up
}

int readADC() {
	ADC_SoftwareStartConv(ADC1);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
		return ADC_GetConversionValue(ADC1);
	}

	return OFF;
}


/*-----------------------------------------------------------*/
static void Flow_Task( void *pvParameters ) {
	int potentiometer = 0;

	while (1) {
		xQueueReset(xQueue_potentiometer);

		potentiometer = readADC();
		printf("Flow - Potentiometer value: %d\n", potentiometer);

		if( xQueueSend(xQueue_potentiometer, &potentiometer, 5000));
		else printf("Error sending potentiometer data in Flow.\n");

		vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH));
	}
}
/*-----------------------------------------------------------*/
static void Creator_Task ( void *pvParameters ) {
	int flowRate = 0;
	int createCar = 0;

	while (1) {
		// Get Potentiometer value.
		if(xQueueReceive(xQueue_potentiometer, &flowRate, 0)) {
			printf("Creator - Received potentiometer value: %d\n", flowRate);

			// Place same potentiometer value back on the queue.
			if( xQueueSend(xQueue_potentiometer, &flowRate, 5000)) {
				printf("Creator - Sent potentiometer value: %d\n", flowRate);
			} else printf("Error receiving potentiometer data in Creator.\n");
		} else printf("Error sending potentiometer data in Creator.\n");


		//use flowRate to change createCar
		createCar = flowRate >= (rand() % MAX_POTENTIOMETER);

		if (createCar == OFF) {
			if( xQueueSend(xQueue_newCar,&createCar,0) ) {
				printf("newCar = 0\n");
			} else printf("NewCar Failed!\n");
		} else if (createCar == ON) {
			if( xQueueSend(xQueue_newCar,&createCar,0) ) {
				printf("newCar = 1\n");
			} else printf("NewCar Failed!\n");
		} else printf("Error: newCar value not binary\n");

		vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH));
	}
}
/*-----------------------------------------------------------*/
static void StopLight_Callback( void *pvParameters ) {
	int lightStatus = GREEN;
	int prevLightStatus = GREEN;
	int flowRate = MAX_POTENTIOMETER;
	int threshold = rand() % MAX_POTENTIOMETER;

	/* Task doesn't succeed unless the potentiometer value can be received and sent. */
	// Get Potentiometer value.
	if(xQueueReceive(xQueue_potentiometer, &flowRate, 0)) {
		printf("StopLight - Received potentiometer value: %d\n", flowRate);

		// Place same potentiometer value back on the queue.
		if( xQueueSend(xQueue_potentiometer, &flowRate, 5000)) {
			printf("StopLight - Sent potentiometer value: %d\n", flowRate);
		} else printf("Error sending potentiometer data in StopLight.\n");
	} else printf("Error receiving potentiometer data in StopLight.\n");

	if(xQueueReceive(xQueue_stopLight, &prevLightStatus, 0));
	else printf("Error receiving stopLight data in StopLight.\n");

	//use flowRate to change lightStatus
	if (prevLightStatus == RED) {
		if (xTimerGetPeriod(xTimer_stopLight) > pdMS_TO_TICKS(500)) {
			xTimerChangePeriod(xTimer_stopLight, xTimerGetPeriod(xTimer_stopLight) / 2, 10);
		}

		if (flowRate < threshold) {
			lightStatus = RED;
		} else {
			xTimerChangePeriod(xTimer_stopLight, pdMS_TO_TICKS(TRAFFICLIGHT_REFRESH *2), 10);
			lightStatus = GREEN;
		}
	} else if (prevLightStatus == YELLOW) {
		lightStatus = RED;
	} else if (prevLightStatus == GREEN) {
		if (xTimerGetPeriod(xTimer_stopLight) != pdMS_TO_TICKS(TRAFFICLIGHT_REFRESH)) {
			xTimerChangePeriod(xTimer_stopLight, pdMS_TO_TICKS(TRAFFICLIGHT_REFRESH), 10);
		}

		if (flowRate < threshold) {
			lightStatus = YELLOW;
		} else {
			lightStatus = GREEN;
		}
	}

	// Change the light status based on the potentiometer value.
	if( xQueueSend(xQueue_stopLight, &lightStatus, 5000)) {
		printf("Changing lights...\n");
	} else printf("LightStatus Failed!\n");
}
/*-----------------------------------------------------------*/
static void Display_Task( void *pvParameters ) {
	int newCar		= OFF;
	int stopLight	= OFF;
	int nextLight	= OFF;

	int bPassing	= 0b000;
	int bLights		= GREEN;
	int bExit		= 0b00000000;
	int bLineup		= 0b00000000;
	int bSystem		= 0b0000000000000000000000;

	while (1) {
		clear_LEDs();

		//shift lights
		bExit >>= 1;
		int passUp = (bPassing) & 1;
		bExit = bExit | (passUp << 7);

		//update traffic light
		if( xQueueReceive( xQueue_stopLight, &stopLight, 0 ) ) {
			bLights = stopLight;
			if( xQueueSend( xQueue_stopLight, &stopLight, 100 ) );
		}


		bPassing >>= 1;
		if (bLights == GREEN) {
			int lineupUp = (bLineup) & 1;
			bPassing |= (lineupUp << 2);
			bLineup >>= 1;
		} else  {
			int hold = 0;
			int tempLineup = bLineup;

			while ((tempLineup) & 1) { //find out how many lights are on in a row from first light
				hold++;
				tempLineup >>= 1;
			}

			int mask = (1 << hold) - 1;
			bLineup = (bLineup >> 1) | mask;
		}

		if(xQueueReceive(xQueue_newCar, &newCar, 0)) {
			if (newCar) bLineup  |= (1<<7);
		} else printf("Error receiving newCar data:%u\n", newCar);

		bSystem = bExit | (bLights << 8) | (bPassing << 11) | (bLineup << 14);

		int bCopySystem = bSystem;
		for (int led_index = 0; led_index < 22; led_index++) {
			nextLight = bCopySystem & 1;
			bCopySystem >>= 1;

			GPIO_WriteBit(GPIOA, GPIO_InitData.GPIO_Pin, nextLight);

			send_clock_pulse();
		}

		vTaskDelay(DISPLAY_REFRESH); //hold delay
	}
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}
