/*
 * FreeRTOS V202212.01
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */


/*
 * This project contains an application demonstrating the use of the
 * FreeRTOS.org mini real time scheduler on the Luminary Micro LM3S811 Eval
 * board.  See http://www.FreeRTOS.org for more information.
 *
 * main() simply sets up the hardware, creates all the demo application tasks,
 * then starts the scheduler.  http://www.freertos.org/a00102.html provides
 * more information on the standard demo tasks.
 *
 * In addition to a subset of the standard demo application tasks, main.c also
 * defines the following tasks:
 *
 * + A 'Print' task.  The print task is the only task permitted to access the
 * LCD - thus ensuring mutual exclusion and consistent access to the resource.
 * Other tasks do not access the LCD directly, but instead send the text they
 * wish to display to the print task.  The print task spends most of its time
 * blocked - only waking when a message is queued for display.
 *
 * + A 'Button handler' task.  The eval board contains a user push button that
 * is configured to generate interrupts.  The interrupt handler uses a
 * semaphore to wake the button handler task - demonstrating how the priority
 * mechanism can be used to defer interrupt processing to the task level.  The
 * button handler task sends a message both to the LCD (via the print task) and
 * the UART where it can be viewed using a dumb terminal (via the UART to USB
 * converter on the eval board).  NOTES:  The dumb terminal must be closed in
 * order to reflash the microcontroller.  A very basic interrupt driven UART
 * driver is used that does not use the FIFO.  19200 baud is used.
 *
 * + A 'check' task.  The check task only executes every five seconds but has a
 * high priority so is guaranteed to get processor time.  Its function is to
 * check that all the other tasks are still operational and that no errors have
 * been detected at any time.  If no errors have every been detected 'PASS' is
 * written to the display (via the print task) - if an error has ever been
 * detected the message is changed to 'FAIL'.  The position of the message is
 * changed for each write.
 */



/* Environment includes. */
#include "DriverLib.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Demo app includes. */
#include "integer.h"
#include "PollQ.h"
#include "semtest.h"
#include "BlockQ.h"
#include "string.h"
#include <stdlib.h>

/* Delay between cycles of the 'check' task. */
#define mainCHECK_DELAY						( ( TickType_t ) 100 / portTICK_PERIOD_MS )
#define printSTATS_DELAY					( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* UART configuration - note this does not use the FIFO so is not very
efficient. */
#define mainBAUD_RATE				( 19200 )
#define mainFIFO_SET				( 0x10 )

/* Demo task priorities. */
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )

/* Demo board specifics. */
#define mainPUSH_BUTTON             GPIO_PIN_4

/* Misc. */
#define mainQUEUE_SIZE				( 3 )
#define mainDEBOUNCE_DELAY			( ( TickType_t ) 150 / portTICK_PERIOD_MS )
#define mainNO_DELAY				( ( TickType_t ) 0 )

#define MAX_BUFFER_SIZE 			20
#define BUFFER_PRINT_SIZE			20

#define DISPLAY_WIDTH				96

/*
 * Configure the processor and peripherals for this demo.
 */
static void prvSetupHardware( void );

/*
 * The 'check' task, as described at the top of this file.
 */
static void vCheckTask( void *pvParameters );

/*
 * The task that is woken by the ISR that processes GPIO interrupts originating
 * from the push button.
 */
static void vButtonHandlerTask( void *pvParameters );

/*
 * The task that controls access to the LCD.
 */
static void vPrintTask( void *pvParameter );

/*
 * The task that generates the temperature values at a 10Hz frequency 
 */
static void vSensorTask( void *pvParameter );

/*
 * Filter of the last N sensor measurements
 */
static void vFilterTask( void *pvParameter );

/*
 *	Reads from the UART port for new N values
 */
void vUARTTask(void *pvParameters);
	
/* String that is transmitted on the UART. */
static char *cMessage = "Task woken by button interrupt! --- ";
static volatile char *pcNextChar;


/*
 * Creates a top-like visualization of system usage by the tasks
 */
static void vTaskGetRunTimeStatsTask(  void *pvParameters );

/* The semaphore used to wake the button handler task from within the GPIO
interrupt handler. */
SemaphoreHandle_t xButtonSemaphore;

/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xPrintQueue;

/* Queue to communicate the last value sensed */
QueueHandle_t xSensorValueQueue;

/* Queue to send the new N values */
QueueHandle_t xNValueQueue;

/*-----------------------------------------------------------*/

int main( void )
{
	/* Configure the clocks, UART and GPIO. */
	prvSetupHardware();

	/* Create the queue used to pass message to vPrintTask. */
	xPrintQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( int ) );

	xSensorValueQueue = xQueueCreate( mainQUEUE_SIZE, sizeof(int));

	xNValueQueue = xQueueCreate( mainQUEUE_SIZE, sizeof(int));


	/* Start the tasks defined within the file. */
	//xTaskCreate( vCheckTask, "Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
	xTaskCreate( vSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL);
	xTaskCreate( vFilterTask, "Filter", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1 , NULL);
	xTaskCreate( vPrintTask, "Print", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
	xTaskCreate( vUARTTask, "UART", configMINIMAL_STACK_SIZE , NULL, mainCHECK_TASK_PRIORITY - 3, NULL );
	xTaskCreate( vTaskGetRunTimeStatsTask, "Stats", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 2 , NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient heap to start the
	scheduler. */

	return 0;
}
/*-----------------------------------------------------------*/

static void vCheckTask( void *pvParameters )
{
portBASE_TYPE xErrorOccurred = pdFALSE;
TickType_t xLastExecutionTime;
const char *pcPassMessage = "PASS";
const char *pcFailMessage = "FAIL";

	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Perform this check every mainCHECK_DELAY milliseconds. */
		vTaskDelayUntil( &xLastExecutionTime, mainCHECK_DELAY );

		/* Has an error been found in any task? */

		if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
		}

		if( xArePollingQueuesStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
		}

		if( xAreSemaphoreTasksStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
		}

		if( xAreBlockingQueuesStillRunning() != pdTRUE )
		{
			xErrorOccurred = pdTRUE;
		}

		/* Send either a pass or fail message.  If an error is found it is
		never cleared again.  We do not write directly to the LCD, but instead
		queue a message for display by the print task. */
		if( xErrorOccurred == pdTRUE )
		{
			xQueueSend( xPrintQueue, &pcFailMessage, portMAX_DELAY );
		}
		else
		{
			xQueueSend( xPrintQueue, &pcPassMessage, portMAX_DELAY );
		}
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
		/* Setup the PLL. */
	SysCtlClockSet( SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ );


	/* Enable the UART.  */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	/* Configure the UART for 8-N-1 operation. */
	UARTConfigSet( UART0_BASE, mainBAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE );

	/* We don't want to use the fifo.  This is for test purposes to generate
	as many interrupts as possible. */
	HWREG( UART0_BASE + UART_O_LCR_H ) &= ~mainFIFO_SET;

	/* Enable Tx interrupts. */
	//HWREG( UART0_BASE + UART_O_IM ) |= UART_INT_RX;
	//IntPrioritySet( INT_UART0, configKERNEL_INTERRUPT_PRIORITY );
	//IntEnable( INT_UART0 );


	/* Initialise the LCD> */
    OSRAMInit( false );
    OSRAMStringDraw("Lab 4 - SOII", 16, 0);
	OSRAMStringDraw("LM3S811 ", 16, 1);
}

/*-----------------------------------------------------------*/

static void vSensorTask(void *pvParameter)
{
	//char* values[10] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
	int values[23] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
	int i = 0;
	int value = 0;
	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();

	for( ;; )
	{
		vTaskDelayUntil( &xLastExecutionTime, mainCHECK_DELAY );
		
		value = values[i];

		xQueueSend( xSensorValueQueue, &value, portMAX_DELAY ); // send the value to the filter
		if(i == 22){ i = 0;}
		else { i++; }

	}

}

/*-----------------------------------------------------------*/

static void vFilterTask( void *pvParameter )
{
	int sensorMessage; // received from the sensor
	int average; // sent to print
	int buffer[MAX_BUFFER_SIZE]; // buffer with last N temperatures
	int N = 15; 
	int receivedCommand;

	for( ;; )
	{
		if (xQueueReceive(xSensorValueQueue, &sensorMessage, portMAX_DELAY) == pdPASS)
        {

        	if (xQueueReceive(xNValueQueue, &receivedCommand, 0) == pdPASS)
        	{
            	if(receivedCommand == 1)
            	{
            		(N + 1 > MAX_BUFFER_SIZE - 1) ? N = MAX_BUFFER_SIZE : N++;
            	}
            	else if(receivedCommand == 0)
            	{
            		(N - 1 <= 0) ? N = 0 : N--;
            	}
        	}

        	average = 0;

        	for(int i = N-1;i > 0; i--)
        	{
       			buffer[i] = buffer[i-1];
       			average += buffer[i];
        	}

        	buffer[0] = sensorMessage;
        	average += sensorMessage;

        	average = average / N; //we take the average

            // Send the average to the print queue
            xQueueSend(xPrintQueue, &average, portMAX_DELAY);
        }	
    }
}

/*-----------------------------------------------------------*/

void vUARTTask(void *pvParameters)
{
	
    char rxChar;
    int newN;

    for (;;)
    {
        if(UARTCharsAvail(UART0_BASE))
        {
        	rxChar = UARTCharGet(UART0_BASE);

		    if (rxChar == '+')
		    {
		        newN = 1;
		        xQueueSend(xNValueQueue, &newN, portMAX_DELAY);
		    }
		    else if (rxChar == '-')
	        {
	            newN = 0;
		        xQueueSend(xNValueQueue, &newN, portMAX_DELAY);
		    }
		}

		vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/*-----------------------------------------------------------*/

void intToChar(unsigned char graph[2*DISPLAY_WIDTH],int buffer)
{
	for(int i = DISPLAY_WIDTH - 1; i > 0;i--)
	{
		graph[i] = graph[i-1];
		graph[i + DISPLAY_WIDTH] = graph[i + DISPLAY_WIDTH - 1];
	}

	graph[DISPLAY_WIDTH] = 0;
	graph[0] = 0;

	if(buffer < 8)
	{
		graph[DISPLAY_WIDTH] = (1 << (7 - buffer));
	}
	else
	{
		graph[0] = (1 << (15 - buffer));
	}
	
}

/*-----------------------------------------------------------*/

static void vPrintTask( void *pvParameters )
{
	int pcMessage;

	char *ms = "Got";
	static unsigned char graph[2 * DISPLAY_WIDTH] = {0};
	unsigned portBASE_TYPE uxLine = 0, uxRow = 0;

	for( ;; )
	{
		/* Wait for a message to arrive. */
		xQueueReceive( xPrintQueue, &pcMessage, portMAX_DELAY );

		intToChar(graph, pcMessage);

		/* Write the message to the LCD. */
		//uxRow++;
		//uxLine++;
		OSRAMClear();
		//OSRAMStringDraw( ms, uxLine & 0x3f, uxRow & 0x01);
		OSRAMImageDraw(graph, 0, 0, DISPLAY_WIDTH, 2);
	}
}

char* ultoa(unsigned long value, char* str, int base) {
    char* ptr = str;
    char* ptr1 = str;
    char tmp_char;
    unsigned long tmp_value;

    // Handle the zero case
    if (value == 0) {
        *ptr++ = '0';
        *ptr = '\0';
        return str;
    }

    while (value) {
        tmp_value = value;
        value /= base;
        *ptr++ = "0123456789ABCDEF"[tmp_value - value * base];
    }

    *ptr = '\0';

    // Reverse the string
    while (ptr1 < ptr) {
        tmp_char = *--ptr;
        *ptr = *ptr1;
        *ptr1++ = tmp_char;
    }

    return str;
}


void vPrintStats(TaskStatus_t *pxTaskStatusArray) {
    char pcWriteBuffer[512] = "";
    char tempStr[32];
    volatile UBaseType_t uxArraySize, x;
    unsigned long ulTotalRunTime, ulStatsAsPercentage;
    
    if (pxTaskStatusArray != NULL) {
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
        ulTotalRunTime /= 100UL;
        if (ulTotalRunTime > 0) {
            for (x = 0; x < uxArraySize; x++) {
                ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;
                
                strcat(pcWriteBuffer, pxTaskStatusArray[x].pcTaskName);
                strcat(pcWriteBuffer, "\t");

                ultoa(pxTaskStatusArray[x].ulRunTimeCounter, tempStr, 10);
                strcat(pcWriteBuffer, tempStr);
                strcat(pcWriteBuffer, "\t");

                ultoa(ulStatsAsPercentage, tempStr, 10);
                strcat(pcWriteBuffer, tempStr);
                strcat(pcWriteBuffer, "%\n");
            }
        }
        OSRAMClear();
        OSRAMStringDraw(pcWriteBuffer, 0, 0);
    }
}


/*-----------------------------------------------------------*/

static void vTaskGetRunTimeStatsTask(  void *pvParameters )
{

    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize;
    

   	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();

	uxArraySize = uxTaskGetNumberOfTasks();

	/* Allocate a TaskStatus_t structure for each task. An array could be
	    	  allocated statically at compile time. */
	pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

	if (pxTaskStatusArray == NULL) {
    	for (;;)
      		;
  		}


   	for(;;)
   	{
   		vTaskDelayUntil( &xLastExecutionTime, printSTATS_DELAY );
	   	/* Take a snapshot of the number of tasks in case it changes while this
	      function is executing. */
	   	
	   	vPrintStats( pxTaskStatusArray);
	}
}
