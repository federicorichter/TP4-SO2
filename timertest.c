/* Modified timer setup from LM3Sxxxx Eclipse demo application */

/* Scheduler includes. */
#include "timer.h"
#include "FreeRTOS.h"

/* Library includes. */
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_timer.h"
#include "hw_types.h"
#include "interrupt.h"
#include "sysctl.h"

/* The set frequency of the interrupt. */
#define timerINTERRUPT_FREQUENCY (20000UL)

/* The highest available interrupt priority. */
#define timerHIGHEST_PRIORITY (0)

/* Misc defines. */
#define timerMAX_32BIT_VALUE (0xffffffffUL)
#define timerTIMER_1_COUNT_VALUE (*((unsigned long *)(TIMER1_BASE + 0x48)))

/*-----------------------------------------------------------*/

/* Interrupt handler */
void Timer0IntHandler(void);

/* Counts the total number of times that the high frequency timer has 'ticked'.
This value is used by the run time stats function to work out what percentage
of CPU time each task is taking. */
volatile unsigned long ulHighFrequencyTimerTicks = 0UL;

/*-----------------------------------------------------------*/

void vSetupHighFrequencyTimer(void) {
  unsigned long ulFrequency;

  /* Timer zero is used to generate the interrupts. */
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);

  /* Set the timer interrupt to be above the kernel - highest. */
  IntPrioritySet(INT_TIMER0A, timerHIGHEST_PRIORITY);

  /* Just used to measure time. */
  TimerLoadSet(TIMER1_BASE, TIMER_A, timerMAX_32BIT_VALUE);

  /* Ensure interrupts do not start until the scheduler is running. */
  portDISABLE_INTERRUPTS();

  /* The rate at which the timer will interrupt. */
  ulFrequency = configCPU_CLOCK_HZ / timerINTERRUPT_FREQUENCY;
  TimerLoadSet(TIMER0_BASE, TIMER_A, ulFrequency);
  IntEnable(INT_TIMER0A);
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

  /* Enable timer. */
  TimerEnable(TIMER0_BASE, TIMER_A);
}
/*-----------------------------------------------------------*/

void Timer0IntHandler(void) {
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

  /* Keep a count of the total number of 20KHz ticks.  This is used by the
  run time stats functionality to calculate how much CPU time is used by
  each task. */
  ulHighFrequencyTimerTicks++;
}
