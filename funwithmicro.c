

//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "lab7pinmux.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

//*****************************************************************************

//global variable count
volatile long count = 0;


void
PortFunctionInit(void)
{
    //
    // Enable Peripheral Clocks 
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable pin PF0 for GPIOInput
    //

    //
    //First open the lock and select the bits we want to modify in the GPIO commit register.
    //
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;

    //
    //Now modify the configuration of the pins that we unlocked.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);

    //
    // Enable pin PF3 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Enable pin PF4 for GPIOInput
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

    //
    // Enable pin PF1 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    //
    // Enable pin PF2 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
		
		//Enable pull up on switches
		GPIO_PORTF_PUR_R |= 0x11;
}

void Interrupt_Init(void)
{
IntEnable(INT_GPIOF);
//Set Priority GPIOF as 2
IntPrioritySet(INT_GPIOF, 0x02);
	//Arm interrupt on PF0 and PF4
	GPIO_PORTF_IM_R|= 0x11;
	//Edge trigger mode where IS 1 is rise edge and IS 0 is level edge trigger
	GPIO_PORTF_IS_R |= 0x11;
	//IBE is interrupt event both edges trigger, don't want
	GPIO_PORTF_IBE_R &= ~0x11;
	//Want some falling action
	GPIO_PORTF_IEV_R &= ~0x11;
	//Master Enable 
	IntMasterEnable();
}

void Timer0A_Init(unsigned long period)
{
	//enable peripheral clocks
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	//configure for 32 bit timer mode
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	//reload value
	TimerLoadSet(TIMER0_BASE, TIMER_A, period -1);
	//configure timer0A priority as 0
	IntPrioritySet(INT_TIMER0A, 0x00);
	//enable interrupt 19 (timer0A) in NVIC
	IntEnable(INT_TIMER0A);
	//arm timeout interrupt
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	//enable timer0A
	TimerEnable(TIMER0_BASE, TIMER_A);
	
}

void GPIOPortF_Handler(void)
{
	//switch debounce, disable and enable interrupt after 10 ms
	IntDisable(INT_GPIOF);
	SysCtlDelay(1066667);
	IntEnable(INT_GPIOF);
	
	//poll the flag for sw1 PF4
	if(GPIO_PORTF_RIS_R &= 0x10)
	{
		//acknowledge the flag for PF4
		GPIO_PORTF_ICR_R |= 0x10;
		count++;
		GPIO_PORTF_DATA_R = 0x0C;
	}
	//poll the flag for sw2 PF0
	if(GPIO_PORTF_RIS_R &= 0x01)
	{
		//acknowledge the flag for PF0
		GPIO_PORTF_ICR_R |= 0x01;
		count--;
		GPIO_PORTF_DATA_R = 0x0A;
	}
}

void Timer0A_Handler(void)
{
	//acknowledge flag for timer0A timeout
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	count++;
}


int main(void)
{	
	//initialize ports
PortFunctionInit();
	
	//configure interrupt
	Interrupt_Init();
	
	//reload value, freq is 16MHz, so period needs to be 16M to make 1second
	unsigned long period = 16000000;

//initialize timer0A and configure interrupt
Timer0A_Init(period);
	
	while(1)
	{
			if(count == 0)
			{
				GPIO_PORTF_DATA_R = 0x02;
			}
			if(count == 1)
			{
				GPIO_PORTF_DATA_R = 0x04;
			}
			if(count == -1)
			{
				GPIO_PORTF_DATA_R = 0x08;
			}
			if(count == -2)
			{
				GPIO_PORTF_DATA_R = 0x06;
			}
			if(count > 1)
			{
				count = -2;
				GPIO_PORTF_DATA_R = 0x06;
			}
			if(count < -2)
			{
				count = 1;
				GPIO_PORTF_DATA_R = 0x04;
			}
		}
	}


