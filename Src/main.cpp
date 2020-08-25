#include "initialisation.h"
#include "USB.h"
#include <string>
#include <sstream>
#include <iomanip>

USB usb;


volatile uint32_t SysTickVal;
bool dumped = false;

uint32_t debugClock = 0;
uint32_t debugClDiff = 0;

volatile uint8_t uartCmdPos = 0;
volatile char uartCmd[100];
volatile bool uartCmdRdy = false;

extern "C" {
#include "interrupts.h"
}


extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();							// Activates floating point coprocessor and resets clock
	SystemClock_Config();					// Configure the clock and PLL
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
	InitUART();
	InitSysTick();
	usb.InitUSB();

	// configure PC13 blue button
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR13_0;			// Set pin to nothing:  01 Pull-up; 10 Pull-down; 11 Reserved
	GPIOC->MODER &= ~GPIO_MODER_MODE13_Msk;

	// PB7 is LD2 Blue
	GPIOB->MODER |= GPIO_MODER_MODER7_0;			// Set to output

//	usb.cdcDataHandler = std::bind(usbSerialdata, std::placeholders::_1, std::placeholders::_2);

	while (1)
	{
//		midiHandler.gateTimer();
//		cfg.SaveConfig();		// Checks if configuration change is pending a save

		if (GPIOC->IDR & GPIO_IDR_ID13 && dumped == 0) {
			GPIOB->ODR |= GPIO_ODR_OD7;
			dumped = 1;
			usb.OutputDebug();
		} else {
			GPIOB->ODR &= ~GPIO_ODR_OD7;
			dumped = 0;
		}


		// Check if a UART command has been received
		if (uartCmdRdy) {
			std::stringstream ss;
			for (uint8_t c = 0; c < 100; ++c) {
				if (uartCmd[c] == 10) {
					if (ss.str().compare("x") == 0) {

					} else {

				}
					break;
				}
				else
					ss << uartCmd[c];
			}
			uartCmdRdy = false;
		}
	}
}

