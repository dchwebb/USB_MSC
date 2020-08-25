#pragma once

#include "stm32f4xx.h"
#include <algorithm>
#include <sstream>

extern volatile uint32_t SysTickVal;

#define MIDIBUFFERSIZE 100

void SystemClock_Config(void);
void InitIO(void);
void InitSysTick();
void InitSampleAcquisition();
void InitCoverageTimer();
void InitDebounceTimer();
void InitEncoders();
void InitUART();
void uartSendChar(char c);
void uartSendString(const std::string& s);
void InitDAC();

