

#ifndef _USB_MAIN
#define _USB_MAIN

#include <stdint.h>

#define BUFFER_LENGTH   255

void ProcessIO(void);

uint16_t readIndex, writeIndex;
uint8_t readBuffer[BUFFER_LENGTH];



#endif