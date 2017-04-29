#include "uart.h"
#include "plib.h"

#define PB_FREQ 40000000UL

void initUART(void){
    PPSInput(3, U1RX, RPA2); // pin 9
    PPSOutput(1, RPA0, U1TX); // pin 2
    mPORTASetPinsDigitalIn(BIT_2);
    mPORTASetPinsDigitalOut(BIT_0);
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART1, PB_FREQ, BAUDRATE);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
}

uint8_t serial_get_byte(void){
// Blocking function that returns one byte received
    
    uint8_t rxchar;
    // wait for a character
    while(!UARTReceivedDataIsAvailable(UART1));
    
    // receive it
    rxchar = UARTGetDataByte(UART1);
    
#ifdef SERIAL_ECHO
    putcUART1(rxchar);
    if (rxchar == '\r') putcUART1('\n');
#endif
    
    return rxchar;
}

uint16_t serial_get_angle(void){
#define BUFMAX  3
    uint8_t rxchar;
    uint8_t rxbuf[BUFMAX+1];
    uint8_t idx;
    
    // Look for start of message
    do{
        rxchar = serial_get_byte();
    } while (rxchar != MSG_BEGIN);
    
    while (1){
        idx = 0;
        /* Wait for a "completed" message.
         * A message is completed if:
         *  - BUFMAX characters have been entered
         *  - MSG_END has been entered
         *  - MSG_BEGIN has been detected
         */
        do{
            rxchar = serial_get_byte();
            if (rxchar >= '0' && rxchar <= '9')
                rxbuf[idx++] = rxchar;
            if (idx == BUFMAX){
                putsUART1("\n\r");
                break;
            }
        } while (rxchar != MSG_END && rxchar != MSG_BEGIN);
        
        // If  a start of message was detected, restart reception
        if (rxchar == MSG_BEGIN){
            continue;
        }
        else{
            rxbuf[idx] = '\0';
            return atoi(rxbuf);
        }
    }
}
