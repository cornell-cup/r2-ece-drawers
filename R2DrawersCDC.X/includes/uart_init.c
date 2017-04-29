
#include "uart_init.h"
#include <plib.h>
#include "global.h"

void init_uart()
{
     __XC_UART = 2;
    PPSInput (2, U2RX, RPB8); //Assign U2RX to pin RPB8 -- Physical pin 17 on 28 PDIP
    PPSOutput(4, RPB9, U2TX); //Assign U2TX to pin RPB9 -- Physical pin 18 on 28 PDIP
    //PPSInput (3, U1RX, RPB13); //Assign U1RX to pin RPB13 -- Physical pin 24 on 28 PDIP
    
    ANSELA =0; //make sure analog is cleared
    ANSELB =0; 
    // init the uart2
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, PB_FREQ, BAUDRATE);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    
    //init uart1 to RFID
    //UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
    //UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    //UARTSetDataRate(UART1, PB_FREQ, BAUDRATE);
    //UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
}
