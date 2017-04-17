#include <plib.h>
#include <stdio.h> // need for printf() and sscanf()
#include "config.h"
#include "global.h"
#include "uart_init.h"
#include <peripheral/ports.h>
#include "protothreads.h"


//***************************************************
unsigned char rxchar = '0'; //receives a byte
unsigned char rfid_buffer[50] = {0}; //stores 10 ASCII RFID data
/********************************* 
	main entry point
*********************************/
enum states {
    WAIT_TRANSMIT, //wait for STX byte to be sent from RFID module
    STX_RECEIVED,  //get 10 data bytes, CR+LF, ETX byte. 
    ETX_RECEIVED,  //check if checksum is correct
    CHECKSUM_CORRECT //print 10 data bytes + checksum
};

enum states state = WAIT_TRANSMIT;

static PT_THREAD (protothread1(struct pt *pt))
{
    unsigned char checkSum[50] = {0}; //stores converted ASCII to hex from rfid_buffer
    unsigned char actualCS1 = '0';
    unsigned char actualCS2 = '0';
    PT_BEGIN(pt);
    
    int i = 0; 
    while (1)
    {
        switch(state)
        {
            case WAIT_TRANSMIT:
                
                while(!UARTReceivedDataIsAvailable(UART1)){}; //await transmission
                rxchar = UARTGetDataByte(UART1);
                if (rxchar == 2)
                    state = STX_RECEIVED;
                break;
            case STX_RECEIVED:
                for (i = 0; i < 10; i++) //get next 10 data bytes
                {
                   while(!UARTReceivedDataIsAvailable(UART1)){};
                   rfid_buffer[i] = UARTGetDataByte(UART1); 
                }
                while(!UARTReceivedDataIsAvailable(UART1)){};
                actualCS1 = UARTGetDataByte(UART1); //receive first checksum byte
                while(!UARTReceivedDataIsAvailable(UART1)){};
                actualCS2 = UARTGetDataByte(UART1);//receive second checksum byte
            
                for (i = 0; i < 2; i++) //receives CR + LF
                {
                   while(!UARTReceivedDataIsAvailable(UART1)){};
                   UARTGetDataByte(UART1);
                }
                while(!UARTReceivedDataIsAvailable(UART1)){};
                rxchar = UARTGetDataByte(UART1); //receives ETX character, stores in rxchar              
                if (rxchar == 3)
                    state = ETX_RECEIVED;
                else
                {
                    state = WAIT_TRANSMIT;
                    for (i = 0; i < 10; i++)
                        rfid_buffer[i] = 0;
                }
                break;
            case ETX_RECEIVED:
                for (i = 0; i < 10; i++) //converts ascii from rfid_buffer to hex into checkSum
                {
                    char x = rfid_buffer[i];
                    if (x < 58 && x > 47)
                        x = x - 48;
                    else 
                        x = x - 55;
                    checkSum[i] = x;
                }
                char CS1 = checkSum[0] ^ checkSum[2] ^ 
                checkSum[4] ^ checkSum[6] ^ checkSum[8]; //calculates 1st checksum byte
                char CS2 = checkSum[1] ^ checkSum[3] ^ 
                checkSum[5] ^ checkSum[7] ^ checkSum[9];//calculates 2nd checksum byte
           
                if (CS1 < 10)
                    CS1 = CS1 + 48;
                else 
                    CS1 = CS1 + 55;
                if (CS2 < 10)
                    CS2 = CS2 + 48;
                else 
                    CS2 = CS2 + 55;
                
                if ((CS1 == actualCS1) && (CS2 == actualCS2)) //if checksum is correct and ETX received
                    state = CHECKSUM_CORRECT;
                else
                {
                    state = WAIT_TRANSMIT;
                    for (i = 0; i < 10; i++)
                        rfid_buffer[i] = 0;
                }
                break;
            case CHECKSUM_CORRECT:
                //rfid_buffer contains the correct 10 ascii bytes
                //send rfid_buffer to CS
                for (i = 0; i < 10; i++)
                        rfid_buffer[i] = 0;
                state = WAIT_TRANSMIT;
                break;      
        }  
    }//end while
    PT_END(pt);
}

int main ( void )
{   
    SYSTEMConfigPerformance(SYS_CLK);
    init_uart(); //initializes uart1 & uart2
    home();
    
}






