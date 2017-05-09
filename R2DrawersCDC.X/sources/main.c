/********************************************************************
 FileName:      main.c
 Dependencies:  See INCLUDES section
 Processor:     PIC18, PIC24, dsPIC, and PIC32 USB Microcontrollers
 Hardware:      This demo is natively intended to be used on Microchip USB demo
                boards supported by the MCHPFSUSB stack.  See release notes for
                support matrix.  This demo can be modified for use on other 
                hardware platforms.
 Complier:      Microchip C18 (for PIC18), XC16 (for PIC24/dsPIC), XC32 (for PIC32)
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Description
  ----  -----------------------------------------
  1.0   Initial release
  2.1   Updated for simplicity and to use common coding style
  2.8   Improvements to USBCBSendResume(), to make it easier to use.
        Added runtime check to avoid buffer overflow possibility if 
        the USB IN data rate is somehow slower than the UART RX rate.
  2.9b  Added support for optional hardware flow control.
  2.9f  Adding new part support   
  2.9j  Updates to support new bootloader features (ex: app version 
        fetching).
********************************************************************/

/** INCLUDES *******************************************************/
#include "../includes/usb/usb.h"
#include "../includes/usb/usb_function_cdc.h"
#include "../includes/HardwareProfile.h"


//#pragma config FNOSC = PRIPLL, POSCMOD = HS, FSOSCEN = OFF, OSCIOFNC = OFF
//#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPBDIV = DIV_1, FPLLODIV = DIV_2
//#pragma config FWDTEN = OFF, JTAGEN = OFF, ICESEL = ICS_PGx3
//#pragma config UPLLIDIV = DIV_2, UPLLEN = ON

/** I N C L U D E S **********************************************************/

#include "../includes/config.h"
#include "../includes/global.h"
#include "GenericTypeDefs.h"
#include "../includes/Compiler.h"
#include "../includes/usb/usb_config.h"
#include "../includes/usb/usb_device.h"
#include <stdlib.h>
#include <stdio.h>
#include "../includes/uart_init.h"
#include "../includes/uart_init.c"
#include "../includes/R2Protocol.h"
#include "../protothreads.h"



#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;

/** C O M M A N D S ********************************************************/
#define CMD_OPEN    "O"
#define CMD_CLOSE   "C"
#define CMD_TOOLS   "T"

#define PERIOD      50000   // 20 ms
#define SERVO_MIN   2000    // 1000 us
#define SERVO_REST  3685    // 1500 us
#define SERVO_MAX   5000    // 2000 us
#define SERVO_RUN_SPEED     30     //400 us
#define SERVO_OPEN  (SERVO_REST + SERVO_RUN_SPEED)
#define SERVO_CLOSE (SERVO_REST - SERVO_RUN_SPEED)

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

/** V A R I A B L E S ********************************************************/
unsigned char RFID[10] = {0}; //stores RFID transmissions
unsigned char rxchar = '0'; //receives a byte
int rfid_ready = 0; //tells drawer thread that RFID buffer is correct, send to R2bot over USB
char TOOLSTATUS;
extern char USB_Out_Buffer[CDC_DATA_OUT_EP_SIZE];
enum states1 {
    WAIT_TRANSMIT, //wait for STX byte to be sent from RFID module
    STX_RECEIVED,  //get 10 data bytes, CR+LF, ETX byte. 
    ETX_RECEIVED,  //check if checksum is correct
    CHECKSUM_CORRECT //print 10 data bytes + checksum
};

enum states1 rfid_state = WAIT_TRANSMIT;

//thread control struct
static struct pt pt1, pt2;
//sem
static struct pt_sem control_t1, control_t2;
/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void initPWM(void);
void setServoSpeed(int speed);
void initToolStatus(void);
void delay_ms(unsigned long i);
uint8_t getToolStatus(void);

/******************************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *****************************************************************************/

static PT_THREAD (protothread1(struct pt *pt))
{
    PT_BEGIN(pt);
    
    unsigned char checkSum[50] = {0}; //stores converted ASCII to hex from rfid_buffer
    unsigned char actualCS1 = '0';
    unsigned char actualCS2 = '0';
    
    int j; 
    while(1)
    {
        switch(rfid_state)
        {
            case WAIT_TRANSMIT:
                PT_YIELD_UNTIL(pt, UARTReceivedDataIsAvailable(UART1));
//                printf("got first byte\n");
                rxchar = UARTGetDataByte(UART1);
                if (rxchar == 2)
                    rfid_state = STX_RECEIVED;
            case STX_RECEIVED:
                for (j = 0; j < 10; j++) //get next 10 data bytes
                {                  
                   while(!UARTReceivedDataIsAvailable(UART1)){};
                   RFID[j] = UARTGetDataByte(UART1); 
//                   printf("getting byte: %c\n", RFID[j]);
                }
                while(!UARTReceivedDataIsAvailable(UART1)){};
                actualCS1 = UARTGetDataByte(UART1); //receive first checksum byte
                while(!UARTReceivedDataIsAvailable(UART1)){};
                actualCS2 = UARTGetDataByte(UART1);//receive second checksum byte
            
                for (j = 0; j < 2; j++) //receives CR + LF
                {
                   while(!UARTReceivedDataIsAvailable(UART1)){};
                   UARTGetDataByte(UART1);
                }
                while(!UARTReceivedDataIsAvailable(UART1)){};
                rxchar = UARTGetDataByte(UART1); //receives ETX character, stores in rxchar              
                if (rxchar == 3)
                    rfid_state = ETX_RECEIVED;
                else
                {
                    rfid_state = WAIT_TRANSMIT;
                    for (j = 0; j < 10; j++)
                    {
                        RFID[j] = 0;
                    }
                }
                break;
            case ETX_RECEIVED:
//                printf("Got ETX\n");
                for (j = 0; j < 10; j++) //converts ascii from rfid_buffer to hex into checkSum
                {
                    char x = RFID[j];
                    if (x < 58 && x > 47)
                        x = x - 48;
                    else 
                        x = x - 55;
                    checkSum[j] = x;
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
                    rfid_state = CHECKSUM_CORRECT;
                else
                {
                    rfid_state = WAIT_TRANSMIT;
                    for (j = 0; j < 10; j++)
                    {
                        RFID[j] = 0;
                    }
                }
                break;
            case CHECKSUM_CORRECT:
                printf("%s", RFID);
                printf("\n");
                //rfid_buffer contains the correct 10 ascii bytes
                rfid_ready = 1;
//                printf("\nRFID ready: %d\n", rfid_ready);
                rfid_state = WAIT_TRANSMIT;
                break;   
        }  
    }//endwhile  
    PT_END(pt);
}

static PT_THREAD (protothread2(struct pt *pt))
{   
    PT_BEGIN(pt);
    while(1)
    {
        //PT_SEM_WAIT(pt, &control_t2);
//        printf("2nd thread\n");
        PT_YIELD(pt);
        //printf("hi\n");
        struct R2ProtocolPacket commandPacket;  //struct to store command received
        uint8_t packetData[32] = {0};
        commandPacket.data_len = 32;
        commandPacket.data = packetData;
        int result = ProcessIO(&commandPacket);
        
        if (result == 1)
        {
            printf("Result: %d\n:", result);
            printf("%c\n", commandPacket.data[0]);
        }
        //continue;
        
        int ck = 0;
        if (result == 1)
        {
           // new data available
            //setServoSpeed(4000);
            if (commandPacket.data[0] == 'C'){
                if (ck % 1 == 0)
                {
                //sprintf("%s", "received cmd to open");
                    printf("Servo Close\n");
                    setServoSpeed(3605);
//                    for (i = 3685; i >= 3650; i--)
//                    {
//                        setServoSpeed(i);
//                        delay_ms(50);
//                    }
//                    ck++;
                }
            }
            else if (commandPacket.data[0] == 'O'){
                //sprintf("%s", "received cmd to close");
                if (ck % 1 == 0)
                {
                    printf("Servo Open\n");
                    setServoSpeed(3770);
//                    for (i = 3685; i <= 3720; i++)
//                    {
//                        setServoSpeed(i);
//                        delay_ms(50);
//                    }
                    
                    ck++;
                }
            }
        
            else if (commandPacket.data[0] == 'T')
            {
                //printf("got command for tools\n");
//                uint8_t data = 'H';
                uint8_t data = getToolStatus();
                int dataLength = 1; 
                struct R2ProtocolPacket dataPacket = {
                    "DRAWER1", "PI","", dataLength, &data, "" 
                };
                uint8_t output[31];
                int len = R2ProtocolEncode(&dataPacket, output, 31);
                printf("%s\n", output);
                printf("Got T\n");
                if (len >= 0) {
                    //printf("Packet about to be sent out!");
                    putUSBUSART((char *) output, len);
                    printf("Tools sent out: \n"BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(data));
                    CDCTxService();
                }
            }
        } //end if USB transmission receied
        
        if (rfid_ready == 1)
        {
            uint8_t *rfidData = RFID;
            int dataLength = 10;
            struct R2ProtocolPacket dataPacketRfid = {
                "RFID", "PI","", dataLength, rfidData, "" 
            };
            uint8_t output[37];
            int lenRfid = R2ProtocolEncode(&dataPacketRfid, output, 37);
            if (lenRfid >= 0) {
                putUSBUSART((char *) output, lenRfid);
                printf("RFID sent out!\n");
                CDCTxService();
            };
            rfid_ready = 0; //set back to zero, latest RFID scan sent

            int ind;
            //clear RFID buffer once it is sent
            for (ind = 0; ind < 10; ind++)
            {
               RFID[ind] = 0;
            }
        }       
    } //endwhile1
    PT_END(pt);
}//end thread2

int main(void)
{
    printf("Started\n");
    mPORTASetPinsDigitalOut(BIT_0);
    int i;
    for (i = 0; i < 12; i++)
    {
        mPORTAToggleBits(BIT_0);
        delay_ms(250);
        WDTCONbits.WDTCLR = 1; // feed the watchdog
    }
    for (i = 0; i < 12; i++)
    {
        mPORTAToggleBits(BIT_0);
        delay_ms(100);
        WDTCONbits.WDTCLR = 1; // feed the watchdog

    }
    mPORTAClearBits(BIT_0);
//    mINT1IntEnable(1);
//    mINT2IntEnable(1);
    SYSTEMConfigPerformance(SYS_FREQ);
    INTEnableSystemMultiVectoredInt();
	PPSInput(1, INT4, RPB15);
	PPSInput(3, INT2, RPA4);
	init_uart();
	ConfigINT4(EXT_INT_ENABLE | FALLING_EDGE_INT | EXT_INT_PRI_2);
	ConfigINT2(EXT_INT_ENABLE | FALLING_EDGE_INT | EXT_INT_PRI_2);
    
    
    //initialize PWM and Limit Switches, Tool Switches, and RFID
    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif
    #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // regularly (such as once every 1.8ms or faster** [see 
        				  // inline code comments in usb_device.c for explanation when
        				  // "or faster" applies])  In most cases, the USBDeviceTasks() 
        				  // function does not take very long to execute (ex: <100 
        				  // instruction cycles) before it returns.
    #endif

        OpenTimer1(T1_ON | T1_PS_1_256, 0xFFFF);
    //initialize protothreads
    PT_INIT(&pt1);
    PT_INIT(&pt2);
    
    //schedule threads, round robin
    while(1)
    {
        PT_SCHEDULE(protothread1(&pt1));
        WDTCONbits.WDTCLR = 1; // feed the watchdog
        PT_SCHEDULE(protothread2(&pt2));
        WDTCONbits.WDTCLR = 1; // feed the watchdog
    }
}//end main

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{

    ANSELA = 0; ANSELB = 0;
    SYSTEMConfigPerformance(40000000);

//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile - (platform).h, and that an appropriate I/O pin 
//  has been mapped	to it.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif
    
    UserInit();

    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
	
	initPWM();
	EnablePullUpA(BIT_4); //back limit switch
	EnablePullUpB(BIT_15); //front limit switch
    initToolStatus();
	
}//end InitializeSystem


/** EOF main.c *************************************************/

void initPWM(void){
    PPSOutput(1, RPB4, OC1);        // pin 11
    
    mPORTBSetPinsDigitalOut(BIT_4);
    
    // pwm mode, fault pin disabled, timer 2 time base
    OC1CON = OCCON_ON | OCCON_OCM1 | OCCON_OCM2;
    
    // 16-bit timer 2, no interrupt, 1:16 prescale, PR2=50000 -> period = 20ms
    OpenTimer2(T2_32BIT_MODE_OFF | T2_INT_OFF | T2_PS_1_16 | T2_ON, PERIOD-1);
	
	setServoSpeed(SERVO_REST);
}

void setServoSpeed(int speed){
    if (speed < SERVO_MIN) speed = SERVO_MIN;
    if (speed > SERVO_MAX) speed = SERVO_MAX;
    SetDCOC1PWM(speed);		//RPB4, OC1
}
//interrupts switched
void __ISR(_EXTERNAL_4_VECTOR, ipl2) StopOpen(void) { //INT1, RPB15, Front Switch (FR_SW)
	mINT4ClearIntFlag();
	setServoSpeed(SERVO_REST);
    printf("Motor close stopped!\n");
}

void __ISR(_EXTERNAL_2_VECTOR, ipl2) StopClose(void) { //INT2, RPA4, Back Switch (BK_SW)
	mINT2ClearIntFlag();
	setServoSpeed(SERVO_REST);
    printf("Motor open stopped!\n");
}

void initToolStatus(void) {
    //Initialize tool switches as digital inputs
    //SW1 = RB1, SW2 = RB2, SW3 = RB3, SW4 = RB7, SW5 = RB14, SW6 = RB15
    mPORTBSetPinsDigitalIn(BIT_1 | BIT_2 | BIT_3 | BIT_7 | BIT_14);
    EnablePullUpB(BIT_1 | BIT_2 | BIT_3 | BIT_7 | BIT_14 );
}

uint8_t getToolStatus(void) {
    uint8_t start = 0;
    uint8_t sw1 =  ((mPORTBReadBits(BIT_1)>0))*64;
    uint8_t sw2 =  ((mPORTBReadBits(BIT_2)>0))*32;
    uint8_t sw3 =  ((mPORTBReadBits(BIT_3)>0))*16;
    uint8_t sw4 =  ((mPORTBReadBits(BIT_7)>0))*8;
    uint8_t sw5 =  ((mPORTBReadBits(BIT_14)>0))*4;
    uint8_t sw6 =  ((mPORTBReadBits(BIT_15)>0))*2;
    uint8_t end = 1;

    uint8_t toolStatus = start + sw1 + sw2 + sw3 + sw4 + sw5 + sw6 + end;
    return toolStatus;
}

void delay_ms(unsigned long i)
{
	/* Create a software delay about i ms long
	 * Parameters:
	 *      i:  equal to number of milliseconds for delay
	 * Returns: Nothing
	 * Note: Uses Core Timer. Core Timer is cleared at the initialiazion of
	 *      this function. So, applications sensitive to the Core Timer are going
	 *      to be affected
	 */
	UINT32 j;
	j = 20000 * i;
	WriteCoreTimer(0);
	while (ReadCoreTimer() < j);
}