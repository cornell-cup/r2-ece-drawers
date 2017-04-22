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
#include "../includes/R2Protocol.h"
#include "config.h"
#include "global.h"
#include "uart_init.h"
#include "protothreads.h"

#pragma config FNOSC = PRIPLL, POSCMOD = HS, FSOSCEN = OFF, OSCIOFNC = OFF
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF, JTAGEN = OFF, ICESEL = ICS_PGx3
#pragma config UPLLIDIV = DIV_2, UPLLEN = ON

/** I N C L U D E S **********************************************************/

#include "GenericTypeDefs.h"
#include "../includes/Compiler.h"
#include "../includes/usb/usb_config.h"
#include "../includes/usb/usb_device.h"

#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;

/** C O M M A N D S ********************************************************/
#define CMD_OPEN    "O"
#define CMD_CLOSE   "C"
#define CMD_TOOLS   "T"

#define PERIOD      50000   // 20 ms
#define SERVO_MIN   2000    // 1000 us
#define SERVO_REST  3750    // 1500 us
#define SERVO_MAX   5000    // 2000 us
#define SERVO_RUN_SPEED     100
#define SERVO_OPEN  (SERVO_REST + SERVO_RUN_SPEED)
#define SERVO_CLOSE (SERVO_REST - SERVO_RUN_SPEED)

/** V A R I A B L E S ********************************************************/
unsigned char rxchar = '0'; //receives a byte
unsigned char RFID[10] = {0};

enum states {
    WAIT_TRANSMIT, //wait for STX byte to be sent from RFID module
    STX_RECEIVED,  //get 10 data bytes, CR+LF, ETX byte. 
    ETX_RECEIVED,  //check if checksum is correct
    CHECKSUM_CORRECT //print 10 data bytes + checksum
};

enum states state = WAIT_TRANSMIT;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void initPWM(void);
void setServoSpeed(int speed);

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
    unsigned char checkSum[10] = {0}; //stores converted ASCII to hex from rfid_buffer
    unsigned char actualCS1 = '0';
    unsigned char actualCS2 = '0';
    PT_BEGIN(pt);
    
    int i = 0; 
    while(1)
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
                   RFID[i] = UARTGetDataByte(UART1); 
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
                        RFID[i] = 0;
                }
                break;
            case ETX_RECEIVED:
                for (i = 0; i < 10; i++) //converts ascii from rfid_buffer to hex into checkSum
                {
                    char x = RFID[i];
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
                        RFID[i] = 0;
                }
                break;
            case CHECKSUM_CORRECT:
                //rfid_buffer contains the correct 10 ascii bytes
                //send rfid_buffer to CS
                
                state = WAIT_TRANSMIT;
                break;      
        }  
    }//end while
    PT_END(pt);
}

static PT_THREAD (protothread2(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1)
    {	
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
        
        char sourceBuffer[2] = {0};
        char transactionBuffer[2] = {0};
        char payloadBuffer[1] = {0};
        char checksumBuffer[2] = {0};
            
		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        int result = ProcessIO(sourceBuffer, payloadBuffer, checksumBuffer, transactionBuffer);
        /* the buffers now contain relevant information;
         * they are updated if result == 1; otherwise, it's old info
         */
        
        if (result){
            // new data available
            
            //print out data obtained:
//            sprintf(readBuffer,
//                "S: %s\n\rT: %s\n\rP: %s\n\rK: %s\n\r",
//                    sourceBuffer, transactionBuffer, 
//                        payloadBuffer, checksumBuffer);
//            putsUSBUSART(readBuffer);
 
            if (strncmp(payloadBuffer, CMD_OPEN, 5)==0){
                setServoSpeed(SERVO_OPEN);
            }
            else if (strncmp(payloadBuffer, CMD_CLOSE, 5)==0){
                setServoSpeed(SERVO_CLOSE);
            }
            else if (strncmp(payloadBuffer, CMD_TOOLS, 5)==0){
                uint32_t dataLength = 1;
                uint8_t data[dataLength] = {getToolStatus()};
                
//                sprintf(readBuffer,
//                "S: %s\n\rT: %s\n\rP: %s\n\rK: %s\n\r",
//                    "DRAWER1", "",  getToolStatus(), "");
//                putsUSBUSART(readBuffer);

                struct R2ProtocolPacket dataPacket;
                dataPacket->source = "DRAWER1";
                dataPacket->destination = "PI";
                dataPacket->id = "";
                dataPacket->data_len = dataLength;
                dataPacket->data = data;

                uint8_t output[256];
                int len = R2ProtocolEncode(&dataPacket, output, 256);

                if (len >= 0) {
                    putsUSBUSART(output, len);
                    CDCTxService();
                }
            }
            else if (RFID != 0){
//                sprintf(readBuffer,
//                "S: %s\n\rT: %s\n\rP: %s\n\rK: %s\n\r",
//                    "DRAWER1", "", RFID, "");
//                putsUSBUSART(readBuffer);
                uint32_t dataLength = 10;
                uint8_t data[dataLength] = {RFID};

                struct R2ProtocolPacket dataPacket;
                dataPacket->source = "DRAWER1";
                dataPacket->destination = "PI";
                dataPacket->id = "";
                dataPacket->data_len = dataLength;
                dataPacket->data = data;

                uint8_t output[256];
                int len = R2ProtocolEncode(&dataPacket, output, 256);

                if (len >= 0) {
                    putsUSBUSART(output, len);
                    CDCTxService();
                };
                int i = 0;
                for (i = 0; i < 10; i++)
                        RFID[i] = 0;
            }
        }
        
    }//end while
    PT_END(pt);
}
int main(void)
{   
	PPSInput(4, INT1, RPB0);
	PPSInput(3, INT2, RPA4);
    INTEnableSystemMultiVectoredInt();
	ConfigINT1(EXT_INT_ENABLE | FALLING_EDGE_INT | EXT_INT_PRI_2);
	ConfigINT2(EXT_INT_ENABLE | FALLING_EDGE_INT | EXT_INT_PRI_2);
	
    //initialize PWM and Limit Switches, Tool Switches, and RFID
    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

    init_uart(); //initializes uart1 for rfid tx
    //home();
    
    //init threads
    PT_INIT(&pt1);
    PT_INIT(&pt2);
    
    //round robin scheduling of threads
    while(1)
    {
        PT_SCHEDULE(protothread1(&pt1));
        PT_SCHEDULE(protothread2(&pt2));
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
	EnablePullUpB(BIT_0); //front limit switch
    initToolStatus();
    initRFID();
	
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

void __ISR(_EXTERNAL_1_VECTOR, ipl1) StopOpen(void) { //INT1, RPB0, Front Switch (FR_SW)
	mINT1ClearIntFlag();
	setServoSpeed(SERVO_REST);
}

void __ISR(_EXTERNAL_2_VECTOR, ipl2) StopClose(void) { //INT2, RPA4, Back Switch (BK_SW)
	mINT2ClearIntFlag();
	setServoSpeed(SERVO_REST);
}

void initToolStatus(void) {
    //Initialize tool switches as digital inputs
    //SW1 = RB1, SW2 = RB2, SW3 = RB3, SW4 = RB7, SW5 = RB14, SW6 = RB15
    mPORTBSetPinsDigitalIn(BIT_1 | BIT_2 | BIT_3 | BIT_7 | BIT_14 | BIT_15);
}

uint8_t getToolStatus(void) {
    uint8_t start = 0;
    uint8_t sw1 =  (mPORTBReadBits(BIT_1)>0)*64;
    uint8_t sw2 =  (mPORTBReadBits(BIT_2)>0)*32;
    uint8_t sw3 =  (mPORTBReadBits(BIT_3)>0)*16;
    uint8_t sw4 =  (mPORTBReadBits(BIT_7)>0)*8;
    uint8_t sw5 =  (mPORTBReadBits(BIT_14)>0)*4;
    uint8_t sw6 =  (mPORTBReadBits(BIT_15)>0)*2;
    uint8_t end = 1;

    uint8_t toolStatus = start + sw1 + sw2 + sw3 + sw4 + sw5 + sw6 + end;
    return toolStatus;
}

void initRFID() {
    
}


