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

#pragma config FNOSC = PRIPLL, POSCMOD = HS, FSOSCEN = OFF, OSCIOFNC = OFF
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF, JTAGEN = OFF, ICESEL = ICS_PGx3
#pragma config UPLLIDIV = DIV_2, UPLLEN = ON

/** I N C L U D E S **********************************************************/

#include "../includes/config.h"
#include "../includes/global.h"
#include "GenericTypeDefs.h"
#include "../includes/Compiler.h"
#include "../includes/usb/usb_config.h"
#include "../includes/usb/usb_device.h"
#include <stdlib.h>
#include <stdio.h>

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
char RFID[10] = {0};
char TOOLSTATUS;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void initPWM(void);
void setServoSpeed(int speed);
void initToolStatus(void);
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

int main(void)
{   
	PPSInput(4, INT1, RPB0);
	PPSInput(3, INT2, RPA4);
		
	ConfigINT1(EXT_INT_ENABLE | FALLING_EDGE_INT | EXT_INT_PRI_2);
	ConfigINT2(EXT_INT_ENABLE | FALLING_EDGE_INT | EXT_INT_PRI_2);
	
    //initialize PWM and Limit Switches, Tool Switches, and RFID
    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

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
        
//        char sourceBuffer[2] = {0};
//        char transactionBuffer[2] = {0};
//        char payloadBuffer[1] = {0};
//        char checksumBuffer[2] = {0};
//            
		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        //int result = ProcessIO(sourceBuffer, payloadBuffer, checksumBuffer, transactionBuffer);
        
        struct R2ProtocolPacket commandPacket;  //struct to store command received
        char USBOutBuffer[CDC_DATA_OUT_EP_SIZE] = {ProcessIO()};  
        
        int endData = R2ProtocolDecode(USBOutBuffer, CDC_DATA_OUT_EP_SIZE, &commandPacket);
        int result = 1 ? endData > 0 : 0;
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
 
            if (strncmp(commandPacket.data, CMD_OPEN, 5)==0){
                sprintf("%s", "received cmd to open");
                setServoSpeed(SERVO_OPEN);
            }
            else if (strncmp(commandPacket.data, CMD_CLOSE, 5)==0){
                setServoSpeed(SERVO_CLOSE);
            }
            else if (strncmp(commandPacket.data, CMD_TOOLS, 5)==0){
//                sprintf("got command for tools");
//                sprintf(readBuffer,
//                "S: %d%s\n\rT: %d%s\n\rP: %d%s\n\rK: %d%s\n\r",
//                    7, "DRAWER1", 2, "D1", 1, getToolStatus(), 2, "CD");
//                putsUSBUSART(readBuffer);
                
                //send data using R2Protocol.h
                uint32_t dataLength = 1;
                uint8_t data[1] = {TOOLSTATUS};

                struct R2ProtocolPacket dataPacket = {
                    "DRAWER1", "NUC", data, dataLength, "" 
                };

                uint8_t output[CDC_DATA_OUT_EP_SIZE];
                int len = R2ProtocolEncode(&dataPacket, output, CDC_DATA_OUT_EP_SIZE);

                if (len >= 0) {
                    putsUSBUSART(output);
                    CDCTxService();
                }
            }
            
//            if (RFID != 0){
////                sprintf(readBuffer,
////                "S: %d%s\n\rT: %d%s\n\rP: %d%s\n\rK: %d%s\n\r",
////                    7, "DRAWER1", 2, "D1", 10, RFID, 2, "CD");
////                putsUSBUSART(readBuffer);
//                
//                //send data using R2Protocol.h
//                uint32_t dataLength = 10;
//                uint8_t data[10] = {RFID};
//
//                struct R2ProtocolPacket dataPacket = {
//                    "DRAWER1", "NUC", data, dataLength, "" 
//                };
//
//                uint8_t output[256];
//                int len = R2ProtocolEncode(&dataPacket, output, 256);
//
//                if (len >= 0) {
//                    putsUSBUSART(output);
//                    CDCTxService();
//                };
//            }
        }
        
    }//end while
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
    TOOLSTATUS = toolStatus;
    return toolStatus;
}