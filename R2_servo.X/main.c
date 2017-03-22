/*
 * File:   main.c
 * Author: Syed Tahmid Mahbub
 *
 * Created on April 28, 2016
 */


// PIC 32MX250F128B

#include "config.h"
#include "uart.h"
#include <stdint.h>
#include <plib.h>
#include <peripheral/ports.h>

#define PERIOD      50000   // 20 ms
#define SERVO_MIN   1500    // 600 us
#define SERVO_REST  3750    // 1500 us
#define SERVO_MAX   6000    // 2400 us

#define SetHeadDC       SetDCOC1PWM     // pin 7
#define SetDrawer1DC    SetDCOC2PWM     // pin 14
#define SetDrawer2DC    SetDCOC3PWM     // pin 25
#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
int speed = 3750;

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

void initPWM(void){
    PPSOutput(1, RPB3, OC1);        // pin 7
    PPSOutput(2, RPB5, OC2);        // pin 14
    PPSOutput(4, RPB14, OC3);       // pin 25
    
    mPORTBSetPinsDigitalOut(BIT_3 | BIT_5 | BIT_14 | BIT_2 | BIT_13);
    
    // pwm mode, fault pin disabled, timer 2 time base
    OC1CON = OCCON_ON | OCCON_OCM1 | OCCON_OCM2;
    OC2CON = OCCON_ON | OCCON_OCM1 | OCCON_OCM2;
    OC3CON = OCCON_ON | OCCON_OCM1 | OCCON_OCM2;
    
    // 16-bit timer 2, no interrupt, 1:16 prescale, PR2=50000 -> period = 20ms
    OpenTimer2(T2_32BIT_MODE_OFF | T2_INT_OFF | T2_PS_1_16 | T2_ON, PERIOD-1);    
}


void main(void) {
    EnablePullUpB(BIT_0);
    SYSTEMConfigPerformance(SYS_CLK);
    INTEnableSystemMultiVectoredInt();
    
    //configure external interrupt
    PPSInput(4, INT1, RPB0);
    //mINT1ClearIntFlag();
    ConfigINT1(EXT_INT_PRI_1 | RISING_EDGE_INT | EXT_INT_ENABLE);
    
//    OpenTimer2( T2_ON | T2_SOURCE_INT | T2_PS_1_256, 0xFFFF);
//	ConfigIntTimer2( T2_INT_ON | T2_INT_PRIOR_2);
	
    initUART();
    initPWM();
   
    //SetHeadDC(SERVO_REST);
    //SetDrawer1DC(SERVO_REST);
    //SetDrawer2DC(SERVO_REST);
    
    //putsUART1("\n\rServo test program for R2\n\r");
    mPORTASetPinsDigitalOut(BIT_0);
    mPORTAClearBits(BIT_0);
    while (1){
        uint16_t rxdat;
        uint32_t head_dc;
        rxdat = 50;
        if (rxdat > 180) rxdat = 180;
        head_dc = SERVO_MIN + 25*rxdat;
        SetHeadDC(speed);
    }
    
}
// Timer2 Interrupt Service Routine
//void __ISR(_TIMER_2_VECTOR, ipl2) handlesTimer2Ints(void){
//		// **make sure iplx matches the timer's interrupt priority level
//    speed = 3500;
//	mT2ClearIntFlag();
//	// Clears the interrupt flag so that the program returns to the main loop
//} // END Timer2 ISR

void __ISR(_EXTERNAL_1_VECTOR, ipl1) pinHandler(void)
{
    mPORTAToggleBits(BIT_0);
    speed = 3500;
    mINT1ClearIntFlag();
    //SetHeadDC(3750);
}