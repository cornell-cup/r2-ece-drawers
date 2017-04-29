/* 
 * File:   config.h
 * Author: Shanee Lu
 *
 * Created on February 18, 2017, 4:54 PM
 */

#ifndef CONFIG_H
#define	CONFIG_H

//  Configuration Bit settings
//  System Clock = 40 MHz,  Peripherial Bus = 40 MHz
//  Internal Osc w/PLL FNOSC = FRCPLL
//  Input Divider    2x Divider FPLLIDIV
//  Multiplier      20x Multiplier FPLLMUL
//  Output divider   2x Divider FPLLODIV
//  peripherial bus divider FPBDIV = 1
//  WDT disabled
//  Disable external oscillator
//#pragma config FNOSC = FRCPLL, POSCMOD = OFF, FSOSCEN = OFF, OSCIOFNC = OFF
//#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPBDIV = DIV_1, FPLLODIV = DIV_2
//#pragma config FWDTEN = OFF, JTAGEN = OFF

#endif	/* CONFIG_H */

