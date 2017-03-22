/*
 * File:   config.h
 * Author: Syed Tahmid Mahbub
 *
 */

#ifndef CONFIG_H
#define	CONFIG_H

#define _SUPPRESS_PLIB_WARNING
#include "plib.h"

#define SYS_CLK     40000000UL
#define PB_FREQ     40000000UL

#pragma config FNOSC = FRCPLL, POSCMOD = OFF
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20
#pragma config FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF, JTAGEN = OFF, FSOSCEN = OFF

#endif	/* CONFIG_H */

