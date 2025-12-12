/*
 * File:   Q_BOT.c
 * Author: Yang
 *
 * Created on November 7, 2025, 5:39 PM
 */

#include "xc.h"
#include <p24Fxxxx.h>
#include <libpic30.h> 
#include <stdlib.h>  // rand(), srand()
#include <time.h>    // time()
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)
// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
                                       // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL 
#include "FinalProject_header.h"



int main(void) {
    Basic_setup();
    PCA_init();
    Timer1_init();
    delay_ms(1000); 
    initAction();
    delay_ms(1000);
    
    while (1) { 
        if (PORTBbits.RB11 == 0) {
            speed();
        } 
        if (PORTBbits.RB15 == 0) {
            moveStraight(step_opt(1));
        }
        else if (PORTBbits.RB14 == 0) {
            moveBack(step_opt(1));
        }
        else if (PORTBbits.RB13 == 0) {   
            turnLeft(step_opt(1));
        }
        else if (PORTBbits.RB12 == 0) {
            turnRight(step_opt(1));
        }
        else {
            initAction();
        }
    }
    return 0;
}

