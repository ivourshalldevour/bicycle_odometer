#include "msp430fr2355.h"
#include <driverlib.h>

void main(void) {

    // Stop watchdog timer
    // WDTPW is Watchdog Timer PassWord. Needs to be written otherwise no other write attempts will work.
    WDTCTL = WDTPW | WDTHOLD;  // set WDTHOLD bit to 1

    // initialise all GPIO pins as output since they are unconnected
    P1DIR = 0xFF;   // all outputs
    P1OUT = 0xFF;   // all LOW
    P2DIR = 0xFF;
    P2OUT = 0xFF;
    P3DIR = 0xFF;
    P3OUT = 0xFF;
    P4DIR = 0xFF;
    P4OUT = 0xFF;
    P5DIR = 0xFF;
    P5OUT = 0xFF;
    P6DIR = 0xFF;
    P6OUT = 0xFF;
    // PxREN is don't care.


    // Setup P1.0 (red LED) as output
    P1DIR |= 0b1;   // P1.0 is output.
    P1REN = 0b0;     // pullup/pulldown resistors disabled.
    P1OUT = 0b0;     // set pin P1.0 to LOW.

    // Setup P6.6 (green LED) as output
    P6DIR |= 0b01000000;    // P6.6 is output
    P6REN = 0b0;            // pullup/downs disabled.
    P6OUT &= 0b10111111;    // set P6.6 to LOW

    // setup P2.3 (S2 button)
    P2DIR &= 0b11110111; // set P2.3 as input
    P2REN |= 0b00001000; // P2.3 has enabled pullup/pulldown resistor.
    P2OUT |= 0b00001000; // make P2.3 have a pullup.

    // setup P4.1 (S1 button)
    P4DIR &= 0b11111101;    // set P4.1 as input
    P4REN |= 0b00000010;    // enabled pullup/down
    P4OUT |= 0b00000010;    // make P4.1 have a pullup.

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;    // clear LOCKLPM5 bit to unlock GPIO pins from high impedance

    while(1) {
        // read input on pin P2.3
        if((P2IN & 0b00001000) == 0) {  // pressed P2.3!
            P1OUT |= 0b1; // output HIGH on P1.0
        }
        else {
            P1OUT &= 0b11111110; // else LOW
        }

        // read input on pin P4.1
        if((P4IN & 0b00000010) == 0) {
            P6OUT |= 0b01000000; // output HIGH on P6.6!
        }
        else {
            P6OUT &= 0b10111111;    // else LOW
        }
    }
}
