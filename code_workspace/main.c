#include "msp430fr2355.h"
#include <driverlib.h>

int main(void) {

    volatile uint32_t i;

    // Stop watchdog timer
    WDTCTL = WDTCTL | WDTHOLD;  // set WDTHOLD bit to 1

    // Setup P1.0 as output
    P1DIR = 0b1;     // only pin P1.0 is an output.
    P1REN = 0b0;     // pullup/pulldown resistors disabled.
    P1OUT = 00;     // set pin P1.0 to LOW.

    // setup P2.3 as input
    P2DIR = 0b0;
    P2REN = 0b00001000; // P2.3 has enabled pullup/pulldown resistor.
    P2OUT = 0b0; // all inputs, but P2.3 has pulldown.


    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;    // clear LOCKLPM5 bit to unlock GPIO pins from high impedance

    while(1)
    {
        // read input on pin P2.3
        if((P2IN & 0b00001000) == 1) {  // pressed P2.3!
            P1OUT = 0b1; // output HIGH on P1.0
        }
        else {
            P1OUT = 00; // else LOW
        }
    }
}
