/*
    Code to test if the ATtiny85 works as a slave properly.
    For now will only send 1 byte back when the master requests a byte.
*/

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 1000000UL // used for util/delay.h
#define SLAVE_ADDRESS 69

uint8_t i2c_start_flag = 0;
uint8_t i2c_rx_flag = 0;    // Raised when 4 bit counter overflows indicating
                            // an entire byte was received.

int main() {
    cli();    // disable interrupts during setup

    DDRB  = 0x00;   // Set all pins to be inputs.
    PORTB = 0xFF;   // Pull-up resistors enabled.

    DDRB |= ((1<<PB2) | (1<<PB0));  // Set SCL & SDA to output mode.
                                    // Outputting high already.
    /* there are 4 registers for the i2c USI
    USIDR   data to send/receive
    USIBR   copy of USIDR that only changes when USIDR has been fully shifted out
    USISR   status reg (contains flags and 4bit counter)
    USICR   control reg (interrupt enable bits and settings)
    */

    // set bits in USICR (control register)
    USICR = 
        // enable start condition interrupt
        (1<<USISIE) |
        // enable 4bit counter overflow interrupt
        (1<<USIOIE) |
        // choosing normal i2c mode
        (1<<USIWM1) | (0<<USIWM0) |
        // source SCL externally on positive edges
        // source 4bit counter clock externally.
        (1<<USICS1) | (0<<USICS0) | (0<<USICLK);

    sei();    // Enable interrupts in status register


    for(;;) {
        //_delay_us(250);

    }

    return 0;
}




// Interrupt service routine for a start condition on i2c port.
ISR(USI_START_vect) {
    i2c_start_flag = 1;
    USISR &= 0b11110000;    // reset 4bit counter
    USISR |= (1<<USISIF);   // clear start flag (by writing 1)
}

ISR(USI_OVF_vect) {
    i2c_rx_flag = 1;    // byte received, get ready to ack/nack

    //clear overflow flag by writing a one.
    USISR |= (1<<USIOIF);
}
