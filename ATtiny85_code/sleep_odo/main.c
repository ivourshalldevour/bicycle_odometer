/*
    This code makes the ATtiny85 count falling edges casued by a reed switch
    connected to PB3 (pin 2). A falling edge triggers an interrrupt which makes
    the main program loop increase the wheel revolution count in SRAM.

    It will display the wheel count by flashing an LED (on pin 5) that many
    times when a button connected to PB4 (pin 3) is held.

    A rising edge on pin 6 (PCINT1) triggers the wheel revolution count stored
    in SRAM to be written permanently to EEPROM. 

    Uses default internal 8MHz osc with divide-by-8 prescaler to make 1MHz
    clock signal.

    Assumes reed switch is hardware debounced with a 1uF capacitor. Assumes
    pin6 is pulled-down by power supply unit digital output. Because it is a
    digital input from a digital output, no debouncing needed. If using a
    mechanical switch during testing, make sure to debounce pin6 with cap.
*/

#define F_CPU 1000000UL // used for util/delay.h
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

// Globals used as flags by interrupt service routines
uint8_t odo_interrupt = 0;      // when another wheel revolution happens
uint8_t disp_interrupt = 0;     // when someone wants to read the odometer count
uint8_t save_interrupt = 0;     // save odo_count to eeprom when pin 6 goes high
uint32_t temp_odo = 0;          // temporary count of wheel rotations stored in SRAM
#define ODO_ADDR 0 // address of odo_count variable

// Helper functions
void doubleDabble();



int main() {
    cli();    // disable interrupts during setup

    DDRB  = 0x00;   // Set all pins to be inputs.
    PORTB = 0xFF;   // Pull-up resistors enabled.

    // Set PB0 (pin 5) as output for LED
    PORTB = PORTB & 0b11111110; // disable pull-up and output LOW
    DDRB = DDRB | (1<<PINB0);   // direction to output
    // disable pull-up on pin6 PB1
    PORTB &= 0b11111101;

    // clear pin change interrupt flag
    GIFR = 1 << PCIF; // done by writing a 1 (yes weird).

    // Set PB4, PB3 & PB1 as pin-change interrupt capable.
    GIMSK = 1 << PCIE; // enable pin-change interrupts
    PCMSK = (1 << PCINT4) | (1 << PCINT3) | (1 << PCINT1); // Mask for only PB4, PB3 & PB1

    // set sleep mode to powerdown
    MCUCR = (MCUCR & ~((1<<SM1) | (1<<SM0))) | (1<<SM1);

    // Disable Timer1, USI and ADC
    PRR = (1<<PRTIM1) | (1<<PRUSI) | (1<< PRADC);

    sei();    // Enable interrupts in status register


    for(;;) {
        // Reed switch from rotating wheel was triggered so increase temp_odo
        if(odo_interrupt) {
            temp_odo++;
            odo_interrupt = 0;
        }
        
        
        // display odo_count stored in eeprom (only while button is held)
        if(disp_interrupt) {
            uint32_t odo_count;
            eeprom_read_block(&odo_count, ODO_ADDR, 4);  // get wheel revs
            uint8_t bcd[10]; // 32bit converts to 10 decimal digits
            doubleDabble(bcd, odo_count); // convert odo_count to BCD
            
            // now blink LED that many times as each BCD digit.
            uint8_t i = 0;
            uint8_t j = 0;
            /*for(i=0; i<10; i++) {
                uint8_t blink_times = bcd[i];
                if(blink_times == 0) {
                    PORTB = PORTB | (1<<PINB0); // on
                    _delay_ms(750);
                    PORTB = PORTB & ~(1<<PINB0);    // off
                    _delay_ms(250);
                }
                for(j=0;j<blink_times; j++) {
                    PORTB = PORTB | (1<<PINB0); // on
                    _delay_ms(250);
                    PORTB = PORTB & ~(1<<PINB0);    // off
                    _delay_ms(250);
                }
                _delay_ms(250); // delay between digits
            }*/
            while((i<10) && (bcd[i] == 0)) i++; // go to first non-zero digit
            if(i==10) { // all digits were zero. Still need to display 0.
                PORTB = PORTB | (1<<PINB0); // on
                _delay_ms(750);
                PORTB = PORTB & ~(1<<PINB0);    // off
            }
            while(i<10) {   // blink for all digits onwards
                uint8_t blink_times = bcd[i];
                if(blink_times == 0) {  // long blink for zero
                    PORTB = PORTB | (1<<PINB0); // on
                    _delay_ms(1000);
                    PORTB = PORTB & ~(1<<PINB0);    // off
                    _delay_ms(250);
                }
                for(j=0; j<blink_times; j++) {
                    PORTB = PORTB | (1<<PINB0); // on
                    _delay_ms(250);
                    PORTB = PORTB & ~(1<<PINB0);    // off
                    _delay_ms(250);
                }
                _delay_ms(500); //delay between digits
                i++;
            }

            /*for(uint32_t i=0; i<odo_count; i++) {   // blink LED that many times
                if((PINB & (1<<PINB4)) != 0) break;  // Stop looping if button is released.
                PORTB = PORTB | (1<<PINB0); // set HIGH
                _delay_ms(250);
                PORTB = PORTB & ~(1<<PINB0);    // set LOW
                _delay_ms(250);
            }*/
            disp_interrupt = 0;
        }
        
        
        // save temp_odo to eeprom when pin 6 goes high
        if(save_interrupt) {
            if(temp_odo != 0) {    // only save if there are new counts
                uint32_t odo_count;
                eeprom_read_block(&odo_count, ODO_ADDR, 4); // read 4 bytes
                odo_count = odo_count + temp_odo;
                temp_odo = 0;   // clear the tally
                eeprom_write_block(&odo_count, ODO_ADDR, 4);    // write 4 bytes
            }
            save_interrupt = 0;
        }


        // go to sleep
        MCUCR |= (1<<SE);   // sleep enable
        __asm__ __volatile__ ("sleep"); // volatile so it doesnt get compiled away

        // CPU will resume here after being woken from sleep
        MCUCR &= ~(1<<SE);  // clear sleep enable
    }

    return 0;
}




// Interrupt service routine for a pin change on either PB3,PB4, or PB1.
ISR(PCINT0_vect) {
    // MCUCR &= ~(1<<SE);  // clear sleep enable

    // check which pin caused the interrupt
    if((PINB & (1<<PINB3)) == 0) {    // active low
        odo_interrupt = 1;
    }
    else if ((PINB & (1<<PINB1)) != 0) {    // active high
        save_interrupt = 1;
    }
    else if((PINB & (1<<PINB4)) == 0) {   // active low
        disp_interrupt = 1;
    }
    
    // keep in mind that this interrupt routine will be triggered for both
    // rising and falling edges of the signals on PB4, PB3 & PB1. It is
    // assumed the inputs are well debounced and sufficiently long to stay in
    // the same state during the interrupt service. If the pin is still LOW it
    // was a falling edge. If the pin is read HIGH, it was a rising edge.   `
}


/*
    Converts 32bit binary into 10 digit BCD. The BCD only takes up 5 bytes,
    because each byte stores two BCD digits. (1 BCD digit is 4bits).
*/
void doubleDabble(uint8_t bcd[10], uint32_t binary) {
    uint8_t scratch[5] = {0,0,0,0,0};   // scratch space
    uint8_t i,j;
    uint8_t carry;
    
    // 32 iterations
    for(i=0; i<32; i++) {
        // check all BCD digits if above 4
        for(j=0; j<5; j++) {    // each scratch[i] contains two BCD digits
            if((scratch[j] & 0xF0) > 0x40) {  // check high digit first
                scratch[j] += 0x30; // plus 3 to high digit
            }
            if((scratch[j] & 0x0F) > 0x04) { // now check low digit
                scratch[j] += 0x03; // plus 3 to low digit
            }
        }
        
        // left shift everything
        carry = (binary & (0b1UL<<31)) ? 1 : 0;   // carry=1 if MSB of binary is 1
        binary <<= 1;
        for(j=5; (j--)>0; ) {
            uint8_t new_carry = (scratch[j] & 0x80) ? 1 : 0;
            scratch[j] <<= 1;
            scratch[j] |= carry;
            carry = new_carry;
        }
    }
    
    // unpack the BCD digits from scratch space.
    bcd[0] = scratch[0] >> 4;       // upper nibble
    bcd[1] = scratch[0] & 0x0F;     // lower nibble
    bcd[2] = scratch[1] >> 4;
    bcd[3] = scratch[1] & 0x0F;     // etc...
    bcd[4] = scratch[2] >> 4;
    bcd[5] = scratch[2] & 0x0F;
    bcd[6] = scratch[3] >> 4;
    bcd[7] = scratch[3] & 0x0F;
    bcd[8] = scratch[4] >> 4;
    bcd[9] = scratch[4] & 0x0F;
}

