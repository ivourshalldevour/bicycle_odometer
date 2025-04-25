#include <EEPROM.h>
#include <Arduino.h>

/*
    This code makes the ATtiny85 count rising edges casued by a reed switch
    connected to PB3 (pin 2). A rising edge triggers an interrrupt which makes
    the main program loop increase the wheel revolution count in EEPROM.

    It will display the wheel count by flashing an LED that many times when a
    button connected to PB4 (pin 3) is pressed.

    Uses default internal 8MHz osc with divide-by-8 prescaler to make 1MHz
    clock signal.
*/

// Globals used as flags by interrupt service routines
byte odo_interrupt = 0;     // when another wheel revolution happens
byte disp_interrupt = 0;    // when someone wants to read the odometer count
#define ODO_ADDR 0 // address of odo_count variable


void setup() {
    cli();    // disable interrupts during setup

    DDRB  = 0x00;   // Set all pins to be inputs.
    PORTB = 0xFF;   // Pull-up resistors enabled.

    // Set PB0 (pin 5) as output for LED
    PORTB = PORTB & 0b11111110; // disable pull-up and output LOW
    DDRB = DDRB | (1<<PINB0);   // direction to output

    // Set PB4 and PB3 as pin-change interrupt capable.
    GIMSK = 1 << PCIE; // enable pin-change interrupts
    PCMSK = (1 << PCINT4) | (1 << PCINT3); // Mask for only PB3 & PB4

    sei();    // Enable interrupts in status register

    uint32_t odo_count = 0;
    EEPROM.put(ODO_ADDR, odo_count);
}

void loop() {
    uint32_t odo_count = 0;

    if(odo_interrupt) {
        odo_interrupt = 0;
        // Reed switch from rotating wheel was triggered so increase odo_count
        EEPROM.get(ODO_ADDR, odo_count);
        odo_count++;
        EEPROM.put(ODO_ADDR, odo_count);
    }

    if(disp_interrupt) {
        disp_interrupt = 0;
        EEPROM.get(ODO_ADDR, odo_count);   // get how many revolutions wheel has had
        for(uint32_t i=0; i<odo_count; i++) {   // blink LED that many times
            PORTB = PORTB | (1<<PINB0); // set HIGH
            delay(500);
            PORTB = PORTB & ~(1<<PINB0);    // set LOW
            delay(500);
        }
    }
}

// Interrupt service routine for a pin change on either PB3 or PB4.
ISR(PCINT0_vect) {
    // clear pin change interrupt flag
    GIFR = 1 << PCIF; // done by writing a 1 (yes weird).

    // check if PB4 or PB3 caused the interrupt
    if((PINB & (1<<PINB3)) == 0) {    // active low
        odo_interrupt = 1;
    }
    else if((PINB & (1<<PINB4)) == 0) {   // active low
        disp_interrupt = 1;
    }
    // keep in mind that this interrupt routine will be triggered for both
    // rising and falling edges of the signals on PB3 and PB4. It is assumed
    // the inputs are well debounced and sufficiently long to stay in the same
    // state during the interrupt service. If the pin is still LOW it was a
    // a falling edge. If the pin is read HIGH, it was a rising edge which we 
    // ignore (dont even have a if statement for that case).
}
