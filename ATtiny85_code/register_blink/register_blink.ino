/*
    Blinks an LED on pin 4 of an ATtiny85 PDIP chip. On for 1 second,
    off for one second.
*/


void setup() {
    DDRB  = 0x00;   // Set all pins to be inputs.
    PORTB = 0x00;   // Pull-up resistors disabled.

    DDRB = DDRB | 0b00000001;   // PB0 is output
}


void loop() {
    PINB = 0b00000001;  // toggle PB0 output state
    delay(1000);
    PINB = 0b00000001;  // toggle PB0 output state
    delay(1000);
}
