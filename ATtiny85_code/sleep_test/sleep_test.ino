
// my personally handwritten code

#include <Arduino.h>

#define LED_PIN PB0      // Pin 5 on ATtiny85
#define WAKE_PIN PB4     // Pin 3 on ATtiny85

volatile bool wake_flag = false;

void setup() {
  // Set LED as output
  DDRB |= (1 << LED_PIN);
  PORTB &= ~(1 << LED_PIN);  // LED OFF

  // Set WAKE_PIN as input with pull-up
  DDRB &= ~(1 << WAKE_PIN);
  PORTB |= (1 << WAKE_PIN);

  // Enable pin change interrupt on PB4
  GIMSK |= (1 << PCIE);               // Enable pin change interrupt system
  PCMSK |= (1 << PCINT4);             // Enable pin change interrupt on PB4
  GIFR |= (1 << PCIF);                // Clear any pending PCINT

  // Set sleep mode to power-down
  MCUCR = (MCUCR & ~((1<<SM1) | (1<<SM0))) | (1<<SM1);   // Power-down mode
  MCUCR &= ~(1 << SE);                // Make sure sleep is off for now

  sei(); // Enable global interrupts
}

void loop() {
  if (wake_flag) {
    // Blink LED once to indicate wake
    PORTB |= (1 << LED_PIN);  // LED ON
    delay(1000);
    PORTB &= ~(1 << LED_PIN); // LED OFF
    delay(200);

    wake_flag = false;
  }

  // Sleep indicator: short blink before sleeping
  PORTB |= (1 << LED_PIN);
  delay(50);
  PORTB &= ~(1 << LED_PIN);
  delay(1000);

  // Enable sleep and go to sleep
  MCUCR |= (1 << SE);                 // Enable sleep
  __asm__ __volatile__("sleep");      // Go to sleep
  asm("NOP");
  MCUCR &= ~(1 << SE);                // Disable sleep after wake
}

// Interrupt Service Routine for pin change on PB4
ISR(PCINT0_vect) {
  wake_flag = true;
}

/*

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define LED_PIN     PB0
#define INPUT_PIN   PB1

void setup() {
    // Set LED pin as output
    DDRB |= (1 << LED_PIN);

    // Set input pin with pull-up
    DDRB &= ~(1 << INPUT_PIN);
    PORTB |= (1 << INPUT_PIN);
}

void flash_led(uint8_t flashes_per_sec, uint8_t duration_sec) {
    uint16_t total_flashes = flashes_per_sec * duration_sec;
    uint16_t delay_ms = 1000 / (flashes_per_sec * 2); // ON and OFF time
    for (uint16_t i = 0; i < total_flashes; i++) {
        PORTB ^= (1 << LED_PIN);
        _delay_ms(delay_ms);
    }
}

void enter_sleep() {
    // Hold LED ON
    PORTB |= (1 << LED_PIN);

    // Set sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    // Enable pin change interrupt on INPUT_PIN
    GIMSK |= (1 << PCIE);          // Enable pin change interrupt
    PCMSK |= (1 << INPUT_PIN);      // Enable for INPUT_PIN

    sei(); // Enable global interrupts

    sleep_cpu(); // Enter sleep mode

    // CPU will continue here after waking up
    sleep_disable();
    GIMSK &= ~(1 << PCIE);  // Disable pin change interrupt
}

ISR(PCINT0_vect) {
    // Interrupt service routine for pin change
    // Wakes up automatically
}

int main(void) {
    setup();

    while (1) {
        // Check if button is pressed
        if (!(PINB & (1 << INPUT_PIN))) {
            // Button pressed - before sleep
            PORTB &= ~(1 << LED_PIN);   // off
            _delay_ms(100);
            PORTB |= (1 << LED_PIN);    // on
            _delay_ms(100);
            PORTB &= ~(1 << LED_PIN);   // off
            _delay_ms(100);
            PORTB |= (1 << LED_PIN);    // on

            enter_sleep();

            // After wake up
            flash_led(5, 3);
        }

        // Main normal loop
        PORTB ^= (1 << LED_PIN);
        _delay_ms(1000);
    }
}
*/