#include <Wire.h>

#define ADDRESS 69  // address of slave.

/*
    Arduino reads a string from serial port and sends it char by char over
    I2C as a master. We expect the slave to echo the exact same string
    back to us. The received string is the re-transmitted back through the
    serial port.

    Basically an echo sketch except it converts from usb to serial to i2c
    and then all the way back. (Technically the usb-serial conversion
    is done by the CH340 chip external to the Arduino.)
*/
void setup() {
    Serial.begin(9600);
    Wire.begin();
    Serial.println("Input a string to send via I2C to the slave.");
}

void loop() {
    char input_str[30] = "";
    byte string_received = 0;   // boolean

    for(byte i=0; Serial.available(); i++) {
        char input_char = (char)Serial.read();
        if(input_char == '\n') {   // newline detected, must be end of string.
            string_received = 1;
            break;  // start doing an action with this string
        } 
        else {
            input_str[i] = input_char;
        }
    }

    if(string_received == 1) {
        Serial.println("Sending inputted string via I2C...");
        char output_str[30] = "";

        Wire.beginTransmission(ADDRESS);     // Transmit to the slave
        Wire.write(input_str);
        Wire.endTransmission(false);    // does a restart not a stop condition
        Wire.requestFrom(ADDRESS, 30);  // request 30 bytes from slave.
        for(byte i=0; i<30; i++) {
            output_str[i] = Wire.read();
        }
        Wire.endTransmission();     // stop condition on I2C

        Serial.println("String received back from I2C slave: ");
        Serial.println(output_str);

        for(byte i=0; i<30; i++) {input_str[i] = 0;}    // clear input string
        string_received = 0;    // clear the flag for next time
    }
}
