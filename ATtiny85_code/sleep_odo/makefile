PORT=/dev/ttyUSB0
MCU=attiny85
CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) -Os
LDFLAGS=-Wl,-gc-sections -Wl,-relax
CC=avr-gcc
TARGET=main
OBJECT_FILES=main.o

all: $(TARGET).hex

clean:
	rm -f *.o *.hex *.obj *.hex

%.hex: %.obj
	avr-objcopy -R .eeprom -O ihex $< $@

%.obj: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

fuse:
	# Write Low fuse (internal 8MHz osc, start-up delay 14ck)
	avrdude -p $(MCU) -c stk500v1 -P $(PORT) -b 19200 -U lfuse:w:0x42:m
	
	# Write High fuse (BOD disabled, preserve EEPROM, SPI program download enabled)
	avrdude -p $(MCU) -c stk500v1 -P $(PORT) -b 19200 -U hfuse:w:0xd7:m

read_mem:
	avrdude -p $(MCU) -c stk500v1 -P $(PORT) -b 19200 -U eeprom:r:eeprom.txt:h -U lfuse:r:lfuse.txt:h -U hfuse:r:hfuse.txt:h

program: $(TARGET).hex
	@echo "\e[1;32mIf program failed. Read makefile comment about 10uF cap.\n\e[0m"
	avrdude -p $(MCU) -c stk500v1 -P $(PORT) -b 19200 -U flash:w:$(TARGET).hex

# This avrdude command is quite temperamental in linux when using an arduino
# as ISP. If using the "-c arduino" option you must use the 10uF cap between
# RST and GND on the programmer. This is because the "-c arduino" option
# includes an extra reset pulse which if not absorbed by the cap, will prevent
# the arduino programmer from responding. The "-c stk500v1" option is exactly
# the same but doesn't have this extra reset pulse. In this case, do NOT use
# the 10uF capacitor.

# Just dont even bother using -c arduinoisp (not working)

# For some reason windows does not have this issue. 
