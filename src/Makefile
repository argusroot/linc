
DEVICE     = atmega328p
CLOCK      = 16000000
PROGRAMMER = stk500v1
PORT	   = /dev/ttyAM0
BAUD       = 19200
FILENAME   = linc
COMPILER    = avr-gcc -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

CFLAGS += -Winvalid-pch -Wall -Wno-long-long -pedantic -Os \
	-ggdb -gstabs -fdata-sections -ffunction-sections -fsigned-char
LDFLAGS += -Wl,-gc-sections -Wl,-u,vfprintf -lprintf_min -fwhole-program

all: clean build
	# upload

build:
	$(COMPILER) $(CFLAGS) -c *.cpp
	$(COMPILER) $(LDFLAGS) -o $(FILENAME).elf *.o
	avr-objcopy -j .text -j .data -O ihex $(FILENAME).elf $(FILENAME).hex
	avr-size --format=avr --mcu=$(DEVICE) $(FILENAME).elf

upload:
	avrdude -v -p $(DEVICE) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$(FILENAME).hex:i

clean:
	rm -rf *.o
	rm -rf *.elf
	rm -rf *.hex
