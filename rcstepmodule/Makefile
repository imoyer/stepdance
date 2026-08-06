# RC Stepper Module Makefile
# Mixing Metaphors Project
# Ilan E. Moyer
# February 16th, 2025

# ---- PROJECT NAME ----
PROJECT=rcstepmodule


# ---- UPLOADER LOCATION ----
UPLOADER = UPPLoader.py


# ---- INTERNAL VARIABLE DEFINITIONS ----
SOURCES=$(PROJECT).c
MMCU=attiny84
F_CPU = 8000000
LFUSE = 0xE2

CFLAGS=-mmcu=$(MMCU) -Wall -Os -DF_CPU=$(F_CPU)


# ---- MAKE RULES ----

all: clean $(PROJECT).hex clean

load: clean $(PROJECT).hex clean flash


$(PROJECT).o: $(SOURCES)
	avr-gcc $(CFLAGS) -c -Wall $(SOURCES)

$(PROJECT).elf:	$(PROJECT).o
	avr-gcc $(CFLAGS) -o $@ $^

$(PROJECT).hex:	$(PROJECT).elf
	avr-objcopy -O ihex $< $@
	avr-size --mcu=$(MMCU) --format=avr $(PROJECT).elf
 
flash: $(PROJECT).hex
	/usr/bin/python3 $(UPLOADER) $(PROJECT).hex $(LFUSE)
	rm $(PROJECT).hex

clean:
	rm -rf *.o *.elf
