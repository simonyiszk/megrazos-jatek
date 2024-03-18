NAME=megrazos_jatek
FREQ=12000000UL

COMMON_COMPILE_FLAGS= -std=c99  -Wall -DF_CPU=$(FREQ)  -mmcu=atmega8 -O3 -I./lcd

.PHONY: clean lcd

all: main_target link lcd

main_target: $(NAME).c
	avr-gcc -c -Wall -DF_CPU=$(FREQ) $(COMMON_COMPILE_FLAGS)  -mmcu=atmega8 -g $(NAME).c -o $(NAME).o

lcd: lcd/lcd.c lcd/lcd.h
	avr-gcc -c -Wall -DF_CPU=$(FREQ) $(COMMON_COMPILE_FLAGS)  -mmcu=atmega8 -g lcd/lcd.c -o lcd.o
	
link: main_target lcd
	avr-gcc -g -mmcu=atmega8 -o $(NAME).elf $(NAME).o lcd.o
	avr-objcopy -j .text -j .data -O ihex  $(NAME).elf  $(NAME)_flash.hex

clean:
	rm -f $(NAME).o $(NAME)_flash.hex  $(NAME).elf

flash: link
	load -r $(NAME)_flash.hex
