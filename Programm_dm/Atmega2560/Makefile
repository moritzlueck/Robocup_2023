#Main application file name
MAIN_APP = main
#Main hex file path in windows format
MAIN_HEX_PATH = C:\Users\morit\Documents\Microcontroller\Robocup\Robocup_2023\Programm_dm\Atmega2560$(MAIN_APP).hex

# Compiler and other Section
CC = avr-gcc
OBJCOPY = avr-objcopy.exe
AVRDUDE := avrdude

#Options for avr-gcc
CFLAGS = -g -Os -o

#Linking options for avr-gcc
LFLAGS = -Os -mmcu=atmega2560 -o

#Options for HEX file generation
HFLAGS = -j .text -j .data -O ihex

#Options for avrdude to burn the hex file
#MMCU model here according to avrdude options
DUDEFLAGS = -c
DUDEFLAGS += -patmega2560 
DUDEFLAGS += -cwiring
DUDEFLAGS += -PCOM4  
DUDEFLAGS += -b115200
DUDEFLAGS += -U flash:w:$(MAIN_HEX_PATH):i
DUDEFLAGS += -C"C:\Program Files (x86)\Arduino\hardware\tools\avr\etc\avrdude.conf"

# Sources files needed for building the application 
SRC = $(MAIN_APP).cpp

# The headers files needed for building the application
INCLUDE = -I. 
INCLUDE += Drive_Funktions.h
INCLUDE += Interrupt_Service_Routines.h
INCLUDE += Init.h
INCLUDE += USART_Functions.h
INCLUDE += I2C.h

# commands Section
Burn : Build
	$(AVRDUDE) $(DUDEFLAGS)

Build : $(MAIN_APP).elf
	$(OBJCOPY) $(HFLAGS) $< $(MAIN_APP).hex
	
$(MAIN_APP).elf: $(MAIN_APP).o
	$(CC) $(SRC) $(INCLUDE) $(LFLAGS) $@
	
$(MAIN_APP).o:$(SRC)
	$(CC) $^ $(INCLUDE) $(CFLAGS) $@