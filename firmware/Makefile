MCU = atmega644p
ARCH = AVR8
BOARD = NONE
F_CPU = 16000000
F_USB = $(F_CPU)

PROJECT            = sidguts
SOURCES            = main.c  uu.c     
HEADERS            = uu.h

EXTRAINCDIRS =

VERSION = 0

#CC                 = /Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avr-gcc
CC = avr-gcc
#OBJCOPY            = /Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avr-objcopy
OBJCOPY = avr-objcopy

F_CPU              = 16000000

ISP_PROG           = -c usbtiny -F
ISP_PORT           = usb

ISP_LOCK_FUSE_PRE  = 0x3f
ISP_LOCK_FUSE_POST = 0x0f

ifeq ($(MCU), atmega328p)
#ISP_HIGH_FUSE      = 0xDE
ISP_HIGH_FUSE      = 0xD8
ISP_LOW_FUSE       = 0xFF
# BOD at 4.3v
ISP_EXT_FUSE       = 0x04
endif

ifeq ($(MCU), atmega644p)
#ISP_HIGH_FUSE      = 0xDE
ISP_HIGH_FUSE      = 0xD8
ISP_LOW_FUSE       = 0xFF
# BOD at 4.3v
ISP_EXT_FUSE       = 0x04
endif


AVRDUDE          = avrdude
AVRDUDE_COM_OPTS =  -V -p $(MCU)
AVRDUDE_ISP_OPTS = -P $(ISP_PORT) $(ISP_PROG)

OBJECTS = $(SOURCES:.c=.o)

CDEFS  = -DF_CPU=$(F_CPU)UL
CDEFS += -DF_USB=$(F_USB)UL
CDEFS += -DBOARD=BOARD_$(BOARD) -DARCH=ARCH_$(ARCH)
CDEFS += -DVERSION=$(strip $(VERSION))

ADEFS += -DF_USB=$(F_USB)UL
ADEFS += -DBOARD=BOARD_$(BOARD) -DARCH=ARCH_$(ARCH)
ADEFS += $(LUFA_OPTS)

CFLAGS      =   -mmcu=$(MCU) -DF_CPU=$(F_CPU) \
		-I. \
		-g -Os -w -Wall \
		-ffunction-sections -fdata-sections -std=gnu99
ASFLAGS       = -mmcu=$(MCU) -I. -x assembler-with-cpp
LDFLAGS       = -mmcu=$(MCU) -lm -Wl,--gc-sections -Os

CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CDEFS)

$(PROJECT).hex: $(PROJECT).out
#	$(OBJCOPY) -j .text -O ihex $(PROJECT).out $(PROJECT).hex
	$(OBJCOPY) -O ihex -R .eeprom $(PROJECT).out $(PROJECT).hex

$(PROJECT).out: $(OBJECTS)
	$(CC) $(LDFLAGS) $(CFLAGS) $(OBJECTS) -o $@ -lc

.c.o:
	$(CC) $(LDFLAGS) $(CFLAGS) -c -I./ $< -o $@ 

ispload: $(PROJECT).hex
		$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ISP_OPTS) -e \
			-U hfuse:w:$(ISP_HIGH_FUSE):m \
			-U lfuse:w:$(ISP_LOW_FUSE):m \
			-U efuse:w:$(ISP_EXT_FUSE):m
		$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ISP_OPTS) -D \
			-U flash:w:$(PROJECT).hex:i
		$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ISP_OPTS) \
			-U lock:w:$(ISP_LOCK_FUSE_POST):m


#			-U lock:w:$(ISP_LOCK_FUSE_PRE):m \


clean:
	rm -f $(PROJECT).out
	rm -f $(PROJECT).hex
	rm -f *.o
