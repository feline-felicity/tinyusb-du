CROSS_COMPILE ?= avr-

include $(TOP)/$(BOARD_PATH)/board.mk
# CPU_CORE ?= avr

PORT ?= 0

CFLAGS += -DCFG_TUSB_MCU=OPT_MCU_AVRDU

SKIP_LIBS = 1
# LDFLAGS_GCC += 
# LD_FILE = $(FAMILY_PATH)/linker/${CH32_FAMILY}.ld

SRC_C += src/portable/microchip/avrdu/dcd_avrdu.c

# SRC_S += 

INC += \
	$(TOP)/$(BOARD_PATH)

# actually in ThirdParty/Partner-Supported-Ports/GCC/AVR_AVRDx
FREERTOS_PORTABLE_SRC = $(FREERTOS_PORTABLE_PATH)/AVR_AVRDx

# flash: flash-avrdude-du
