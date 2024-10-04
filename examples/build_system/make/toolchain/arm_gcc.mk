# makefile for arm gcc toolchain

# Can be set by family, default to ARM GCC
CROSS_COMPILE ?= arm-none-eabi-

CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
AS = $(CC) -x assembler-with-cpp
LD = $(CC)

GDB = $(CROSS_COMPILE)gdb
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size

CFLAGS += \
  -fsingle-precision-constant \

ifndef SKIP_LIBS
  LIBS += -lgcc -lm -lnosys
endif

include ${TOP}/examples/build_system/make/toolchain/gcc_common.mk
