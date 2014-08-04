############################################################################
#
#    Makefile for STM32F4-H407
#
#    Run 'make' to compile code. 
#
#    Run 'make doc' do compile documentation. Doxygen and Dot are required.
#    Both are available in the general Ubuntu repository, or can be compiled
#    from scratch. Doxygen version >1.8 is required.
#
############################################################################

# project name
PRJNAME = provant_v_0_1

# toolchain configuration
TOOLCHAIN_PREFIX = arm-none-eabi-

# cpu configuration
THUMB = -mthumb
MCU   = cortex-m4

# path, root and common dir
PRJDIR := $(shell pwd)
PATH   := ${PATH}:$(PRJDIR)
COMMON := $(PRJDIR)/../common

# project directories
OUTDIR       := $(PRJDIR)/bin
MODDIR       := $(COMMON)/modules
INCDIR       := $(PRJDIR)/inc
LIBDIR       := $(PRJDIR)/../lib

# modules 
MODULES	      = $(MODDIR)/common
MODULES	     += $(MODDIR)/rc
MODULES	     += $(MODDIR)/io

# CMSIS directory
CMSISDIR     := $(LIBDIR)/cmsis
CMSISSRCDIR  := $(CMSISDIR)/src
CMSISINCDIR  := $(CMSISDIR)/inc

# FreeRTOS directories
FRTDIR    := $(LIBDIR)/FreeRTOSV7.5.2/FreeRTOS
FRTSRCDIR := $(FRTDIR)/Source
FRTINCDIR := $(FRTSRCDIR)/include
FRTMEMDIR := $(FRTSRCDIR)/portable/MemMang/
FRTPORDIR := $(FRTSRCDIR)/portable/GCC/ARM_CM4F

# C source files

#modules
C_SRC += $(PRJDIR)/*.c
C_SRC += $(COMMON)/system/*.c
C_SRC += $(MODDIR)/rc/*.c
C_SRC += $(MODDIR)/io/*.c
C_SRC += $(MODDIR)/common/*.c
#freertos
C_SRC += $(FRTSRCDIR)/list.c 
C_SRC += $(FRTSRCDIR)/queue.c 
C_SRC += $(FRTSRCDIR)/tasks.c 
C_SRC += $(FRTSRCDIR)/timers.c 
C_SRC += $(FRTPORDIR)/port.c  
C_SRC += $(FRTMEMDIR)/heap_2.c
# FreeRTOS+Trace
C_SRC += $(LIBDIR)/trace/*.c

# Assembler source files
A_SRC = 
	
###################################################

# executables
DOXYGEN = doxygen

# include directories
INCDIRS = $(INCDIR) $(LIBDIR) $(MODULES) $(COMMON)/system 											\
	  	  $(CMSISDIR) $(CMSISINCDIR) $(CMSISINCDIR)/peripherals $(CMSISINCDIR)/core \
	  	  $(FRTINCDIR) $(FRTPORDIR)													\
	  	  $(LIBDIR)/trace/config $(LIBDIR)/trace/inc		 						

# add startup file to build
C_SRC += $(CMSISDIR)/startup_stm32f4xx.s	

# Define programs and commands.
CC      = $(TOOLCHAIN_PREFIX)gcc
OBJCOPY = $(TOOLCHAIN_PREFIX)objcopy
OBJDUMP = $(TOOLCHAIN_PREFIX)objdump
NM      = $(TOOLCHAIN_PREFIX)nm

# compiler flags
CFLAGS  = -g -O2 -Wall -T$(COMMON)/system/stm32_flash.ld #-ftime-report
CFLAGS += -mlittle-endian -mthumb -mcpu=$(MCU) -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I. $(patsubst %,-I%,$(INCDIRS)) 
# cpp related flags
CFLAGS += -Os -ffunction-sections -fdata-sections -fno-exceptions --specs=nano.specs -Wl,--gc-sections 
CFLAGS += -std=c99 

###################################################

# build again when Makefile changes
BUILDONCHANGE = Makefile

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: doc lib proj

all: directories lib proj

doc:
	cd $(PRJDIR)/doc && $(DOXYGEN)
	cd ..

directories:
	mkdir -p bin

lib:
	$(MAKE) -C $(CMSISDIR)

proj: 	$(PRJNAME).elf

$(PRJNAME).elf: $(C_SRC) 
	$(CC) $(CFLAGS) $^ -o $(OUTDIR)/$@ -L$(CMSISDIR) -lc -lstm32f4 -lstdc++ -lnosys
	$(OBJCOPY) -O ihex $(OUTDIR)/$(PRJNAME).elf $(OUTDIR)/$(PRJNAME).hex
	$(OBJCOPY) -O binary $(OUTDIR)/$(PRJNAME).elf $(OUTDIR)/$(PRJNAME).bin

clean:
	rm -f *.o
	rm -f $(OUTDIR)/$(PRJNAME).elf
	rm -f $(OUTDIR)/$(PRJNAME).hex
	rm -f $(OUTDIR)/$(PRJNAME).bin
