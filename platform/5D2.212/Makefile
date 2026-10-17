MODEL = 5D2
FW_VERSION = 212

TOP_DIR = ../..

AUTOEXEC_BASE = 0x800000
FIR_BASE = 0x800120

FIRMWARE_ID	= 0x80000218

MAIN_FIRMWARE_ADDR = 0xFF810000
PLATFORM_ARCH = armv5te
ML_BOOT_OBJ = boot-d45.o

# DryOSmemory map
# RESTARTSTART is selected to be just above the end of the bss
#
RESTARTSTART = 0x4E000

ML_SRC_EXTRA_OBJS = $(BUILD_DIR)/afma.o $(BUILD_DIR)/asm.o

# Boot progress on the card LED, for cameras that hang with a black screen:
#   make BOOT_LED_DIAG=y
# See contrib/recovery/5D2.212/BOOT-LED-DIAG.md
ifeq ($(BOOT_LED_DIAG),y)
ML_SRC_EXTRA_OBJS += $(BUILD_DIR)/boot-diag.o
CFLAGS_PLATFORM += -DCONFIG_BOOT_LED_DIAG
endif
ifeq ($(BOOT_DIAG_LATE_HOOKS),y)
CFLAGS_PLATFORM += -DCONFIG_BOOT_DIAG_LATE_HOOKS
endif

include ../Makefile
.DEFAULT_GOAL := $(BUILD_DIR)/cam_complete
