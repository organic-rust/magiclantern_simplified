# define the module name - make sure name is max 8 characters
MODULE_NAME = dmlog
MODULE_OBJS = $(BUILD_DIR)/dmlog.o \
              $(BUILD_DIR)/hooks_thumb_M6II.o
TOP_DIR = ../../..

# include modules environment
include $(TOP_DIR)/modules/Makefile
CFLAGS_ARCHLESS = $(filter-out -march=armv7-r -march=armv7-a -march=armv5te,$(CFLAGS))
.DEFAULT_GOAL := $(BUILD_DIR)/module_complete

# hook stubs replay Thumb instructions, so they must be Thumb themselves
$(BUILD_DIR)/hooks_thumb_M6II.o: hooks_thumb_M6II.c
	$(CC) $(CFLAGS_ARCHLESS) -mthumb -march=armv7 $(CDEPS_FLAGS) -c -o $@ $<
