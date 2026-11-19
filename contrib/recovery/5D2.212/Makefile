# Standalone bootloader-context AUTOEXEC.BIN for 5D2 (no ML runtime).
# Build: make            (needs arm-none-eabi-gcc in PATH)

name    = AUTOEXEC
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
CFLAGS  = -nostdlib -ffreestanding -march=armv5te -marm -Os -Wall -Wextra

OBJS = entry.o restore_mainfirm.o

all: $(name).BIN

$(name).BIN: $(name).elf
	$(OBJCOPY) -O binary $< $@
	@ls -l $@

$(name).elf: $(OBJS) link.script
	$(CC) $(CFLAGS) -Wl,-T,link.script -o $@ $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.S
	$(CC) $(CFLAGS) -c -o $@ $<

# Inverse tool: disable the FIRMWARE flag so the bootloader loads a .FIR from
# the card (see README.md). Output: disable/AUTOEXEC.BIN
disable: disable/$(name).BIN

disable/$(name).BIN: disable_mainfirm.elf
	mkdir -p disable
	$(OBJCOPY) -O binary $< $@
	@ls -l $@

disable_mainfirm.elf: entry.o disable_mainfirm.o link.script
	$(CC) $(CFLAGS) -Wl,-T,link.script -o $@ entry.o disable_mainfirm.o

disable_mainfirm.o: restore_mainfirm.c
	$(CC) $(CFLAGS) -DDISABLE_MAINFIRM -c -o $@ $<

clean:
	rm -f *.o *.elf $(name).BIN
	rm -rf disable

.PHONY: all clean disable
