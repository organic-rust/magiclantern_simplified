/*
 * Entry point: the 5D2 bootloader loads AUTOEXEC.BIN at 0x800000 and jumps
 * to its first instruction.
 */
    .text
    .org 0
    .globl _start
_start:
    B       restore_mainfirm
