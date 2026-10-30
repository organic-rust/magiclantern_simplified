#include "hooks.h"

// M6II 1.1.1 CRaw encoder hooks.  Each one saves the argument registers,
// calls the logger, then replays the 8 bytes the hook overwrote and jumps
// back.  Both overwritten sequences end in "ldr rX, [pc, #imm]", which
// loads 0x143b8 (CRaw driver globals); we load it directly instead.
// 8 registers are pushed to keep the stack 8-byte aligned for the call;
// the call goes through a register so it works whether the module code is
// ARM or Thumb.

// 0xe0300ed0:  stmdb sp!, {r0-r8, lr}; mov r6, r0; ldr r7, [pc, #208]
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_EncInit_M6II(void)
{
    asm(
        "push {r0-r5, r12, lr}\n"
        "ldr r12, =dmlog_EncInit\n"
        "blx r12\n"
        "pop {r0-r5, r12, lr}\n"

        "stmdb sp!, {r0-r8, lr}\n"
        "mov r6, r0\n"
        "ldr r7, =0x143b8\n"
        "ldr pc, =0xe0300ed9\n"
    );
}

// 0xe0301306:  stmdb sp!, {r0-r2, r4-r11, lr}; mov r4, r1; ldr r0, [pc, #388]
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_EncSetParam_M6II(void)
{
    asm(
        "push {r0-r5, r12, lr}\n"
        "ldr r12, =dmlog_EncSetParam\n"
        "blx r12\n"
        "pop {r0-r5, r12, lr}\n"

        "stmdb sp!, {r0-r2, r4-r11, lr}\n"
        "mov r4, r1\n"
        "ldr r0, =0x143b8\n"
        "ldr pc, =0xe030130f\n"
    );
}

// EDMAC setup hooks (one 64 KB page, 0xe0580000): find the Canon code that
// programs the LV raw writer (ch 1) and reader (ch 48).  The original
// caller's LR is passed to the logger.  Both prologues end in
// "ldr r0, [pc, #imm]", loading 0xe1008944 (channel table).

// 0xe0581942 read pack setter (ch, fields):
//   push {r2-r6, lr}; mov r5, r0; ldr r0, [pc, #508]; mov r4, r1
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_ReadPack_M6II(void)
{
    asm(
        "push {r0-r5, r12, lr}\n"
        "mov r2, lr\n"
        "mov r3, #0\n"
        "ldr r12, =dmlog_edmac_read_pack\n"
        "blx r12\n"
        "pop {r0-r5, r12, lr}\n"

        "push {r2-r6, lr}\n"
        "mov r5, r0\n"
        "ldr r0, =0xe1008944\n"
        "mov r4, r1\n"
        "ldr pc, =0xe058194b\n"
    );
}

// 0xe058096e edmac_set_size (ch, info):
//   stmdb sp!, {r4-r11, lr}; mov r5, r0; ldr r0, [pc, #576]
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_SetSize_M6II(void)
{
    asm(
        "push {r0-r5, r12, lr}\n"
        "mov r2, lr\n"
        "mov r3, #1\n"
        "ldr r12, =dmlog_edmac_set_size\n"
        "blx r12\n"
        "pop {r0-r5, r12, lr}\n"

        "stmdb sp!, {r4-r11, lr}\n"
        "mov r5, r0\n"
        "ldr r0, =0xe1008944\n"
        "ldr pc, =0xe0580977\n"
    );
}

// 0xe0580782 StartEDmac (ch):
//   push {r4-r6, lr}; mov r4, r0; bl 0xe057ffdc
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_Start_M6II(void)
{
    asm(
        "push {r0-r5, r12, lr}\n"
        "mov r1, lr\n"
        "ldr r12, =dmlog_edmac_start\n"
        "blx r12\n"
        "pop {r0-r5, r12, lr}\n"

        "push {r4-r6, lr}\n"
        "mov r4, r0\n"
        "ldr r12, =0xe057ffdd\n"
        "blx r12\n"
        "ldr pc, =0xe058078b\n"
    );
}

// 0xe0580502 ConnectWriteEDmac (ch, conn):
//   stmdb sp!, {r4-r12, lr}; mov r4, r0; ldr.w fp, [pc, #676] (0xe1008944)
// (the ldr.w ends at 0xe058050c, past the 8 patched bytes)
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_ConnectWrite_M6II(void)
{
    asm(
        "push {r0-r5, r12, lr}\n"
        "mov r2, lr\n"
        "mov r3, #0\n"
        "ldr r12, =dmlog_edmac_connect\n"
        "blx r12\n"
        "pop {r0-r5, r12, lr}\n"

        "stmdb sp!, {r4-r12, lr}\n"
        "mov r4, r0\n"
        "ldr r11, =0xe1008944\n"
        "ldr pc, =0xe058050d\n"
    );
}

// 0xe05806e4 ConnectReadEDmac (ch, conn):
//   stmdb sp!, {r4-r8, lr}; mov r5, r0; ldr r7, [pc, #212] (0xe1008d6c)
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_ConnectRead_M6II(void)
{
    asm(
        "push {r0-r5, r12, lr}\n"
        "mov r2, lr\n"
        "mov r3, #1\n"
        "ldr r12, =dmlog_edmac_connect\n"
        "blx r12\n"
        "pop {r0-r5, r12, lr}\n"

        "stmdb sp!, {r4-r8, lr}\n"
        "mov r5, r0\n"
        "ldr r7, =0xe1008d6c\n"
        "ldr pc, =0xe05806ed\n"
    );
}

// 0xe06662f6 StartEncode (jobs, n, cbr, ctx):
//   stmdb sp!, {r0-r11, lr}; mov r8, r0; ldr.w r9, [pc, #124] (0x5bbc)
// (the ldr.w ends at 0xe0666300, past the 8 patched bytes).  The logger
// returns the callback to use, which replaces r2.
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_StartEncode_M6II(void)
{
    asm(
        "push {r0-r5, r12, lr}\n"
        "ldr r12, =dmlog_StartEncode\n"
        "blx r12\n"
        "str r0, [sp, #8]\n"
        "pop {r0-r5, r12, lr}\n"

        "stmdb sp!, {r0-r11, lr}\n"
        "mov r8, r0\n"
        "ldr r9, =0x5bbc\n"
        "ldr pc, =0xe0666301\n"
    );
}

// CMOS (sensor register) writers, ImageSequence::Sio0Driver.c, one page
// (0xe06a0000).  Both prologues are stmdb; mov; ldr.w (10 bytes): the
// 8-byte hook ends inside the ldr.w, so all three are replayed and we
// jump back after it.  Both ldr.w load 0x67c84.  The original caller's
// LR goes to the logger.

// 0xe06a0dee cmos_write_pio (u32 *frames):
//   stmdb sp!, {r2-r12, lr}; mov r4, r0; ldr.w fp, [pc, #448]
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_CmosPio_M6II(void)
{
    asm(
        "push {r0-r5, r12, lr}\n"
        "mov r2, lr\n"
        "ldr r12, =dmlog_cmos_pio\n"
        "blx r12\n"
        "pop {r0-r5, r12, lr}\n"

        "stmdb sp!, {r2-r12, lr}\n"
        "mov r4, r0\n"
        "ldr fp, =0x67c84\n"
        "ldr pc, =0xe06a0df9\n"
    );
}

// 0xe06a0f6a cmos_write_dma (u16 *data, n):
//   stmdb sp!, {r3-r9, lr}; mov r6, r0; ldr.w r8, [pc, #68]
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_CmosDma_M6II(void)
{
    asm(
        "push {r0-r5, r12, lr}\n"
        "mov r2, lr\n"
        "ldr r12, =dmlog_cmos_dma\n"
        "blx r12\n"
        "pop {r0-r5, r12, lr}\n"

        "stmdb sp!, {r3-r9, lr}\n"
        "mov r6, r0\n"
        "ldr r8, =0x67c84\n"
        "ldr pc, =0xe06a0f75\n"
    );
}
