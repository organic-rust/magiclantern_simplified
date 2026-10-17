/*
 * 5D Mark II: restore the MAIN FIRMWARE boot flag from a bootable card.
 *
 * UNTESTED ON HARDWARE. Read README.md before running this on a camera.
 *
 * Use case: the FIRMWARE boot flag at 0xF8000000 has been left disabled
 * (0xFFFFFFFF), e.g. by a firmware update that failed at the last step, so the
 * camera boots straight into "Firmware update program / Update file cannot be
 * found". The BOOTDISK flag must still be enabled, so the bootloader loads this
 * file as AUTOEXEC.BIN from a bootable card.
 *
 * Method: the 5D2 bootloader (Boot loader Ver 0.30, K218) copies its
 * FROMUTILITY half from ROM 0xFFFF2B18 to RAM 0x100000 and keeps a RAM shadow
 * of the 16-byte flag block. FROMUTILITY has setters for BOOTDISK / RAM_EXE /
 * UPDATE but none for FIRMWARE, so we:
 *     flags_read();  shadow[0] = 0;  flags_write();
 * flags_write() erases the 8 KB parameter sector at 0xF8000000 (the flags are
 * alone in it), programs 16 bytes and verifies them. An interrupted write
 * leaves an erased sector = FIR=-1 BOOT=-1, i.e. the state this tool starts
 * from, so the card escape hatch survives.
 *
 * Addresses were derived from a 5D2 ROM1 dump with
 * find_bl_flag_stubs.py; every one is guarded before use.
 *
 * LED feedback (card LED):
 *     solid on            success, FIRMWARE flag now enabled (0)
 *     slow blink          flag was already enabled, nothing written
 *     fast blink          a guard failed, nothing written
 *     long-short blink    flags_write() returned an error or readback mismatch
 */

#include <stdint.h>

#define MEM(x) (*(volatile uint32_t *)(x))

/* card LED, from platform/5D2.212/consts.h */
#define CARD_LED  0xC02200BC
#define LEDON     0x46
#define LEDOFF    0x44

/* boot flag block in ROM1 */
#define BOOTFLAGS 0xF8000000

/* FROMUTILITY RAM copy: RAM = ROM - 0xFFFF0000 + 0xFD4E8 */
#define BL_FLAGS_READ     0x102160  /* ROM FFFF4C78 */
#define BL_FLAGS_WRITE    0x102148  /* ROM FFFF4C60 */
#define BL_SHADOW_LIT     0x10215C  /* ROM FFFF4C74: literal = shadow buffer address */
#define BL_SHADOW         0x10D770

/* expected instruction / data words at those addresses */
#define G_FLAGS_READ      0xE51F000C  /* ldr r0, [pc, #-0xc] */
#define G_FLAGS_WRITE     0xE59F100C  /* ldr r1, [pc, #0xc]  */
#define G_SHADOW_LIT      0x0010D770

/* ROM string "<K218 Board> SystemCLK 132MHz" at 0xFFFF0760 */
#define BL_BOARD_STR      0xFFFF0760

static void delay(int n)
{
    for (volatile int i = 0; i < n * 100000; i++)
        ;
}

static void __attribute__((noreturn)) blink(int on, int off)
{
    while (1)
    {
        MEM(CARD_LED) = LEDON;  delay(on);
        MEM(CARD_LED) = LEDOFF; delay(off);
    }
}

static void __attribute__((noreturn)) blink_long_short(void)
{
    while (1)
    {
        MEM(CARD_LED) = LEDON;  delay(30);
        MEM(CARD_LED) = LEDOFF; delay(10);
        MEM(CARD_LED) = LEDON;  delay(5);
        MEM(CARD_LED) = LEDOFF; delay(30);
    }
}

static int is_k218_bootloader(void)
{
    const char *want = "<K218 Board>";
    const volatile char *s = (const volatile char *) BL_BOARD_STR;
    for (int i = 0; want[i]; i++)
        if (s[i] != want[i])
            return 0;
    return 1;
}

static void irq_disable(void)
{
    uint32_t cpsr;
    asm volatile ("mrs %0, cpsr" : "=r"(cpsr));
    cpsr |= 0xC0;   /* I and F */
    asm volatile ("msr cpsr_c, %0" :: "r"(cpsr));
}

void __attribute__((noreturn)) restore_mainfirm(void)
{
    irq_disable();

    /* guards: nothing is written unless all of these match */
    if (!is_k218_bootloader())                 blink(3, 3);
    if (MEM(BL_FLAGS_READ)  != G_FLAGS_READ)   blink(3, 3);
    if (MEM(BL_FLAGS_WRITE) != G_FLAGS_WRITE)  blink(3, 3);
    if (MEM(BL_SHADOW_LIT)  != G_SHADOW_LIT)   blink(3, 3);

    void (*flags_read)(void)  = (void *) BL_FLAGS_READ;
    int  (*flags_write)(void) = (void *) BL_FLAGS_WRITE;
    volatile uint32_t *shadow = (volatile uint32_t *) BL_SHADOW;

    /* note: flags_read copies 0x10 halfwords (32 bytes) into Canon's buffer */
    flags_read();

    for (int i = 0; i < 4; i++)
        if (shadow[i] != MEM(BOOTFLAGS + 4 * i))
            blink(3, 3);

    if (shadow[0] == 0)
        blink(20, 20);                          /* already enabled */

    uint32_t bootdisk = shadow[1];

    shadow[0] = 0;        /* FIRMWARE: negative logic, 0 = enabled            */
                          /* BOOTDISK, RAM_EXE, UPDATE left as they were       */

    int r = flags_write();  /* 0 on success; -1 erase, -3 program, -2 verify   */

    if (r != 0 || MEM(BOOTFLAGS) != 0 || MEM(BOOTFLAGS + 4) != bootdisk)
        blink_long_short();

    MEM(CARD_LED) = LEDON;
    while (1)
        ;
}
