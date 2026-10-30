#include <module.h>
#include <dryos.h>
#include <property.h>
#include <bmp.h>
#include <menu.h>
#include <config.h>
#include <console.h>
#include <shoot.h>
#include <beep.h>
#include <edmac.h>
#include <timer.h>
#include <asm.h>

static int is_5d3 = 0;

/* DIGIC 8 has a different register layout, and the core's struct edmac_info
 * is larger than the D4/D5 one this module is built with.  On D8 we read
 * the geometry registers ourselves and never touch struct edmac_info. */
static int is_d8 = 0;
static uint32_t num_channels = 0;

extern WEAK_FUNC(ret_0) char * asm_guess_func_name_from_string(uint32_t func_addr);

/* not stubbed on every model; only find_free_edmac_channels() uses them,
 * and it doesn't run on D8 */
extern WEAK_FUNC(ret_0) struct LockEntry *CreateResLockEntry(uint32_t *resIds, uint32_t resIdCount);
extern WEAK_FUNC(ret_0) unsigned int LockEngineResources(struct LockEntry *lockEntry);
extern WEAK_FUNC(ret_0) unsigned int UnLockEngineResources(struct LockEntry *lockEntry);

static const char * edmac_format_size_short(struct edmac_info * info, uint32_t maxlen)
{
    const char * sz = edmac_format_size(info);

    if (strlen(sz) > maxlen)
    {
        /* description too complex? just print the total transfer size */
        return format_memory_size(edmac_get_total_size(info, 0));
    }

    return sz;
}

/* D8 geometry registers, in the order Canon's set-size call writes them
 * (M6II 0xE058096E): +0x48 .. +0x70 */
struct d8_geometry
{
    uint32_t ys_xs;
    uint32_t ya_xa;
    uint32_t yb_xb;
    uint32_t yn_xn;
    uint32_t off1s;
    uint32_t off2s;
    uint32_t off1a;
    uint32_t off2a;
    uint32_t off1b;
    uint32_t off2b;
    uint32_t off3;
};

#define LO16(x) ((x) & 0xFFFF)
#define HI16(x) ((x) >> 16)

static struct d8_geometry d8_get_geometry(uint32_t channel)
{
    uint32_t base = edmac_get_base(channel);
    struct d8_geometry g;
    uint32_t * regs = (uint32_t *) &g;

    for (uint32_t i = 0; i < sizeof(g) / 4; i++)
    {
        regs[i] = MEM(base + 0x48 + 4 * i);
    }
    return g;
}

/* Bytes transferred, using the D4/D5 formula on the a/b/n fields.
 * How the D8-only "s" fields contribute is not known yet. */
static uint32_t d8_transfer_size(struct d8_geometry * g)
{
    uint32_t xa = LO16(g->ya_xa), ya = HI16(g->ya_xa);
    uint32_t xb = LO16(g->yb_xb), yb = HI16(g->yb_xb);
    uint32_t xn = LO16(g->yn_xn), yn = HI16(g->yn_xn);

    return (xa * (ya+1) * xn + xb * (ya+1)) * yn +
           (xa * (yb+1) * xn + xb * (yb+1));
}

/* "XxY" for plain rectangles, otherwise the total; "~" when s is set */
static const char * d8_format_size(struct d8_geometry * g)
{
    static char buf[32];

    if (!g->ya_xa && !g->yn_xn && !g->ys_xs)
    {
        snprintf(buf, sizeof(buf), "%dx%d", LO16(g->yb_xb), HI16(g->yb_xb) + 1);
    }
    else
    {
        snprintf(buf, sizeof(buf), "%s%s",
            g->ys_xs ? "~" : "", format_memory_size(d8_transfer_size(g))
        );
    }
    return buf;
}

static const char * channel_size_short(uint32_t channel)
{
    if (is_d8)
    {
        struct d8_geometry g = d8_get_geometry(channel);
        return d8_format_size(&g);
    }

    struct edmac_info info = edmac_get_info(channel);
    return edmac_format_size_short(&info, 10);
}

static int edmac_selection;

/* D8: reading registers of an engine that is powered down hangs the camera
 * (the overview and the all-channel snapshot both froze on M6II), so only
 * the one channel chosen here is ever read. */
static int probe_channel = 1;

static void edmac_display_page_d8(int i0, int x0, int y0)
{
    bmp_printf(FONT_MONO_20, x0, y0, "EDM# Base      Dir\n");
    y0 += fontspec_font(FONT_MONO_20)->height * 2;

    for (int i = 0; i < 16; i++)
    {
        uint32_t ch = i0 + i;
        if (ch >= num_channels)
        {
            break;
        }

        uint32_t dir = edmac_get_dir(ch);
        bmp_printf(
            FONT(FONT_MONO_20, ch == (uint32_t) probe_channel ? COLOR_GREEN1 : COLOR_GRAY(50), COLOR_BLACK),
            x0, y0 + i * fontspec_font(FONT_MONO_20)->height,
            "[%2d] %8x %s", ch, edmac_get_base(ch),
            dir == EDMAC_DIR_READ ? "rd" : dir == EDMAC_DIR_WRITE ? "wr" : "--"
        );
    }
}

static void edmac_display_page(int i0, int x0, int y0)
{
    bmp_printf(
        FONT_MONO_20,
        x0, y0,
        "EDM# Address  Size\n"
    );

    y0 += fontspec_font(FONT_MONO_20)->height * 2;

    for (int i = 0; i < 16; i++)
    {
        char msg[100];
        int ch = i0 + i;

        if ((uint32_t) ch >= num_channels)
        {
            break;
        }

        uint32_t addr = edmac_get_address(ch);

        int state = edmac_get_state(ch);

        const char * sz = channel_size_short(ch);
        snprintf(msg, sizeof(msg), "[%2d] %8x: %s", ch, addr, sz);

        if (state != 0 && state != 1)
        {
            STR_APPEND(msg, " (%x)", state);
        }

        uint32_t dir     = edmac_get_dir(ch);
        uint32_t conn_w  = edmac_get_connection(ch, EDMAC_DIR_WRITE);
        uint32_t conn_r  = edmac_get_connection(ch, EDMAC_DIR_READ);

        int color =
            dir == EDMAC_DIR_UNUSED ? COLOR_GRAY(20) :   /* unused? */
            state == 0              ? COLOR_GRAY(50) :   /* inactive? */
            state == 1              ? COLOR_GREEN1   :   /* active? */
                                      COLOR_RED      ;   /* no idea */

        if (dir == EDMAC_DIR_WRITE)
        {
            if (conn_w == 0 && !is_d8)
            {
                /* Write EDMAC, but could not figure out where it's connected */
                /* May be either unused, or connected to 0 (RAW data) */
                STR_APPEND(msg, " <w!>");
            }
            else
            {
                STR_APPEND(msg, " <w%x>", conn_w);
            }
        }
        else if (dir == EDMAC_DIR_READ)
        {
            if (conn_r == 0xFF)
            {
                /* Read EDMAC, but could not figure out where it's connected */
                STR_APPEND(msg, " <r!>");
            }
            else
            {
                STR_APPEND(msg, " <r%x>", conn_r);
            }
        }

        if (dir != EDMAC_DIR_UNUSED && strchr(msg, '!'))
        {
            color = COLOR_YELLOW;
        }


        bmp_printf(
            FONT(FONT_MONO_20, color, COLOR_BLACK),
            x0, y0 + i * fontspec_font(FONT_MONO_20)->height,
            msg
        );
    }
}

/* D4/D5 only: D8 connections are per-engine routing values, not one shared
 * numbering, so this map doesn't apply there */
static void edmac_display_conn(int page)
{
    uint32_t conn0 = page * 16;

    bmp_printf(
        FONT_LARGE,
        50, 20,
        "EDMAC connections %d - %d", conn0, conn0 + 15
    );

    int y = 50;
    int font_height = fontspec_font(FONT_MONO_20)->height;
    int line_height = font_height + 4;

    /* memory map */
    for (int i = 0; i < 400; i++)
    {
        int y = i + 50;
        draw_line(10, y, 20, y, COLOR_GRAY(50));
        draw_line(700, y, 710, y, COLOR_GRAY(50));
    }

    bmp_printf(FONT_MONO_20 | FONT_ALIGN_CENTER, 15, 460, "RD");
    bmp_printf(FONT_MONO_20 | FONT_ALIGN_CENTER, 720-15, 460, "WR");

    bmp_printf(FONT_SMALL | FONT_ALIGN_LEFT, 30, 45, "0");
    bmp_printf(FONT_SMALL | FONT_ALIGN_RIGHT, 720-30, 45, "0");

    bmp_printf(FONT_SMALL | FONT_ALIGN_LEFT, 30, 445, "512MB");
    bmp_printf(FONT_SMALL | FONT_ALIGN_RIGHT, 720-30, 445, "512MB");

    const char * labels[64] = {
        [0]  = "RAW",    /* most models, photo + LV; also used for TTL */
        [1]  = "DEF",    /* DEF input? */
        [2]  = "WB",     /* OBWB related */
        [3]  = "YUV",    /* RSZ: reads YUV from <3>, writes YUV to <3> */
        [5]  = "JPG",    /* JPG DEC: reads JPG from <5>, writes YUV to <3> */
        [8]  = "DEF",    /* DEF input? */
        [15] = "HIV",    /* input only (row/column offsets for pattern noise correction) */
        [16] = "DEF",    /* DEF output? */
        [35] = "RAW",    /* 5D3 photo */
    };

    for (uint32_t conn = conn0; conn < conn0 + 16; conn++)
    {
        y += line_height;

        bmp_printf(FONT_MONO_20 | FONT_ALIGN_CENTER, 360, y, ">%02d      %02d>", conn, conn);

        if (labels[conn])
        {
            /* fixme: how to show the endpoints are *not* connected? */
            bmp_printf(FONT_MONO_20 | FONT_ALIGN_CENTER, 360, y, "(%s)", labels[conn]);
        }

        if (conn == 6 || conn == 7)
        {
            /* passthru */
            draw_line(360-30, y + font_height / 2, 360+30, y + font_height / 2, COLOR_WHITE);
        }

        uint32_t ch_r = 0xFFFFFFFF;
        uint32_t ch_w = 0xFFFFFFFF;

        for (uint32_t chan = 0; chan < num_channels; chan++)
        {
            uint32_t dir = edmac_get_dir(chan);
            uint32_t c   = edmac_get_connection(chan, dir);

            if (dir == EDMAC_DIR_READ)
            {
                if (c == conn)
                {
                    if (conn != 0 || edmac_get_pointer(chan))
                    {
                        /* connection #0: finding the associated channel is not reliable
                         * heuristic: pick the channel with configured address */
                        ch_r = chan;
                    }
                }
            }
            else if (dir == EDMAC_DIR_WRITE)
            {
                if (c == conn)
                {
                    if (conn != 0 || edmac_get_pointer(chan))
                    {
                        /* connection #0: finding the associated channel is not reliable
                         * heuristic: pick the channel with configured address */
                        ch_w = chan;
                    }
                }
            }
        }

        /* fixme: handle conflicts (multiple channels on the same connection) */

        uint32_t chs[] = { ch_r, ch_w };
        for (int i = 0; i < COUNT(chs); i++)
        {
            uint32_t ch = chs[i];
            if (ch != 0xFFFFFFFF)
            {
                /* show the channels on the memory map */
                struct edmac_info info = edmac_get_info(ch);
                uint32_t dir  = edmac_get_dir(ch);
                uint64_t addr = (uint32_t)CACHEABLE(edmac_get_address(ch));
                uint64_t ptr  = (uint32_t)CACHEABLE(edmac_get_pointer(ch));
                uint64_t last = addr + edmac_get_total_size(&info, 1);

                int ya = 50 + addr * 400 / 0x20000000;
                int yp = 50 + ptr  * 400 / 0x20000000;
                int yl = 50 + last * 400 / 0x20000000;
                int xm = (dir == EDMAC_DIR_READ) ? 10 : 710;
                int xr = (dir == EDMAC_DIR_READ) ? 20 : 700;
                int xc = (dir == EDMAC_DIR_READ) ? 100 : 720-100;
                int xf = (dir == EDMAC_DIR_READ) ? 250 : 720-250;
                int yc = y + font_height / 2;

                /* not all channels have known start address
                 * maybe they are managed by a secondary CPU? */
                if (addr)
                {
                    /* highlight the memory read or written by the current channel */
                    for (int yi = ya; yi <= yl; yi++)
                    {
                        draw_line(xm, yi, xr, yi, COLOR_BLUE);
                        draw_line(xr, yi, xc, yc, COLOR_BLUE);
                    }
                }
                /* draw the current EDMAC pointer (read from hardware) */
                draw_line(xm, yp, xr, yp, COLOR_RED);
                draw_line(xr, yp, xc, yc, COLOR_RED);
                draw_line(xc, yc, xf, yc, COLOR_RED);

                /* show size for each channel */
                const char * sz = edmac_format_size_short(&info, 10);
                int state = edmac_get_state(ch);

                int color =
                    dir == EDMAC_DIR_UNUSED ? COLOR_GRAY(20) :   /* unused? */
                    state == 0              ? COLOR_GRAY(50) :   /* inactive? */
                    state == 1              ? COLOR_GREEN1   :   /* active? */
                                              COLOR_RED      ;   /* no idea */

                int fnt = FONT(FONT_MONO_20, color, COLOR_BLACK);

                if (dir == EDMAC_DIR_READ)
                {
                    bmp_printf(fnt | FONT_ALIGN_RIGHT, 270, y, "%s #%02d", sz, ch);
                }
                else
                {
                    bmp_printf(fnt | FONT_ALIGN_LEFT, 720-270, y, "#%02d %s", ch, sz);
                }
            }
        }
    }
}

static const char * dir_name(uint32_t dir)
{
    return
        dir == EDMAC_DIR_READ  ? "read"  :
        dir == EDMAC_DIR_WRITE ? "write" :
                                 "unused";
}

static void edmac_display_detailed_d8(int channel)
{
    uint32_t base = edmac_get_base(channel);
    uint32_t dir  = edmac_get_dir(channel);

    int y = 50;
    bmp_printf(
        FONT_LARGE,
        50, y-30,
        "EDMAC channel #%d - %x\n",
        channel,
        base
    );
    y += font_large.height;

    int fh = fontspec_font(FONT_MONO_20)->height;

    if (channel != probe_channel)
    {
        /* scrolling past a powered-down engine would hang the camera */
        bmp_printf(FONT_MONO_20, 50, y += fh, "Direction  : %s", dir_name(dir));
        bmp_printf(FONT_MONO_20, 50, y += 2 * fh, "Not read. Set \"Probe channel\" to %d", channel);
        bmp_printf(FONT_MONO_20, 50, y += fh, "to read its registers here.");
        return;
    }

    /* connection lookup reads the Boomer routing registers; not done on D8
     * until they are known to be safe to read */
    struct d8_geometry g = d8_get_geometry(channel);

    bmp_printf(FONT_MONO_20, 50, y += fh, "Address    : %8x ", edmac_get_address(channel));
    bmp_printf(FONT_MONO_20, 50, y += fh, "Pointer    : %8x ", edmac_get_pointer(channel));
    bmp_printf(FONT_MONO_20, 50, y += fh, "Run (+04)  : %8x ", edmac_get_state(channel));
    bmp_printf(FONT_MONO_20, 50, y += fh, "Mode (+08) : %8x ", MEM(base + 0x08));
    bmp_printf(FONT_MONO_20, 50, y += fh, "Xfer (+C0) : %8x ", edmac_get_flags(channel));
    bmp_printf(FONT_MONO_20, 50, y += fh, "Size       : %s (%s)", d8_format_size(&g), format_memory_size(d8_transfer_size(&g)));
    y += fh;
    bmp_printf(FONT_MONO_20, 50, y += fh, "Size S     : %8x (%d x %d) ", g.ys_xs, LO16(g.ys_xs), HI16(g.ys_xs));
    bmp_printf(FONT_MONO_20, 50, y += fh, "Size A     : %8x (%d x %d) ", g.ya_xa, LO16(g.ya_xa), HI16(g.ya_xa));
    bmp_printf(FONT_MONO_20, 50, y += fh, "Size B     : %8x (%d x %d) ", g.yb_xb, LO16(g.yb_xb), HI16(g.yb_xb));
    bmp_printf(FONT_MONO_20, 50, y += fh, "Size N     : %8x (%d x %d) ", g.yn_xn, LO16(g.yn_xn), HI16(g.yn_xn));
    bmp_printf(FONT_MONO_20, 50, y += fh, "off1 s/a/b : %8x %8x %8x", g.off1s, g.off1a, g.off1b);
    bmp_printf(FONT_MONO_20, 50, y += fh, "off2 s/a/b : %8x %8x %8x", g.off2s, g.off2a, g.off2b);
    bmp_printf(FONT_MONO_20, 50, y += fh, "off3       : %8x ", g.off3);
    y += fh;
    bmp_printf(FONT_MONO_20, 50, y += fh, "Direction  : %s", dir_name(dir));
}

static void edmac_display_detailed(int channel)
{
    if (is_d8)
    {
        edmac_display_detailed_d8(channel);
        return;
    }

    uint32_t base = edmac_get_base(channel);

    int x = 50;
    int y = 50;
    bmp_printf(
        FONT_LARGE,
        x, y-30,
        "EDMAC channel #%d - %x\n",
        channel,
        base
    );
    y += font_large.height;

    /* http://magiclantern.wikia.com/wiki/Register_Map#EDMAC */

    uint32_t dir     = edmac_get_dir(channel);
    char* dir_s      = (char *) dir_name(dir);

    if (dir == EDMAC_DIR_UNUSED)
    {
        /* attempting to get info from unused channels may crash on old models */
        /* if it doesn't crash, the additional info doesn't make any sense */
        bmp_printf(FONT_MONO_20, 50, y + font_large.height, "Unused");
        return;
    }

    uint32_t state = edmac_get_state(channel);
    uint32_t flags = edmac_get_flags(channel);
    uint32_t addr  = edmac_get_address(channel);

    char flags_desc[32] = "";
    if (flags)
    {
        snprintf(flags_desc, sizeof(flags_desc),
            "(%d bytes per transfer)",
            edmac_bytes_per_transfer(flags)
        );
    }

    union edmac_size_t
    {
        struct { short x, y; } size;
        uint32_t raw;
    };

    union edmac_size_t size_n = (union edmac_size_t) shamem_read(base + 0x0C);
    union edmac_size_t size_b = (union edmac_size_t) shamem_read(base + 0x10);
    union edmac_size_t size_a = (union edmac_size_t) shamem_read(base + 0x14);

    uint32_t off1b = shamem_read(base + 0x18);
    uint32_t off2b = shamem_read(base + 0x1C);
    uint32_t off1a = shamem_read(base + 0x20);
    uint32_t off2a = shamem_read(base + 0x24);
    uint32_t off3  = shamem_read(base + 0x28);
    uint32_t off40 = shamem_read(base + 0x40);

    uint32_t conn_w  = edmac_get_connection(channel, EDMAC_DIR_WRITE);
    uint32_t conn_r  = edmac_get_connection(channel, EDMAC_DIR_READ);

    struct edmac_info info = edmac_get_info(channel);

    int fh = fontspec_font(FONT_MONO_20)->height;

    bmp_printf(FONT_MONO_20, 50, y += fh, "Address    : %8x ", addr);
    bmp_printf(FONT_MONO_20, 50, y += fh, "State      : %8x ", state);
    bmp_printf(FONT_MONO_20, 50, y += fh, "Flags      : %8x %s", flags, flags_desc);
    bmp_printf(FONT_MONO_20, 50, y += fh, "Size       : %s ", edmac_format_size(&info));
    y += fh;
    bmp_printf(FONT_MONO_20, 50, y += fh, "Size A     : %8x (%d x %d) ", size_a.raw, size_a.size.x, size_a.size.y);
    bmp_printf(FONT_MONO_20, 50, y += fh, "Size B     : %8x (%d x %d) ", size_b.raw, size_b.size.x, size_b.size.y);
    bmp_printf(FONT_MONO_20, 50, y += fh, "Size N     : %8x (%d x %d) ", size_n.raw, size_n.size.x, size_n.size.y);
    bmp_printf(FONT_MONO_20, 50, y += fh, "off1a      : %8x ", off1a);
    bmp_printf(FONT_MONO_20, 50, y += fh, "off1b      : %8x ", off1b);
    bmp_printf(FONT_MONO_20, 50, y += fh, "off2a      : %8x ", off2a);
    bmp_printf(FONT_MONO_20, 50, y += fh, "off2b      : %8x ", off2b);
    bmp_printf(FONT_MONO_20, 50, y += fh, "off3       : %8x ", off3);
    bmp_printf(FONT_MONO_20, 50, y += fh, "off40      : %8x ", off40);
    y += fh;
    bmp_printf(FONT_MONO_20, 50, y += fh, "Connection : write=0x%x read=0x%x dir=%s", conn_w, conn_r, dir_s);

    if (is_5d3)
    {
        /**
         * ConnectReadEDmac(channel, conn)
         * RAM:edmac_register_interrupt(channel, cbr_handler, ...)
         * => *(8 + 32*arg0 + *0x12400) = arg1
         * and also: *(12 + 32*arg0 + *0x12400) = arg1
         */
        uint32_t cbr1 = MEM(8 + 32*(channel) + MEM(0x12400));
        uint32_t cbr2 = MEM(12 + 32*(channel) + MEM(0x12400));
        bmp_printf(FONT_MONO_20, 50, y += fh, "CBR handler: %8x %s", cbr1, asm_guess_func_name_from_string(cbr1));
        bmp_printf(FONT_MONO_20, 50, y += fh, "CBR abort  : %8x %s", cbr2, asm_guess_func_name_from_string(cbr2));
    }
}

/* display selection: overview screens (32 channels each),
 * then connection maps (D4/D5 only), then one screen per channel */
static int overview_screens()
{
    return (num_channels + 31) / 32;
}

static int conn_screens()
{
    return is_d8 ? 0 : 4;
}

static MENU_UPDATE_FUNC(edmac_display)
{
    if (!info->can_custom_draw) return;
    info->custom_drawing = CUSTOM_DRAW_THIS_MENU;
    bmp_fill(COLOR_BLACK, 0, 0, 720, 480);

    int selection = edmac_selection;

    if (selection < overview_screens())
    {
        if (is_d8)
        {
            /* table data only: no register reads, see probe_channel */
            edmac_display_page_d8(32 * selection, 0, 30);
            edmac_display_page_d8(32 * selection + 16, 360, 30);
            bmp_printf(FONT_MONO_20, 20, 450, "DIGIC 8: registers only for the probe channel");
            return;
        }

        edmac_display_page(32 * selection, 0, 30);
        edmac_display_page(32 * selection + 16, 360, 30);

        bmp_printf(
            FONT_MONO_20,
            20, 450, "EDMAC state: "
        );

        bmp_printf(
            FONT(FONT_MONO_20, COLOR_GRAY(50), COLOR_BLACK),
            20+200, 450, "inactive"
        );

        bmp_printf(
            FONT(FONT_MONO_20, COLOR_GREEN1, COLOR_BLACK),
            20+350, 450, "running"
        );

        bmp_printf(
            FONT_MONO_20,
            720 - fontspec_font(FONT_MONO_20)->width * 13, 450, "[Scrollwheel]"
        );
        return;
    }
    selection -= overview_screens();

    if (selection < conn_screens())
    {
        edmac_display_conn(selection);
        return;
    }
    selection -= conn_screens();

    edmac_display_detailed(selection);
}


/* attempt to detect available edmac channels */
static volatile int trying_to_lock = 0;
static volatile int timed_out = 0;

static void check_timeout()
{
    timed_out = 1;

    if (lv && trying_to_lock)
    {
        int key_play = module_translate_key(MODULE_KEY_PLAY, MODULE_KEY_CANON);
        fake_simple_button(key_play);
    }
}

static int try_lock_edmac_channel(uint32_t resource)
{
    uint32_t res[] = { resource };
    struct LockEntry * resLock = CreateResLockEntry(res, 1);

    /* the Lock call will block if the resource is used by someone else */
    /* to get past it, try to enter PLAY mode after a timeout */
    delayed_call(1000, check_timeout, 0);
    timed_out = 0;
    trying_to_lock = 1;
    LockEngineResources(resLock);
    trying_to_lock = 0;
    UnLockEngineResources(resLock);
    int success = !timed_out;

    while (!timed_out)
        msleep(10);

    return success;
}

static void find_free_edmac_channels()
{
    if (is_d8)
    {
        /* resource IDs for EDMAC channels are not known on D8 yet */
        NotifyBox(3000, "Not available on DIGIC 8 yet.");
        return;
    }

    console_show();
    msleep(2000);

    for (int ch = 0; ch < 32; ch++)
    {
        force_liveview();
        wait_lv_frames(5);

        int dir = edmac_get_dir(ch);
        int i = edmac_channel_to_index(ch);
        uint32_t resource = 0;

        switch (dir)
        {
            case EDMAC_DIR_WRITE:
            {
                printf("Trying write channel #%d... ", ch);
                resource = 0x00000000 + i; /* write edmac channel */
                break;
            }
            case EDMAC_DIR_READ:
            {
                printf("Trying read channel #%d... ", ch);
                resource = 0x00010000 + i; /* read edmac channel */
                break;
            }
            default:
            {
                printf("Skipping unused channel #%d...\n", ch);
                continue;
            }
        }

        if (try_lock_edmac_channel(resource))
        {
            printf("seems to work!\n");
        }
        else
        {
            printf("\n");
        }
    }
}

/* log EDMAC state every X microseconds */
static CONFIG_INT("log.interval", log_interval, 500);

#define LOG_SAMPLES 2048

/* register bases of the channels we poll, filled at startup from the core */
static uint32_t * log_regs = 0;
static uint32_t num_log_regs = 0;

/* where state and current pointer live in a channel's register block;
 * D8 offsets are inferred from Canon's Edmac.c, untested */
static uint32_t state_offset = 0x00;
static uint32_t pointer_offset = 0x08;

/* this should be called at startup */
static void edmac_regs_init()
{
    log_regs = malloc(num_channels * sizeof(log_regs[0]));
    if (!log_regs)
    {
        return;
    }

    for (uint32_t ch = 0; ch < num_channels; ch++)
    {
        if (edmac_get_dir(ch) != EDMAC_DIR_UNUSED)
        {
            log_regs[num_log_regs++] = edmac_get_base(ch);
        }
    }

    if (is_d8)
    {
        state_offset = 0x04;
        pointer_offset = 0xEC;
    }
}

/* one row per sample: one word per polled channel, then these */
#define CLK_IDX (num_log_regs)
#define TSK_IDX (num_log_regs+1)
#define OVH_IDX (num_log_regs+2)
#define XTR_IDX (num_log_regs+3)
#define ROW_WORDS (num_log_regs+4)
#define STATE(i, j) edmac_states[(i) * ROW_WORDS + (j)]

static uint32_t * edmac_states = 0;
static uint32_t edmac_index = 0;

struct edmac_extra_info
{
    uint32_t ch;
    uint32_t addr;
    uint32_t conn;
    uint32_t cbr;
    union
    {
        struct edmac_info info;     /* D4/D5 */
        struct d8_geometry d8;      /* D8 */
    };
};

static struct edmac_extra_info edmac_extra_infos[512];
static int edmac_extra_index = 0;

static void FAST edmac_spy_poll(int last_expiry, void* unused)
{
    uint32_t start_clock = GET_DIGIC_TIMER();

    if (!edmac_states || edmac_index >= LOG_SAMPLES)
    {
        /* finished */
        return;
    }

    /* schedule next call */
    SetHPTimerNextTick(last_expiry, log_interval, edmac_spy_poll, edmac_spy_poll, 0);

    /* this routine requires LCLK enabled (D4/D5 clock register) */
    if (!is_d8 && !(MEM(0xC0400008) & 0x2))
    {
        return;
    }

    for (uint32_t i = 0; i < num_log_regs; i++)
    {
        /* edmac_get_state/pointer(channel), just faster */
        uint32_t state = MEM(log_regs[i] + state_offset) & 1;
        uint32_t ptr   = MEM(log_regs[i] + pointer_offset);
        STATE(edmac_index, i) = ptr | (state << 31);

        uint32_t prev_state = (edmac_index)
            ? (STATE(edmac_index-1, i) & 0x80000000)
            : 0;

        if (state && !prev_state)
        {
            /* when a channel is starting, record some extra info,
             * such as connection and geometry
             * (this is slow, so we don't record it continuously)
             */

            if (edmac_extra_index < COUNT(edmac_extra_infos))
            {
                uint32_t ch = edmac_get_channel(log_regs[i]);
                struct edmac_extra_info * extra = &edmac_extra_infos[edmac_extra_index];
                extra->ch   = ch;
                extra->addr = edmac_get_address(ch);
                extra->conn = edmac_get_connection(ch, edmac_get_dir(ch));
                extra->cbr  = is_5d3 ? MEM(8 + 32*(ch) + MEM(0x12400)) : 0;  /* hardcoded for 5D3 */

                if (is_d8)
                {
                    extra->d8 = d8_get_geometry(ch);
                }
                else
                {
                    extra->info = edmac_get_info(ch);
                }

                edmac_extra_index++;
            }
        }
    }

    /* also store:
     * - microsecond timer
     * - current task name (pointer)
     * - estimated overhead for this routine (microseconds)
     * - extra index (for infos that are not logged continuously)
     *   (if it was 5 at edmax_index-1 and now it's 7,
     *    this step stored extra infos at indices 5 and 6)
     */
    STATE(edmac_index, CLK_IDX) = start_clock;
    STATE(edmac_index, TSK_IDX) = (uint32_t)get_current_task_name();
    STATE(edmac_index, OVH_IDX) = GET_DIGIC_TIMER() - start_clock;
    STATE(edmac_index, XTR_IDX) = edmac_extra_index;
    edmac_index++;
}

static const char * extra_info_size(struct edmac_extra_info * info)
{
    if (is_d8)
    {
        static char buf[96];
        snprintf(buf, sizeof(buf), "%s B=%x A=%x N=%x S=%x",
            d8_format_size(&info->d8),
            info->d8.yb_xb, info->d8.ya_xa, info->d8.yn_xn, info->d8.ys_xs
        );
        return buf;
    }

    return edmac_format_size(&info->info);
}

static void edmac_spy_dump()
{
    int len = 0;
    int maxlen = 8*1024*1024;
    char* out = fio_malloc(maxlen);
    if (!out)
    {
        NotifyBox(2000, "Out of memory");
        return;
    }
    memset(out, ' ', maxlen);

    uint32_t extra_index = 0;

    for (uint32_t i = 0; i < edmac_index; i++)
    {
        for (; extra_index < STATE(i, XTR_IDX); extra_index++)
        {
            struct edmac_extra_info * info = &edmac_extra_infos[extra_index];
            char * buf = out+len;
            len += snprintf(out+len, maxlen-len,
                "EDMAC#%d: addr=0x%x conn=%d cbr=%x name='''%s''' size='''%s'''\n",
                info->ch, info->addr, info->conn, info->cbr,
                asm_guess_func_name_from_string(info->cbr),
                extra_info_size(info)
            );

            /* remove any newlines (output easier to parse) */
            for (char * c = buf; c < out+len-1; c++)
            {
                if (*c == '\n')
                {
                    *c = ' ';
                }
            }
        }

        snprintf(out+len, maxlen-len, "%08X %s          ",
            STATE(i, CLK_IDX), (char *) STATE(i, TSK_IDX)
        );
        len += 16;

        for (uint32_t j = 0; j < num_log_regs; j++)
        {
            int ch = edmac_get_channel(log_regs[j]);
            len += snprintf(out+len, maxlen-len, " %d:%08X", ch, STATE(i, j));
        }

        len += snprintf(out+len, maxlen-len, " OVH:%d\n", STATE(i, OVH_IDX));
    }

    len += snprintf(out+len, maxlen-len, "\nSaved %d events, %d extra infos.\n", edmac_index, edmac_extra_index);

    NotifyBox(2000, "Saved %d events, %d extra infos.", edmac_index, edmac_extra_index);


    char log_filename[100];
    get_numbered_file_name("edmac%03d.log", 999, log_filename, sizeof(log_filename));
    save_mem_to_file(out, len, log_filename);
    free(out);

    edmac_index = 0;
    edmac_extra_index = 0;
}

static void log_edmac_usage()
{
    if (!log_regs)
    {
        NotifyBox(2000, "Out of memory");
        return;
    }

    if (is_d8)
    {
        /* only the probe channel is safe to read, see probe_channel */
        log_regs[0] = edmac_get_base(probe_channel);
        num_log_regs = 1;
    }

    edmac_states = fio_malloc(LOG_SAMPLES * ROW_WORDS * sizeof(edmac_states[0]));
    if (!edmac_states)
    {
        NotifyBox(2000, "Out of memory");
        return;
    }
    edmac_index = 0;
    edmac_extra_index = 0;

    NotifyBox(100000, "Press shutter halfway to start.");
    while (!get_halfshutter_pressed())
    {
        msleep(10);
    }

    NotifyBox(1000000, "Logging EDMAC usage...");

    SetHPTimerAfterNow(1000, edmac_spy_poll, edmac_spy_poll, 0);

    /* wait until buffer full */
    while (edmac_index < LOG_SAMPLES)
    {
        info_led_on();
        msleep(100);
    }
    info_led_off();

    NotifyBox(2000, "Saving log...");
    edmac_spy_dump();

    uint32_t * states = edmac_states;
    edmac_states = 0;
    free(states);
}

static MENU_UPDATE_FUNC(log_interval_update)
{
    int log_duration = (int)roundf((float) log_interval * LOG_SAMPLES / 1e6 * 100.0);

    MENU_SET_WARNING(MENU_WARN_INFO,
        "Will log %d samples = %s%d.%02d seconds.",
        LOG_SAMPLES, FMT_FIXEDPOINT2(log_duration)
    );
}

/* D8: the probe channel only.  Registers are read one at a time, offsets
 * Canon itself reads first, each drawn on screen beforehand (bmp_printf
 * draws immediately; the console only refreshes later), so if one hangs
 * the camera the screen shows which.
 * Canon reads +0xEC (current pointer), +0x30 (irq status), +0xB4, +0xC4;
 * the rest it only writes, as far as seen. */
static void save_register_snapshot_d8()
{
    static const uint8_t canon_first[] = {
        0xEC, 0x30, 0xB4, 0xC4,
        0xA0, 0x48, 0x4C, 0x50, 0x54, 0x58, 0x5C, 0x60, 0x64, 0x68, 0x6C, 0x70,
        0x04, 0x08, 0xC0, 0xD0,
    };

    uint32_t ch = probe_channel;
    uint32_t base = edmac_get_base(ch);
    uint32_t regs[0x40] = {0};
    int read_ok[0x40] = {0};

    console_show();
    printf("EDMAC #%d at %x (%s)\n", ch, base, dir_name(edmac_get_dir(ch)));

    for (uint32_t k = 0; k < COUNT(canon_first) + 0x40; k++)
    {
        int off = (k < COUNT(canon_first)) ? canon_first[k] : 4 * (k - COUNT(canon_first));
        if (read_ok[off / 4])
        {
            continue;
        }
        /* stays on screen if the read hangs */
        bmp_printf(FONT(FONT_LARGE, COLOR_YELLOW, COLOR_BLACK), 20, 400,
            "EDMAC #%d reading +%02x (%08x)...   ", ch, off, base + off);
        regs[off / 4] = MEM(base + off);
        read_ok[off / 4] = 1;
        printf("+%02x = %08x\n", off, regs[off / 4]);
    }

    /* the channel's pack unit holds the bit depth; Canon RMWs this register */
    uint32_t pack_reg = edmac_get_pack_reg(ch);
    uint32_t pack = 0;
    if (pack_reg)
    {
        bmp_printf(FONT(FONT_LARGE, COLOR_YELLOW, COLOR_BLACK), 20, 400,
            "EDMAC #%d reading pack (%08x)...   ", ch, pack_reg);
        pack = MEM(pack_reg);
        printf("pack %x = %08x\n", pack_reg, pack);
    }

    char filename[100];
    int num = get_numbered_file_name("edsnp%03d.log", 999, filename, sizeof(filename));
    FILE * f = FIO_CreateFile(filename);
    if (f)
    {
        my_fprintf(f, "EDMAC #%d base=%08x dir=%s\n", ch, base, dir_name(edmac_get_dir(ch)));
        my_fprintf(f, "addr=%08x ptr=%08x run=%x mode=%x xfer=%x pack=%x\n",
            regs[0xA0/4], regs[0xEC/4], regs[0x04/4], regs[0x08/4], regs[0xC0/4], regs[0xD0/4]);
        my_fprintf(f, "S=%08x A=%08x B=%08x N=%08x (B: %d bytes x %d rows)\n",
            regs[0x48/4], regs[0x4C/4], regs[0x50/4], regs[0x54/4],
            LO16(regs[0x50/4]), HI16(regs[0x50/4]) + 1);
        /* L-class channels keep xa/xb at +0x58/+0x5C instead */
        my_fprintf(f, "+58/+5C (xa/xb on L channels) = %d / %d bytes; pointer - address = %d\n",
            regs[0x58/4], regs[0x5C/4], regs[0xEC/4] - regs[0xA0/4]);
        if (pack_reg)
        {
            /* bits 4-5: 0/1/2 = 10/12/14 bit; bit 0: LSB-first, bit 20:
             * ML's 16-bit words (neither: unpacked) */
            my_fprintf(f, "pack reg %08x = %08x (depth code %d, packing %s)\n",
                pack_reg, pack, (pack >> 4) & 3, (pack & 1) ? "on" : "off");
        }
        for (int i = 0; i < 0x40; i += 4)
        {
            my_fprintf(f, "+%02x: %08x %08x %08x %08x\n",
                4 * i, regs[i], regs[i+1], regs[i+2], regs[i+3]);
        }
        FIO_CloseFile(f);
    }

    NotifyBox(2000, "Saved edsnp%03d.log", num);
}

/* D8: save the frame the probe channel last wrote, for offline analysis of
 * the raw packing.  Size from its registers: rows from yb (+0x50 high),
 * row bytes from xb (+0x50 low, or +0x5C on L channels).  The buffer is
 * plain RAM, so this reads no further engine registers.  Copied first,
 * since the channel keeps writing into it. */
static void save_probe_buffer()
{
    if (!is_d8)
    {
        NotifyBox(3000, "DIGIC 8 only.");
        return;
    }

    uint32_t ch = probe_channel;
    uint32_t base = edmac_get_base(ch);
    uint32_t addr = MEM(base + 0xA0);
    uint32_t yb_xb = MEM(base + 0x50);
    uint32_t xb = LO16(yb_xb) ? LO16(yb_xb) : MEM(base + 0x5C);
    uint32_t rows = HI16(yb_xb) + 1;
    uint32_t size = xb * rows;
    uint32_t pack_reg = edmac_get_pack_reg(ch);
    uint32_t pack = pack_reg ? MEM(pack_reg) : 0;

    if (!addr || !xb || size > 64*1024*1024)
    {
        NotifyBox(3000, "No sane buffer: addr %x, %d x %d", addr, xb, rows);
        return;
    }

    void * copy = fio_malloc(size);
    if (!copy)
    {
        NotifyBox(3000, "Out of memory (%s)", format_memory_size(size));
        return;
    }
    memcpy(copy, UNCACHEABLE(addr), size);

    char filename[100];
    int num = get_numbered_file_name("edbuf%03d.bin", 999, filename, sizeof(filename));
    save_mem_to_file(copy, size, filename);
    free(copy);

    snprintf(filename, sizeof(filename), "edbuf%03d.log", num);
    FILE * f = FIO_CreateFile(filename);
    if (f)
    {
        my_fprintf(f, "EDMAC #%d addr=%08x xb=%d rows=%d size=%d pack=%08x (depth code %d, packing %s)\n",
            ch, addr, xb, rows, size, pack, (pack >> 4) & 3, (pack & 1) ? "on" : "off");
        my_fprintf(f, "lv=%d movie=%d\n", lv, is_movie_mode());
        FIO_CloseFile(f);
    }

    NotifyBox(3000, "Saved edbuf%03d: %d x %d", num, xb, rows);
}

/* One-shot snapshot of every channel: a readable summary.
 * Meant for finding image buffers, e.g. take one with LV raw on and look
 * for a raw-sized write.  D8 reads only the probe channel, see above. */
static void save_register_snapshot()
{
    if (is_d8)
    {
        save_register_snapshot_d8();
        return;
    }

    int maxlen = 64*1024;
    char * out = fio_malloc(maxlen);
    if (!out)
    {
        NotifyBox(2000, "Out of memory");
        return;
    }

    int len = 0;
    for (uint32_t ch = 0; ch < num_channels; ch++)
    {
        uint32_t dir = edmac_get_dir(ch);
        if (dir == EDMAC_DIR_UNUSED)
        {
            continue;
        }

        len += snprintf(out+len, maxlen-len,
            "#%02d %-5s base=%08x addr=%08x ptr=%08x state=%x conn=%x size=%s",
            ch, dir_name(dir), edmac_get_base(ch), edmac_get_address(ch),
            edmac_get_pointer(ch), edmac_get_state(ch),
            edmac_get_connection(ch, dir), channel_size_short(ch)
        );

        len += snprintf(out+len, maxlen-len, "\n");
    }

    char filename[100];
    int num = get_numbered_file_name("edsnp%03d.log", 999, filename, sizeof(filename));
    save_mem_to_file(out, len, filename);
    free(out);

    NotifyBox(2000, "Saved edsnp%03d", num);
}

/* Turn Canon's LiveView raw stream on or off, as raw.c does,
 * so a snapshot can show which channel carries it. */
static int lv_raw_stream = 0;

static MENU_SELECT_FUNC(lv_raw_stream_toggle)
{
    lv_raw_stream = !lv_raw_stream;

    if (lv_raw_stream && is_d8)
    {
        /* on D8, lv_save_raw saves YUV unless lv_set_mm selects RAW */
        call("lv_set_mm", 1);
    }
    call("lv_save_raw", lv_raw_stream);
}

/* LV raw bit depth (raw.c, applied once per frame on D8) */
extern WEAK_FUNC(ret_0) void raw_lv_request_bpp(int bpp);
extern WEAK_FUNC(ret_0) int raw_lv_get_bpp_reverts();
static int lv_raw_depth = 0;

static MENU_SELECT_FUNC(lv_raw_depth_toggle)
{
    lv_raw_depth = !lv_raw_depth;
    /* 14 = "most the camera can do", i.e. Canon's depth on D8 */
    raw_lv_request_bpp(lv_raw_depth ? 10 : 14);
}

static MENU_UPDATE_FUNC(lv_raw_depth_update)
{
    if (lv_raw_depth)
    {
        MENU_SET_RINFO("Canon resets: %d", raw_lv_get_bpp_reverts());
    }
}

/* the mlv_lite copy path; not every model provides it */
extern WEAK_FUNC(ret_0) void* edmac_copy_rectangle_cbr_start(void *dst, void *src,
    int src_width, int src_x, int src_y, int dst_width, int dst_x, int dst_y,
    int w, int h, void (*cbr_r)(void *), void (*cbr_w)(void *), void *cbr_ctx);
extern WEAK_FUNC(ret_0) void edmac_copy_rectangle_adv_cleanup();

static void m2m_test_cbr_r(void *ctx)
{
}

/* same as mlv_lite's edmac_cbr_w */
static void m2m_test_cbr_w(void *ctx)
{
    *(volatile int *)ctx = 1;
    edmac_copy_rectangle_adv_cleanup();
}

/* Copy a cropped rectangle out of a test pattern through
 * edmac_copy_rectangle_cbr_start(), the call mlv_lite uses, and check
 * every byte.  D8 only: there it is blocking; D4/D5 is asynchronous. */
static void m2m_copy_test()
{
    if (!is_d8)
    {
        NotifyBox(3000, "DIGIC 8 only.");
        return;
    }

    /* bytes; the crop is offset in both directions to test off1b */
    const int src_w = 4096, src_h = 64;
    const int x = 256, y = 8, w = 1024, h = 32;

    uint8_t * src_buf = fio_malloc(src_w * src_h);
    uint8_t * dst_buf = fio_malloc(w * h);
    if (!src_buf || !dst_buf)
    {
        NotifyBox(2000, "Out of memory");
        if (src_buf) free(src_buf);
        if (dst_buf) free(dst_buf);
        return;
    }

    /* cacheable source exercises the cache sync, as in mlv_lite */
    uint8_t * src = CACHEABLE(src_buf);
    uint8_t * dst = UNCACHEABLE(dst_buf);

    for (int i = 0; i < src_w * src_h; i++)
    {
        src[i] = (i * 7 + i / src_w) & 0xFF;
    }
    memset(dst, 0xAA, w * h);

    console_show();
    printf("m2m copy test: %dx%d from (%d,%d) of %dx%d\n", w, h, x, y, src_w, src_h);

    volatile int done = 0;
    int t0 = get_ms_clock();
    void * ret = edmac_copy_rectangle_cbr_start(
        dst, src,
        src_w, x, y,
        w, 0, 0,
        w, h,
        m2m_test_cbr_r, m2m_test_cbr_w, (void *) &done
    );
    int t1 = get_ms_clock();

    int errors = 0, first = -1;
    for (int row = 0; row < h; row++)
    {
        for (int col = 0; col < w; col++)
        {
            if (dst[row * w + col] != src[(y + row) * src_w + x + col])
            {
                if (first < 0) first = row * w + col;
                errors++;
            }
        }
    }

    printf("returned %x, cbr %s, %d ms\n", ret, done ? "called" : "NOT called", t1 - t0);
    if (errors)
    {
        printf("FAILED: %d of %d bytes differ, first at row %d col %d (got %02x, want %02x)\n",
            errors, w * h, first / w, first % w,
            dst[first], src[(y + first / w) * src_w + x + first % w]);
    }
    else
    {
        printf("OK: all %d bytes match\n", w * h);
    }

    free(src_buf);
    free(dst_buf);
}

/* edmac_test.c */
extern void edmac_test();

static void edmac_model_test()
{
    if (is_d8)
    {
        /* edmac_test.c uses struct edmac_info, which has the D4/D5 layout here */
        NotifyBox(3000, "Not available on DIGIC 8 yet.");
        return;
    }

    edmac_test();
}

static struct menu_entry edmac_menu[] =
{
    {
        .name   = "EDMAC tools",
        .select = menu_open_submenu,
        .children =  (struct menu_entry[]) {
            {
                .name       = "Show EDMAC channels",
                .select     = menu_open_submenu,
                .icon_type  = IT_ACTION,
                .help       = "Useful for finding image buffers.",
                .children =  (struct menu_entry[]) {
                    {
                        .name   = "EDMAC display",
                        .priv   = &edmac_selection,
                        .max    = 49,   /* updated at startup */
                        .update = edmac_display,
                    },
                    MENU_EOL
                },
            },
            {
                .name   = "Probe channel",
                .priv   = &probe_channel,
                .min    = 0,
                .max    = 75,   /* updated at startup */
                .help   = "DIGIC 8: the only channel whose registers are read.",
                .help2  = "Reading a powered-down engine hangs the camera.",
            },
            {
                .name   = "Save register snapshot",
                .select = run_in_separate_task,
                .priv   = save_register_snapshot,
                .help   = "Save state, address and size of every channel to a file.",
                .help2  = "On DIGIC 8: the probe channel's registers, shown as read.",
            },
            {
                .name   = "Save probe channel buffer",
                .select = run_in_separate_task,
                .priv   = save_probe_buffer,
                .help   = "Save the frame the probe channel last wrote (raw .bin + .log).",
                .help2  = "DIGIC 8 only. For working out the raw packing offline.",
            },
            {
                .name   = "LV raw stream",
                .priv   = &lv_raw_stream,
                .max    = 1,
                .select = lv_raw_stream_toggle,
                .help   = "Ask Canon firmware to output raw in LiveView (lv_save_raw).",
                .help2  = "Take a snapshot with this on and off to find the raw channel.",
            },
            {
                .name   = "LV raw depth",
                .priv   = &lv_raw_depth,
                .max    = 1,
                .select = lv_raw_depth_toggle,
                .update = lv_raw_depth_update,
                .choices = CHOICES("Canon", "10-bit"),
                .help   = "DIGIC 8, EXPERIMENTAL: switch the LV raw channel to 10-bit.",
                .help2  = "Sets the pack register and xb each frame. Check with a probe buffer.",
            },
            {
                .name   = "Mem-to-mem copy test",
                .select = run_in_separate_task,
                .priv   = m2m_copy_test,
                .help   = "Copy a test pattern the way mlv_lite does, and check it.",
                .help2  = "DIGIC 8 only. Results on the console.",
            },
            {
                .name   = "Find free EDMAC channels",
                .select = run_in_separate_task,
                .priv   = find_free_edmac_channels,
                .help   = "Useful to find which channels can be used in LiveView.\n",
            },
            {
                .name   = "Log EDMAC activity",
                .select = menu_open_submenu,
                .help   = "Sample EDMAC activity on all channels and save a log file.",
                .help2  = "Useful for figuring out how image capture/processing works.",
                .children =  (struct menu_entry[]) {
                    {
                        .name   = "Start logging",
                        .select = run_in_separate_task,
                        .priv   = log_edmac_usage,
                        .help   = "Start logging EDMAC activity.",
                        .help2  = "Press shutter halfway to choose the exact moment.",
                    },
                    {
                        .name       = "Log every",
                        .priv       = &log_interval,
                        .min        = 50,
                        .max        = 10000,
                        .update     = log_interval_update,
                        .unit       = UNIT_TIME_US,
                        .edit_mode  = EM_ROUND_1_2_5_10,
                        .help       = "Sampling interval (how often EDMAC channels are polled).",
                        .help2      = "The logging buffer size is fixed at 2048 samples.",
                    },
                    MENU_EOL
                },
            },
            {
                .name   = "EDMAC model test",
                .select = run_in_separate_task,
                .priv   = edmac_model_test,
                .help   = "Tests to confirm our hypothesis on how EDMAC works.",
                .help2  = "Should be executed on both camera and QEMU."
            },
            MENU_EOL
        }
    },
};

static unsigned int edmac_init()
{
    is_5d3 = is_camera("5D3", "*");
    is_d8 = get_digic_version() == 8;
    num_channels = edmac_get_num_channels();
    edmac_regs_init();

    /* one display screen per overview page, connection page and channel */
    edmac_menu[0].children[0].children[0].max =
        overview_screens() + conn_screens() + num_channels - 1;
    edmac_menu[0].children[1].max = num_channels - 1;   /* Probe channel */

    menu_add("Debug", edmac_menu, COUNT(edmac_menu));
    return 0;
}

static unsigned int edmac_deinit()
{
    menu_remove("Debug", edmac_menu, COUNT(edmac_menu));
    free(log_regs);
    log_regs = 0;
    return 0;
}

MODULE_INFO_START()
    MODULE_INIT(edmac_init)
    MODULE_DEINIT(edmac_deinit)
MODULE_INFO_END()

MODULE_CONFIGS_START()
    MODULE_CONFIG(log_interval)
MODULE_CONFIGS_END()
