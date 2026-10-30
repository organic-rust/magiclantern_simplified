/* M6II: LiveView in movie mode with the mode dial on photo, for raw video
 * started with REC in photo mode (mlv_lite).
 *
 * Canon's own REC in photo mode sets PROP_FIXED_MOVIE and restarts
 * LiveView in movie mode, but starts its H.264 recording with it.  Setting
 * the property alone (or with Canon's lv_set_fixed_movie) leaves LiveView
 * as it is; a LiveView stop and start through PROP_LV_ACTION (1, then 0)
 * brings it back in the mode the property says: movie LiveView (sensor
 * slot 15 at 1080p), ML's lv set, nothing recording.  Checked over USB.
 *
 * The sequence is the one that worked over USB, step for step: Canon's
 * lv_set_fixed_movie, for movie mode also the LiveView object calls its
 * photo path makes (0xE04F9A86, 0xE077888C with 4 and 1), the stop, 1.5 s,
 * the start.  LiveView then gets 2 s to settle before anything else
 * (a take locks Canon's GUI, whose acks LiveView's start-up needs: a take
 * started straight away left LiveView stopped and the display frozen).
 *
 * PROP_LV_ACTION goes straight to Canon's _prop_request_change: it stays
 * off ML's write allow-list, where it would wake up PauseLiveView /
 * ResumeLiveView all over ML (no-ops on this port so far). */

#include "dryos.h"
#include "property.h"
#include "propvalues.h"

extern void lv_set_fixed_movie(int *movie);
extern void _prop_request_change(unsigned property, const void *addr, size_t len);

#define SEQ_MODE        (0x10D88 + 8)   /* LiveView's sensor slot */
#define LV_OBJ          MEM(0x969C + 4) /* LiveView object lv_set_fixed_movie uses */

/* calls on the LiveView object, as lv_set_fixed_movie's photo path makes
 * them (Canon ROM, Thumb) */
static void (*const lv_obj_prepare)(uint32_t obj) = (void *) 0xE04F9A87;
static int (*const lv_obj_event)(uint32_t obj, int ev, int a, int b) = (void *) 0xE077888D;

#define SETTLE_MS       2000

static void lv_action(int stop)
{
    int x = stop;
    _prop_request_change(PROP_LV_ACTION, &x, 4);
}

/* each step to the console: a switch that stalls shows where */
static int lvm_t0 = 0;
static void lvm_log(const char *what)
{
    printf("lv_mode %5d ms: %s (lv %d slot %d movie %d)\n", get_ms_clock() - lvm_t0, what,
           lv, MEM(SEQ_MODE), is_movie_mode());
}

static int lvm_wait_started(int slot0)
{
    for (int t = 0; t < 3000; t += 20)
    {
        if (lv && (int) MEM(SEQ_MODE) != slot0)
        {
            return 1;
        }
        msleep(20);
    }
    return 0;
}

/* 1 when LiveView runs in the mode asked for (movie: 1, photo: 0), in a
 * slot other than before; takes ~5-8 s.  Not while recording; from a task
 * that may wait, with Canon's GUI not locked. */
int lv_set_movie_mode(int movie)
{
    if (RECORDING)
    {
        return 0;
    }
    movie = movie ? 1 : 0;
    lvm_t0 = get_ms_clock();
    lvm_log(movie ? "to movie: start" : "to photo: start");
    int slot0 = MEM(SEQ_MODE);

    lv_set_fixed_movie(&movie);
    if (movie)
    {
        /* lv_set_fixed_movie makes these itself only on its photo path */
        uint32_t obj = LV_OBJ;
        if (obj)
        {
            lv_obj_prepare(obj);
            lv_obj_event(obj, 4, 0, 0);
            msleep(100);
            lv_obj_event(obj, 1, 0, 0);
        }
    }
    lvm_log("property set");

    /* restart LiveView: it starts in the mode the property says */
    lv_action(1);
    lvm_log("LiveView stop sent");
    msleep(1500);
    lv_action(0);
    lvm_log("LiveView start sent");

    int started = lvm_wait_started(slot0);
    if (!started)
    {
        /* not back in the other slot: one more stop / start */
        lvm_log("not restarted: once more");
        lv_action(1);
        msleep(1500);
        lv_action(0);
        started = lvm_wait_started(slot0);
    }
    lvm_log(started ? "LiveView restarted" : "LiveView NOT restarted");

    /* settle (and is_movie_mode() follows PROP_FIXED_MOVIE after 300 ms) */
    msleep(SETTLE_MS);
    int ok = started && lv && is_movie_mode() == movie;
    lvm_log(ok ? "done: ok" : "done: FAILED");
    return ok;
}
