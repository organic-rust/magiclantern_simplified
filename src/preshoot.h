#ifndef _preshoot_h_
#define _preshoot_h_

/* 1:1 crop takes (platform preshoot.c, CONFIG_PRESHOOT_1TO1): Canon's 1:1
 * sensor mode started at REC, each frame's crop encoded by Canon's CR3
 * encoder.  A sensor-mode module (d8_modes) chooses the source, the band
 * the crop may use and the rate with preshoot_take_config; the recorder
 * (mlv_lite) reads them with preshoot_take_get and cuts the crop from the
 * band with its own resolution, aspect ratio, bit depth and compression
 * settings.  It reports the crop with preshoot_take_set_crop, for the
 * sensor-mode module's cropmark. */
struct preshoot_take
{
    int on;                     /* the next take records the 1:1 source */
    int x0, y0, width, height;  /* the band in the 1:1 frame (its Readout) */
    int fps_24p;                /* 23.976 instead of Canon's 29.97 */
};

void preshoot_take_config(const struct preshoot_take *take);
int preshoot_take_get(struct preshoot_take *take);    /* 1 if on */
void preshoot_take_set_crop(int width, int height);   /* centred in the band */
int preshoot_take_get_crop(int *width, int *height);  /* 0 if none reported */

/* Canon's 1:1 mode alone (PoCom MechalessStart / Stop; 0 on success), and
 * our own encodes of the crop from EDMAC ch 1.  bits: 14 (Canon's), 12 or
 * 10 (EDMAC ch 1 switched every frame; frames Canon wrote at another depth
 * are dropped and counted).  preshoot_movie_frame() once per LiveView
 * frame starts one (1) unless the last is still running or the 1:1 mode
 * isn't on yet (0); frame_cbr(handle) runs on AlgsMgr's task with each
 * result and returns 1 if it keeps it, which it then gives back with
 * preshoot_movie_release() after copying the parts, in order; 0 frees it
 * at once. */
int preshoot_mode_start(void);
void preshoot_mode_stop(void);
int preshoot_movie_enable(int x0, int width, int y0, int height, int bits, int (*frame_cbr)(uint32_t handle));
void preshoot_movie_disable(void);
int preshoot_movie_frame(void);
void preshoot_movie_release(uint32_t handle);
/* started, skipped (encoder busy), failed, dropped for the depth, encoder
 * set-ups, of the last (or current) take */
void preshoot_movie_stats(int *started, int *skipped, int *failed, int *wrong_depth, int *resets);
/* ch 1's writer row (in the full frame, -1 = outside it) at vsync, a copy
 * of that, after the encode start and at its end (min, mean, max each),
 * then the encode time max and mean (us), in v[14]; returns the frames
 * measured.  The encoder reads a frame while the next one is written into
 * the same buffer: this shows how close the writer gets. */
int preshoot_movie_race(int *v);
/* before preshoot_movie_enable: 1 = lossless (Canon's full RAW job, type 0
 * = encoder 0, as its lossless stills: 2 tiles, encType 0, 0 DWT levels),
 * 0 = C-RAW (the RAW burst's job, type 9 = encoder 0x12, 2 tiles) */
void preshoot_movie_set_lossless(int on);
/* Canon's 1:1 mode at 23.976 (on) or its own 29.97; applies from the next
 * mode start (not while it runs).  Returns that frame rate x 1000. */
int preshoot_set_23976(int on);
/* Canon's RAW burst (0) / its pre-shooting (1) setting: 0 off, 1 on, -1 not
 * reported yet (dmlog's tests) */
int preshoot_get_setting(int prerec);

#endif
