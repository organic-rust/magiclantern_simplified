#ifndef _crx_enc_h_
#define _crx_enc_h_

/* Canon's CR3 (CRX) encoder, driven through AlgsMgr as Canon's stills do
 * (CONFIG_CRX_ENCODER).
 *
 * Input: raw rows packed LSB-first at 10/12/14 bit (the D8 layout, as the
 * LV raw stream delivers it for CRX takes), any pitch; output: one CRX
 * sample (tile, plane and subband headers + 4 plane bitstreams: lossless,
 * or Canon's C-RAW with 3 DWT levels), as stored in a CR3 track.  A
 * reader rebuilds the CMP1 parameters from width, height and bits.  1:1
 * takes use Canon's own 2-tile jobs instead (preshoot.h). */

/* sample header for 1 tile, 4 planes, 0 wavelet levels */
#define CRX_SAMPLE_HDR_SIZE 112

/* 1 if this camera has the encoder */
int crx_enc_available(void);

/* 1 if Canon has set the encoder up (AlgsMgr running) */
int crx_enc_ready(void);

/* call before the first frame of a size/depth; the encoder setup runs
 * with these parameters on the next frame */
/* lossy: Canon's C-RAW (3 DWT levels) instead of lossless */
int crx_enc_setup(int width, int height, int bits, int pitch_px, int lossy);

/* encode one frame from src (raw rows at pitch_px), starting x_off pixels
 * into each row (even; src itself 4-byte aligned), write the sample to
 * out (uncacheable); returns its size, or a negative error.
 * enc_us / copy_us: time in the encoder / copying its output (may be NULL) */
int crx_enc_frame(void *src, int x_off, void *out, int out_max, int *enc_us, int *copy_us);

/* recording finished (no encode running): Canon sets its encoders up for
 * its own sizes again; without this its next CR3 still finds encoder 1
 * set up for our frames */
void crx_enc_done(void);

/* copy the non-empty parts 1..n-1 of an encoder result (addresses and
 * sizes as AlgsMgr's result handle lists them; entry 0 is Canon's sample
 * header) one after another to dst, by scatter-gather DMA where it works;
 * returns the bytes copied.  dst must be reachable by DMA (first GB). */
int crx_copy_parts(uint8_t *dst, uint32_t n, const uint32_t *addrs, const uint32_t *sizes);

/* decode one sample (as crx_enc_frame writes it, lossless or C-RAW) with
 * Canon's decoder into rows of pitch_px pixels at `bits`, LSB-first.  The
 * first subband (sample + header size: 0x70 lossless, 0x220 C-RAW) and out
 * must be 128-byte aligned; the sample is rewritten in place (subbands
 * padded to 128 bytes) and needs CRX_DEC_SPARE bytes after it.  0 on
 * success. */
#define CRX_DEC_SPARE (39 * 127)
int crx_dec_frame(void *sample, int size, void *out, int width, int height, int bits, int pitch_px);

/* playback finished: hand the decoder slot back to Canon */
void crx_dec_done(void);

#endif
