/* Lossless raw compression */

#ifndef _lossless_h_
#define _lossless_h_

/* to be called before anything else; 1 = success */
int lossless_init();

/* returns output size if successful, negative on error */
int lossless_compress_raw(struct raw_info * raw_info, struct memSuite * output_memsuite);

/* compress a rectangle cropped from a source buffer */
/* similar to edmac_copy_rectangle */
int lossless_compress_raw_rectangle(
    struct memSuite * dst_suite, void * src,
    int src_width, int src_x, int src_y,
    int width, int height
);

/* layout of the compressed frames, for cameras whose encoder doesn't produce
 * a plain raster LJ92 stream (5D2: CR2-style slices).
 * Returns 1 and fills *layout if the output is sliced, 0 otherwise. */
struct lossless_layout
{
    int slice_count;        /* number of slices of slice_width (CR2 tag 0xC640, [0]) */
    int slice_width;        /* [1] */
    int last_slice_width;   /* [2]: one more slice of this width at the right */
    int crop_top;           /* rows of the input buffer the encoder skips at the top */
    int width, height;      /* frame size in the stream */
};
int lossless_get_layout(struct lossless_layout * layout);

int lossless_decompress_raw(
    struct memSuite * src, void * dst,
    int width, int height,
    int output_bpp
);


#endif
