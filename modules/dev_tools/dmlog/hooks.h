#include <stdint.h>

/* in dmlog.c */
void dmlog_EncInit(uint32_t id, const void *params);
void dmlog_EncSetParam(uint32_t id, const void *block, const void *ctx);

/* Thumb entry stubs (hooks_thumb_M6II.c): log, then run the 8 bytes of
 * Canon's code the hook replaced and jump back */
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_EncInit_M6II(void);
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_EncSetParam_M6II(void);

/* EDMAC setup logging (dmlog.c) and Thumb entry stubs */
void dmlog_edmac_read_pack(uint32_t ch, const uint32_t *fields, uint32_t lr, uint32_t kind);
void dmlog_edmac_set_size(uint32_t ch, const uint32_t *info, uint32_t lr, uint32_t kind);
void dmlog_edmac_set_addr(uint32_t ch, uint32_t addr);
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_ReadPack_M6II(void);
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_SetSize_M6II(void);
void dmlog_edmac_start(uint32_t ch, uint32_t lr);
void dmlog_edmac_connect(uint32_t ch, uint32_t conn, uint32_t lr, uint32_t kind);
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_Start_M6II(void);
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_ConnectWrite_M6II(void);
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_ConnectRead_M6II(void);
uint32_t dmlog_StartEncode(uint32_t *jobs, uint32_t n, uint32_t cbr, uint32_t ctx);
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_StartEncode_M6II(void);

/* CMOS (sensor register) writers: logging (dmlog.c) and Thumb entry stubs */
void dmlog_cmos_pio(const uint32_t *frames, uint32_t unused, uint32_t lr);
void dmlog_cmos_dma(const uint16_t *data, uint32_t n, uint32_t lr);
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_CmosPio_M6II(void);
void __attribute__((noreturn,noinline,naked,aligned(4))) hook_CmosDma_M6II(void);
