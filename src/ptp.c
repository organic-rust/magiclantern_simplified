#include "ptp.h"
#include "dryos.h"

int recv_ptp_data(struct ptp_context *data, char *buf, int size)
	// repeated calls per transaction are ok
{
	while ( size >= BUF_SIZE )
	{
		data->recv_data(data->handle,buf,BUF_SIZE,0,0);
		// XXX check for success??

		size -= BUF_SIZE;
		buf += BUF_SIZE;
	}
	if ( size != 0 )
	{
		data->recv_data(data->handle,buf,size,0,0);
		// XXX check for success??
	}

	return 1;
}

int send_ptp_data(struct ptp_context *data, const char *buf, int size)
	// repeated calls per transaction are *not* ok
{
	int tmpsize;

	tmpsize = size;
	while ( size >= BUF_SIZE )
	{
		if ( data->send_data(data->handle,(void *)buf,BUF_SIZE,tmpsize,0,0,0) )
		{
			return 0;
		}

		tmpsize = 0;
		size -= BUF_SIZE;
		buf += BUF_SIZE;
	}
	if ( size != 0 )
	{
		if ( data->send_data(data->handle,(void *)buf,size,tmpsize,0,0,0) )
		{
			return 0;
		}
	}

	return 1;
}

uint32_t ptp_register_all_handlers()
{
	uint32_t ret = 0x0;
	
	extern struct ptp_handler _ptp_handlers_start[];
	extern struct ptp_handler _ptp_handlers_end[];

	struct ptp_handler * handler = _ptp_handlers_start;

	for( ; handler < _ptp_handlers_end ; handler++ )
	{
#if defined(POSITION_INDEPENDENT)
        handler->handler = PIC_RESOLVE(handler->handler);
        handler->priv = PIC_RESOLVE(handler->priv);
#endif
		//DebugMsg("[ML] PTP_INIT reg: id=0x%08X h=0x%08X p=0x%08X", handler->id, handler->handler, handler->priv);		
		ret |= ptp_register_handler(
				handler->id,
				handler->handler,
				handler->priv
				);
	}

	return ret;
}

/* PTP mailbox: fixed, exported, so a host can find it in
 * magiclantern.sym and read module data (whose addresses vary) with
 * PTP_CHDK_GetMemory.  Slots are claimed by name. */
struct ptp_mailbox ptp_mailbox = { .magic = PTP_MAILBOX_MAGIC };

void ptp_mailbox_set(int slot, const char *name, void *ptr, volatile uint32_t *len)
{
    if (slot < 0 || slot >= PTP_MAILBOX_SLOTS)
        return;
    ptp_mailbox.slot[slot].ptr = ptr;
    ptp_mailbox.slot[slot].len = (uint32_t *)len;
    snprintf(ptp_mailbox.slot[slot].name, sizeof(ptp_mailbox.slot[slot].name), "%s", name ? name : "");
}

static void ptp_init( void *unused )
{
#ifndef CONFIG_40D 
#ifdef CONFIG_DIGIC_8X
	/* D8 (M6II): the registration wrapper returns 0x17 until Canon has
	 * set up its operation list; retry for a while */
	for (int i = 0; i < 20; i++)
	{
		uint32_t ret = ptp_register_all_handlers();
		if (ret == 0 || ret == 0x13)    /* 0x13: already registered */
			break;
		msleep(1000);
	}
#else
	ptp_register_all_handlers();
#endif
#endif
}

INIT_FUNC( __FILE__, ptp_init );
