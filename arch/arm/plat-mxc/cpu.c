
#include <linux/module.h>

unsigned int __mxc_cpu_type;
EXPORT_SYMBOL(__mxc_cpu_type);

void mxc_set_cpu_type(unsigned int type)
{
	__mxc_cpu_type = type;
}

int mxc_jtag_enabled;		/* OFF: 0 (default), ON: 1 */
int uart_at_24; 			/* OFF: 0 (default); ON: 1 */
/*
 * Here are the JTAG options from the command line. By default JTAG
 * is OFF which means JTAG is not connected and WFI is enabled
 *
 *       "on" --  JTAG is connected, so WFI is disabled
 *       "off" -- JTAG is disconnected, so WFI is enabled
 */

static int __init jtag_wfi_setup(char *p)
{
	if (memcmp(p, "on", 2) == 0) {
		mxc_jtag_enabled = 1;
		p += 2;
	} else if (memcmp(p, "off", 3) == 0) {
		mxc_jtag_enabled = 0;
		p += 3;
	}
	return 0;
}
early_param("jtag", jtag_wfi_setup);


