#ifndef U_TMC_H
#define U_TMC_H

#include <linux/usb/composite.h>


struct f_tmc_opts {
	struct usb_function_instance	func_inst;
	int				minor;
	int				port_num;
	
	/*
	 * Protect the data from concurrent access by read/write
	 * and create symlink/remove symlink
	 */
	struct mutex			lock;
	int				refcnt;
};

#endif /* U_TMC_H */
