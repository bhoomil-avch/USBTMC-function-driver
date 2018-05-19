/*
 * f_usbtmc.c - USBTMC function driver
 *
 * usbtmc.c -- USBTMC gadget driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/idr.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/cdev.h>

#include <asm/byteorder.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <asm/unaligned.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/tmc.h>

#include "u_tmc.h"

#define TMC_DEBUG
#define USBTMC_MINORS			4
#define USBTMC_CLASS			3
#define USBTMC_SUBCLASS_STANDARD	0
#define USBTMC_SUBCLASS_USB488		1


#define USB_TMC_HEADER_SIZE			12  /* length of a USBTMC header */

/* USBTMC MsgID. Values */
#define DEV_DEP_MSG_OUT			1   /* device dependent command message */
#define REQUEST_DEV_DEP_MSG_IN		2   /* command message that requests the device to send a USBTMC response */
#define DEV_DEP_MSG_IN			2   /* response message to the REQUEST_DEV_DEP_MSG_IN */
#define VENDOR_SPECIFIC_OUT		126 /* vendor specific command message */
#define REQUEST_VENDOR_SPECIFIC_IN	127 /* command message that requests the device to send a vendor specific USBTMC response */
#define VENDOR_SPECIFIC_IN		127 /* response message to the REQUEST_VENDOR_SPECIFIC_IN */

static int major, minors;
static struct class *usb_gadget_class;
static DEFINE_IDA(usbtmc_ida);
static DEFINE_MUTEX(usbtmc_ida_lock); /* protects access do usbtmc_ida */


#define TMC_IOCTL_MAGIC 'c'
#define TMC_ERR_PAYLOAD_STUCK       _IOW(TMC_IOCTL_MAGIC, 0, unsigned)
#define TMC_ATS_ENABLE              _IOR(TMC_IOCTL_MAGIC, 1, unsigned)

#define TMC_BULK_BUFFER_SIZE          (10*1024+12+3) 
#define TX_REQ_MAX 1

static const char tmc_shortname[] = "usb_tmc";

/** struct to hold all usb tmc transfer relevant information */
typedef struct 
{
	uint8_t bTag; /**< contains the bTag value of the currently active transfer */
	uint32_t nbytes_rxd; /**< contains the number of bytes received in active tmc OUT transfer */
	uint32_t nbytes_txd; /**< contains the number of bytes transmitted in active tmc IN transfer */
}TMCTransferInfo;

#define USB_TMC_HEADER_SIZE 12  /**< length of a USBTMC header */

/* USBTMC MsgID. Values, Ref.: Table 2 */
#define DEV_DEP_MSG_OUT              1   /**< device dependent command message */
#define REQUEST_DEV_DEP_MSG_IN       2   /**< command message that requests the device to send a USBTMC response */
#define DEV_DEP_MSG_IN               2   /**< response message to the REQUEST_DEV_DEP_MSG_IN */
#define VENDOR_SPECIFIC_OUT          126 /**< vendor specific command message */
#define REQUEST_VENDOR_SPECIFIC_IN   127 /**< command message that requests the device to send a vendor specific USBTMC response */
#define VENDOR_SPECIFIC_IN           127 /**< response message to the REQUEST_VENDOR_SPECIFIC_IN */


/* USBTMC USB488 Subclass commands (bRequest values, Ref.: Table 9) */
#define READ_STATUS_BYTE             128
#define REN_CONTROL                  160
#define GO_TO_LOCAL                  161
#define LOCAL_LOCKOUT                162

/* bmTransfer attributes */
#define bmTA_EOM       0x01  /**< bmTransfer Attribute: End of Message */
#define bmTA_TERMCHAR  0x02  /**< bmTransfer Attribute: Terminate transfer with Terminate Character */

/* defines for the device capablilities, Ref.: Table 37 and Table 8 USB488 */
#define HAS_INDICATOR_PULSE 0x04
#define TALK_ONLY       0x02
#define LISTEN_ONLY     0x01
#define TERMCHAR_BULKIN 0x01
#define IS_488_2        0x04
#define ACCEPTS_LOCAL_LOCKOUT 0x02
#define TRIGGER         0x01
#define SCPI_COMPILIANT 0x08
#define SR1_CAPABLE     0x04
#define RL1_CAPABLE     0x02
#define DT1_CAPABLE     0x01

typedef struct 
{
	uint8_t USBTMC_status;
	uint8_t reserved0;
	uint8_t bcdUSBTMC_lsb;
	uint8_t bcdUSBTMC_msb;
	uint8_t TMCInterface;
	uint8_t TMCDevice;
	uint8_t reserved1[6];
	/* place here USB488 subclass capabilities */
	uint8_t bcdUSB488_lsb;
	uint8_t bcdUSB488_msb;
	uint8_t USB488Interface;
	uint8_t USB488Device;
	uint8_t reserved2[8];
} USB_TMC_Capabilities;

struct tmc_dev 
{
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	atomic_t online;
	//atomic_t error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;

	struct list_head tx_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	struct usb_request *rx_req;
	int rx_done;

	char buf[512];
	TMCTransferInfo usb_tmc_transfer; 
	int usb_tmc_status; 
	uint8_t term_char_enabled;
	uint8_t term_char;
	uint8_t usbtmc_last_write_bTag;
	uint8_t usbtmc_last_read_bTag;


	/* char driver specific members */
	int			minor;
	struct cdev		tmc_cdev;
	u8			tmc_cdev_open;
};

static volatile const USB_TMC_Capabilities USB_TMC_CAPABILITIES = {
  	USBTMC_STATUS_SUCCESS,
	0,
	0x10, 0x01,/* BCD version number of TMC specification, 2.00 */
	0x00,
	0,
	{0,0,0,0,0,0},

    	/* place here USB488 subclass capabilities */
    	0x10, 0x01, /* BCD version number of USB488 specification, 2.00 */
      	0,
      	0,
	{0,0,0,0,0,0,0,0}
};


static struct usb_interface_descriptor tmc_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = USB_CLASS_APP_SPEC,
	.bInterfaceSubClass     = 0x03,
	.bInterfaceProtocol     = 1,
};

/*USB 2.0 specific*/

static struct usb_endpoint_descriptor tmc_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor tmc_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor tmc_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor tmc_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_tmc_descs[] = {
	(struct usb_descriptor_header *) &tmc_interface_desc,
	(struct usb_descriptor_header *) &tmc_fullspeed_in_desc,
	(struct usb_descriptor_header *) &tmc_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_tmc_descs[] = {
	(struct usb_descriptor_header *) &tmc_interface_desc,
	(struct usb_descriptor_header *) &tmc_highspeed_in_desc,
	(struct usb_descriptor_header *) &tmc_highspeed_out_desc,
	NULL,
};

/* USB 3.0 */

static struct usb_endpoint_descriptor tmc_superspeed_in_desc = {
        .bLength =              USB_DT_ENDPOINT_SIZE,
        .bDescriptorType =      USB_DT_ENDPOINT,
        .bmAttributes =         USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize =       cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor tmc_superspeed_in_comp_desc = {
        .bLength =              sizeof(tmc_superspeed_in_comp_desc),
        .bDescriptorType =      USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_endpoint_descriptor tmc_superspeed_out_desc = {
        .bLength =              USB_DT_ENDPOINT_SIZE,
        .bDescriptorType =      USB_DT_ENDPOINT,
        .bmAttributes =         USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize =       cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor tmc_superspeed_out_comp_desc = {
        .bLength =              sizeof(tmc_superspeed_out_comp_desc),
        .bDescriptorType =      USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_descriptor_header *ss_tmc_descs[] = {
        (struct usb_descriptor_header *) &tmc_interface_desc,
        (struct usb_descriptor_header *) &tmc_superspeed_in_desc,
        (struct usb_descriptor_header *) &tmc_superspeed_in_comp_desc,
        (struct usb_descriptor_header *) &tmc_superspeed_out_desc,
        (struct usb_descriptor_header *) &tmc_superspeed_out_comp_desc,
        NULL
};



static struct tmc_dev *_tmc_dev;

static inline struct tmc_dev *func_to_tmc(struct usb_function *f)
{
	return container_of(f, struct tmc_dev, function);
}


static struct usb_request *tmc_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
	{
		return NULL;
	}

	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf)
       	{
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void tmc_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) 
	{
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int tmc_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) 
	{
		return 0;
	}
       	else
       	{
		atomic_dec(excl);
		return -1;
	}
}

static inline void tmc_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

void tmc_req_put(struct tmc_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

struct usb_request *tmc_req_get(struct tmc_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) 
	{
		req = 0;
	}
       	else
       	{
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}


static void tmc_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_dev *dev = _tmc_dev;

	if (req->status != 0) 
	{
		if (req->status != -ESHUTDOWN)
		{
#ifdef TMC_DEBUG
			printk(KERN_INFO "[USB] %s: warning (%d)\n", __func__, req->status);
#endif
		}
	}

	tmc_req_put(dev, &dev->tx_idle, req);
	wake_up(&dev->write_wq);
}

static void tmc_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_dev *dev = _tmc_dev;

	dev->rx_done = 1;
	if (req->status != 0) 
	{
		if (req->status != -ESHUTDOWN)
		{
#ifdef TMC_DEBUG
			printk(KERN_INFO "[USB] %s: warning (%d)\n", __func__, req->status);
#endif
		}
	}

	wake_up(&dev->read_wq);
}

#if 0
static void tmc_complete_in(struct usb_ep *ep, struct usb_request *req)
{

	struct tmc_dev *dev = ep->driver_data;

	if (req->status != 0) 
	{
		if (req->status != -ESHUTDOWN)
		{
#ifdef TMC_DEBUG
			printk(KERN_INFO "[USB] %s: warning (%d)\n", __func__, req->status);
#endif
		}
	}

	tmc_req_put(dev, &dev->tx_idle, req);
	wake_up(&dev->write_wq);
}

static void tmc_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_dev *dev = ep->driver_data;
printk("----1. check--------------\n");
	dev->rx_done = 1;
printk("----2. check--------------\n");
	if (req->status != 0) 
	{
		if (req->status != -ESHUTDOWN)
		{
#ifdef TMC_DEBUG
			printk(KERN_INFO "[USB] %s: warning (%d)\n", __func__, req->status);
#endif
		}
	}
printk("---3. check --------------\n");
	wake_up(&dev->read_wq);
}

#endif

static int tmc_create_bulk_endpoints(struct tmc_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

#ifdef TMC_DEBUG
	printk("create_bulk_endpoints dev: %p\n", dev);
#endif
	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep)
       	{
		printk("usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
#ifdef TMC_DEBUG
	printk("usb_ep_autoconfig for ep_in got %s\n", ep->name);
#endif
	ep->driver_data = dev;		
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) 
	{
		printk("usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}

#ifdef TMC_DEBUG
	printk("usb_ep_autoconfig for tmc ep_out got %s\n", ep->name);
#endif
	ep->driver_data = dev;		
	dev->ep_out = ep;
	
	req = tmc_request_new(dev->ep_out, TMC_BULK_BUFFER_SIZE);
	if (!req)
	{
		goto fail;
	}

	req->complete = tmc_complete_out;
	dev->rx_req = req;

	for (i = 0; i < TX_REQ_MAX; i++) 
	{
		req = tmc_request_new(dev->ep_in, TMC_BULK_BUFFER_SIZE);
		if (!req)
		{
			goto fail;
		}

		req->complete = tmc_complete_in;
		tmc_req_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	printk(KERN_ERR "tmc_bind() could not allocate requests\n");
	return -1;
}

static int tmc_queue_req(struct usb_ep *ep,
			       struct usb_request *req, gfp_t gfp_flags)
{
	int ret = 0;
	int try_cnt = 0;

	do
	{
		ret = usb_ep_queue(ep, req, gfp_flags);
		try_cnt++;
		if(try_cnt > 3)
		{
			printk("queue is always not empty, so empty it forcely, and queue the new req\n");
			usb_ep_fifo_flush(ep);
			ret = usb_ep_queue(ep, req, gfp_flags);
			break;
		}
	}while(ret == -EINVAL);

	return ret;
}

static ssize_t tmc_read(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	struct tmc_dev *dev = fp->private_data;
	struct usb_request *req;
	int r = count, xfer;
	int ret;
	unsigned long int n_characters, done = 0;
	u8 *req_buf;
	
#ifdef TMC_DEBUG
	printk("tmc_read(%d)\n", count);
#endif
	//check params
	if (!_tmc_dev)
	{
		printk("tmc_read !tmc_dev\n");
		return -ENODEV;
	}
	if (count > TMC_BULK_BUFFER_SIZE-12-3)
	{
		printk("tmc_read count > %d\n", TMC_BULK_BUFFER_SIZE-12-3);
		return -EINVAL;
	}
	if (tmc_lock(&dev->read_excl))
	{
		printk("tmc_read lock read excl wrong\n");
		return -EBUSY;
	}

	
	/*waiting for state of ready to read*/
	if(!(atomic_read(&dev->online))) 
	{
#ifdef TMC_DEBUG
		printk("tmc_read: waiting for online state\n");
#endif
		tmc_unlock(&dev->read_excl);
		return -EIO;
	}

	req  = dev->rx_req;
       	req_buf = (u8 *)req->buf;
	xfer = count;

#ifdef TMC_DEBUG
	printk("tmc_read start read\n");
#endif
	while(1)
	{
		/*read pdu*/
		dev->rx_done = 0;
		req->length = xfer+12;
#ifdef TMC_DEBUG
		printk("tmc_read: queue req %p\n", req);
#endif
		ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
		if (ret < 0) 
		{
			printk("tmc_read: failed to queue req %p (%d)\n", req, ret);
			r = -EIO;
			break;
		}

		/*wait the request complete, if something wrong, just dequeue it*/	
		ret = wait_event_timeout(dev->read_wq, dev->rx_done, 20);
		if (ret <= 0) 
		{
			if (ret != -ERESTARTSYS)
			{
			}

			r = ret;
			usb_ep_dequeue(dev->ep_out, req);
#ifdef TMC_DEBUG
			printk("read data always undone\n");
#endif
			break;
		}
#ifdef TMC_DEBUG
		printk("tmc_read: req is processed %d, %d\n", dev->rx_done, ret);
#endif

		/*if no error, then copy to user*/
		{
			int i;

#ifdef TMC_DEBUG
			printk("%s:rx %p %d\n", __func__, req, req->actual);

			for(i=0; i<=req->actual; i++)
			{
				printk("%s:buf[%d]=%x\n", __func__, i, req_buf[i]);	
			}
#endif

			if(req_buf[0] == DEV_DEP_MSG_OUT)	
			{
				u8 btag = req_buf[1];
				u8 btag_rev = ~req_buf[2];

				if(btag == btag_rev)	
				{
		  			dev->usb_tmc_transfer.bTag = req_buf[1];
					dev->usbtmc_last_read_bTag = req_buf[1];
					//dev->usb_tmc_transfer.bTag++;
					if(!dev->usb_tmc_transfer.bTag)
					{
						//dev->usb_tmc_transfer.bTag++;
					}

					n_characters=req_buf[4]+(req_buf[5]<<8)+(req_buf[6]<<16)+(req_buf[7]<<24);

					xfer = req->actual - 12;

					if(n_characters <= xfer)
					{
						if (copy_to_user(buf+done, req->buf+12, n_characters))
						{
							printk("copy to user wrong\n");
							r = -EFAULT;
							break;
						}

						done += n_characters;	
						if(req_buf[8] & 0x1)
						{
							printk("the pdu read comletely\n");
							break;
						}
						else
						{
							if(done >= count)
							{
								printk("pc send too much data than wanted, the pdu can not read comletely\n");
								r = -EIO;
								break;	
							}
							else
							{
								xfer = count - done;
							}
						}
					}
					else
					{
						if (copy_to_user(buf+done, req->buf+12, xfer))
						{
							printk("copy to user wrong 2\n");
							r = -EFAULT;
							break;
						}
						r = -EIO;
						printk("pc send too much data than wanted, the pdu can not read comletely 2\n");
						break;

					}
				}
				else
				{
					printk("btag and btag rev wrong:0x%x, 0x%x\n", btag, btag_rev);	
					r = -EIO;
					break;
				}
			}
			else
			{
				printk("read not supported msg id %d\n", req_buf[0]);	
				r = -EIO;
				break;
			}
		}
	}

	tmc_unlock(&dev->read_excl);
	if(r < 0)
	{
		dev->usb_tmc_transfer.nbytes_rxd = 0;
		pr_debug("tmc_read succes returning %d\n", r);
		return r;
	}
	else
	{
		dev->usb_tmc_transfer.nbytes_rxd = done;
		pr_debug("tmc_read wrong returning %ld\n", done);
		return done;
	}
}

static ssize_t tmc_read_REQUEST_DEV_DEP_MSG_IN(struct tmc_dev *dev)
{
	struct usb_request *req;
	unsigned long int n_characters;
	int ret, r;
	u8 *req_buf;

	if (tmc_lock(&dev->read_excl))
	{
		return -EBUSY;
	}

	/*send the pdu*/
	req = dev->rx_req;
       	req_buf = (u8 *)req->buf;
	req->length = 12;
	dev->rx_done = 0;
	ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
	if (ret < 0) 
	{
		printk("tmc_read_REQUEST_DEV_DEP_MSG_IN: read failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		tmc_unlock(&dev->read_excl);
		return r;
	}

	/*wait the request complete, if not dequeue it*/	
	ret = wait_event_interruptible(dev->read_wq, dev->rx_done);
	if (ret < 0) 
	{
		if (ret != -ERESTARTSYS)
		{
		}
		r = ret;
		usb_ep_dequeue(dev->ep_out, req);
		tmc_unlock(&dev->read_excl);
		return r;
	}

	/*if no error, then check it*/
	{
		int i;
#ifdef TMC_DEBUG
		printk("%s:rx %p %d\n", __func__, req, req->actual);
		for(i=0; i<=req->actual; i++)
		{
			printk("%s:buf[%d]=%x\n", __func__, i, req_buf[i]);	
		}
#endif

		if (req->actual == 12)
		{
			if(req_buf[0] == REQUEST_DEV_DEP_MSG_IN)	
			{
				u8 btag = req_buf[1];
				u8 btag_rev = ~req_buf[2];

				if(btag == btag_rev)	
				{
	  				dev->usb_tmc_transfer.bTag = req_buf[1];
					dev->usbtmc_last_read_bTag = req_buf[1];
					//dev->usb_tmc_transfer.bTag++;
					if(!dev->usb_tmc_transfer.bTag)
					{
						//dev->usb_tmc_transfer.bTag++;
					}

					n_characters=req_buf[4]+(req_buf[5]<<8)+(req_buf[6]<<16)+(req_buf[7]<<24);
					dev->term_char_enabled = ((req_buf[8])&0x2)>>1;
					dev->term_char = req_buf[9];
#ifdef TMC_DEBUG
					printk("%s:%d, %d, 0x%x\n", __func__, n_characters, dev->term_char_enabled, dev->term_char);
#endif

					tmc_unlock(&dev->read_excl);
					return n_characters;
				}
				else
				{
					printk("REQUEST_DEV_DEP_MSG_IN btag and btag rev wrong:0x%x, 0x%x\n", btag, btag_rev);	
					r = -EIO;
					tmc_unlock(&dev->read_excl);
					return r;
				}
			}
			else
			{
				printk("not REQUEST_DEV_DEP_MSG_IN, %d\n", req_buf[0]);
				r = -EIO;
				tmc_unlock(&dev->read_excl);
				return r;
			}
		}
		else
		{
			printk("rx %p %d, wrong nm\n", req, req->actual);
			tmc_unlock(&dev->read_excl);
			r = -EIO;
			return r;
		}
	}
}	

static ssize_t tmc_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *pos)
{
	struct tmc_dev *dev = fp->private_data;
	struct usb_request *req = 0;
	int r = count, xfer, remaining;
	int ret;
	ssize_t  max_transfer_size;
	u8 term_char_enabled = 0;
	u8 term_char= 0;
	u8 *req_buf;

#ifdef TMC_DEBUG
	printk("tmc_write: write cnt is%d\n", count);
#endif


	if (!_tmc_dev)
		return -ENODEV;

	if (tmc_lock(&dev->write_excl))
		return -EBUSY;

	/*wait state is get ready to write*/
	req = 0;
	ret = wait_event_interruptible(dev->write_wq,
		(req = tmc_req_get(dev, &dev->tx_idle)));
	if (ret<0 || req==0) 
	{
		printk("wait state to get ready to write wrong\n");
		r = ret;
		if(req)
		{
			tmc_req_put(dev, &dev->tx_idle, req);
		}
		tmc_unlock(&dev->write_excl);
		return r;
	}

       	req_buf = (u8 *)req->buf;
	remaining = count;
	while (remaining > 0) 
	{
		max_transfer_size = tmc_read_REQUEST_DEV_DEP_MSG_IN(dev);

		if(max_transfer_size < 0)
		{
			r = -EIO;
			printk("get pc max transfer size wrong\n");
			break;
		}
		else
		{
			term_char_enabled = dev->term_char_enabled;
			term_char = dev->term_char;
#ifdef TMC_DEBUG
			printk("get pc max transfer size %d, %d, %c\n", max_transfer_size, term_char_enabled, term_char);
#endif
		}

		{
			if(remaining > TMC_BULK_BUFFER_SIZE-12-3)
			{
				xfer = TMC_BULK_BUFFER_SIZE-12-3;
			}
			else
			{
				xfer = remaining;	
			}

			if (xfer > max_transfer_size)
			{
				xfer = max_transfer_size;
				req_buf[8] = 0;
			}
			else
			{
				req_buf[8] = 1;
			}

			/*get user's data*/
			req_buf[0]=DEV_DEP_MSG_IN; 
			req_buf[1]=dev->usb_tmc_transfer.bTag; // Transfer ID (bTag)
			req_buf[2]=~(dev->usb_tmc_transfer.bTag); // Inverse of bTag
			req_buf[3]=0; // Reserved
			req_buf[4]=xfer&255; // Transfer size (first byte)
			req_buf[5]=(xfer>>8)&255; // Transfer size (second byte)
			req_buf[6]=(xfer>>16)&255; // Transfer size (third byte)
			req_buf[7]=(xfer>>24)&255; // Transfer size (fourth byte)
			//req_buf[8] is set above...
			req_buf[9]=0; // Reserved
			req_buf[10]=0; // Reserved
			req_buf[11]=0; // Reserved

			if (copy_from_user(req->buf+12, buf, xfer)) 
			{
				printk("copy data from user error\n");
				r = -EFAULT;
				break;
			}

			/*Add zero bytes to achieve 4-byte alignment*/
			req->length = xfer+12;
			if(xfer%4) 
			{
				int n;

				req->length += 4-xfer%4;
				for(n=12+xfer; n<req->length; n++)
				{
					req_buf[n]=0;
				}
			}
			
			/*send the pdu*/
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if (ret < 0) 
			{
				printk("tmc_write: xfer error %d\n", ret);
				r = -EIO;
				break;
			}

			dev->usbtmc_last_write_bTag = dev->usb_tmc_transfer.bTag;
			//dev->usb_tmc_transfer.bTag++;
			if(!dev->usb_tmc_transfer.bTag)
			{
				//dev->usb_tmc_transfer.bTag++;
			}

			buf += xfer;
			remaining -= xfer;
		}
	}

	if(req)
	{
		tmc_req_put(dev, &dev->tx_idle, req);
	}

	tmc_unlock(&dev->write_excl);

	if(r < 0)
	{
		dev->usb_tmc_transfer.nbytes_txd= 0;
#ifdef TMC_DEBUG
		printk("tmc_write success returning  %d\n", r);
#endif
		return r;
	}
	else
	{
		dev->usb_tmc_transfer.nbytes_txd= count-remaining;
#ifdef TMC_DEBUG
		printk("tmc_write wrong returning %d\n", count - remaining);
#endif
		return count - remaining;
	}
}


static int tmc_open(struct inode *ip, struct file *fp)
{
#ifdef TMC_DEBUG
	printk(KERN_INFO "[USB] tmc_open: %s(parent:%s): tgid=%d\n",
			current->comm, current->parent->comm, current->tgid);
#endif
	if (!_tmc_dev)
		return -ENODEV;

	if (tmc_lock(&_tmc_dev->open_excl))
		return -EBUSY;

	fp->private_data = _tmc_dev;

	return 0;
}

static int tmc_release(struct inode *ip, struct file *fp)
{
#ifdef TMC_DEBUG
	printk(KERN_INFO "[USB] tmc_release: %s(parent:%s): tgid=%d\n",
			current->comm, current->parent->comm, current->tgid);
#endif
	tmc_unlock(&_tmc_dev->open_excl);
	return 0;
}


static const struct file_operations tmc_fops = 
{
	.owner = THIS_MODULE,
	.read = tmc_read,
	.write = tmc_write,
	.open = tmc_open,
	.release = tmc_release,
};


static int
tmc_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct tmc_dev	*dev = func_to_tmc(f);
	int			id;
	int			ret;
	dev_t devt;
	struct device *pdev;

	dev->cdev = cdev;
#ifdef TMC_DEBUG
	printk("tmc_function_bind dev: %p\n", dev);
#endif
	
	id = usb_interface_id(c, f);
	if (id < 0)
	{
		printk("%s:id<0\n", __func__);
		return id;
	}

	tmc_interface_desc.bInterfaceNumber = id;
	
	ret = tmc_create_bulk_endpoints(dev, &tmc_fullspeed_in_desc,
			&tmc_fullspeed_out_desc);
	if (ret)
		return ret;

#if 0
        /* assumes that all endpoints are dual-speed */
        tmc_highspeed_in_desc.bEndpointAddress = tmc_fullspeed_in_desc.bEndpointAddress;
        tmc_highspeed_in_desc.bEndpointAddress = tmc_fullspeed_out_desc.bEndpointAddress;
        tmc_superspeed_in_desc.bEndpointAddress = tmc_fullspeed_in_desc.bEndpointAddress;
        tmc_superspeed_in_desc.bEndpointAddress = tmc_fullspeed_out_desc.bEndpointAddress;

        ret = usb_assign_descriptors(f, fs_tmc_function,
                        hs_tmc_function, ss_tmc_function);
#endif

#if 1	
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		tmc_highspeed_in_desc.bEndpointAddress =
			tmc_fullspeed_in_desc.bEndpointAddress;
		tmc_highspeed_out_desc.bEndpointAddress =
			tmc_fullspeed_out_desc.bEndpointAddress;
	}

	if (gadget_is_superspeed(c->cdev->gadget)) {
		tmc_superspeed_in_desc.bEndpointAddress =
			tmc_fullspeed_in_desc.bEndpointAddress;
		tmc_superspeed_out_desc.bEndpointAddress =
			tmc_fullspeed_out_desc.bEndpointAddress;
	}



#ifdef TMC_DEBUG
	printk("%s speed",gadget_is_superspeed(c->cdev->gadget) ? "superspeed " : " not superspeed ");
	printk("%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
#endif
#endif
	/* Setup the sysfs files for the usbtmc gadget. */
	devt = MKDEV(major, dev->minor);
	pdev = device_create(usb_gadget_class, NULL, devt,
				  NULL, "g_tmc%d", dev->minor);
	if (IS_ERR(pdev)) {
		printk("Failed to create device: g_tmc\n");
		//TODO : delete the filled memory for rx/tx req
		return 0;
	}

	/*
	 * Register a character device as an interface to a user mode
	 * program that handles the printer specific functionality.
	 */
	cdev_init(&dev->tmc_cdev, &tmc_fops);
	dev->tmc_cdev.owner = THIS_MODULE;
	ret = cdev_add(&dev->tmc_cdev, devt, 1);
	if (ret) {
		printk("Failed to open char device\n");
		goto fail_cdev_add;
	}

	return 0;

fail_cdev_add:
	device_destroy(usb_gadget_class, devt);



	return 0;
}

static void
tmc_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct tmc_dev	*dev = func_to_tmc(f);
	struct usb_request *req;

	device_destroy(usb_gadget_class, MKDEV(major, dev->minor));

	atomic_set(&dev->online, 0);
	//wake_up(&dev->read_wq);

	tmc_request_free(dev->rx_req, dev->ep_out);
	while ((req = tmc_req_get(dev, &dev->tx_idle)))
		tmc_request_free(req, dev->ep_in);
}

static int tmc_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct tmc_dev	*dev = func_to_tmc(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

#ifdef TMC_DEBUG
	printk("tmc_function_set_alt intf: %d alt: %d\n", intf, alt);
#endif

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret) {
		dev->ep_in->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
				dev->ep_in->name, ret);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_in);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
			dev->ep_in->name, ret);
		return ret;
	}

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
	if (ret) {
		dev->ep_out->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
			dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_out);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
				dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}

	atomic_set(&dev->online, 1);
	wake_up(&dev->read_wq);

	return 0;
}

static void tmc_function_disable(struct usb_function *f)
{
	struct tmc_dev	*dev = func_to_tmc(f);
	struct usb_composite_dev	*cdev = dev->cdev;

	atomic_set(&dev->online, 0);
	usb_ep_disable(dev->ep_in);
#ifdef TMC_DEBUG
	printk("tmc_function_disable 2:%p\n", dev->ep_out);
#endif
	usb_ep_disable(dev->ep_out);
#ifdef TMC_DEBUG
	printk("tmc_function_disable 3\n");
#endif
	wake_up(&dev->read_wq);

#ifdef TMC_DEBUG
	printk("%s disabled\n", dev->function.name);
#endif
}


static void tmc_function_free(struct usb_function *f)
{
#ifdef TMC_DEBUG
        printk("tmc_function_free\n");
#endif
	struct tmc_dev *dev = func_to_tmc(f);
	struct f_tmc_opts *opts;

	opts = container_of(f->fi, struct f_tmc_opts, func_inst);
	kfree(dev);
	mutex_lock(&opts->lock);
	--opts->refcnt;
	mutex_unlock(&opts->lock);

}

static int tmc_function_setup(struct usb_function *f, const struct usb_ctrlrequest *c)
{
	int value = -EOPNOTSUPP;
	u16 wIndex = le16_to_cpu(c->wIndex);
	u16 wValue = le16_to_cpu(c->wValue);
	u16 wLength = le16_to_cpu(c->wLength);
	struct tmc_dev	*dev = func_to_tmc(f);
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req = cdev->req;

#ifdef TMC_DEBUG
	printk("%s:%d, %d\n", __func__, c->bRequestType, c->bRequest);
#endif

	switch ((c->bRequestType << 8) | c->bRequest) 
	{
		case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_ENDPOINT) << 8 | USBTMC_REQUEST_INITIATE_ABORT_BULK_OUT):

			/* check if the active transfer has the requested bTag value */
#ifdef TMC_DEBUG
			printk("%s:USBTMC_REQUEST_INITIATE_ABORT_BULK_OUT\n", __func__);
#endif
			//if (c->bRequestType == USB_DIR_IN)
			{
				if(dev->usb_tmc_transfer.bTag == wValue) 
				{
					dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;

					usb_ep_set_halt(dev->ep_in);
				}
				else 
				{
					dev->usb_tmc_status = USBTMC_STATUS_TRANSFER_NOT_IN_PROGRESS;
				}

				dev->buf[0] = dev->usb_tmc_status;
				dev->buf[1] = dev->usb_tmc_transfer.bTag;
				value = 2;
				req->zero = 0;
				req->length = value;
				memcpy(req->buf, dev->buf, value);
				if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
				{
					printk(KERN_ERR "ep0 in queue failed\n");
				}
			}

			break;
		case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_ENDPOINT) << 8 | USBTMC_REQUEST_CHECK_ABORT_BULK_OUT_STATUS):
#ifdef TMC_DEBUG
			printk("%s:USBTMC_REQUEST_CHECK_ABORT_BULK_OUT_STATUS\n", __func__);
#endif
			//if (c->bRequestType == USB_DIR_IN)
			{
				/* send number of transmitted bytes */ 
				dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
				dev->buf[0] = dev->usb_tmc_status;
				dev->buf[1] = 0;
				dev->buf[2] = 0;
				dev->buf[3] = 0;
				dev->buf[4] = (dev->usb_tmc_transfer.nbytes_rxd & 0x0000FF);
				dev->buf[5] = (dev->usb_tmc_transfer.nbytes_rxd & 0x00FF00)>>8;
				dev->buf[6] = (dev->usb_tmc_transfer.nbytes_rxd & 0xFF0000)>>16;
				dev->buf[7] = dev->usb_tmc_transfer.nbytes_rxd >>24;

				value = 8;
				req->zero = 0;
				req->length = value;
				memcpy(req->buf, dev->buf, value);
				if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
				{
					printk(KERN_ERR "ep0 in queue failed\n");
				}
			}
			break;
		case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_ENDPOINT) << 8 | USBTMC_REQUEST_INITIATE_ABORT_BULK_IN):
#ifdef TMC_DEBUG
			printk("%s:USBTMC_REQUEST_INITIATE_ABORT_BULK_IN\n", __func__);
#endif
			//if (c->bRequestType == USB_DIR_IN)
			{
				/* check if the active transfer has the requested bTag value */
				if(dev->usb_tmc_transfer.bTag == wValue) 
				{
					dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
					usb_ep_set_halt(dev->ep_out);
				}
				else 
				{
					dev->usb_tmc_status = USBTMC_STATUS_TRANSFER_NOT_IN_PROGRESS;
				}

				dev->buf[0] = dev->usb_tmc_status;
				dev->buf[1] = wValue;
				value = 2;
				req->zero = 0;
				req->length = value;
				memcpy(req->buf, dev->buf, value);
				if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
				{
					printk(KERN_ERR "ep0 in queue failed\n");
				}
			}
			break;
		case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_ENDPOINT) << 8 | USBTMC_REQUEST_CHECK_ABORT_BULK_IN_STATUS):
#ifdef TMC_DEBUG
			printk("%s:USBTMC_REQUEST_CHECK_ABORT_BULK_IN_STATUS\n", __func__);
#endif
			//if (c->bRequestType == USB_DIR_IN)
			{
				/* send number of transmitted bytes */ 
				dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
				dev->buf[0] = dev->usb_tmc_status;
				dev->buf[1] = 0;
				dev->buf[2] = 0;
				dev->buf[3] = 0;
				dev->buf[4] = (dev->usb_tmc_transfer.nbytes_txd & 0x0000FF);
				dev->buf[5] = (dev->usb_tmc_transfer.nbytes_txd & 0x00FF00)>>8;
				dev->buf[6] = (dev->usb_tmc_transfer.nbytes_txd & 0xFF0000)>>16;
				dev->buf[7] = dev->usb_tmc_transfer.nbytes_txd >>24;

				value = 8;
				req->zero = 0;
				req->length = value;
				memcpy(req->buf, dev->buf, value);
				if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
				{
					printk(KERN_ERR "ep0 in queue failed\n");
				}
			}
			break;
		case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8 | USBTMC_REQUEST_INITIATE_CLEAR):
#ifdef TMC_DEBUG
			printk("%s:USBTMC_REQUEST_INITIATE_CLEAR\n", __func__);
#endif
			//if (c->bRequestType == USB_DIR_IN)
			{
				dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
				usb_ep_set_halt(dev->ep_in);
				usb_ep_set_halt(dev->ep_out);

				dev->buf[0] = dev->usb_tmc_status;
				value = 1;
				req->zero = 0;
				req->length = value;
				memcpy(req->buf, dev->buf, value);
				if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
				{
					printk(KERN_ERR "ep0 in queue failed\n");
				}
			}
			break;
		case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8 | USBTMC_REQUEST_CHECK_CLEAR_STATUS):
#ifdef TMC_DEBUG
			printk("%s:USBTMC_REQUEST_CHECK_CLEAR_STATUS\n", __func__);
#endif
			//if (c->bRequestType == USB_DIR_IN)
			{
				dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
				dev->buf[0] = dev->usb_tmc_status;
				dev->buf[1] = wValue;
				value = 2;
				req->zero = 0;
				req->length = value;
				memcpy(req->buf, dev->buf, value);
				if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
				{
					printk(KERN_ERR "ep0 in queue failed\n");
				}
			}
			break;
		case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8 | USBTMC_REQUEST_GET_CAPABILITIES):
#ifdef TMC_DEBUG
			printk("%s:USBTMC_REQUEST_GET_CAPABILITIES\n", __func__);
#endif
			//if (c->bRequestType == USB_DIR_IN)
			{
				dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
				value = sizeof(USB_TMC_Capabilities);
				req->zero = 0;
				req->length = value;
				memcpy(req->buf, (char *)&USB_TMC_CAPABILITIES, value);
				if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
				{
					printk(KERN_ERR "ep0 in queue failed\n");
				}
			}
			break;
		case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8 | USBTMC_REQUEST_INDICATOR_PULSE):
			/* optional command, not implemented */
			value = 0;
			break;
		default:
			printk("Unknown request 0x%x\n",c->bRequest);
			break;
	}

	return value;
}


/*--------------------------------------------------------------------------------------------------------------*/
/*                                              ConfigFS Attribute                                              */                                     
#if 1
static inline struct f_tmc_opts * to_f_tmc_opts(struct config_item *item)
{
        return container_of(to_config_group(item), struct f_tmc_opts, func_inst.group);
}

static void tmc_attr_release(struct config_item *item)
{
        struct f_tmc_opts *opts = to_f_tmc_opts(item);

        usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations tmc_item_ops = {
        .release        = tmc_attr_release,
};

static ssize_t f_tmc_opts_port_num_show(struct config_item *item, char *page)
{
        return sprintf(page, "%u\n", to_f_tmc_opts(item)->port_num);
}

CONFIGFS_ATTR_RO(f_tmc_opts_, port_num);

static struct configfs_attribute *tmc_attrs[] = {
        &f_tmc_opts_attr_port_num,
        NULL,
};

static struct config_item_type tmc_func_type = {
        .ct_item_ops    = &tmc_item_ops,
        .ct_attrs       = tmc_attrs,
        .ct_owner       = THIS_MODULE,
};

#endif
//For Old Kernel 
#if 0
static inline struct f_tmc_opts *to_f_tmc_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_tmc_opts,
			    func_inst.group);
}

CONFIGFS_ATTR_STRUCT(f_tmc_opts);
#if 0
static ssize_t f_tmc_attr_show(struct config_item *item,
				  struct configfs_attribute *attr,
				  char *page)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	struct f_tmc_opts_attribute *f_tmc_opts_attr =
		container_of(attr, struct f_tmc_opts_attribute, attr);
	ssize_t ret = 0;

	if (f_tmc_opts_attr->show)
		ret = f_tmc_opts_attr->show(opts, page);

	return ret;
}
#endif

static void tmc_attr_release(struct config_item *item)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations tmc_item_ops = {
	.release	= tmc_attr_release,
	//.show_attribute = f_tmc_attr_show,
};

static ssize_t f_tmc_port_num_show(struct f_tmc_opts *opts, char *page)
{
	return sprintf(page, "%u\n", opts->port_num);
}

static struct f_tmc_opts_attribute f_tmc_port_num =
	__CONFIGFS_ATTR_RO(port_num, f_tmc_port_num_show);

static struct configfs_attribute *tmc_attrs[] = {
	&f_tmc_port_num.attr,
	NULL,
};

static struct config_item_type tmc_func_type = {
	.ct_item_ops	= &tmc_item_ops,
	.ct_attrs	= tmc_attrs,
	.ct_owner	= THIS_MODULE,
};


#endif

/*--------------------------------------------------------------------------------------------------------------*/
/*				USBMTC device allocation with composite framework				*/

static int tmc_setup(int);
static void tmc_cleanup(void);

static inline int tmc_get_minor(void)
{
	int ret;

	ret = ida_simple_get(&usbtmc_ida, 0, 0, GFP_KERNEL);
	if (ret >= USBTMC_MINORS) {
		ida_simple_remove(&usbtmc_ida, ret);
		ret = -ENODEV;
	}

	return ret;
}

static inline void tmc_put_minor(int minor)
{
	ida_simple_remove(&usbtmc_ida, minor);
}

static void tmc_free_inst(struct usb_function_instance *f)
{
	struct f_tmc_opts *opts;

	opts = container_of(f, struct f_tmc_opts, func_inst);

	mutex_lock(&usbtmc_ida_lock);

	tmc_put_minor(opts->minor);
	if (idr_is_empty(&usbtmc_ida.idr))
		tmc_cleanup();

	mutex_unlock(&usbtmc_ida_lock);

	kfree(opts);
}

static struct usb_function_instance *tmc_alloc_inst(void)
{
	struct f_tmc_opts *opts;
	struct usb_function_instance *ret;
	int status = 0;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);

	mutex_init(&opts->lock);
	opts->func_inst.free_func_inst = tmc_free_inst;
	ret = &opts->func_inst;

	mutex_lock(&usbtmc_ida_lock);

	if (idr_is_empty(&usbtmc_ida.idr)) {
		status = tmc_setup(USBTMC_MINORS);
		if (status) {
			ret = ERR_PTR(status);
			kfree(opts);
			goto unlock;
		}
	}

	opts->minor = tmc_get_minor();
	if (opts->minor < 0) {
		ret = ERR_PTR(opts->minor);
		kfree(opts);
		if (idr_is_empty(&usbtmc_ida.idr))
			tmc_cleanup();
		goto unlock;
	}

	config_group_init_type_name(&opts->func_inst.group, "", &tmc_func_type);
unlock:
	mutex_unlock(&usbtmc_ida_lock);
	return ret;
}

static struct usb_function *tmc_alloc(struct usb_function_instance *fi)
{
	struct tmc_dev	*dev;
	struct f_tmc_opts	*opts;

	opts = container_of(fi, struct f_tmc_opts, func_inst);

	mutex_lock(&opts->lock);
	if (opts->minor >= minors) {
		mutex_unlock(&opts->lock);
		return ERR_PTR(-ENOENT);
	}

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		mutex_unlock(&opts->lock);
		return ERR_PTR(-ENOMEM);
	}

	++opts->refcnt;
	dev->minor = opts->minor;
	mutex_unlock(&opts->lock);

	spin_lock_init(&dev->lock);

	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);

	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);

	INIT_LIST_HEAD(&dev->tx_idle);

	_tmc_dev = dev;	

	dev->tmc_cdev_open = 0;

	dev->usb_tmc_transfer.bTag = 0;
	dev->usb_tmc_transfer.nbytes_rxd = 0;
	dev->usb_tmc_transfer.nbytes_txd = 0;
	
	dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;

	dev->function.name = "tmc";
	dev->function.fs_descriptors = fs_tmc_descs;
	dev->function.hs_descriptors = hs_tmc_descs;
	dev->function.ss_descriptors = ss_tmc_descs;
	dev->function.bind = tmc_function_bind;
	dev->function.unbind = tmc_function_unbind;
	dev->function.set_alt = tmc_function_set_alt;
	dev->function.disable = tmc_function_disable;
	dev->function.setup   = tmc_function_setup;
	dev->function.free_func = tmc_function_free;

	return &dev->function;
}

DECLARE_USB_FUNCTION_INIT(tmc, tmc_alloc_inst, tmc_alloc);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bhoomil Chavda - bhoomil.chavda@gmail.com");

static int tmc_setup(int count)
{
	int status;
	dev_t devt;

	usb_gadget_class = class_create(THIS_MODULE, "usbtmc_gadget");
	if (IS_ERR(usb_gadget_class)) {
		status = PTR_ERR(usb_gadget_class);
		usb_gadget_class = NULL;
		pr_err("unable to create usb_gadget class %d\n", status);
		return status;
	}

	status = alloc_chrdev_region(&devt, 0, count, "USBTMC gadget");
	if (status) {
		pr_err("alloc_chrdev_region %d\n", status);
		class_destroy(usb_gadget_class);
		usb_gadget_class = NULL;
		return status;
	}

	major = MAJOR(devt);
	minors = count;

	return status;
}

static void tmc_cleanup(void)
{
	if (major) {
		unregister_chrdev_region(MKDEV(major, 0), minors);
		major = minors = 0;
	}
	class_destroy(usb_gadget_class);
	usb_gadget_class = NULL;
	kfree(_tmc_dev);
	_tmc_dev = NULL;
}
