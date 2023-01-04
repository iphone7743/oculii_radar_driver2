/* SPDX-License-Identifier: GPL-2.0 */
/*
 * pcan_usbpro.c - the inner parts for pcan-usb-pro support
 *
 * Copyright (C) 2001-2020 PEAK System-Technik GmbH <www.peak-system.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Contact:      <linux@peak-system.com>
 * Author:       Stephane Grosjean <s.grosjean@peak-system.com>
 */
/* #define DEBUG */
/* #undef DEBUG */

#include "src/pcan_common.h"

#ifdef USB_SUPPORT

#include <linux/sched.h>
#include "src/pcan_main.h"
#include "src/pcan_fifo.h"
#include "src/pcan_usbpro.h"

#ifdef NETDEV_SUPPORT
#include "src/pcan_netdev.h"     /* for hotplug pcan_netdev_register() */
#endif

#include "src/pcan_usbpro_fw.h"

#ifdef DEBUG
/* if defined, tell how timestamp are handled */
#define DEBUG_TIMESTAMP
/* if defined, tell how incoming messages and records are handled */
#define DEBUG_DECODE
#define DEBUG_BITTIMING
#define DEBUG_CAN_ONOFF
#else
//#define DEBUG_TIMESTAMP
//#define DEBUG_TS_HWTYPE			HW_USB_PRO
//#define DEBUG_DECODE
//#define DEBUG_BITTIMING
//#define DEBUG_CAN_ONOFF
//#define DEBUG_BUS_LOAD
#endif

/* TODO: check if PCAN-USB Pro sync period is the same */
#define PCAN_USB_FD_SYNC_PERIOD		USEC_PER_SEC

#define PCAN_USBPRO_SYSCLK_HZ		(56 * MHz)

#define PCAN_USBPRO_RTR			0x01
#define PCAN_USBPRO_EXT			0x02

#define PCAN_USBPRO_SR			0x80

/* PCAN-USB-PRO status flags */
#define PCAN_USBPRO_BUS_HEAVY		0x01
#define PCAN_USBPRO_BUS_OVERRUN		0x0c

/* interface private flags */
#define PCAN_USBPRO_WAIT_FOR_CALIBRATION_MSG	0x00000001UL

/* device state flags */
#define PCAN_USBPRO_SHOULD_WAKEUP	0x00000001UL

/* pcan_usbpro_cmd_send_ex() flags */
#define PCAN_USBPRO_SEND_SYNC_MODE	0x00000001
#define PCAN_USBPRO_SEND_GET_TOD	0x00000002

/*
 * Private Data Structures
 */

/* Internal structure used to handle messages sent to bulk urb */

#define PCAN_USB_PRO_CMD_BUFFER_SIZE	512

struct pcan_usbpro_msg {
	u8 *	rec_ptr;
	int	rec_buffer_size;
	int	rec_buffer_len;
	union {
		u16 * rec_counter_read;
		u32 * rec_counter;
		u8 *  rec_buffer;
	} u;
};

const struct pcanfd_bittiming_range sja2010_caps = {

	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,

	.tseg1_min = 1,		/* pcan_timing.c constant for v <= 7.13 */
	.tseg1_max = 16,
	.tseg2_min = 1,		/* pcan_timing.c constant for v <= 7.13 */
	.tseg2_max = 8,

	.sjw_min = 1,
	.sjw_max = 4,		/* Windows driver says 5 here */
};

const pcanfd_mono_clock_device sja2010_clocks = {
	.count = 1,
	.list = {
		[0] = { .clock_Hz = 56*MHz, .clock_src = 56*MHz, },
	}
};

static int pcan_usbpro_devices = 0;

/*
 * Private Functions
 */
#define pcan_usbpro_cmd_send(a,b)\
	        pcan_usbpro_cmd_send_ex(a,b,PCAN_USBPRO_SEND_SYNC_MODE)
#define pcan_usbpro_cmd_send_async(a,b)\
	        pcan_usbpro_cmd_send_ex(a,b,0)

#define pcan_usbpro_calibration_request(a,b)\
	        pcan_usbpro_calibration_request_ex(a,b,1)
#define pcan_usbpro_calibration_request_async(a,b)\
	        pcan_usbpro_calibration_request_ex(a,b,0)

/*
 * u32 pcan_timeval_delta_us(struct timeval *tv0, struct timeval *tv1)
 *
 * do tv1 - tv0, return number of usec
 * Note: tv1 is supposed to be more recent than tv0.
 *       if numerical value tv1 < tv0, supposing 2^32 wrapping
 */
u32 pcan_timeval_delta_us(struct timeval *tv0, struct timeval *tv1)
{
	u32 s, u;

	s = (tv0->tv_sec <= tv1->tv_sec) ? tv1->tv_sec - tv0->tv_sec :
	                               0xffffffff - tv0->tv_sec + tv1->tv_sec;

	if (tv0->tv_usec <= tv1->tv_usec)
		u = tv1->tv_usec - tv0->tv_usec;
	else {
		u = USEC_PER_SEC - tv0->tv_usec + tv1->tv_usec;
		s--;
	}

	return USEC_PER_SEC*s + u;
}

/*
 * static int pcan_usbpro_sizeof_rec(u8 data_type)
 */
static int pcan_usbpro_sizeof_rec(u8 data_type)
{
	switch (data_type)
	{
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_8:
		return sizeof(struct pcan_usbpro_canmsg_rx);
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_4:
		return sizeof(struct pcan_usbpro_canmsg_rx) - 4;
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_0:
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RTR_RX:
		return sizeof(struct pcan_usbpro_canmsg_rx) - 8;

	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_STATUS_ERROR_RX:
		return sizeof(struct pcan_usbpro_canmsg_status_error_rx);

	case DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX:
		return sizeof(struct pcan_usbpro_calibration_ts_rx);

	case DATA_TYPE_USB2CAN_STRUCT_BUSLAST_RX:
		return sizeof(struct pcan_usbpro_buslast_rx);

	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8:
		return sizeof(struct pcan_usbpro_canmsg_tx);
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4:
		return sizeof(struct pcan_usbpro_canmsg_tx) - 4;
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0:
		return sizeof(struct pcan_usbpro_canmsg_tx) - 8;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE:
		return sizeof(struct pcan_usbpro_baudrate);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE:
		return sizeof(struct pcan_usbpro_bus_activity);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE:
		return sizeof(struct pcan_usbpro_silent_mode);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETDEVICENR:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
		return sizeof(struct pcan_usbpro_dev_nr);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETWARNINGLIMIT:
		return sizeof(struct pcan_usbpro_warning_limit);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_EXPLICIT:
		return sizeof(struct pcan_usbpro_lookup_explicit);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_GROUP:
		return sizeof(struct pcan_usbpro_lookup_group);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE:
		return sizeof(struct pcan_usbpro_filter_mode);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETRESET_MODE:
		return sizeof(struct pcan_usbpro_reset_mode);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETERRORFRAME:
		return sizeof(struct pcan_usbpro_error_frame);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS:
		return sizeof(struct pcan_usbpro_error_status);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETREGISTER:
		return sizeof(struct pcan_usbpro_set_register);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETREGISTER:
		return sizeof(struct pcan_usbpro_get_register);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG:
		return sizeof(struct pcan_usbpro_calibration);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG:
		return sizeof(struct pcan_usbpro_buslast);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSTRING:
		return sizeof(struct pcan_usbpro_set_string);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETSTRING:
		return sizeof(struct pcan_usbpro_get_string);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_STRING:
		return sizeof(struct pcan_usbpro_string);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SAVEEEPROM:
		return sizeof(struct pcan_usbpro_save_eeprom);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_USB_IN_PACKET_DELAY:
		return sizeof(struct pcan_usbpro_packet_delay);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_TIMESTAMP_PARAM:
		return sizeof(struct pcan_usbpro_timestamp_param);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_ID:
		return sizeof(struct pcan_usbpro_error_gen_id);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_NOW:
		return sizeof(struct pcan_usbpro_error_gen_now);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_SOFTFILER:
		return sizeof(struct pcan_usbpro_softfiler);

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
		return sizeof(struct pcan_usbpro_set_can_led);

	default:
		pr_info(DEVICE_NAME ": %s(%d): unsupported data type\n",
			__func__, data_type);
	}

	return -1;
}

/*
 * static u8 * pcan_usbpro_msg_init(struct pcan_usbpro_msg *pm,
 *                                       void *buffer_addr, int buffer_size)
 *
 * Initialize PCAN USB-PRO message data structure
 */
static u8 * pcan_usbpro_msg_init(struct pcan_usbpro_msg *pm,
				 void *buffer_addr, int buffer_size)
{
	if (buffer_size < 4)
		return NULL;

	pm->u.rec_buffer = (u8 *)buffer_addr;
	pm->rec_buffer_size = pm->rec_buffer_len = buffer_size;
	pm->rec_ptr = pm->u.rec_buffer + 4;

	return pm->rec_ptr;
}

static u8 * pcan_usbpro_msg_init_empty(struct pcan_usbpro_msg *pm,
				       void *buffer_addr, int buffer_size)
{
	u8 *pr = pcan_usbpro_msg_init(pm, buffer_addr, buffer_size);
	if (pr) {
		pm->rec_buffer_len = 4;
		*pm->u.rec_counter = 0;
	}
	return pr;
}

/*
 * static int pcan_usbpro_msg_add_rec(struct pcan_usbpro_msg *pm,
 *                                    int id, ...)
 *
 * Add one record to a message being built.
 */
static int pcan_usbpro_msg_add_rec(struct pcan_usbpro_msg *pm,
	                                int id, ...)
{
	int l, i;
	u8 *pc;
	va_list ap;

	va_start(ap, id);

	pc = pm->rec_ptr + 1;

	i = 0;
	switch (id) {
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8:
		i += 4;
		/* fall through */
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4:
		i += 4;
		/* fall through */
	case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0:
		*pc++ = (u8 )va_arg(ap, int);	// client
		*pc++ = (u8 )va_arg(ap, int);	// flags
		*pc++ = (u8 )va_arg(ap, int);	// len
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32));  // id
		pc += 4;
		memcpy(pc, va_arg(ap, int *), i);
		pc += i;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETDEVICENR:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		pc += 2; // dummy
		/* CCBT, devicenr */
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32));
		pc += 4;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETWARNINGLIMIT:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETRESET_MODE:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETERRORFRAME:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_TIMESTAMP_PARAM:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_NOW:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		/* onoff, silentmode, warninglimit, filter_mode, reset, mode, */
		/* start_or_end, bit_pos */
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int));
		pc += 2;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_EXPLICIT:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // id_type,mode
		pc += 2;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // id, timeout
		pc += 4;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_GROUP:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // id_type
		pc += 2;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // id_start
		pc += 4;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // id_end
		pc += 4;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETREGISTER:
		*pc++ = (u8 )va_arg(ap, int);	// irq_off
		pc += 2; // dummy
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // address
		pc += 4;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // value
		pc += 4;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // mask
		pc += 4;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETREGISTER:
		*pc++ = (u8 )va_arg(ap, int);	// irq_off
		pc += 2; // dummy
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // address
		pc += 4;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // value
		pc += 4;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_USB_IN_PACKET_DELAY:
		pc++; // dummy
		/* mode, delay */
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // mode
		pc += 2;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		pc++; // dummy
		*pc++ = (u8 )va_arg(ap, int);	// mode
		//*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // prescaler
		pc += 2; // prescale (readonly)
		//*(u16 *)pc = cpu_to_le32((u16 )va_arg(ap, int)); // sampletimequanta
		*(u16 *)pc = cpu_to_le16(4096); // sampletimequanta
		pc += 2;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSTRING:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		*pc++ = (u8 )va_arg(ap, int);	// offset
		*pc++ = (u8 )va_arg(ap, int);	// len
		memcpy(pc, va_arg(ap, u8 *), 60);
		pc += 60;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_GETSTRING:
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SAVEEEPROM:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		pc += 2; // dummy
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_STRING:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		pc += 2; // dummy
		memcpy(pc, va_arg(ap, u8 *), 250);
		pc += 250;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_ID:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // bit_pos
		pc += 2;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // id
		pc += 4;
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); // ok_counter
		pc += 2;
		*(u16 *)pc = cpu_to_le16((u16 )va_arg(ap, int)); //error_counter
		pc += 2;
		break;

	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_SOFTFILER:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		pc += 2;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // accmask
		pc += 4;
		*(u32 *)pc = cpu_to_le32(va_arg(ap, u32)); // acccode
		pc += 4;
		break;

#if 0
	case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
		*pc++ = (u8 )va_arg(ap, int);	// channel
		*(u16 *)pc =  (u16 )va_arg(ap, int); // mode
		pc += 2;
		*(u32 *)pc =  va_arg(ap, u32); // timeout
		pc += 4;
		break;
#endif

	default:
		pr_info(DEVICE_NAME ": %s(): unknown data type %02Xh (%d)\n",
			__func__, id, id);
		pc--;
		break;
	}

	l = pc - pm->rec_ptr;
	if (l > 0) {
		*pm->u.rec_counter = cpu_to_le32(*pm->u.rec_counter+1);
		*(pm->rec_ptr) = (u8 )id;

		pm->rec_ptr = pc;
		pm->rec_buffer_len += l;
	}

	va_end(ap);

	return l;
}

/* URB status (LDD3 p339):
 * 0
 * -ENOENT(2)                The URB was stopped by call to usb_kill_urb()
 * -EOVERFLOW(75)            Too large packet
 * -EINPROGRESS(115)         The URB is always being processed by device
 */
/*
 * static void pcan_usbpro_submit_cmd_complete(struct urb *purb,
 *                                             struct pt_regs *regs)
 *
 * Called when URB has been submitted to hardware
 */
#ifdef LINUX_26
static void pcan_usbpro_submit_cmd_complete(struct urb *purb,
	                                         struct pt_regs *regs)
#else
static void pcan_usbpro_submit_cmd_complete(purb_t purb)
#endif
{
#ifdef PCAN_USB_CMD_PER_DEV
	struct pcandev *dev = purb->context;
	struct pcan_usb_port *u = &dev->port.usb;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(pcan%u) = %d\n",
		__func__, dev->nMinor, purb->status);
#endif
#else
	struct pcan_usb_interface *usb_if = purb->context;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s) = %d\n",
		__func__, usb_if->adapter->name, purb->status);
#endif
#endif

#ifdef PCAN_USB_CMD_PER_DEV
#ifndef PCAN_USB_ALLOC_URBS
	if (purb == &u->urb_cmd_async)
#else
	if (purb == u->urb_cmd_async)
#endif
		atomic_set(&u->cmd_async_complete, 1);
	else
		atomic_set(&u->cmd_sync_complete, 1);
#else
#ifndef PCAN_USB_ALLOC_URBS
	if (purb == &usb_if->urb_cmd_async)
#else
	if (purb == usb_if->urb_cmd_async)
#endif
		atomic_set(&usb_if->cmd_async_complete, 1);
	else
		atomic_set(&usb_if->cmd_sync_complete, 1);
#endif
}

#ifdef PCAN_USB_CMD_PER_DEV
/*
 * static int pcan_usbpro_cmd_send(struct pcandev *dev,
 *                                 struct pcan_usbpro_msg *pum)
 */
static int pcan_usbpro_cmd_send_ex(struct pcandev *dev,
	                                struct pcan_usbpro_msg *pum,
	                                u32 flags)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	struct pcan_usb_port *up = &dev->port.usb;
	struct urb *urb;
	u32 ms_timeout;
	int i = 0, err = 0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(pcan%u, flags=%u): ->EP#%02X\n",
		__func__, dev->nMinor, flags,
		usb_if->pipe_cmd_out.ucNumber);
#endif

	/* don't do anything with non-existent hardware */
	if (!dev->is_plugged)
		return -ENODEV;

	if (!(flags & PCAN_USBPRO_SEND_SYNC_MODE)) {

		if (!atomic_read(&up->cmd_async_complete)) {
			err = -EWOULDBLOCK;
			goto fail;
		}

#ifndef PCAN_USB_ALLOC_URBS
		urb = &up->urb_cmd_async;
	} else {

		urb = &up->urb_cmd_sync;

#else
		urb = up->urb_cmd_async;
	} else {

		urb = up->urb_cmd_sync;
#endif
	}

#ifdef DEBUG
	dump_mem("sent message", &pum->u.rec_buffer[0], pum->rec_buffer_len);
#endif

	FILL_BULK_URB(urb, usb_if->usb_dev,
	              usb_sndbulkpipe(usb_if->usb_dev,
	                              usb_if->pipe_cmd_out.ucNumber),
	              &pum->u.rec_buffer[0], pum->rec_buffer_len,
	              pcan_usbpro_submit_cmd_complete, dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	urb->timeout = TICKS(PCAN_USB_CMD_TIMEOUT);
#endif

	/* sometimes, 1st write fails and needs a retry */
	do {
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err) {
			pr_err(DEVICE_NAME
				": %s(): usb_submit_urb() failure (err %d)\n",
				__func__, err);
			goto fail;
		}

		/* check if we need to wait for completion */
		if (!(flags & PCAN_USBPRO_SEND_SYNC_MODE))
			return 0;

		/* wait until submit is finished, either normal or
		 * thru timeout */
		ms_timeout = get_mtime() + PCAN_USB_CMD_TIMEOUT;
		while (!atomic_read(&up->cmd_sync_complete)) {
			schedule();

			if (get_mtime() >= ms_timeout) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
				usb_kill_urb(urb);
#else
				usb_unlink_urb(urb);
#endif
				err = -ETIMEDOUT;
				break;
			}
		}

		/* reset to 0 for next command */
		atomic_set(&up->cmd_sync_complete, 0);

		if (err == -ETIMEDOUT && ++i < 3)
			continue;

		err = urb->status;

		break;

	} while (1);

fail:

#ifdef DEBUG
	if (err)
		pr_info(DEVICE_NAME
			": %s(pcan%u): failed to send sync cmd (err %d)\n",
			__func__, dev->nMinor, err);
#endif
	return err;
}

#else
/*
 * static int pcan_usbpro_cmd_send(struct pcan_usb_interface *usb_if,
 *                                 struct pcan_usbpro_msg *pum)
 */
static int pcan_usbpro_cmd_send_ex(struct pcan_usb_interface *usb_if,
	                                struct pcan_usbpro_msg *pum,
	                                u32 flags)
{
	int err = 0;
	struct urb *urb;
	u32 ms_timeout;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(pcan%u, flags=%u): ->EP#%02X\n",
		__func__, dev->nMinor, flags,
		usb_if->pipe_cmd_out.ucNumber);
#endif

	/* don't do anything with non-existent hardware */
	if (!usb_if_dev(usb_if, 0)->is_plugged)
		return -ENODEV;

	if (!(flags & PCAN_USBPRO_SEND_SYNC_MODE)) {

		if (!atomic_read(&usb_if->cmd_async_complete))
			return -EWOULDBLOCK;

#ifndef PCAN_USB_ALLOC_URBS
		urb = &usb_if->urb_cmd_async;
#else
		urb = usb_if->urb_cmd_async;
#endif
	} else {

		if (!atomic_read(&usb_if->cmd_sync_complete))
			return -EWOULDBLOCK;

#ifndef PCAN_USB_ALLOC_URBS
		urb = &usb_if->urb_cmd_sync;
#else
		urb = usb_if->urb_cmd_sync;
#endif
	}

#ifdef DEBUG
	dump_mem("sent message", &pum->u.rec_buffer[0], pum->rec_buffer_len);
#endif

	FILL_BULK_URB(urb, usb_if->usb_dev,
	              usb_sndbulkpipe(usb_if->usb_dev,
	                              usb_if->pipe_cmd_out.ucNumber),
	              &pum->u.rec_buffer[0], pum->rec_buffer_len,
	              pcan_usbpro_submit_cmd_complete, usb_if);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	urb->timeout = TICKS(PCAN_USB_CMD_TIMEOUT);
#endif

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err) {
		pr_err(DEVICE_NAME
			": %s(): usb_submit_urb() failure error %d\n",
			__func__, err);
		goto fail;
	}

	/* check if we need to wait for completion */
	if (!(flags & PCAN_USBPRO_SEND_SYNC_MODE))
		return 0;

	// wait until submit is finished, either normal or thru timeout
	ms_timeout = get_mtime() + PCAN_USB_CMD_TIMEOUT;
	while (!atomic_read(&usb_if->cmd_sync_complete)) {
		schedule();

		if (get_mtime() >= ms_timeout) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
			usb_kill_urb(urb);
#else
			usb_unlink_urb(urb);
#endif
			break;
		}
	}

fail:
	err = urb->status;

	atomic_set(&usb_if->cmd_sync_complete, 0);

	return err;
}
#endif

#ifdef PCAN_USB_CMD_PER_DEV
#ifdef LINUX_26
static void pcan_usbpro_read_complete(struct urb *purb, struct pt_regs *regs)
#else
static void pcan_usbpro_read_complete(purb_t purb)
#endif
{
	struct pcan_usb_interface *usb_if = purb->context;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s() = %d\n", __func__, purb->status);
#endif

	/* un-register outstanding urb */
	atomic_set(&usb_if->cmd_sync_complete, 1);
}
#endif

/*
 * static int pcan_usbpro_read_rec(struct pcan_usb_interface *usb_if,
 *                                 void *pv, int maxlen)
 *
 * Wait for a record given in 'pv'.
 * Returns first one read if pv->data_type = 0;
 */
static int pcan_usbpro_read_rec(struct pcan_usb_interface *usb_if,
	                             void *pv, int maxlen)
{
	int err = 0, resubmit = 1;
	u32 ms_timeout;
#ifndef PCAN_USB_ALLOC_URBS
	register purb_t urb = &usb_if->urb_cmd_sync;
#else
	register purb_t urb = usb_if->urb_cmd_sync;
#endif
	u8 *tmp = pcan_malloc(PCAN_USB_PRO_CMD_BUFFER_SIZE, GFP_KERNEL|GFP_DMA);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(): <-EP#%02X\n",
		__func__, usb_if->pipe_cmd_in.ucNumber);
#endif

	if (!tmp) 
		return -ENOMEM;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	urb->timeout = TICKS(PCAN_USB_CMD_TIMEOUT);
#endif

	ms_timeout = get_mtime() + PCAN_USB_CMD_TIMEOUT;
	for (;;) {
		if (resubmit) {
			/* if not using actual length, should use
			*(u32 *)&tmp[0] = 0;
			  instead
			 */

			FILL_BULK_URB(urb, usb_if->usb_dev,
					usb_rcvbulkpipe(usb_if->usb_dev,
					usb_if->pipe_cmd_in.ucNumber),
					tmp, PCAN_USB_PRO_CMD_BUFFER_SIZE,
#ifdef PCAN_USB_CMD_PER_DEV
					pcan_usbpro_read_complete,
#else
					pcan_usbpro_submit_cmd_complete,
#endif
					usb_if);

			err = __usb_submit_urb(urb);
			if (err) {
				pr_err(DEVICE_NAME ": %s() "
					"__usb_submit_urb() err=%d\n",
					__func__, err);
				break;
			}

			resubmit = 0;
		}

		/* Wait for callback to tell us that is finished */
		if (atomic_read(&usb_if->cmd_sync_complete)) {
			int r, rec_counter;
			u8 *pc = tmp;

#ifdef DEBUG
			dump_mem("received message", tmp, urb->actual_length);
#endif

			/* Sometimes, device wakes us up with 0 length msg... */
			if (urb->actual_length > 0) {
				/* Should read into tmp if record is present.
				 * If not, loop again... */
				rec_counter = le32_to_cpu(*(u32 *)pc);

				/* This rec_counter sometimes contains other
				 * info in HIWORD */
				rec_counter &= 0xffff;

				pc += 4;

				for (r = 0; r < rec_counter; r++) {
					int found_rec = 0;
					union pcan_usbpro_rec *pr =
						(union pcan_usbpro_rec *)pc;
					union pcan_usbpro_rec *ps;

					int rec_size =
						pcan_usbpro_sizeof_rec(pr->data_type);
					if (rec_size <= 0) {
						pr_info(DEVICE_NAME
							": %s(): unable to read"
							" record in rsp\n",
							__func__);

						dump_mem("unknown rec", pc,
							urb->actual_length);
						err = -EBADMSG;
						goto fail;
					}

					ps = (union pcan_usbpro_rec *)pv;
					if (ps->data_type != 0) {
						if (pr->data_type ==
								ps->data_type) {

							switch (pr->data_type) {
							case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE:
							case DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE:
							case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
								found_rec = (pr->bus_activity.channel == \
								                              ps->bus_activity.channel);
								break;
							default:
								/* no other criteria defined for data type %02Xh: considering response received */
								found_rec = 1;
								break;
							}
						}
					}
					else found_rec = 1;

					if (found_rec) {
						if (maxlen != rec_size)
							pr_info(DEVICE_NAME
							       ": %s(): maxlen=%d while rec_size=%d\n",
							       __func__,
							       maxlen,
							       rec_size);

						memcpy(pv, pr, maxlen);

						break;
					}

					pc += rec_size;
				}

				/* Record found */
				if (r < rec_counter)
					break;
			}

			/* Did not receive waited record. Resubmit and loop
			 * again... */
			resubmit = 1;
			atomic_set(&usb_if->cmd_sync_complete, 0);

#ifdef DEBUG
			pr_info(DEVICE_NAME ": %s(): resubmitting URB...\n",
				__func__);
#endif
		}

		schedule();

		if (get_mtime() > ms_timeout) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
			usb_kill_urb(urb);
#else
			/* any better solution here for Kernel 2.4 ? */
			usb_unlink_urb(urb);
#endif
			break;
		}
	}

	err = urb->status;

fail:
	atomic_set(&usb_if->cmd_sync_complete, 0);

	pcan_free(tmp);
	return err;
}

#if 0
/*
 * int pcan_usbpro_setget_busload(struct pcandev *dev, u8 mode)
 */
static int pcan_usbpro_setget_busload(struct pcandev *dev, u8 mode)
{
	struct pcan_usbpro_msg ub;

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG,
	                        dev->can_idx,
	                        mode);
#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
}
#endif

/*
 * Hardware Callbacks
 */
static int pcan_usbpro_set_can_on(struct pcandev *dev)
{
	struct pcan_usbpro_msg ub;
	int err;

#if defined(DEBUG_TRACE) || defined(DEBUG_CAN_ONOFF)
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n",
		__func__, dev->adapter->name, dev->can_idx+1);
#endif

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);

	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE,
	                        dev->can_idx,
	                        1);

	/* request BUS_LOAD only when the device is opened */
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG,
	                        dev->can_idx,
	                        1);

#ifdef PCAN_USB_CMD_PER_DEV
	err = pcan_usbpro_cmd_send(dev, &ub);
#else
	err = pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
	return err;
}

static int pcan_usbpro_set_can_off(struct pcandev *dev)
{
	struct pcan_usbpro_msg ub;
	int err;

#if defined(DEBUG_TRACE) || defined(DEBUG_CAN_ONOFF)
	pr_info(DEVICE_NAME ": %s(%s CAN%u): tx_frames_counter=%d nStored=%d\n",
		__func__, dev->adapter->name, dev->can_idx+1,
		dev->tx_frames_counter, dev->writeFifo.nStored);
#endif

	/* wait a bit for last data to be written on CAN bus */
	if (dev->tx_frames_counter > 0)
		pcan_msleep_interruptible(20);

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	/* stop BUSLOAD notification */
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG,
	                        dev->can_idx,
	                        0);

	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE,
	                        dev->can_idx,
	                        0);

#ifdef PCAN_USB_CMD_PER_DEV
	err = pcan_usbpro_cmd_send(dev, &ub);
#else
	err = pcan_usbpro_cmd_send(u->usb_if, &ub);
#endif

	/* wait a bit before deleting USB pipes */
	pcan_msleep_interruptible(20);

	return err;
}

/*
 * int pcan_usbpro_set_silent_mode(struct pcandev *dev, int silent_on)
 */
static int pcan_usbpro_set_silent_mode(struct pcandev *dev, int silent_on)
{
	struct pcan_usbpro_msg ub;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u, silent_on=%d)\n",
		__func__, dev->adapter->name, dev->can_idx+1, silent_on);
#endif
	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE,
	                        dev->can_idx,
	                        silent_on);

#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
}

/*
 * int pcan_usbpro_set_filter_mode(struct pcandev *dev, u16 filter_mode)
 */
static int pcan_usbpro_set_filter_mode(struct pcandev *dev, u16 filter_mode)
{
	struct pcan_usbpro_msg ub;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u, filter_mode=%d)\n",
		__func__, dev->adapter->name, dev->can_idx+1, filter_mode);
#endif

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE,
	                        dev->can_idx,
	                        filter_mode);

#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
}

/*
 * int pcan_usbpro_calibration_request_ex(struct pcan_usb_interface *usb_if,
 *                                        u16 onoff, int sync_mode)
 */
static int pcan_usbpro_calibration_request_ex(struct pcan_usb_interface *usb_if,
	                                    u16 onoff, int sync_mode)
{
	struct pcandev *dev = usb_if_dev(usb_if, 0);
	struct pcan_usbpro_msg ub;
	u32 send_flags = 0;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u, onoff=%d, sync_mode=%d)\n",
		__func__, dev->adapter->name, dev->can_idx+1,
		onoff, sync_mode);
#endif

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                    DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG,
	                    onoff);
	if (onoff) {
		send_flags |= PCAN_USBPRO_SEND_GET_TOD;

		/* Remember we ask for receiving calibration messages */
		usb_if->state |= PCAN_USBPRO_WAIT_FOR_CALIBRATION_MSG;

#ifdef DEBUG_TIMESTAMP
#ifdef DEBUG_TS_HWTYPE
		if (dev->wType == DEBUG_TS_HWTYPE)
#endif
			pr_info(DEVICE_NAME
				": requesting for calibration messages...\n");
#endif
	} else {
		/* Remember we ask for no more receiving any calibration msg */
		usb_if->state &= ~PCAN_USBPRO_WAIT_FOR_CALIBRATION_MSG;

#ifdef DEBUG_TIMESTAMP
#ifdef DEBUG_TS_HWTYPE
		if (dev->wType == DEBUG_TS_HWTYPE)
#endif
			pr_info(DEVICE_NAME
				": stopping calibration messages...\n");
#endif
	}

	if (sync_mode)
		send_flags |= PCAN_USBPRO_SEND_SYNC_MODE;

#ifdef PCAN_USB_CMD_PER_DEV
	err = pcan_usbpro_cmd_send_ex(dev, &ub, send_flags);
#else
	err = pcan_usbpro_cmd_send_ex(usb_if, &ub, send_flags);
#endif
	if (err) {
		if (onoff)
			usb_if->state &= ~PCAN_USBPRO_WAIT_FOR_CALIBRATION_MSG;
		else
			usb_if->state |= PCAN_USBPRO_WAIT_FOR_CALIBRATION_MSG;
	}

	return err;
}

/*
 * int pcan_usbpro_set_can_led(struct pcandev *dev, u8 mode, u32 timeout)
 */
static int pcan_usbpro_set_can_led(struct pcandev *dev, u8 mode, u32 timeout)
{
	struct pcan_usbpro_msg ub;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u, mode=%d, timeour=%d)\n",
		__func__, dev->adapter->name, dev->can_idx+1,
		mode, timeout);
#endif

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED,
	                        dev->can_idx,
	                        mode,
	                        timeout);

#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
}

/*
 * int pcan_usbpro_get_device_nr(struct pcandev *dev, u32 *p_device_nr)
 */
static int pcan_usbpro_get_device_nr(struct pcandev *dev, u32 *p_device_nr)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	struct pcan_usbpro_dev_nr *pdn;
	struct pcan_usbpro_msg ub;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n",
		__func__, dev->adapter->name, dev->can_idx+1);
#endif

	/* Keep record addr in message, to get the response */
	pdn = (struct pcan_usbpro_dev_nr *)\
			pcan_usbpro_msg_init_empty(&ub,
						dev->port.usb.cout_baddr,
						dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR,
	                        dev->can_idx);

#ifdef PCAN_USB_CMD_PER_DEV
	err = pcan_usbpro_cmd_send(dev, &ub);
#else
	err =  pcan_usbpro_cmd_send(usb_if, &ub);
#endif
	if (!err) {
		u32 tmp32;

#ifndef PCAN_USB_CMD_PER_DEV
		// heuristic result - wait a little bit
		mdelay(5);
#endif
		err = pcan_usbpro_read_rec(usb_if, pdn, sizeof(*pdn));
		tmp32 = le32_to_cpu(pdn->serial_num);

		dev->flags &= ~PCAN_DEV_USES_ALT_NUM;
		if (tmp32 != 0xffffffff)
			dev->flags |= PCAN_DEV_USES_ALT_NUM;

		if (p_device_nr)
			*p_device_nr = tmp32;
	}

	return err;
}

/*
 * int pcan_usbpro_set_device_nr(struct pcandev *dev, u32 device_nr)
 */
static int pcan_usbpro_set_device_nr(struct pcandev *dev, u32 device_nr)
{
	struct pcan_usbpro_msg ub;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u, device_nr=%08xh)\n",
		__func__, dev->adapter->name, dev->can_idx+1,
		device_nr);
#endif

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETDEVICENR,
	                        dev->can_idx,
	                        device_nr);

#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
}

/*
 * int pcan_usbpro_set_baudrate(struct pcandev *dev, u32 baudrate)
 */
static int pcan_usbpro_set_baudrate(struct pcandev *dev, u32 baudrate)
{
	struct pcan_usbpro_msg ub;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u, baudrate=%08xh)\n",
		__func__, dev->adapter->name, dev->can_idx+1,
		baudrate);
#endif

	pcan_usbpro_msg_init_empty(&ub, dev->port.usb.cout_baddr,
					dev->port.usb.cout_bsize);
	pcan_usbpro_msg_add_rec(&ub,
	                        DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE,
	                        dev->can_idx,
	                        baudrate);

#ifdef PCAN_USB_CMD_PER_DEV
	return pcan_usbpro_cmd_send(dev, &ub);
#else
	return pcan_usbpro_cmd_send(dev->port.usb.usb_if, &ub);
#endif
}

/*
 * Since Kernel 4.13, transfer data must be dma capable. Stack might not be.
 */
static int pcan_usb_control_msg(struct usb_device *dev, unsigned int pipe,
		__u8 request, __u8 requesttype, __u16 value, __u16 index,
		void *data, __u16 size, int timeout)
{
	int err;
	void *b = pcan_malloc(size, GFP_KERNEL|GFP_DMA);
	if (!b)
		return -ENOMEM;

	memcpy(b, data, size);
	err = usb_control_msg(dev, pipe, request, requesttype, value, index,
			      b, size, timeout);

	memcpy(data, b, size);
	pcan_free(b);

	return err;
}

/*
 * int pcan_usbpro_get_serial_nr(struct pcan_usb_interface *usb_if, u32 *pdwSNR)
 *
 * Retrieve serial number from bootloader info
 */
static int pcan_usbpro_get_serial_nr(struct pcan_usb_interface *usb_if,
					u32 *pdwSNR)
{
	struct pcan_usbpro_bootloader_info bi;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif
	err = pcan_usb_control_msg(usb_if->usb_dev,
		                   usb_rcvctrlpipe(usb_if->usb_dev, 0),
		                   USB_VENDOR_REQUEST_INFO,
		                   USB_DIR_IN | USB_TYPE_VENDOR|USB_RECIP_OTHER,
		                   USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER,
		                   0, &bi, sizeof(bi),
		                   USB_CTRL_GET_TIMEOUT);
	if (err >= 0) {
		if (pdwSNR) {
			*pdwSNR = le32_to_cpu(bi.serial_num_high);
		}

		err = 0;
	} else {
		pr_err(DEVICE_NAME ": %s(): unable to retrieve S/N (err=%d)\n",
			__func__, err);
	}

	return err;
}

/*
 * int pcan_usbpro_open_ex(struct pcandev *dev, struct pcanfd_init *pfdi)
 */
static int pcan_usbpro_open_ex(struct pcandev *dev, struct pcanfd_init *pfdi)
{
	int err = 0;
	u32 ccbt;

	ccbt = (pfdi->nominal.tsam) << 23;
	ccbt |= ((pfdi->nominal.tseg2 - 1) << 20);
	ccbt |= ((pfdi->nominal.tseg1 - 1) << 16);
	ccbt |= ((pfdi->nominal.sjw - 1) << 14);
	ccbt |= (pfdi->nominal.brp - 1);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, bitrate=%u bps CCBT=%08Xh)\n",
	        __func__, dev->can_idx+1, pfdi->nominal.bitrate, ccbt);
#endif
	err = pcan_usbpro_set_baudrate(dev, ccbt);
	if (err)
		goto fail;

	err = pcan_usbpro_set_silent_mode(dev,
				pfdi->flags & PCANFD_INIT_LISTEN_ONLY);
	if (err)
		goto fail;

	/* Set filter mode: 0-> All OFF; 1->bypass */
	err = pcan_usbpro_set_filter_mode(dev, 1);
	if (err)
		goto fail;

	/* No EROR_ACTIVE notification => simulate it now */
	pcan_soft_error_active(dev);
fail:
	return err;
}

/* do blink LED */
static int pcan_usbpro_identify(struct pcandev *dev, u32 delay_ms)
{
	int err;

	if (delay_ms) {

		/* set (FAST) blinking mode */
		err = pcan_usbpro_set_can_led(dev, FW_USBPRO_LED_BLINK_FAST,
					      0xffffffff);
		if (err)
			return err;

		pcan_msleep_interruptible(delay_ms);
	}

	/* restore "normal" mode */
	err = pcan_usbpro_set_can_led(dev, FW_USBPRO_LED_DEVICE, 1);

	return !err ? FW_USBPRO_LED_DEVICE : err;
}

/*
 * Data Messages Handlers
 */
/*
 * int pcan_usbpro_get_frame_number(struct pcan_usb_interface *usb_if)
 */
static int pcan_usbpro_get_frame_number(struct pcan_usb_interface *usb_if)
{
	const int bus_fi = usb_get_current_frame_number(usb_if->usb_dev);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(): bus raw frame index=%d (mask=0x%x)\n",
		__func__, bus_fi, PCAN_USB_PRECISION_MASK);
#endif
	//return bus_fi & PCAN_USB_PRECISION_MASK;
	return bus_fi;
}

#if 0
/*
 * High speed has 125 us per (micro) frame; others are 1 ms per
 *
 * us_per_frame = (HIGH_SPEED) ? 1000 : 125;
 */
static u32 pcan_usbpro_fi_to_us(struct pcan_usb_interface *usb_if, u32 fi)
{
	return (usb_if->usb_dev->speed == USB_SPEED_HIGH) ? 
					fi * US_PER_MSEC : fi * 125;
}
#endif

/* int pcan_usbpro_timestamp_decode(struct pcandev *dev,
 *					u32 ts_low, struct timeval *tv)
 */
struct pcan_timeval *pcan_usbpro_timestamp_decode(struct pcandev *dev,
					u32 ts_low, struct pcan_timeval *tv)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	u32 ts_high = 0;

	/* do nothing (that is, set host time rather than hw timestamps in msgs
	 * while we don't have received the 1st calibration msg */
	if (dev->time_sync.ts_us) {
	
#ifdef DEBUG_TRACE
		pr_info(DEVICE_NAME
			": %s CAN%u: msg.ts=%u.%u vs usb_if.msg.ts=%u.%u\n",
			dev->adapter->name, dev->can_idx+1,
			ts_high, ts_low,
			usb_if->ts_high, usb_if->ts_low);
#endif
		/* since calibration msgs donot use ts_high (always 0), then
		 * we must ignore ts_high read from the record too, and
		 * simulate our own ts_high by handling ts_low wrapping.
		 * Note that calibration msgs ts_low is always <= ts_low read
		 * from the record. */
		ts_high = usb_if->ts_high;
		if (ts_low < usb_if->ts_low) {

			/* if ts_low is lower than sync ts_low, it's maybe
			 * because:
			 * - ts counter has wrapped or
			 * - event has been delayed because of sync
			 * if ts counter has wrapped, then this ts_low 
			 * shouldn't be greater than the sync period */
			if (ts_low < PCAN_USB_FD_SYNC_PERIOD) {

				/* because the driver might receive event(s)
				 * AFTER having received the calibration msg
				 * *BUT* with a timestamp LOWER than the
				 * ts of the calibration msg, we must
				 * consider wrapping only if event ts_low is
				 * REALLY lower than calibration ts_low:
				 * the counter has wrapped if NEXT calibration
				 * ts will wrapp too */
				u32 next_ts_low = usb_if->ts_low +
							PCAN_USB_FD_SYNC_PERIOD;
				if (next_ts_low < usb_if->ts_low) {

					ts_high++;

#ifdef DEBUG_TIMESTAMP
#ifdef DEBUG_TS_HWTYPE
					if (dev->wType == DEBUG_TS_HWTYPE)
#endif
					pr_info(DEVICE_NAME
						": device ts counter wrapped: "
						" %u:%u -> %u.%u\n",
						usb_if->ts_high, usb_if->ts_low,
						ts_high, ts_low);
#endif
				}
			}
		}	
	}

#ifdef DEBUG_TIMESTAMP
#ifdef DEBUG_TS_HWTYPE
	if (dev->wType == DEBUG_TS_HWTYPE)
#endif
		pr_info(DEVICE_NAME
			": %s CAN%u -> MSG[ts=%u.%u]: usb_if[ts=%u.%u]\n",
			dev->adapter->name, dev->can_idx+1,
			ts_high, ts_low, usb_if->ts_high, usb_if->ts_low);
#endif

	if (usb_if->cm_ignore_count < 0) {

		/* special case when calibration msgs are ignored:
		 * first event will set its own timestamps so that we are sure
		 * that both sync and timestamps are based on the same clock.
		 * This time base will be used until next CM */
		if (!dev->time_sync.ts_us)
			pcan_sync_times(dev, ts_low, ts_high, 0);
	}

	/* when time_sync.ts_us is 0, then pcan_sync_decode() fails:
	 * caller should consider host time only. */
	return pcan_sync_decode(dev, ts_low, ts_high, tv) ? tv : NULL;
}

/*
 * int pcan_usbpro_handle_canmsg_rx(struct pcan_usb_interface *usb_if,
 *                                  struct pcan_usbpro_canmsg_rx *rx)
 *
 * Handler of record types:
 * - DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_x
 * - DATA_TYPE_USB2CAN_STRUCT_CANMSG_RTR_RX
 */
static int pcan_usbpro_handle_canmsg_rx(struct pcan_usb_interface *usb_if,
	                                struct pcan_usbpro_canmsg_rx *rx)
{
	const int ctrl_index = (rx->len >> 4) & 0x0f;
	struct pcandev *dev = usb_if_dev(usb_if, ctrl_index);
	struct pcanfd_rxmsg f = {
		.msg = {
			.type = PCANFD_TYPE_CAN20_MSG,
			.flags = PCANFD_MSG_STD,
		},
	};
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u): "
		"rx=[client=0x%02x flags=0x%02x len=0x%02x "
		"timestamp32=0x%08x id=0x%08x]\n",
		__func__, ctrl_index+1,
		rx->client, rx->flags, rx->len, rx->timestamp32,
		le32_to_cpu(rx->id));
#endif
	dev->dwInterruptCounter++;

#if 1
	/* such test is now made by pcan_chardev_rx() func */
#else
	/* Don't send any message when device not opened */
	if (dev->nOpenPaths <= 0) {
		pr_info("%s: incoming message 0x%x (flags=%x) discarded: "
		        "CAN%u not yet opened\n",
		        DEVICE_NAME, rx->id, rx->flags, ctrl_index+1);
		return 0;
	}
#endif
	f.msg.id = le32_to_cpu(rx->id);
	f.msg.data_len = rx->len & 0x0f;

	if (rx->flags & PCAN_USBPRO_RTR)
		f.msg.flags |= PCANFD_MSG_RTR;
	if (rx->flags & PCAN_USBPRO_EXT)
		f.msg.flags |= PCANFD_MSG_EXT;
	if (rx->client & PCAN_USBPRO_SR)
		f.msg.flags |= PCANFD_MSG_SLF;

	memcpy(f.msg.data, rx->data, f.msg.data_len);

	if (pcan_usbpro_timestamp_decode(dev,
					 le32_to_cpu(rx->timestamp32), &f.hwtv))
		f.msg.flags |= PCANFD_HWTIMESTAMP|PCANFD_TIMESTAMP;

	err = pcan_xxxdev_rx(dev, &f);
	if (err > 0)
		dev->port.usb.state |= PCAN_USBPRO_SHOULD_WAKEUP;

	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/*
 * int pcan_usbpro_handle_status_err_rx(struct pcan_usb_interface *usb_if,
 *                              struct pcan_usbpro_canmsg_status_error_rx *er)
 */
static int pcan_usbpro_handle_status_err_rx(struct pcan_usb_interface *usb_if,
	                    struct pcan_usbpro_canmsg_status_error_rx *er)
{
	const u32 raw_status = le32_to_cpu(er->status);
	const int ctrl_index = (er->channel >> 4) & 0x0f;
	struct pcandev *dev = usb_if_dev(usb_if, ctrl_index);
	struct pcanfd_rxmsg f = {
		.msg = {
			.type = PCANFD_TYPE_NOP,
		}
	};
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u): "
	        "rx=[status=0x%04x timestamp32=0x%08x error_frame=0x%08x]\n",
	        __func__, ctrl_index+1,
	        raw_status, le32_to_cpu(er->timestamp32),
	        le32_to_cpu(er->error_frame));
#endif

	dev->dwInterruptCounter++;

	if (raw_status & FW_USBPRO_STATUS_MASK_BUS_S) {
		/* Bus Off */
		pcan_handle_busoff(dev, &f);
	} else {
		dev->rx_error_counter =
				(le32_to_cpu(er->error_frame) >> 16) & 0xff;

		dev->tx_error_counter =
				(le32_to_cpu(er->error_frame) >> 24) & 0xff;

		if (!pcan_handle_error_status(dev, &f,
			(raw_status & FW_USBPRO_STATUS_MASK_ERROR_S), /* EW */
			(dev->rx_error_counter > 127 || /* EP */
					dev->tx_error_counter > 127))) {

			/* no error bit (so, no error, back to active state) */
			pcan_handle_error_active(dev, &f);
		}
	}

	if (raw_status & FW_USBPRO_STATUS_MASK_OVERRUN_S)
		pcan_handle_error_protocol(dev, &f, PCANFD_RX_OVERFLOW);

	if (raw_status & FW_USBPRO_STATUS_MASK_QOVERRUN_S)
		pcan_handle_error_ctrl(dev, &f, PCANFD_RX_OVERFLOW);

	/* Don't send any null message */
	if (f.msg.type == PCANFD_TYPE_NOP)
		return 0;

	if (pcan_usbpro_timestamp_decode(dev,
					 le32_to_cpu(er->timestamp32), &f.hwtv))
		f.msg.flags |= PCANFD_HWTIMESTAMP|PCANFD_TIMESTAMP;

	err = pcan_xxxdev_rx(dev, &f);
	if (err > 0)
		dev->port.usb.state |= PCAN_USBPRO_SHOULD_WAKEUP;

	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

int pcan_usbpro_handle_calibration(struct pcan_usb_interface *usb_if,
					u32 ts_low, u32 dev_frame_index)
{
	const int bus_frame_index = pcan_usbpro_get_frame_number(usb_if);
	struct pcandev *dev0 = usb_if_dev(usb_if, 0);
	int d, dfi;

#ifdef DEBUG_TIMESTAMP
#ifdef DEBUG_TS_HWTYPE
	if (dev0 && dev0->wType == DEBUG_TS_HWTYPE)
#endif
		pr_info(DEVICE_NAME ": %s [CM]: ts=%u "
			"dev#=%u (old=%d) bus#=%d (old=%d)\n",
			usb_if->adapter->name,
			ts_low,
			dev_frame_index & PCAN_USB_PRECISION_MASK,
			usb_if->dev_frame_index & PCAN_USB_PRECISION_MASK,
			bus_frame_index & PCAN_USB_PRECISION_MASK,
			usb_if->bus_frame_index & PCAN_USB_PRECISION_MASK);
#endif

	if (bus_frame_index < 0)
		return bus_frame_index;

	usb_if->bus_frame_index = bus_frame_index;

	if (usb_if->cm_ignore_count > 0) {
		usb_if->cm_ignore_count--;
		return 0;
	}

	/* dev_frame_index is the USB cyclic ms. clock of the device */
	if (usb_if->dev_frame_index < 0) {
		usb_if->dev_frame_index = dev_frame_index;

		usb_if->ts_low = 0;
		usb_if->ts_high = 0;

#ifdef PCAN_USB_USE_AV_DFI
		usb_if->dfi_total = 0;
		usb_if->dfi_count = 1;
#endif
		return 0;
	}

	usb_if->dev_frame_index = dev_frame_index;

	/* if calibration msgs have to be ignored, then reset ts_us so that
	 * first next event will set its own timings */
	if (usb_if->cm_ignore_count < 0)
		ts_low = 0;

	/* device ts_high always 0, so we have to handle it by ourselves: */
	if (ts_low < usb_if->ts_low)
		usb_if->ts_high++;

	usb_if->ts_low = ts_low;

	/* dfi is normally 0. When it is not 0, then it gives the number of
	 * ms to add to local host time. */
	dfi = (dev_frame_index & PCAN_USB_PRECISION_MASK) -
		(bus_frame_index & PCAN_USB_PRECISION_MASK);

	/* convert in µs */
	dfi *= USEC_PER_MSEC;

#ifdef PCAN_USB_USE_AV_DFI
	if (usb_if->dfi_count) {
		usb_if->dfi_total += dfi;
		usb_if->dfi_av = usb_if->dfi_total / usb_if->dfi_count++;
	}

	dfi = usb_if->dfi_av;
#endif

#ifdef DEBUG_TIMESTAMP
#ifdef DEBUG_TS_HWTYPE
	if (dev0 && dev0->wType == DEBUG_TS_HWTYPE)
#endif
		pr_info(DEVICE_NAME ": %s [CM]: usb_if[ts=%u-%u] dfi=%d\n",
			usb_if->adapter->name,
			usb_if->ts_high, usb_if->ts_low, dfi);
#endif

	/* handle time sync in dev[0] even if it is not opened
	 * -50 µs gives times ~ those with PCIe FD for the USB FD devices */
	if (dev0 && pcan_sync_times(dev0, ts_low, usb_if->ts_high, dfi))

		/* do a copy of time_sync object for ALL channels devices */
		for (d = 1; d < usb_if->can_count; d++) {
			struct pcandev *dev = usb_if_dev(usb_if, d);
			if (dev)
				memcpy(&dev->time_sync, &dev0->time_sync,
					sizeof(dev0->time_sync));
		}

	return 0;
}

/*
 * int pcan_usbpro_handle_calibration_msg(struct pcan_usb_interface *usb_if,
 *                                   struct pcan_usbpro_calibration_ts_rx *ts)
 * Handler of record types:
 * - DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX
 */
static int pcan_usbpro_handle_calibration_msg(struct pcan_usb_interface *usb_if,
				struct pcan_usbpro_calibration_ts_rx *ts)
{
	pcan_usbpro_handle_calibration(usb_if,
			le32_to_cpu(ts->timestamp64[1]),
			le32_to_cpu(ts->timestamp64[0]));
	return 0;
}

/*
 * int pcan_usbpro_handle_buslast_rx(struct pcan_usb_interface *usb_if,
 *                                   struct pcan_usbpro_buslast_rx *bl)
 */
static int pcan_usbpro_handle_buslast_rx(struct pcan_usb_interface *usb_if,
	                                 struct pcan_usbpro_buslast_rx *bl)
{
	const int ctrl_index = (bl->channel >> 4) & 0x0f;
	struct pcandev *dev = usb_if_dev(usb_if, ctrl_index);
	struct pcanfd_rxmsg f = {
		.msg = {
			.type = PCANFD_TYPE_STATUS,
			.id = PCANFD_BUS_LOAD,
		}
	};
	int err;

#if defined(DEBUG_TRACE) || defined(DEBUG_BUS_LOAD)
	pr_info(DEVICE_NAME ": %s(CAN%u): "
	        "bl=[buslast_val=0x%04x timestamp=0x%08x]\n",
	        dev->adapter->name, ctrl_index+1,
	        le16_to_cpu(bl->buslast_val), le32_to_cpu(bl->timestamp32));
#endif

	dev->bus_load = (10000UL * le16_to_cpu(bl->buslast_val)) >> 12;

#if 1
	/* no need to decode the timestamp from BUS_LOAD msgs: since 8.6,
	 * the driver is in charge to periodically send an average value to
	 * the application. */
#else
	/* next, prepare the corresponding STATUS message */
	if (pcan_usbpro_timestamp_decode(dev,
					 le32_to_cpu(bl->timestamp32), &f.hwtv))
		f.msg.flags |= PCANFD_HWTIMESTAMP|PCANFD_TIMESTAMP;
#endif
	err = pcan_xxxdev_rx(dev, &f);
	if (err > 0)
		dev->port.usb.state |= PCAN_USBPRO_SHOULD_WAKEUP;

	return err;
}

/*
 * int pcan_usbpro_msg_decode(struct pcan_usb_interface *usb_if,
 *                            u8 *msg_addr, int msg_len)
 *
 * Decode a message from PCAN-USB-PRO.
 */
static int pcan_usbpro_msg_decode(struct pcan_usb_interface *usb_if,
	                          u8 *msg_addr, int msg_len)
{
	int err = 0;
	struct pcan_usbpro_msg usb_msg;
	u8 *rec_ptr, *msg_end;
	u16 rec_count;
	int d;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(msg_len=%d)\n", __func__, msg_len);
#endif
	//dump_mem("received msg", msg_addr, msg_len);

	/* Apply any fragmentation offset to the packet address to seek on */
	/* the (simulated) record count */
	msg_addr -= usb_if->frag_rec_offset;
	msg_len += usb_if->frag_rec_offset;

	/* For USB-PRO, messages MUST be at least 4-bytes length */
	rec_ptr = pcan_usbpro_msg_init(&usb_msg, msg_addr, msg_len);
	if (rec_ptr == NULL) {
		pr_err(DEVICE_NAME ": %s() invalid msg len %d (ignored)\n",
			__func__, msg_len);
		return 0;
	}

#ifdef DEBUG_DECODE
	pr_info(DEVICE_NAME ": decoding incoming message: "
		"rec_counter=%d (0x%08x) len=%d:\n",
		*usb_msg.u.rec_counter_read, *usb_msg.u.rec_counter, msg_len);
#endif

	/* do some init for each controller */
	for (d = 0; d < usb_if->can_count; d++) {
		struct pcandev *dev = usb_if_dev(usb_if, d);
		dev->port.usb.state &= ~PCAN_USBPRO_SHOULD_WAKEUP;
	}

	/* Loop reading all the records from the incoming message */
	msg_end = msg_addr + msg_len;
	for (rec_count=le16_to_cpu(*usb_msg.u.rec_counter_read); rec_count > 0;
	                                                          rec_count--) {
		union pcan_usbpro_rec *pr = (union pcan_usbpro_rec *)rec_ptr;
		int sizeof_rec = pcan_usbpro_sizeof_rec(pr->data_type);

#ifdef DEBUG_DECODE
		pr_info(DEVICE_NAME ": %02u: record=0x%02x (%d) sizeof=%d\n",
			rec_count, pr->data_type, pr->data_type, sizeof_rec);
#endif
		if (sizeof_rec <= 0) {
			pr_info(DEVICE_NAME
				": got unsupported record in rx msg:\n");
			dump_mem("message content", msg_addr, msg_len);
			err = -EOPNOTSUPP;
			break;
		}

		/* Check if the record goes out of current packet */
		if (rec_ptr + sizeof_rec > msg_end) {
			/* Yes, it does: donot handle it here but wait for next
			 * loop decoding on the next packet.
			 * Uses current packet to simulate message header with
			 * the correct record counter */
			rec_ptr -= 4;
			*(u32 *)rec_ptr = cpu_to_le32(rec_count);

			err = rec_count;
			break;
		}

		switch (pr->data_type) {
		case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_8:
		case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_4:
		case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_0:
		case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RTR_RX:
			err = pcan_usbpro_handle_canmsg_rx(usb_if,
							&pr->canmsg_rx);
			if (err < 0)
				goto fail;
			break;

		case DATA_TYPE_USB2CAN_STRUCT_CANMSG_STATUS_ERROR_RX:
			err = pcan_usbpro_handle_status_err_rx(usb_if,
			                           &pr->canmsg_status_error_rx);
			if (err < 0)
				goto fail;
			break;

		case DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX:
			err = pcan_usbpro_handle_calibration_msg(usb_if,
			                                &pr->calibration_ts_rx);
			break;

		case DATA_TYPE_USB2CAN_STRUCT_BUSLAST_RX:
			err = pcan_usbpro_handle_buslast_rx(usb_if,
							&pr->buslast_rx);
			break;

		default:
			pr_info(DEVICE_NAME
				": %s(): unhandled rec 0x%02x(%d)\n",
				__func__, pr->data_type, pr->data_type);
			dump_mem("message content", msg_addr, msg_len);
			break;
		}

		rec_ptr += sizeof_rec;
	}

	/* Always compute next packet offset to seek on (simulated) record
	 * counter. If all records have been processed here, next packet will
	 * start with a correct message header */
	usb_if->frag_rec_offset = (rec_count > 0) ? (msg_end - rec_ptr) : 0;

fail:

	/* check if something is to be waken up */
	for (d = 0; d < usb_if->can_count; d++) {
		struct pcandev *dev = usb_if_dev(usb_if, d);
		if (dev->port.usb.state & PCAN_USBPRO_SHOULD_WAKEUP) {

			dev->dwInterruptCounter++;
#ifdef DEBUG_DECODE
			pr_info(DEVICE_NAME
				": wakeup task reading CAN%u\n", d+1);
#endif
#ifndef NETDEV_SUPPORT
			pcan_event_signal(&dev->in_event);
#endif
		}
	}

	return err;
}

/*
 * int pcan_usbpro_msg_encode(struct pcandev *dev,
 *                            u8 *buffer_addr, int *buffer_size)
 *
 * Reads into CAN fifo and encode data in USB messages.
 *
 * @return -ENODATA  if no more data in CAN fifo,
 *         -ENOSPC if *buffer_size is not large enough to store a TX_x
 *                   record
 *         0         if output buffer is full of TX_x records,
 *         any other -ERR.
 */
static int pcan_usbpro_msg_encode(struct pcandev *dev,
	                          u8 *buffer_addr, int *buffer_size)
{
	const int rec_max_len = \
	   pcan_usbpro_sizeof_rec(DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8);

	int err = 0;
	int buffer_high_water;
	struct pcan_usbpro_msg usb_msg;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(buffer_size=%d) rec_max_size=%d "
	        "msg_in_fifo=%d fifo_empty=%d\n",
		__func__, *buffer_size, rec_max_len,
	        dev->writeFifo.nStored, pcan_fifo_empty(&dev->writeFifo));
#endif

	/* In order to accelerate things... */
	if (pcan_fifo_empty(&dev->writeFifo)) {
		*buffer_size = 0;
		return -ENODATA;
	}

	if (*buffer_size < rec_max_len) {
		pr_err(DEVICE_NAME ": %s(): not enough room to store %d "
			"bytes record into %d bytes buffer\n",
		        __func__, rec_max_len, *buffer_size);

		*buffer_size = 0;
		return -ENOSPC;
	}

	if (!pcan_usbpro_msg_init_empty(&usb_msg, buffer_addr, *buffer_size)) {
		*buffer_size = 0;
		return -ENOSPC;
	}

	/* Loop while sure to have enough room to store any TX_x record */
	buffer_high_water = *buffer_size - rec_max_len;

	while (usb_msg.rec_buffer_len < buffer_high_water) {
		struct pcanfd_txmsg tx;
		u8 data_type, client, len, flags;

		// release fifo buffer and step forward in fifo
		if ((err = pcan_fifo_get(&dev->writeFifo, &tx))) {
			if (err != -ENODATA) {
				pr_err(DEVICE_NAME
					": %s(): can't get data out of "
					"writeFifo, available data=%d err=%d\n",
				       __func__, dev->writeFifo.nStored, err);
			}

			break;
		}

#ifdef MSGTYPE_PARAMETER
		if (tx.msg.flags & (MSGTYPE_STATUS | MSGTYPE_PARAMETER)) {
#else
		if (tx.msg.flags & MSGTYPE_STATUS) {
#endif
			pr_info(DEVICE_NAME
				": %s() CAN msg type %02Xh ignored\n",
				__func__, tx.msg.flags);
			continue;
		}

		if ((tx.msg.flags & MSGTYPE_RTR) || (tx.msg.data_len == 0)) {
			data_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0;
		} else if (tx.msg.data_len <= 4) {
			data_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4;
		} else {
			data_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8;
		}

#if 0
		/* Check if enough room to add one more TX_X */
		if (usb.msg.rec_buffer_len + pcan_usbpro_sizeof_rec(data_type) > \
		     usb_msg.rec_buffer_size) {
			err = -ENOSPC;
			break;
		}
#endif

		len = (dev->can_idx << 4) | (tx.msg.data_len & 0x0f);

		flags = 0;
#ifdef MSGTYPE_SINGLESHOT
		if (tx.msg.flags & MSGTYPE_SINGLESHOT)
			flags |= 0x08;
#endif
		if (tx.msg.flags & MSGTYPE_EXTENDED)
			flags |= 0x02;
		if (tx.msg.flags & MSGTYPE_RTR)
			flags |= 0x01;

		client = 0;

		if (tx.msg.flags & MSGTYPE_SELFRECEIVE) {
			flags |= 0x04;
			client |= PCAN_USBPRO_SR;
		}

		pcan_usbpro_msg_add_rec(&usb_msg,
		                        data_type,
		                        client,
		                        flags,
		                        len,
		                        tx.msg.id,
		                        tx.msg.data);

		dev->tx_frames_counter++;
	}

	/* Don't return rec_buffer_len if no record at all has been copied,
	 * since it is initialized to 4... */
	*buffer_size = (*usb_msg.u.rec_counter == 0) ? 0
						: usb_msg.rec_buffer_len;

	return err;
}

/*
 * int pcan_usbpro_request(struct pcan_usb_interface *usb_if,
 *                         int req_id, int req_value,
 *                         void *req_addr, int req_size)
 * Send USB Vendor type request
 */
int pcan_usbpro_request(struct pcan_usb_interface *usb_if,
			int req_id, int req_value,
			void *req_addr, int req_size)
{
	int err;
	u8 req_type;
	unsigned int p;

	req_type = USB_TYPE_VENDOR | USB_RECIP_OTHER;
	switch (req_id) {
	case USB_VENDOR_REQUEST_FKT:
		/* Host to device */
		p = usb_sndctrlpipe(usb_if->usb_dev, 0);
		break;

	case USB_VENDOR_REQUEST_INFO:
	default:
		/* Device to host */
		p = usb_rcvctrlpipe(usb_if->usb_dev, 0);
		req_type |= USB_DIR_IN;
		memset(req_addr, '\0', req_size);
	}

	err = pcan_usb_control_msg(usb_if->usb_dev,
	                      p,
	                      req_id,
	                      req_type,
	                      req_value,
	                      0, req_addr, req_size,
	                      2*USB_CTRL_GET_TIMEOUT);
	if (err < 0)
		pr_err(DEVICE_NAME ": %s: failed to send usb request "
			"[type=%d value=%d]: err %d\n",
		        usb_if->adapter->name, req_id, req_value, err);

	else {
		//dump_mem("request content", req_addr, req_size);
		err = 0;
	}

	return err;
}

int pcan_usbpro_driver_loaded(struct pcan_usb_interface *usb_if,
	                            int can_lin, int loaded)
{
	int err = 0;

#ifdef USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED
	/* Tell the USB-PRO that the Driver is loaded */
	u8 buffer[16];

	buffer[0] = (u8 )can_lin;	/* Interface CAN=0 LIN=1 */
	buffer[1] = !!loaded;		/* Driver loaded 0/1 */

	err = pcan_usbpro_request(usb_if,
		USB_VENDOR_REQUEST_FKT,
		USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED,
		buffer, sizeof(buffer));
#endif
	return err;
}

/*
 * void pcan_usbpro_cleanup(struct pcandev *dev)
 *
 * Last chance to submit URB before driver removal.
 */
static void pcan_usbpro_cleanup(struct pcandev *dev)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n",
	        __func__, dev->adapter->name, dev->can_idx+1);
#endif

	/* Sometimes, bus off request can't be submit when driver is removed so,
	 * when the device was not properly closed. So, move the bus off request
	 * here to be sure it is sent. */
	pcan_usbpro_set_can_off(dev);

#ifdef USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED
	/* No more need to switch off the LEDs by ourselves!
	 * Fw does it when we notify it from driver unload! */
#else
	/* Switch LED off */
	pcan_usbpro_set_can_led(dev, FW_USBPRO_LED_OFF, 0xffffffff);
#endif

	/* If last device, tell module that driver is unloaded */
	if (dev->can_idx == (usb_if->can_count-1)) {

		/* turn off calibration message */
		pcan_usbpro_calibration_request(usb_if, 0);

		/* Tell module the CAN driver is unloaded */
		pcan_usbpro_driver_loaded(usb_if, 0, 0);
	}
}

/*
 * int pcan_usbpro_ctrl_init(struct pcandev *dev)
 *
 * Do CAN controller specific initialization.
 */
static int pcan_usbpro_ctrl_init(struct pcandev *dev)
{
	struct pcan_usb_interface *usb_if = pcan_usb_get_if(dev);
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n",
	        __func__, dev->adapter->name, dev->can_idx+1);
#endif

	/* set bittiming capabilities */
	dev->bittiming_caps = &sja2010_caps;

	/* now, seems that may avoid to retry... 
	 * request now for 1st calibration message (on last ctrlr only) */
	if (dev->can_idx == (usb_if->can_count-1)) {
		/* Tell device the CAN driver is loaded */
		//pcan_usbpro_driver_loaded(usb_if, 0, 1);

		err = pcan_usbpro_calibration_request(usb_if, 1);
		if (err)
			return err;
	}

#if 0
	pr_info(DEVICE_NAME ": %s channel %d device number=%u\n",
	        usb_if->adapter->name, dev->can_idx+1, dev->device_alt_num);
#endif
	/* set LED in default state (end of init phase) */
	pcan_usbpro_set_can_led(dev, FW_USBPRO_LED_DEVICE, 1);

	return 0;
}

/*
 * void pcan_usbpro_free(struct pcan_usb_interface *usb_if)
 */
static void pcan_usbpro_free(struct pcan_usb_interface *usb_if)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s)\n", __func__, usb_if->adapter->name);
#endif

	/* release adapter object */
	usb_if->adapter = pcan_free_adapter(usb_if->adapter);
	pcan_usbpro_devices--;
}

/*
 * int pcan_usbpro_init(struct pcan_usb_interface *usb_if)
 *
 * Do device specifc initialization.
 */
int pcan_usbpro_init(struct pcan_usb_interface *usb_if)
{
	int c;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif
	usb_if->adapter = pcan_alloc_adapter("PCAN-USB Pro",
						pcan_usbpro_devices++, 2);
	if (!usb_if->adapter)
		return -ENOMEM;

	/* Set PCAN-USB-PRO hardware specific callbacks */
	usb_if->device_ctrl_init = pcan_usbpro_ctrl_init;
	usb_if->device_get_snr = pcan_usbpro_get_serial_nr;
	usb_if->device_msg_decode = pcan_usbpro_msg_decode;
	usb_if->device_free = pcan_usbpro_free;

	usb_if->device_ctrl_cleanup = pcan_usbpro_cleanup;
	usb_if->device_ctrl_open_fd = pcan_usbpro_open_ex;
	usb_if->device_ctrl_set_bus_on = pcan_usbpro_set_can_on;
	usb_if->device_ctrl_set_bus_off = pcan_usbpro_set_can_off;
	usb_if->device_ctrl_set_dnr = pcan_usbpro_set_device_nr;
	usb_if->device_ctrl_get_dnr = pcan_usbpro_get_device_nr;
	usb_if->device_ctrl_msg_encode = pcan_usbpro_msg_encode;

#ifdef USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE
	/* Firmware info */
	{
		struct pcan_usbpro_ext_firmware_info fi;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE,
		                        &fi, sizeof(fi)) >= 0) {
			dev_info(&usb_if->usb_intf->dev,
			       "%s fw v%d.%d.%d (%02d/%02d/%02d)\n",
			       usb_if->adapter->name,
			       fi.version[0], fi.version[1], fi.version[2],
			       fi.day, fi.month, fi.year);

#ifdef DEBUG
			pr_info(DEVICE_NAME ": type=%08xh\n", fi.fw_type);
#endif
			usb_if->hw_ver.major = fi.version[0];
			usb_if->hw_ver.minor = fi.version[1];
			usb_if->hw_ver.subminor = fi.version[2];

			usb_if->adapter->hw_ver = usb_if->hw_ver;
		}
	}
#endif

#ifdef USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER
	/* Bootloader info */
	{
		struct pcan_usbpro_bootloader_info bi;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER,
		                        &bi, sizeof(bi)) >= 0) {
#ifdef DEBUG
			pr_info(DEVICE_NAME
			       ": bootloader v%d.%d.%d (%02d/%02d/%02d)\n",
			       bi.version[0], bi.version[1], bi.version[2],
			       bi.day, bi.month, bi.year);

			pr_info(DEVICE_NAME
			       ": S/N=%08Xh.%08Xh hw=%08xh.%08xh\n",
			       bi.serial_num_high, bi.serial_num_low,
			       bi. hw_type, bi.hw_rev);
#endif
			usb_if->ucRevision = (u8 )bi.hw_rev;
		}
	}
#endif

#if 0//def USB_VENDOR_REQUEST_wVALUE_INFO_DEVICENR
	/* Device Number */
	{
		struct pcan_usbpro_device_nr dnr;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_DEVICENR,
		                        &dnr, sizeof(dnr)) >= 0) {
			pr_info(DEVICE_NAME ": Device #%08xh\n",
				dnr.device_nr);

			usb_if->ucHardcodedDevNr = (u8 )dnr.device_nr;
		}
	}
#endif

#ifdef USB_VENDOR_REQUEST_wVALUE_INFO_CPLD
	/* CPLD */
	{
		struct pcan_usbpro_cpld_info ci;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_CPLD,
		                        &ci, sizeof(ci)) >= 0) {
#ifdef DEBUG
			pr_info(DEVICE_NAME ": CPLD=%08xh\n", ci.cpld_nr);
#endif
		}
	}
#endif

#if 0//def USB_VENDOR_REQUEST_wVALUE_INFO_MODE
	/* Mode */
	{
		struct pcan_usbpro_info_mode mi;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_MODE,
		                        &mi, sizeof(mi)) >= 0) {
			pr_info(DEVICE_NAME ": mode[%d] = %02xh\n",
				dev->can_idx, mi.mode[dev->can_idx]);
		}
	}
#endif

#if 0//def USB_VENDOR_REQUEST_wVALUE_INFO_TIMEMODE
	/* Time Mode */
	{
		struct pcan_usbpro_time_mode tm;

		if (pcan_usbpro_request(usb_if,
		                        USB_VENDOR_REQUEST_INFO,
		                        USB_VENDOR_REQUEST_wVALUE_INFO_TIMEMODE,
		                        &tm, sizeof(tm)) >= 0) {
			pr_info(DEVICE_NAME ": Time mode %d flags=%04xh\n",
				tm.time_mode, tm.flags);
		}
	}
#endif

	usb_if->cm_ignore_count = 3;
	usb_if->dev_frame_index = -1;
	usb_if->bus_frame_index = pcan_usbpro_get_frame_number(usb_if);

	/* request for hardware periodic calibration messages is done now when
	 * last controller is initialized */
	//pcan_usbpro_calibration_request_async(usb_if, 1);

	/* setup identification callback */
	for (c = 0; c < usb_if->can_count; c++) {
		usb_if_dev(usb_if, c)->device_identify = pcan_usbpro_identify;
	}

	/* Tell module the CAN driver is loaded */
	pcan_usbpro_driver_loaded(usb_if, 0, 1);

	return 0;
}
#endif /* USB_SUPPORT */
