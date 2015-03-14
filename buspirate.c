/* 
 * Bus Pirate I2C bus driver
 *
 * This is an I2C bus driver for the Dangerous Prototypes "Bus Pirate" board
 * (http://dangerousprototypes.com/docs/Bus_Pirate).  The Bus Pirate is bus
 * analyzer and master for a variety of embedded bus protocols (I2C, SPI, JTAG,
 * etc).  This driver implements only the I2C side.
 *
 * The BP uses an onboard FTDI chip to present itself to Linux as a /dev/ttyUSB*
 * serial port. As a result, this driver is implemented on top of that serial
 * port as a line discipline driver.  To instantiate the I2C bus, simply
 * register the N_BUSPIRATE line discipline on the BP's serial port.  The I2C
 * bus will appear after that.
 *
 * This driver assumes your I2C device needs the BP's integrated pull-up
 * resistors and voltage rails enabled, and automatically turns them on/off.
 *
 * TODO:
 * - Cleanup close() to properly quiesce the BP and any I2C transactions in
 *   progress.
 * - Properly handle unplugging the BP while the driver is in use.
 * - Optimize I2C block writes to send more than one byte at a time.
 * - Use a better I2C bus name than "Bus Pirate I2C".  Maybe incorporate the
 *   TTY name into it.
 *
 * Copyright (C) 2015 Tony Hutter <tony.hutter at gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
  */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tony Hutter");
MODULE_DESCRIPTION("Bus Pirate I2C driver");

struct buspirate {
	struct tty_struct *tty;
	struct i2c_adapter adap;
	struct device *dev;

	/* Amount of time to wait for the next byte to be received */
#define BUSPIRATE_RX_TIMEOUT	20	/* milliseconds */

	struct timer_list recv_timer;

	u8 *rx_buf;		/* Pointer to buffer to store RX data */
	unsigned int rx_len;	/* Number of RX bytes we need to read before
				 * marking current transaction as complete.
				 * Use 0 here if you don't know how many bytes
				 * you will be receiving in the current
				 * transaction. */
	unsigned int rx_read;	/* Number of RX bytes we've received in the
			 	 * current transaction. */

	struct completion data_comp;
	struct semaphore sem;
	atomic_t expecting;	/* Set when we're expecting data to arrive.
				 * Used to detect spurious data. */
	int shutdown;		/* Crude flag to indicate that we're in the
				 * process of shutting down the I2C bus. */ 

	/* BP binary mode commands, also used by our state machine */
#define BUSPIRATE_MODE_TEXT	-2	/* Text mode */
#define BUSPIRATE_MODE_UNKNOWN	-1	/* Initial mode */
#define BUSPIRATE_MODE_BITBANG	0	/* Binary bitbang */
#define BUSPIRATE_MODE_I2C	2	/* Binary I2C */
	s8 mode;

	u8 peripherals_enabled;	/* Flag to record if power and pull-ups are enabled */
};

/* BP binary I2C commands */
#define BUSPIRATE_I2C_START	0x02
#define BUSPIRATE_I2C_STOP	0x03
#define	BUSPIRATE_I2C_READ	0x04
#define BUSPIRATE_I2C_ACK	0x06
#define BUSPIRATE_I2C_NACK	0x07
#define BUSPIRATE_I2C_BULK	0x10	/* Bulk write */

static u32 buspirate_i2c_func(struct i2c_adapter *adap)
{
    u32 rc =  I2C_FUNC_SMBUS_QUICK |
           I2C_FUNC_SMBUS_BYTE |
           I2C_FUNC_SMBUS_BYTE_DATA |
           I2C_FUNC_SMBUS_WORD_DATA |
           I2C_FUNC_SMBUS_BLOCK_DATA;

	return rc;
};

/**
 * buspirate_recv_timer_callback() - Timeout callback for read data
 * @data:	Pointer to BP device
 * 
 * Read timeout callback function used by buspirate_write_and_read()
 */
static void buspirate_recv_timer_callback(unsigned long data)
{
	struct buspirate *bp = (struct buspirate *) data;
	complete(&bp->data_comp);
}

/**
 * buspirate_receive_buf() - Receive "interrupt" for RX data
 * @tty:	TTY device
 * @rx_buf:	RX data
 * @fp:		File pointer
 * @count:	Size of rx_buf in bytes
 * 
 * This is the TTY's version of a receive "interrupt".  Any time we receive
 * a block of data, this gets called.
 */
static void buspirate_receive_buf(struct tty_struct *tty,
						const unsigned char *rx_buf,
						char *fp, int count)
{
	struct buspirate *bp = tty->disc_data;
	struct device *dev;

	/* Sanity */
	if (unlikely(!bp))
		goto end;

	dev = &bp->adap.dev;

	if (atomic_read(&bp->expecting) == 0) {
		/* Spurious data received.  If you Ctrl-C a transfer, you
		 * may get it, so not a big deal.  Just ignore it. */
		dev_info(dev, "Got %d bytes spurious data\n", count);
		goto end;
	}

	/* We got data in, but do we care about it? */
	if (bp->rx_buf) {
		/* We do care about it.  Save it. */
		if (bp->rx_len && (bp->rx_read + count > bp->rx_len)) {
			/* Data won't fit in our buffer - truncate it */
			dev_dbg(dev, "Buffer overrun, %d bytes truncated to %d\n",
					count, bp->rx_len - bp->rx_read);
			count = bp->rx_len - bp->rx_read;
		}
		memcpy(&bp->rx_buf[bp->rx_read], rx_buf, count);
	}
	bp->rx_read += count;

	if ((bp->rx_len > 0) && (bp->rx_read == bp->rx_len)) {
		/* We've got all the data we wanted.  Signal we're done. */
		del_timer_sync(&bp->recv_timer);
		complete(&bp->data_comp);
	} else {
		/* Keep waiting for more data */
		mod_timer(&bp->recv_timer, jiffies + msecs_to_jiffies(BUSPIRATE_RX_TIMEOUT));
	}

end:
	return;
}

/**
 * buspirate_wait_for_data() - Wait for data to come back (if any)
 * @bp:		BP device
 *
 * Wait for data to come in after a send.  If bp->rx_len is 0, then we don't
 * know how much data is coming back, so just wait for BUSPIRATE_RX_TIMEOUT
 * milliseconds after every RX byte for more data.  If no more data comes in
 * after that timeout, then assume the transfer is finished.
 */
static unsigned long buspirate_wait_for_data(struct buspirate *bp)
{
	mod_timer(&bp->recv_timer, jiffies + msecs_to_jiffies(BUSPIRATE_RX_TIMEOUT));
	return wait_for_completion_interruptible(&bp->data_comp);
}

/**
 * buspirate_write_and_read() - Send out data and optionally wait for data back
 * @bp:		BP device
 * @tx_buf:	Pointer to data to send, if any (NULL otherwise).
 * @tx_len:	Length of data to send in bytes (0 otherwise).
 * @rx_buf:	Pointer to receive data buffer, if any.  NULL to ignore the 
 *		contents of the data being returned.
 * @rx_len:	Length of data to receive, in bytes.  If you don't know or
 *		don't care how much data is coming back, use 0 here.  0 will
 *		keep reading data until the sender stops sending, as determined
 *		by an idle RX for BUSPIRATE_RX_TIMEOUT milliseconds.  If
 *		you know how much data is coming back, but don't care what the
 *		data is, you should still specify rx_len, as it will avoid
 *		having to wait for a RX timeout to mark the end of the
 *		transmission.  If you know that no bytes are coming back, you
 *		should use 0 here.  This will still incur the timeout, but
 *		shouldn't be an issue in practice, as the BP almost always
 *		returns something after a transmission.
 *
 * Returns the number of bytes read back, or negative value on error.
 */
static int buspirate_write_and_read(struct buspirate *bp, const u8 *tx_buf,
				int tx_len, u8 *rx_buf, int rx_len)
{
	int rc = 0;
	int actual;
	int tx_sent = 0;

	/* Make sure we're the only ones using the device */
	rc = down_interruptible(&bp->sem);
	if (rc) {
		/* User probably Ctrl-C'd the transfer.  */
		rc = -EINTR;
		goto end;
	}

	bp->rx_len = rx_len;
	bp->rx_read = 0;
	bp->rx_buf = rx_buf;
	init_completion(&bp->data_comp);

	/* We can have valid RX data start coming while were calling
	 * bp->tty->ops->write(), so set a flag to tell buspirate_receive_buf()
	 * that it's not spurious data. */
	atomic_set(&bp->expecting, 1);

	/* Send out data */
	while (tx_sent < tx_len) {
		actual = bp->tty->ops->write(bp->tty, &tx_buf[tx_sent], tx_len - tx_sent);
		tty_wait_until_sent(bp->tty, 0);
		tx_sent += actual;
	}

	/* Everything is sent out.  Wait for remaining data to come in. */
	buspirate_wait_for_data(bp);
	atomic_set(&bp->expecting, 0);

	rc = bp->rx_read;

	up(&bp->sem);
end:
	return rc;
}

static int buspirate_write_str_and_read(struct buspirate *bp, const char *str,
						u8 *rx_buf, int rx_len) {
	return buspirate_write_and_read(bp, str, strlen(str) + 1, rx_buf, rx_len);
}

static int buspirate_writeb_and_read(struct buspirate *bp, u8 byte,
						u8 *rx_buf, int rx_len) {
	return buspirate_write_and_read(bp, &byte, 1, rx_buf, rx_len);
}

/**
 * buspirate_writeb_and_check()- Write a single byte out and check response 
 * 
 * @bp:		BP device
 * @byte:	Byte to write out
 * @expected:	Value of byte you expect to get back
 *
 * Write a single byte and check for a specific one byte response.  This is
 * a convenience function used to send the BP a command and look for its
 * response code.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int buspirate_writeb_and_check(struct buspirate *bp, u8 byte,
						u8 expected)
{
	int rc;
	u8 resp;
	rc = buspirate_writeb_and_read(bp, byte, &resp, 1);
	if (rc != 1) {
		dev_dbg(bp->dev, "Expected one byte, got %d bytes\n", rc);
		rc = -EIO;
	} else if (expected != resp) {
		rc = -EIO;
		dev_dbg(bp->dev, "Expected 0x%x, got 0x%x\n", expected, resp);
	} else {
		/* No errors */
		rc = 0;
	}

	return rc;
}

/**
 * buspirate_enter_mode() - Enter a specific BP mode.
 * @bp:		BP device
 * @mode:	Mode
 *
 * Enter a specific BP mode.  We assume the BP is not in text mode when
 * calling this.
 *
 * Returns 0 on success, non-zero otherwise
 */
static int buspirate_enter_mode(struct buspirate *bp, s8 mode)
{
	/* Strings returned by BP after we send a mode command */
	const char *ret_str[] = {
		[BUSPIRATE_MODE_BITBANG] = "BBIO",
		[BUSPIRATE_MODE_I2C] = "I2C"
	};

	/* buf[] must always be large enough to hold the largest string in
	 * ret_str[], plus 1 more byte. */
	char buf[6] = {0};
	int rc = 0;

	if (mode >= ARRAY_SIZE(ret_str) || mode < 0) {
		dev_err(bp->dev, "Invalid mode %d\n", mode);
		rc = -EINVAL;
		goto end;
	}

	/* Are we already in the right mode? */
	if (mode == bp->mode)
		goto end;
		
	/* The return strings all have a one character version number affixed
	 * to them. So we'll really see "BBIO1", "SPI1", and "I2C1", returned.
	 * That's why we add the extra byte after the strlen() */
	buspirate_writeb_and_read(bp, mode, buf, strlen(ret_str[mode]) + 1);

	if (strnstr(buf, ret_str[mode], sizeof(buf)) != buf) {
		rc = -EINVAL;
		goto end;
	}

	/* Success */
	bp->mode = mode;
end:
	return rc;
}

/**
 * buspirate_set_tty_baud() - Set TTY speed
 * @tty:	TTY device
 * @baud:	Baud rate
 *
 * This sets the TTY speed on the Linux side (not the BP side).  Use
 * buspirate_set_bp_and_tty_baud() to set them both.
 */
static void buspirate_set_tty_baud(struct tty_struct *tty, speed_t baud)
{
	struct ktermios new_termios;
	new_termios = tty->termios;	
	tty_termios_encode_baud_rate(&new_termios, baud, baud);
	tty_set_termios(tty, &new_termios);
}


/**
 * buspirate_enter_initial_bitbang_mode() - Set the BP to a known initial state
 * @bp:	BP device
 *
 * We have no idea at startup what mode the Bus Pirate is in, but we know
 * know that we can get into we can get into "bitbang" mode from any mode.
 * So enter bitbang mode and use that as a starting off point to enter other
 * modes.
 *
 * Returns 0 on success, non-zero otherwise
 */
static int buspirate_enter_initial_bitbang_mode(struct buspirate *bp)
{
	struct device *dev = &bp->adap.dev;
	int rc = -EINVAL;
	int tries = 21;

	/* "Some terminals send a NULL character (0x00) on start-up, causing
	 * the Bus Pirate to enter binary mode when it wasn't wanted. To get
	 * around this, you must now enter 0x00 at least 20 times to enter
	 * raw bitbang mode." */
	do {
		rc = buspirate_enter_mode(bp, BUSPIRATE_MODE_BITBANG);
		if (rc == 0)
			break;
	} while (--tries);

	if (rc == 0)
		dev_dbg(dev, "Entered bitbang mode on try %d\n", tries);

	return rc;
}

/**
 * buspirate_set_bp_and_tty_baud() - Set Bus Pirate and TTY baud rate
 * @bp:		BP device
 * @baud:	Baud rate
 *
 * Set BP's PIC UART speed and our TTY speed.
 *
 * Returns 0 on success, non-zero otherwise.
 */ 
static int buspirate_set_bp_and_tty_baud(struct buspirate *bp, speed_t baud)
{
	struct device *dev = &bp->adap.dev;
	int rc = 0;
	const char *brg;

	switch (baud) {
	/* Default BP speed */
	case 115200:
		brg = "34\n";
		break;

	/* Highest speed BP runs at (found though experimentation). */
	case 460800:
		brg = "8\n";
		break;
	default:
		dev_err(dev, "Unsupported baud %d\n", baud);
		rc = -EINVAL;
		goto end;
	}

	/* Set baud rate by manually setting the BRG divisor */
	buspirate_write_str_and_read(bp, "\n", NULL, 0);
	buspirate_write_str_and_read(bp, "b\n", NULL, 0);
	buspirate_write_str_and_read(bp, "10\n", NULL, 0);
	buspirate_write_str_and_read(bp, brg, NULL, 0);

	/* After entering in the BRG, the buspirate will say:
	 *
	 *	"Adjust your terminal
	 *	 Space to continue"
	 *
	 * So send a space. */
	buspirate_set_tty_baud(bp->tty, baud);
	buspirate_writeb_and_read(bp, ' ', NULL, 0);

end:
	return rc;
}

/**
 * buspirate_reset - Reset the BP back to its defaults
 *
 * @bp:	BP device
 *
 * Reset the Bus Pirate.  Afterwards it will in text mode and set to the
 * default 115200 baud rate.
 */
static void buspirate_reset(struct buspirate *bp) {
	buspirate_enter_initial_bitbang_mode(bp);

	/* Exit binary mode and reset.  We will be in text mode afterwards */
	buspirate_writeb_and_read(bp, 0x0f, NULL, 0);
	bp->mode = BUSPIRATE_MODE_TEXT;
}

/**
 * buspirate_firsttime_setup() - Setup initial BP configuration
 * @bp:		BP device
 *
 * If we've never talked to this BP before, run though all these steps
 * to get it to a sane initial state.  After this is run, the BP will be in
 * binary bitbang mode.  Normally, we would do something like this this in
 * open(), but can't since receive_buf() isn't available at that point.
 */
static void buspirate_firsttime_setup(struct buspirate *bp)
{
	buspirate_reset(bp);

	/* Talk to the BP at the fastest speed it can handle (460800).  This
	 * was found though experimentation. */ 
	buspirate_set_bp_and_tty_baud(bp, 460800);

	/* Put ourselves back in binary bitbang mode so we can quickly switch
	 * to the next mode. */
	buspirate_enter_initial_bitbang_mode(bp);
}

/**
 * buspirate_i2c_set_peripherals() - Enable/disable pull-ups and power rails
 * @bp:		BP device
 * @enable:	1 = enable pull-ups/power, 0 = disable
 *
 * Turn on/off power and pull-ups on the BP.  Most devices hooked to the BP will
 * be using them, so we enable it by default.
 *
 * Returns 0 on success, non-zero otherwise.
 */
static int buspirate_i2c_set_peripherals(struct buspirate *bp, int enable)
{
	int rc = 0;

	/* 0100wxyz – Configure peripherals w=power, x=pull-ups, y=AUX, z=CS
	 * So, 0x4C = Enable power and pull-ups, 0x40 = disable */
	const u8 cmds[] = {0x40, 0x4C};
	if (bp->peripherals_enabled)
		goto end;

	buspirate_enter_mode(bp, BUSPIRATE_MODE_I2C);

	/* BP responds with 0x1 on success. */
	if (buspirate_writeb_and_check(bp, cmds[enable] , 0x1)) {
		rc = -EIO;
	} else {
		bp->peripherals_enabled = 1;
	}
end:
	return rc;
}



/**
 * buspirate_i2c_single_xfer() - Do a single I2C transaction
 * @adap:	I2C adapter
 * @msg:	Single I2C message
 *
 * Return 0 if message sent, non-zero otherwise.
 */
static int buspirate_i2c_single_xfer(struct i2c_adapter *adap,
		struct i2c_msg *msg)
{
	struct buspirate *bp = i2c_get_adapdata(adap);
	struct device *dev = &adap->dev;
	int rc = 0;
	u8 *tx_buf = NULL;
	u16 write_len, read_len;
	int i;
	u8 addr;
	u8 tx_tmp[2];
	u8 rx_tmp[2];

	/* Sanity */
	if (!msg)
		goto end;

	/* BP allows for 4096 byte writes & reads, but that's technically out
	 * of I2C spec (32B or less).  */
	if ((msg->len > 32)) {
		dev_dbg(dev, "I2C msg length > 32! (got %d)\n",
			msg->len);
		rc = -EINVAL;
		goto end;
	}

	/* Send start bit */
	if ((rc = buspirate_writeb_and_check(bp, BUSPIRATE_I2C_START, 0x1))) {
		dev_dbg(dev, "%s: Couldn't send start, rc %d\n", __func__, rc);
		rc = -EIO;
		goto end;
	}

	if (msg->flags & I2C_M_RD) {
		/* address byte only */
		write_len = 1;
		read_len = msg->len;
		addr = (msg->addr << 1) | 1;
	} else {
		/* address byte + data */
		write_len = 1 + msg->len;
		read_len = 0;
		addr = (msg->addr << 1);
	}
	
	tx_buf = kzalloc(write_len, GFP_KERNEL);
	if (!tx_buf) {
		rc = -ENOMEM;
		goto end;
	}

	tx_buf[0] = addr;
	memcpy(&tx_buf[1], msg->buf, msg->len);
	
	for (i = 0; i < write_len; i++) {
		/* TODO: send more than one byte per bulk transfer; BP
		 * supports up to 16 bytes. */
		tx_tmp[0] = BUSPIRATE_I2C_BULK;
		tx_tmp[1] = tx_buf[i];
		if ((rc = buspirate_write_and_read(bp, tx_tmp, 2, rx_tmp, 2)) < 1) {
			dev_dbg(dev, "%s: Couldn't write out\n", __func__);
			rc = -EIO;
			goto end;
		}

		/* "BP replies 0×01 to the bulk I2C command. After each data
		 * byte the Bus Pirate returns the ACK (0x00) or NACK (0x01)
		 * bit from the slave device." */
		if (!(rx_tmp[0] == 0x1 && rx_tmp[1] == 0)) {
			dev_dbg(dev, "%s: Sending error (reply: 0x%x 0x%x)\n",
				__func__, rx_tmp[0], rx_tmp[1]);
			rc = -EIO;
			goto end;
		}
	}
	
	for (i = 0; i < read_len; i++) {
		if ((rc = buspirate_writeb_and_read(bp, BUSPIRATE_I2C_READ, rx_tmp, 1)) < 1) {
			dev_dbg(dev, "%s: Couldn't read back I2C data\n", __func__);
			rc = -EIO;
			goto end;
		}
		msg->buf[i] = rx_tmp[0];
		if ((rc = buspirate_writeb_and_read(bp, BUSPIRATE_I2C_ACK, rx_tmp, 1)) < 1) {
			dev_dbg(dev, "%s: Couldn't ACK\n", __func__);
			rc = -EIO;
			goto end;
		}
	}
	rc = 0;
end:
	if (tx_buf)
		kfree(tx_buf);

	return rc;
}

s32 buspirate_i2c_xfer (struct i2c_adapter *adap, struct i2c_msg *msgs,
                        int count)
{
	struct buspirate *bp = i2c_get_adapdata(adap);
	int i = 0;
	int rc = 0;

	if (!msgs || bp->shutdown)
		goto end;
	
	if (bp->mode == BUSPIRATE_MODE_UNKNOWN) {
		buspirate_firsttime_setup(bp);
		buspirate_enter_mode(bp, BUSPIRATE_MODE_I2C);

		 /* Enable BP's pull-ups and voltage to I2C device*/
		buspirate_i2c_set_peripherals(bp, 1);
	}

	for (i = 0; i < count; i++) {
		if ((rc = buspirate_i2c_single_xfer(adap, &msgs[i]))) {
			/* Some sort of failure */
			break;
		}
	}
	buspirate_writeb_and_check(bp, BUSPIRATE_I2C_STOP, 0x1);

	/* If rc is 0, then there were no errors and we should the number of
	 * successfully sent messages.  Otherwise, return negative error. */
end:
	return rc == 0 ? i : rc;
};

static struct i2c_algorithm buspirate_algo = {
	.master_xfer = buspirate_i2c_xfer,
	.functionality  = buspirate_i2c_func,
};


static int buspirate_init_i2c(struct buspirate *bp)
{
	/* Setup I2C */
	bp->adap.owner = THIS_MODULE;
	bp->adap.algo = &buspirate_algo;
	strncpy(bp->adap.name, "Bus Pirate I2C", sizeof(bp->adap.name));
	bp->adap.dev.parent = bp->dev;
	bp->mode = BUSPIRATE_MODE_UNKNOWN;

	setup_timer(&bp->recv_timer, buspirate_recv_timer_callback, (unsigned long) bp);

	i2c_set_adapdata(&bp->adap, bp);
	return i2c_add_adapter(&bp->adap);
}

static int buspirate_open(struct tty_struct *tty) {
	struct buspirate *bp;

	/* We cheat a little here since we know the BP is an USB FTDI device.
	 * Look up it's associated 'struct dev' so we can use it for our
	 * dev_dbg() prints.  Not strictly kosher but... */
	struct usb_serial_port *usb_port = tty->driver_data;
	struct device *dev = &usb_port->dev;
	int rc = 0;

	/* Linux defaults TTYs to 9600 baud.  Need 115200 to talk to BP */
	if ((tty->termios.c_ispeed != 115200) || (tty->termios.c_ospeed != 115200))
		buspirate_set_tty_baud(tty, 115200);

	tty->receive_room = 65536;

	bp = kzalloc(sizeof(*bp), GFP_KERNEL);
	if (!bp) {
		dev_err(dev, "Can't allocate memory\n");
		rc = -ENOMEM;
		goto error;
	}
	
	bp->tty = tty;
	bp->dev = dev;
	tty->disc_data = bp;

	rc = buspirate_init_i2c(bp);
	if (rc) {
		dev_err(bp->dev, "Couldn't add I2C adapter, rc=%d\n", rc);
		goto error;
	}

	init_completion(&bp->data_comp);
	sema_init(&bp->sem, 1);

	/* Everything is setup */
	goto end;
	
error:
	kfree(bp);
end:
	return rc;
}

static void buspirate_close(struct tty_struct *tty)
{
	struct buspirate *bp = tty->disc_data;

	/* Both hangup and close can call this function.  Check to see if we've
	 * already shut things down. */
	if (bp) {
		bp->shutdown = 1;
		del_timer_sync(&bp->recv_timer);
		i2c_del_adapter(&bp->adap);

		/* Disable pull-ups and voltage to I2C device */
		buspirate_i2c_set_peripherals(bp, 0);
		buspirate_reset(bp);	/* Get us back into text mode.  It also resets
					 * the baud rate. */
		buspirate_set_tty_baud(tty, 115200);
		kfree(bp);
	}
};

static int buspirate_ioctl(struct tty_struct *tty, struct file *file,
                                         unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int buspirate_hangup(struct tty_struct *tty)
{
	buspirate_close(tty);
	return 0;
}

static struct tty_ldisc_ops buspirate_ldisc = {
         .owner          = THIS_MODULE,
         .magic          = TTY_LDISC_MAGIC,
         .name           = "buspirate",
         .open           = buspirate_open,
         .close          = buspirate_close,
         .hangup         = buspirate_hangup,
         .ioctl          = buspirate_ioctl,
         .receive_buf    = buspirate_receive_buf,
};

static int __init buspirate_init(void)
{
	int rc;
	if ((rc = tty_register_ldisc(N_BUSPIRATE, &buspirate_ldisc)))
		printk(KERN_ERR "buspirate: can't register line discipline (rc = %d)\n", rc);
	else
		printk("buspirate: module loaded\n");
		
	return rc;
}

static void __exit buspirate_cleanup(void)
{
	int rc;
	rc = tty_unregister_ldisc(N_BUSPIRATE);
	if (rc) {
                 printk(KERN_ERR "buspirate: can't unregister line discipline (rc = %d)\n", rc);
	}
}

module_init(buspirate_init);
module_exit(buspirate_cleanup);
