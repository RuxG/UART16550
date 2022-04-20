// SPDX-License-Identifier: GPL-2.0
  /*
   * uart16550.c - UART16550 driver
   *
   * Author: Grigorie Ruxandra <ruxi.grigorie@gmail.com>
   * Author: Orzata Miruna Narcisa <mirunaorzata21@gmail.com>
   */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/moduleparam.h>
#include "uart16550.h"

#define MODULE_NAME	"uart16550"

#define DEFAULT_MAJOR	42
#define DEFAULT_OPTION	OPTION_BOTH

#define COM1_MINOR		0
#define COM2_MINOR		1
#define DEFAULT_MINOR   0

#define COM1_BASEPORT	0x3f8
#define COM2_BASEPORT	0x2f8
#define NR_PORTS	8

#define IRQ_COM1	4
#define IRQ_COM2	3

#define BUFFER_SIZE		1024

static int major = DEFAULT_MAJOR;
module_param(major, int, 0);

static int option = DEFAULT_OPTION;
module_param(option, int, 0);

struct uart16550_dev {
	struct cdev cdev;
	int port_type;
	char read_buf[BUFFER_SIZE];
	char write_buf[BUFFER_SIZE];
	atomic_t read_buf_size;
	atomic_t write_buf_size;
	size_t put_idx, get_idx;
	spinlock_t lock;
	wait_queue_head_t wq_reads, wq_writes;
}; 
static struct uart16550_dev devs[MAX_NUMBER_DEVICES];

static int uart16550_open(struct inode *inode, struct file *file)
{
	struct uart16550_dev *data;

	data = container_of(inode->i_cdev, struct uart16550_dev, cdev);

	file->private_data = data;
	pr_info("%s opened\n", MODULE_NAME);

	return 0;
}

static int uart16550_release(struct inode *inode, struct file *file)
{
	pr_info("%s closed\n", MODULE_NAME);
	return 0;
}

static bool get_char(char *c, struct uart16550_dev *data)
{
	/* get char from buffer; update size and get_idx */
	if (atomic_read(&data->read_buf_size) > 0) {
		*c = data->read_buf[data->get_idx];
		data->get_idx = (data->get_idx + 1) % BUFFER_SIZE;
		atomic_dec(&data->read_buf_size);
		return true;
	}

	return false;
}

static bool write_char(char c, struct uart16550_dev *data)
{
	/* write char to buffer; update size and put_idx */
	if (atomic_read(&data->write_buf_size) < BUFFER_SIZE) {
		data->write_buf[data->put_idx] = c;
		data->put_idx = (data->put_idx + 1) % BUFFER_SIZE;
		atomic_inc(&data->write_buf_size);
		return true;
	}

	return false;
}

static ssize_t uart16550_read(struct file *file,  char __user *user_buffer,
							size_t size, loff_t *offset)
{
	struct uart16550_dev *data =
		(struct uart16550_dev *) file->private_data;
	int i = 0;
	char c;
	unsigned long flags;
	bool read_status = true;

	// Un apel read blocant înseamnă că rutina de read apelată din user-space se va bloca
	// până la citirea a cel puţin un octet (buffer-ul de read din kernel este gol și
	// nu se pot citi date).

	if (wait_event_interruptible(data->wq_reads, atomic_read(&data->read_buf_size) > 0))
		return -ERESTARTSYS;

	while (size > 0) {
		spin_lock_irqsave(&data->lock, flags);
		read_status = get_char(&c, data);
		spin_unlock_irqrestore(&data->lock, flags);

		if (read_status == false)
			break;
		
		if (put_user(c, &user_buffer[i]))
			return -EFAULT;
		
		i++;
		size--;
	}

	return i;
}

static ssize_t uart16550_write(struct file *file,
								const char __user *user_buffer,
								size_t size, loff_t *offset)
{
	struct uart16550_dev *data =
		(struct uart16550_dev *) file->private_data;

	int i = 0;
	char c;
	unsigned long flags;
	bool write_status = true;

	// Un apel write blocant înseamnă că rutina de write apelată din user-space se va bloca
	// până la scrierea a cel puţin un octet (buffer-ul de write din kernel este plin și
	// nu se pot scrie date).
	if (wait_event_interruptible(data->wq_writes, atomic_read(&data->write_buf_size) < BUFFER_SIZE))
			return -ERESTARTSYS;

	while (size > 0) {
		if (get_user(c, &user_buffer[i]))
			return -EFAULT;

		spin_lock_irqsave(&data->lock, flags);
		write_status = write_char(c, data);
		spin_unlock_irqrestore(&data->lock, flags);

		if (write_status == false)
			break;

		i++;
		size--;
	}

	// enable write interrupt


	return i;
}

static int uart16550_line_info_set_baud(unsigned char baud)
{
	int ret = -EINVAL;
	int size = 8;
	int i;

	unsigned char valid_baud[8] = { UART16550_BAUD_1200, UART16550_BAUD_2400,
								   UART16550_BAUD_4800, UART16550_BAUD_9600,
								   UART16550_BAUD_19200, UART16550_BAUD_38400,
								   UART16550_BAUD_56000, UART16550_BAUD_115200 };

	for (i = 0; i < size; i++) {
		if (baud == valid_baud[i]) {
			
			if (option == OPTION_COM1 || option == OPTION_BOTH) {
				/* Turn off interrupts */
				outb(COM1_BASEPORT + UART_IER, 0);

				/* Set DLAB ON - enable baud generator divisor latches */
				outb(COM1_BASEPORT + UART_LCR, UART_LCR_DLAB);

				/* Set DLL - set baud divisor low byte */
				outb(COM1_BASEPORT + UART_DLL, baud);

				/* Set DLAB OFF */
				outb(COM1_BASEPORT + UART_LCR, inb(COM1_BASEPORT + UART_LCR)&(~UART_LCR_DLAB));
			}

			if (option == OPTION_COM2 || option == OPTION_BOTH) {
				/* Turn off interrupts */
				outb(COM2_BASEPORT + UART_IER, 0);

				/* Set DLAB ON - enable baud generator divisor latches */
				outb(COM2_BASEPORT + UART_LCR, UART_LCR_DLAB);

				/* Set DLL - set baud divisor low byte */
				outb(COM2_BASEPORT + UART_DLL, baud);

				/* Set DLAB OFF */
				outb(COM2_BASEPORT + UART_LCR, inb(COM2_BASEPORT + UART_LCR)&(~UART_LCR_DLAB)); 
			}

			ret = 0;
			break;
		}
	}

	return ret;
}

static int uart16550_line_info_set_len(unsigned char len)
{
	int ret = -EINVAL;
	int size = 4;
	int i;
	int valid_len[] = { UART16550_LEN_5, UART16550_LEN_6, UART16550_LEN_7, UART16550_LEN_8 };

	for (i = 0; i < size; i++) {
		if (len == valid_len[i]) {

			if (option == OPTION_COM1 || option == OPTION_BOTH) {
				/* Set word length */
				outb(COM1_BASEPORT + UART_LCR , inb(COM1_BASEPORT + UART_LCR) | len);
			}

			if (option == OPTION_COM2 || option == OPTION_BOTH) {
				/* Set word length */
				outb(COM2_BASEPORT + UART_LCR , inb(COM2_BASEPORT + UART_LCR) | len);
			}

			ret = 0;
			break;
		}
	}

	return ret;
}

static int uart16550_line_info_set_parity(unsigned char parity)
{
	int ret = -EINVAL;
	int size = 4;
	int i;
	int valid_parity[] = { UART16550_PAR_NONE, UART16550_PAR_ODD, UART16550_PAR_EVEN, UART16550_PAR_STICK };

	for (i = 0; i < size; i++) {
		if (parity == valid_parity[i]) {

			if (option == OPTION_COM1 || option == OPTION_BOTH) {
				/* Set parity */
				outb(COM1_BASEPORT + UART_LCR , inb(COM1_BASEPORT + UART_LCR) | parity);
			}

			if (option == OPTION_COM2 || option == OPTION_BOTH) {
				/* Set parity */
				outb(COM2_BASEPORT + UART_LCR , inb(COM2_BASEPORT + UART_LCR) | parity);
			}

			ret = 0;
			break;
		}
	}

	return ret;
}

static int uart16550_line_info_set_stop(unsigned char stop)
{
	int ret = -EINVAL;
	int size = 2;
	int i;
	int valid_stop[] = { UART16550_STOP_1, UART16550_STOP_2};

	for (i = 0; i < size; i++) {
		if (stop == valid_stop[i]) {

			if (option == OPTION_COM1 || option == OPTION_BOTH) {
				/* Set number of stop bits */
				outb(COM1_BASEPORT + UART_LCR , inb(COM1_BASEPORT + UART_LCR) | stop);
			}

			if (option == OPTION_COM2 || option == OPTION_BOTH) {
				/* Set number of stop bits */
				outb(COM2_BASEPORT + UART_LCR , inb(COM2_BASEPORT + UART_LCR) | stop);
			}

			ret = 0;
			break;
		}
	}

	return ret;
}

static void configure_fifo(void)
{
	/* Configure FIFO control register: enable FIFO, set trigger level to 1 bit */
	if (option == OPTION_COM1 || option == OPTION_BOTH) {
		outb(COM1_BASEPORT + UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR |
									UART_FCR_CLEAR_XMIT | UART_FCR_R_TRIG_00);
	}

	if (option == OPTION_COM2 || option == OPTION_BOTH) {
		outb(COM2_BASEPORT + UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR |
									UART_FCR_CLEAR_XMIT | UART_FCR_R_TRIG_00);
	}
}

static void configure_modem(void)
{
	/* Configure Modem Control to enable interrupts */
	if (option == OPTION_COM1 || option == OPTION_BOTH) {
		outb(COM1_BASEPORT + UART_MCR, UART_MCR_OUT2 | UART_MCR_RTS | UART_MCR_DTR);
	}

	if (option == OPTION_COM2 || option == OPTION_BOTH) {
		outb(COM2_BASEPORT + UART_MCR, UART_MCR_OUT2| UART_MCR_RTS | UART_MCR_DTR);
	}
}

static void configure_interrupts(void)
{
	/* Configure Interrupts: enable Received data available and
		Transmitter Holding Register Empty interrupts */
	if (option == OPTION_COM1 || option == OPTION_BOTH) {
		outb(COM1_BASEPORT + UART_IER, UART_IER_RDI | UART_IER_THRI);
	}

	if (option == OPTION_COM2 || option == OPTION_BOTH) {
		outb(COM2_BASEPORT + UART_IER, UART_IER_RDI | UART_IER_THRI);
	}
}

static long
uart16550_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct uart16550_dev *data =
		(struct uart16550_dev *) file->private_data;
	
	struct uart16550_line_info uli;
	int ret = 0;

	switch (cmd) {
	case UART16550_IOCTL_SET_LINE:

		if (copy_from_user(&uli, (const void *)arg, sizeof(struct uart16550_line_info)))
			return -EFAULT;

		if (uart16550_line_info_set_baud(uli.baud)) {
			ret = -EINVAL;
			pr_err("%s: ioctl invalid communication parameters: invalid baud divisor: %u.\n", MODULE_NAME, uli.baud);
			break;
		}

		if (uart16550_line_info_set_len(uli.len)) {
			ret = -EINVAL;
			pr_err("%s: ioctl invalid communication parameters: invalid len: %u.\n", MODULE_NAME, uli.len);
			break;
		}
		
		if (uart16550_line_info_set_parity(uli.par)) {
			ret = -EINVAL;
			pr_err("%s: ioctl invalid communication parameters: invalid parity: %u.\n", MODULE_NAME, uli.par);
			break;
		}

		if (uart16550_line_info_set_stop(uli.stop)) {
			ret = -EINVAL;
			pr_err("%s: ioctl invalid communication parameters: invalid stop: %u.\n", MODULE_NAME, uli.stop);
			break;
		}

		//configure_fifo();
		
		//configure_modem();

		configure_interrupts();

		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct file_operations uart16550_fops = {
		.owner		 = THIS_MODULE,
		.open		 = uart16550_open,
		.release	 = uart16550_release,
		.read		 = uart16550_read,
		.write		 = uart16550_write,
		.unlocked_ioctl = uart16550_ioctl
};

irqreturn_t uart16550_interrupt_handler(int irq_no, void *dev_id)
{
	struct uart16550_dev *my_data = (struct uart16550_dev *) dev_id;
	
	/* if interrupt is not for this device (shared interrupts) */
	/* return IRQ_NONE;*/
	/* clear interrupt-pending bit */
	/* read from device or write to device*/
	int base_addr;

	if (my_data->port_type == 0)
		base_addr = COM1_BASEPORT;
	else
		base_addr = COM2_BASEPORT;

	// write interrupt
	// TODO disable write interrupt when the kernel write_buffer is empty
	// spinlock
	// change write_buffer_size + put_idx
	// spinlock
	// wake_up(&my_data->wq_writes)


	// read interrupt
	// spinlock
	// change read_buffer_size + get_idx
	// spinlock
	// wake_up(&my_data->wq_reads)


	// base + 5
	// interrupt status
	// LSR
	// THRI dezactivare

	// outb(base_addr + UART_IIR, (inb(base_addr + UART_IIR)&(~UART_IIR_NO_INT)));	/* interrupts pending */
	// int count = 0;
	// do { c = inb(base_addr + UART_LSR);
	// 		if (c & 1) {
	// 			buffer_rec[count] = inb(base_addr);
	// 			count++;
	// 		}
	// } while (c & 1);

	// how to transmit a byte:
	// 1. Read the Line Status Register - LSR
	// 2. Transmit Holding Register is Empty? (yes) - THR bit 5
	// 3. Write byte to the Transmitter Data Register

	// how to receive a byte:
	// 1. Read the Line Status Register - LCR
	// 2. Received Data is Ready? (yes) - RDR bit 0
	// 3. Read byte from the Receiver Data Register
 
	return IRQ_HANDLED;
}

static int init_char_dev(const char *dev_name, int start_minor, int nr_minors)
{
	int err = 0;
	int i;

	/* register char device region for major and nr minors */
	err = register_chrdev_region(MKDEV(major, start_minor), nr_minors, dev_name);
	if (err != 0) {
		pr_err("%s register_chrdev_region failed: %d\n", dev_name, err);
		return err;
	}

	for (i = 0; i < nr_minors; i++) {
		/* init and add cdev to kernel core */
		init_waitqueue_head(&devs[i].wq_reads);
		init_waitqueue_head(&devs[i].wq_writes);
		cdev_init(&devs[i].cdev, &uart16550_fops);
		cdev_add(&devs[i].cdev, MKDEV(major, start_minor + i), 1);
	}

	return err;
}

static int init_io_region(const char *dev_name, int port_base_addr)
{
	/* request I/O port */
	if (!request_region(port_base_addr, NR_PORTS, dev_name)) {
		pr_err("%s request_region failed\n", dev_name);
		return -ENODEV;
	}

	return 0;
}

static int init_irq_handler(int dev_id, const char *dev_name, int irq_line)
{
	int err = 0;

	/* register IRQ handler for IRQ line */
	err = request_irq(irq_line, uart16550_interrupt_handler, IRQF_SHARED,
					MODULE_NAME, &devs[dev_id]);
	if (err < 0) {
		pr_err("%s register_irq failed: %d\n", dev_name, err);
		return err;
	}

	return err;
}

static void uart16550_init_registers(void)
{
	/* Configure line parameters */
	uart16550_line_info_set_baud(UART_DEFAULT_BAUD);
	uart16550_line_info_set_len(UART_DEFAULT_LEN);
	uart16550_line_info_set_parity(UART_DEFAULT_PARITY);
	uart16550_line_info_set_stop(UART_DEFAULT_STOP);

	configure_fifo();

	configure_modem();

	configure_interrupts();
}

static int __init uart16550_init(void)
{
	int err;
	int num_ports;

	switch (option) {
	case OPTION_BOTH:
		pr_info("option both");
		devs[0].port_type = 0;
		devs[1].port_type = 1;
		spin_lock_init(&devs[0].lock);
		spin_lock_init(&devs[1].lock);

		num_ports = 2;
		err = init_char_dev("uart16550", COM1_MINOR, 2);
		if (err != 0) {
			goto out;
		}

		err = init_io_region("uart16550", COM1_BASEPORT);
		if (err != 0) {
			goto out_unregister_chrdev_com1;
		}

		err = init_io_region("uart16550", COM2_BASEPORT);
		if (err != 0) {
			goto out_release_io_regions_com1;
		}
		
		err = init_irq_handler(0, "com1", IRQ_COM1);
		if (err != 0) {
			goto out_release_io_regions_com1_com2;
		}

		err = init_irq_handler(1, "com2", IRQ_COM2);
		if (err != 0) {
			goto out_free_irq1;
		}
		break;

	case OPTION_COM1:
		devs[0].port_type = 0;
		spin_lock_init(&devs[0].lock);

		num_ports = 1;
		pr_info("option com1");
		err = init_char_dev("uart16550", COM1_MINOR, 1);
		if (err != 0) {
			pr_info("Cant register I tried\n");
			goto out;
		}

		err = init_io_region("uart16550", COM1_BASEPORT);
		if (err != 0) {
			pr_info("Cant register I tried 2\n");
			goto out_unregister_chrdev_com1;
		}

		err = init_irq_handler(0, "com1", IRQ_COM1);
		if (err != 0) {
			pr_info("Cant register I tried 3\n");
			goto out_unregister_and_release_com1;
		}

		break;

	case OPTION_COM2:
		devs[0].port_type = 1;
		spin_lock_init(&devs[0].lock);

		num_ports = 1;
		pr_info("option com2");
		err = init_char_dev("uart16550", COM2_MINOR, 1);
		if (err != 0) {
			goto out;
		}

		err = init_io_region("uart16550", COM2_BASEPORT);
		if (err != 0) {
			goto out_unregister_chrdev_com2;
		}
		
		err = init_irq_handler(0, "com2", IRQ_COM2);
		if (err != 0) {
			goto out_unregister_and_release_com2;
		}

		break;
	default:
		err = -EINVAL;
		goto out;
	}

	uart16550_init_registers();
	

// TODO verifica ce e mai jos
out_unregister_chrdev_com2:
	unregister_chrdev_region(MKDEV(major, COM2_MINOR), 1);
	goto out;

out_unregister_and_release_com1:
	release_region(COM1_BASEPORT, NR_PORTS);
	unregister_chrdev_region(MKDEV(major, COM1_MINOR), 1);
	goto out;

out_unregister_and_release_com2:
	release_region(COM2_BASEPORT, NR_PORTS);
	unregister_chrdev_region(MKDEV(major, COM2_MINOR), 1);
	goto out;

out_free_irq1:
	free_irq(IRQ_COM1, &devs[0]);
out_release_io_regions_com1_com2:
	release_region(COM2_BASEPORT, NR_PORTS);
out_release_io_regions_com1:
	release_region(COM1_BASEPORT, NR_PORTS);
out_unregister_chrdev_com1:
	unregister_chrdev_region(MKDEV(major, COM1_MINOR), num_ports);
out:
	return err;
}

static void __exit uart16550_exit(void)
{
	switch (option) {
	case OPTION_BOTH:
		/* delete cdev from kernel core */
		cdev_del(&devs[0].cdev);
		cdev_del(&devs[1].cdev);

		/* unregister char device region, for major and 1 minor */
		unregister_chrdev_region(MKDEV(major, COM1_MINOR), 2);

		/* release I/O ports */
		release_region(COM1_BASEPORT, NR_PORTS);
		release_region(COM2_BASEPORT, NR_PORTS);

		/* free IRQ_COM1 & IRQ_COM2 */
		free_irq(IRQ_COM1, &devs[0]);
		free_irq(IRQ_COM2, &devs[1]);

		break;
	case OPTION_COM1:
		cdev_del(&devs[0].cdev);
		unregister_chrdev_region(MKDEV(major, COM1_MINOR), 1);
		release_region(COM1_BASEPORT, NR_PORTS);
		free_irq(IRQ_COM1, &devs[0]);

		break;
	case OPTION_COM2:
		cdev_del(&devs[0].cdev);
		unregister_chrdev_region(MKDEV(major, COM2_MINOR), 1);
		release_region(COM2_BASEPORT, NR_PORTS);
		free_irq(IRQ_COM2, &devs[0]);

		break;
	default:
		break;
	}
}

module_init(uart16550_init);
module_exit(uart16550_exit);

MODULE_DESCRIPTION("UART16550 Driver");
MODULE_AUTHOR("Grigorie Ruxandra <ruxi.grigorie@gmail.com");
MODULE_AUTHOR("Orzata Miruna Narcisa <mirunaorzata21@gmail.com");
MODULE_LICENSE("GPL-2.0");
