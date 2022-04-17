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
	char read_buf[BUFFER_SIZE];
	char write_buf[BUFFER_SIZE];
	int read_buf_size;
	int write_buf_size;
	size_t put_idx, get_idx;
	bool empty;
	bool full;
	// TODO wait_queue_head_t, atomic_t?
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

static ssize_t uart16550_read(struct file *file,  char __user *user_buffer,
							size_t size, loff_t *offset)
{
	struct uart16550_dev *data =
		(struct uart16550_dev *) file->private_data;
	size_t to_read = 0;

	// Un apel read blocant înseamnă că rutina de read apelată din user-space se va bloca
	// până la citirea a cel puţin un octet (buffer-ul de read din kernel este gol și
	// nu se pot citi date).

	return to_read;
}

static ssize_t uart16550_write(struct file *file,
								const char __user *user_buffer,
								size_t size, loff_t *offset)
{
	struct uart16550_dev *data =
		(struct uart16550_dev *) file->private_data;

	size_t to_write = 0;

	// Un apel write blocant înseamnă că rutina de write apelată din user-space se va bloca
	// până la scrierea a cel puţin un octet (buffer-ul de write din kernel este plin și
	// nu se pot scrie date). 

	return to_write;
}

static long
uart16550_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct uart16550_dev *data =
		(struct uart16550_dev *) file->private_data;
	
	int ret = 0;
	// TODO verify -> struct uart16550_line_info

	switch (cmd) {
	case UART16550_IOCTL_SET_LINE:
		// TODO change communication parameters
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
		cdev_init(&devs[i].cdev, &uart16550_fops);
		cdev_add(&devs[i].cdev, MKDEV(major, i), 1);
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

static int __init uart16550_init(void)
{
	int err;
	// TODO initialize communication with com1 and com2 serial ports (control registers)

	switch (option) {
	case OPTION_BOTH:
		pr_info("option both");
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
		pr_info("option com1");
		err = init_char_dev("uart16550", COM1_MINOR, 1);
		if (err != 0) {
			goto out;
		}

		err = init_io_region("uart16550", COM1_BASEPORT);
		if (err != 0) {
			goto out_unregister_chrdev_com1;
		}

		err = init_irq_handler(0, "com1", IRQ_COM1);
		if (err != 0) {
			goto out_unregister_and_release_com1;
		}

		break;
	case OPTION_COM2:
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
	unregister_chrdev_region(MKDEV(major, COM1_MINOR), 1);
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
