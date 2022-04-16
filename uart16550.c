// SPDX-License-Identifier: GPL-2.0
  /*
   * uart16550.c - UART16550 driver
   *
   * Author: Grigorie Ruxandra <ruxi.grigorie@gmail.com>
   * Author: Orzata Miruna Narcisa <mirunaorzata21@gmail.com>
   */
  struct uart16550_dev {
     struct cdev cdev;
     /*TODO */
  };
  static struct uart16550_dev devs[MAX_NUMBER_DEVICES];

  static int uart16550_open(struct inode *inode, struct file *file)
  {
      /*TODO */
      return 0;
  }

  static int uart16550_release(struct inode *inode, struct file *file)
  {
     /*TODO */
     return 0;
  }

  static ssize_t uart16550_read(struct file *file,  char __user *user_buffer,
                                size_t size, loff_t *offset)
  {
        /*TODO */
  }
  static ssize_t uart16550_write(struct file *file,
                                 const char __user *user_buffer,
                                 size_t size, loff_t *offset)
  {
       /*TODO */
  }
  static long
  uart16550_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
  {
        /*TODO */
        return 0;
  }
  static const struct file_operations uart16550_fops = {
         .owner		 = THIS_MODULE,
         .open		 = uart16550_open,
         .release	 = uart16550_release,
         .read		 = uart16550_read,
         .write		 = uart16550_write,
         .unlocked_ioctl = uart16550_ioctl
  };

  static int __init uart16550_init(void)
  {
    /* TODO: */
  }
  static void __exit uart16550_exit(void)
  {
     /* TODO: */
  }
  module_init(uart16550_init);
  module_exit(uart16550_exit);

  MODULE_DESCRIPTION("UART16550 Driver");
  MODULE_AUTHOR("Grigorie Ruxandra <ruxi.grigorie@gmail.com");
  MODULE_AUTHOR("Orzata Miruna Narcisa <mirunaorzata21@gmail.com");
