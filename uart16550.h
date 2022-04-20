#ifndef _UART16550_H
#define _UART16550_H

#define	OPTION_COM1			1
#define OPTION_COM2			2
#define OPTION_BOTH			3

#define UART16550_COM1_SELECTED		0x01
#define UART16550_COM2_SELECTED		0x02

#define MAX_NUMBER_DEVICES		2

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

#ifndef _UART16550_REGS_H

#define UART16550_BAUD_1200		96
#define UART16550_BAUD_2400		48
#define UART16550_BAUD_4800		24
#define UART16550_BAUD_9600		12
#define UART16550_BAUD_19200		6
#define UART16550_BAUD_38400		3
#define UART16550_BAUD_56000		2
#define UART16550_BAUD_115200		1

#define UART16550_LEN_5			0x00
#define UART16550_LEN_6			0x01
#define UART16550_LEN_7			0x02
#define UART16550_LEN_8			0x03

#define UART16550_STOP_1		0x00
#define UART16550_STOP_2		0x04

#define UART16550_PAR_NONE		0x00
#define UART16550_PAR_ODD		0x08
#define UART16550_PAR_EVEN		0x18
#define UART16550_PAR_STICK		0x20

#define UART_DEFAULT_BAUD	96
#define UART_DEFAULT_LEN	0x03
#define UART_DEFAULT_STOP	0x00
#define UART_DEFAULT_PARITY	0x00

#define UART_DLL			0x00	/* Out: Divisor Latch Low */


#define UART_FCR				2		/* Out: FIFO Control Register */
#define UART_FCR_ENABLE_FIFO	0x01	/* Enable the FIFO */
#define UART_FCR_CLEAR_RCVR		0x02 	/* Clear the RCVR FIFO */
#define UART_FCR_CLEAR_XMIT		0x04 	/* Clear the XMIT FIFO */
#define UART_FCR_R_TRIG_00		0x00	/* Receive Interrupt trigger level */
#define UART_FCR_R_TRIG_01		0x40	/* Receive Interrupt trigger level */
#define UART_FCR_R_TRIG_10		0x80    /* Receive Interrupt trigger level */


#define UART_IER			1			/* Out: Interrupt Enable Register */
#define UART_IER_MSI		0x08 		/* Enable Modem status interrupt */
#define UART_IER_RLSI		0x04 		/* Enable receiver line status interrupt */
#define UART_IER_THRI		0x02 		/* Enable Transmitter holding register int. */
#define UART_IER_RDI		0x01 		/* Enable receiver data interrupt */


#define UART_IIR			2		/* In:  Interrupt ID Register */
#define UART_IIR_NO_INT		0x01 	/* No interrupts pending */
#define UART_IIR_ID			0x0e 	/* Mask for the interrupt ID */
#define UART_IIR_MSI		0x00 	/* Modem status interrupt */
#define UART_IIR_THRI		0x02 	/* Transmitter holding register empty */
#define UART_IIR_RDI		0x04 	/* Receiver data interrupt */
#define UART_IIR_RLSI		0x06 	/* Receiver line status interrupt */
#define UART_IIR_TIMEOUT	12 		/* Receiver line status interrupt */

#define UART_LCR			3		/* Out: Line Control Register */
#define UART_LCR_DLAB		0x80	/* Divisor latch access bit */

#define UART_LSR			5		/* In:  Line Status Register */
#define UART_LSR_DR			0x01 	/* Receiver data ready */
#define UART_LSR_THRE		0x20 	/* Transmit-hold-register empty */

#define UART_MCR			4		/* Out: Modem Control Register */
#define UART_MCR_OUT2		0x08 	/* Out1 complement */
#define UART_MCR_RTS		0x02 	/* RTS complement */
#define UART_MCR_DTR		0x01 	/* DTR complement */

#define FIFO_SIZE			16		/* Number of bytes FIFO can hold */

#endif

#define	UART16550_IOCTL_SET_LINE	1



struct uart16550_line_info {
	unsigned char baud, len, par, stop;
};

#endif
