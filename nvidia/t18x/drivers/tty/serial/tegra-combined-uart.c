/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/serial_core.h>
#include <linux/console.h>
#include <linux/tty_flip.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqreturn.h>
#include <asm/io.h>

#define HSP_INT_IE_0		0x100
#define TOP0_SHARED_MBOX0	0x0
#define MBOX0_FULL_BIT		8

#define PKT_RCVD_BIT		26
#define NUM_BYTES_FIELD_BIT	27
#define LAST_PKT_BIT		29
#define FIRST_PKT_BIT		30
#define INTR_TRIGGER_BIT	31

static u8 __iomem *top0_mbox01_base;
static u8 __iomem *spe_mbox_reg;
static u8 __iomem *top0_cmn_base;

static struct uart_port tegra_combined_uart_port;
static struct console tegra_combined_uart_console;
static struct uart_driver tegra_combined_uart_driver;

static void tegra_combined_uart_console_write(struct console *co,
						const char *s,
						unsigned int count);

/*
 * This function does nothing. This function is used to fill in the function
 * pointers in struct uart_ops tegra_combined_uart_ops, which we don't
 * implement.
 */
static int uart_null_func(struct uart_port *port)
{
	return 0;
}

static void tegra_combined_uart_start_tx(struct uart_port *port)
{
	unsigned long tail;
	unsigned long count;
	struct circ_buf *xmit = &port->state->xmit;

	tail = (unsigned long)&xmit->buf[xmit->tail];
	count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
	if (!count)
		return;

	tegra_combined_uart_console_write(NULL, (char *)tail, count);
	xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
}

/*
 * This function splits the string to be printed (const char *s) into multiple
 * packets. Each packet contains a max of 3 characters. Packets are sent to the
 * SPE-based combined UART server for printing. Communication with SPE is done
 * through mailbox registers which can generate interrupts for SPE and Linux.
 */
static void tegra_combined_uart_console_write(struct console *co,
						const char *s,
						unsigned int count)
{
	int num_packets, curr_packet_bytes, last_packet_bytes;
	u32 reg_val;
	int i, j;

	num_packets = count / 3;
	if (count % 3 != 0) {
		last_packet_bytes = count % 3;
		num_packets += 1;
	} else {
		last_packet_bytes = 3;
	}

	/* Loop for processing each 3 char packet */
	for (i = 0; i < num_packets; i++) {
		reg_val = BIT(INTR_TRIGGER_BIT);
		if (i == 0)
			reg_val |= BIT(FIRST_PKT_BIT);

		if (i == num_packets - 1) {
			reg_val |= BIT(LAST_PKT_BIT);
			curr_packet_bytes = last_packet_bytes;
		} else {
			curr_packet_bytes = 3;
		}

		/*
		 * Extract the current 3 chars from the
		 * string buffer (const char *s) and store them in the mailbox
		 * register value.
		 */
		reg_val |= curr_packet_bytes << NUM_BYTES_FIELD_BIT;
		for (j = 0; j < curr_packet_bytes; j++)
			reg_val |= s[i*3 + j] << (j * 8);

		/* Send current packet to SPE */
		writel(reg_val, spe_mbox_reg);

		/*
		 * Wait for SPE to ACK that it received the current packet.
		 * TODO: Add support for interrupt based operation.
		 */
		while (!(readl(top0_mbox01_base + TOP0_SHARED_MBOX0)
					& BIT(PKT_RCVD_BIT)))
			cpu_relax();

		/* Clear the ACK */
		writel(0, top0_mbox01_base + TOP0_SHARED_MBOX0);
	}
}

static int __init tegra_combined_uart_console_setup(struct console *co,
							char *options)
{
	int baud = 115200;
	int parity = 'n';
	int bits = 8;
	int flow = 'n';

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&tegra_combined_uart_port, co, baud, parity,
				bits, flow);
}

static irqreturn_t tegra_combined_uart_rx(int irq, void *dev_id)
{
	int i;
	char ch = 0;
	int bytes;
	u32 reg_val;
	struct tty_port *port = &((tegra_combined_uart_port.state)->port);

	/* read the mailbox register: top0_shared_mbox0 */
	reg_val = readl(top0_mbox01_base + TOP0_SHARED_MBOX0);
	if (reg_val & BIT(PKT_RCVD_BIT)) {
		pr_err("%s: ACK received but was expecting data\n", __func__);
		return IRQ_HANDLED;
	}

	bytes = (reg_val >> NUM_BYTES_FIELD_BIT) & 0x3;
	for (i = 0; i < bytes; i++) {
		ch = (reg_val >> i * 8) & 0xFF;
		tty_insert_flip_char(port, ch, TTY_NORMAL);
	}
	if (reg_val & BIT(LAST_PKT_BIT))
		tty_flip_buffer_push(port);

	/* clear the mailbox register: top0_shared_mbox0 */
	writel(0, top0_mbox01_base + TOP0_SHARED_MBOX0);
	/* ack by writing to spe mailbox register */
	writel(BIT(PKT_RCVD_BIT) | BIT(INTR_TRIGGER_BIT), spe_mbox_reg);

	return IRQ_HANDLED;
}

static int tegra_combined_uart_startup(struct uart_port *port)
{
	int ret;
	u32 reg_val;
	/* allocate IRQ */
	ret = request_irq(tegra_combined_uart_port.irq, tegra_combined_uart_rx,
		0, "combined_uart rx", port);
	if (ret) {
		pr_err("%s: request_irq error\n", __func__);
		return ret;
	}
	/*
	 * enable interrupt
	 * WARNING: HSP_INT_IE_0 is not protected for RMW.
	 */
	reg_val = readl(top0_cmn_base + HSP_INT_IE_0);
	reg_val |= (1 << MBOX0_FULL_BIT);
	writel(reg_val, top0_cmn_base + HSP_INT_IE_0);
	return ret;
}

static int tegra_combined_uart_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;

	if (!np)
		return -ENODEV;

	top0_mbox01_base = (u8 *)(of_io_request_and_map(np, 0,
		    "Tegra Combined UART TOP0_HSP Linux mailbox"));
	if (IS_ERR(top0_mbox01_base))
		return PTR_ERR(top0_mbox01_base);

	spe_mbox_reg = (u8 *)(of_io_request_and_map(np, 1,
		    "Tegra Combined UART SPE mailbox"));
	if (IS_ERR(spe_mbox_reg)) {
		ret = PTR_ERR(spe_mbox_reg);
		goto err_mapping;
	}

	top0_cmn_base = (u8 *)(of_io_request_and_map(np, 2,
		    "Tegra Combined UART TOP0_HSP Linux mailbox interrrupt"));
	if (IS_ERR(top0_cmn_base)) {
		ret = PTR_ERR(top0_cmn_base);
		goto err_mapping;
	}

	ret = uart_register_driver(&tegra_combined_uart_driver);
	if (ret < 0) {
		pr_err("%s: Could not register driver\n", __func__);
		goto err_mapping;
	}

	tegra_combined_uart_port.irq = irq_of_parse_and_map(np, 0);
	if (!tegra_combined_uart_port.irq) {
		pr_err("%s: Failed to get irq from device tree\n", __func__);
		ret = -EINVAL;
		goto err_irq;
	}

	ret = uart_add_one_port(&tegra_combined_uart_driver,
				&tegra_combined_uart_port);
	if (ret < 0) {
		pr_err("%s: Failed to add uart port\n", __func__);
		goto err_irq;
	}

	return ret;

err_irq:
	uart_unregister_driver(&tegra_combined_uart_driver);
err_mapping:
	if (spe_mbox_reg != NULL && !IS_ERR(spe_mbox_reg))
		iounmap(spe_mbox_reg);
	if (top0_mbox01_base != NULL && !IS_ERR(top0_mbox01_base))
		iounmap(top0_mbox01_base);
	if (top0_cmn_base != NULL && !IS_ERR(top0_cmn_base))
		iounmap(top0_cmn_base);
	return ret;
}

static int tegra_combined_uart_remove(struct platform_device *pdev)
{
	uart_remove_one_port(&tegra_combined_uart_driver,
				&tegra_combined_uart_port);
	uart_unregister_driver(&tegra_combined_uart_driver);
	iounmap(spe_mbox_reg);
	iounmap(top0_mbox01_base);
	iounmap(top0_cmn_base);
	return 0;
}

static struct of_device_id tegra_combined_uart_of_match[] = {
	{
		.compatible     = "nvidia,tegra186-combined-uart",
	}, {
	},
};

MODULE_DEVICE_TABLE(of, tegra_combined_uart_of_match);

static unsigned int tegra_combined_uart_tx_empty(struct uart_port *port)
{
	return TIOCSER_TEMT;
}

static struct uart_ops tegra_combined_uart_ops = {
	.pm		= (void (*)(struct uart_port *,
					unsigned int,
					unsigned int)) &uart_null_func,
	.tx_empty	= tegra_combined_uart_tx_empty,
	.get_mctrl	=
		(unsigned int (*)(struct uart_port *)) &uart_null_func,
	.set_mctrl	= (void (*)(struct uart_port *,
					unsigned int)) &uart_null_func,
	.stop_tx	= (void (*)(struct uart_port *)) &uart_null_func,
	.start_tx	= tegra_combined_uart_start_tx,
	.stop_rx	= (void (*)(struct uart_port *)) &uart_null_func,
	.break_ctl	= (void (*)(struct uart_port *, int)) &uart_null_func,
	.startup	= tegra_combined_uart_startup,
	.shutdown	= (void (*)(struct uart_port *)) &uart_null_func,
	.set_termios	= (void (*)(struct uart_port *,
					struct ktermios *,
					struct ktermios *)) &uart_null_func,
	.type		=
		(const char * (*)(struct uart_port *)) &uart_null_func,
	.release_port	= (void (*)(struct uart_port *)) &uart_null_func,
	.request_port	= uart_null_func,
	.config_port	= (void (*)(struct uart_port *, int)) &uart_null_func,
	.verify_port	= (int (*)(struct uart_port *,
				struct serial_struct *)) &uart_null_func,

#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char = uart_null_func,
	.poll_put_char = (void (*)(struct uart_port *,
					unsigned char)) &uart_null_func,
#endif
};

static struct uart_port tegra_combined_uart_port = {
	.lock		= __SPIN_LOCK_UNLOCKED(tegra_combined_uart_port.lock),
	.iotype		= UPIO_MEM,
	.uartclk	= 0,
	.fifosize	= 16,
	.flags		= UPF_BOOT_AUTOCONF,
	.line		= 0,
	.type		= 111,
	.ops		= &tegra_combined_uart_ops,
};

static struct console tegra_combined_uart_console = {
	.name		= "ttyTCU",
	.device		= uart_console_device,
	.flags		= CON_PRINTBUFFER | CON_ANYTIME,
	.index		= -1,
	.write		= tegra_combined_uart_console_write,
	.setup		= tegra_combined_uart_console_setup,
	.data		= &tegra_combined_uart_driver,
};

static struct uart_driver tegra_combined_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "tegra-combined-uart",
	.dev_name	= "ttyTCU",
	.major		= TTY_MAJOR,
	.minor		= 70,
	.cons		= &tegra_combined_uart_console,
	.nr		= 1,
};

static struct platform_driver tegra_combined_uart_platform_driver = {
	.probe		= tegra_combined_uart_probe,
	.remove		= tegra_combined_uart_remove,
	.driver		= {
		.name	= "tegra-combined-uart",
		.of_match_table = of_match_ptr(tegra_combined_uart_of_match),
	},
};

static int __init tegra_combined_uart_init(void)
{
	int ret;

	ret = platform_driver_register(&tegra_combined_uart_platform_driver);
	if (ret < 0) {
		pr_err("%s: Platform driver register failed!\n", __func__);
		return ret;
	}

	register_console(&tegra_combined_uart_console);

	return 0;
}

static void __exit tegra_combined_uart_exit(void)
{
	pr_info("Unloading Tegra combined UART driver\n");
	platform_driver_unregister(&tegra_combined_uart_platform_driver);
}

module_init(tegra_combined_uart_init);
module_exit(tegra_combined_uart_exit);

MODULE_ALIAS("tegra-combined-uart");
MODULE_DESCRIPTION(
	"Linux client driver for Tegra combined UART");
MODULE_AUTHOR("Adeel Raza <araza@nvidia.com>");
MODULE_LICENSE("GPL v2");
