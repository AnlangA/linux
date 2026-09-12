// SPDX-License-Identifier: GPL-2.0-only

#include <linux/serial_core.h>

#if IS_ENABLED(CONFIG_SERIAL_CORE)

/* Only inline/macro adapters; controller logic lives in Rust. */
__rust_helper void rust_helper_uart_port_lock_irqsave(struct uart_port *port,
						      unsigned long *flags)
{
	uart_port_lock_irqsave(port, flags);
}

__rust_helper void rust_helper_uart_port_unlock_irqrestore(struct uart_port *port,
							   unsigned long flags)
{
	uart_port_unlock_irqrestore(port, flags);
}

__rust_helper unsigned int rust_helper_uart_fifo_get(struct uart_port *port,
						     unsigned char *ch)
{
	return uart_fifo_get(port, ch);
}

__rust_helper unsigned int rust_helper_uart_xmit_pending(struct uart_port *port)
{
	return kfifo_len(&port->state->port.xmit_fifo);
}

__rust_helper int rust_helper_uart_tx_stopped(struct uart_port *port)
{
	return uart_tx_stopped(port);
}

__rust_helper int rust_helper_uart_handle_break(struct uart_port *port)
{
	return uart_handle_break(port);
}

#endif
