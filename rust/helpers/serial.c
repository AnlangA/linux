// SPDX-License-Identifier: GPL-2.0-only

#include <linux/serial_core.h>
#include <linux/tty_flip.h>

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

__rust_helper unsigned int rust_helper_uart_fifo_peek(struct uart_port *port,
						    unsigned char *buf,
						    unsigned int count)
{
	return kfifo_out_peek(&port->state->port.xmit_fifo, buf, count);
}

__rust_helper void rust_helper_uart_xmit_advance(struct uart_port *port,
					       unsigned int count)
{
	uart_xmit_advance(port, count);
}

__rust_helper int rust_helper_tty_insert_flip_string(struct tty_port *port,
						   const unsigned char *buf,
						   size_t size)
{
	return tty_insert_flip_string(port, buf, size);
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
