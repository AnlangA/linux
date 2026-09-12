// SPDX-License-Identifier: GPL-2.0-only
#include <linux/dmaengine.h>
#include <linux/workqueue.h>

#if IS_BUILTIN(CONFIG_DMA_ENGINE)
__rust_helper int rust_helper_dmaengine_slave_config(struct dma_chan *chan,
						    struct dma_slave_config *cfg)
{
	return dmaengine_slave_config(chan, cfg);
}

__rust_helper struct dma_async_tx_descriptor *
rust_helper_dmaengine_prep_slave_single(struct dma_chan *chan, dma_addr_t addr,
				       size_t len, enum dma_transfer_direction dir,
				       unsigned long flags)
{
	return dmaengine_prep_slave_single(chan, addr, len, dir, flags);
}

__rust_helper dma_cookie_t
rust_helper_dmaengine_submit(struct dma_async_tx_descriptor *desc)
{
	return dmaengine_submit(desc);
}

__rust_helper void rust_helper_dma_async_issue_pending(struct dma_chan *chan)
{
	dma_async_issue_pending(chan);
}

__rust_helper int rust_helper_dmaengine_pause(struct dma_chan *chan)
{
	return dmaengine_pause(chan);
}

__rust_helper int rust_helper_dmaengine_terminate_async(struct dma_chan *chan)
{
	return dmaengine_terminate_async(chan);
}

__rust_helper int rust_helper_dmaengine_terminate_sync(struct dma_chan *chan)
{
	return dmaengine_terminate_sync(chan);
}

__rust_helper enum dma_status
rust_helper_dmaengine_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
				struct dma_tx_state *state)
{
	return dmaengine_tx_status(chan, cookie, state);
}
#endif

__rust_helper void rust_helper_init_delayed_work(struct delayed_work *work,
					       work_func_t func)
{
	INIT_DELAYED_WORK(work, func);
}

__rust_helper bool rust_helper_schedule_delayed_work(struct delayed_work *work,
						   unsigned long delay)
{
	return schedule_delayed_work(work, delay);
}

__rust_helper bool rust_helper_mod_delayed_work(struct delayed_work *work,
					      unsigned long delay)
{
	return mod_delayed_work(system_wq, work, delay);
}
