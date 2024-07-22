/*
 * Copyright (c) 2024 Microchip Technology Inc. and its subsidiaries.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_dmac_u2223

#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/dma.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(dma_microchip_dmac_u2223, CONFIG_DMA_LOG_LEVEL);

#define DMA_REGS	((dmac_registers_t *)DT_INST_REG_ADDR(0))

struct dma_u2223channel {
	dma_callback_t cb;
	void *user_data;
};

struct dma_u2223data {
	__aligned(16) dmac_descriptor_registers_t descriptors[DMAC_CH_NUM];
	__aligned(16) dmac_descriptor_registers_t descriptors_wb[DMAC_CH_NUM];
	struct dma_u2223channel channels[DMAC_CH_NUM];
};

/* Handles DMA interrupts and dispatches to the individual channel */
static void dma_u2223isr(const struct device *dev)
{
	struct dma_u2223data *data = dev->data;
	struct dma_u2223channel *chdata;
	uint16_t pend = DMA_REGS->DMAC_INTPEND;
	uint32_t channel;

	/* Acknowledge all interrupts for the channel in pend */
	DMA_REGS->DMAC_INTPEND = pend;

	channel = (pend & DMAC_INTPEND_ID_Msk) >> DMAC_INTPEND_ID_Pos;
	chdata = &data->channels[channel];

	if (pend & DMAC_INTPEND_TERR_Msk) {
		if (chdata->cb) {
			chdata->cb(dev, chdata->user_data,
				   channel, -DMAC_INTPEND_TERR_Msk);
		}
	} else if (pend & DMAC_INTPEND_TCMPL_Msk) {
		if (chdata->cb) {
			chdata->cb(dev, chdata->user_data, channel, 0);
		}
	}

	/*
	 * If more than one channel is pending, we'll just immediately
	 * interrupt again and handle it through a different INTPEND value.
	 */
}

/* Configure a channel */
static int dma_u2223config(const struct device *dev, uint32_t channel,
			   struct dma_config *config)
{
	struct dma_u2223data *data = dev->data;
	dmac_descriptor_registers_t *desc = &data->descriptors[channel];
	struct dma_block_config *block = config->head_block;
	struct dma_u2223channel *channel_control;
	uint16_t btctrl = 0;
	unsigned int key;

	if (channel >= DMAC_CH_NUM) {
		LOG_ERR("Unsupported channel");
		return -EINVAL;
	}

	if (config->block_count > 1) {
		LOG_ERR("Chained transfers not supported");
		/* TODO: add support for chained transfers. */
		return -ENOTSUP;
	}

	if (config->dma_slot >= DMAC_TRIG_NUM) {
		LOG_ERR("Invalid trigger number");
		return -EINVAL;
	}

	/* Lock and page in the channel configuration */
	key = irq_lock();

	/*
	 * The "bigger" DMAC on some SAM0 chips (e.g. SAMD5x) has
	 * independently accessible registers for each channel, while
	 * the other ones require an indirect channel selection before
	 * accessing shared registers.  The simplest way to detect the
	 * difference is the presence of the DMAC_CHID_ID macro from the
	 * ASF HAL (i.e. it's only defined if indirect access is required).
	 */
#ifdef DMAC_CHID_ID_Msk
	/* Select the channel for configuration */
	DMA_REGS->DMAC_CHID = DMAC_CHID_ID(channel);
	DMA_REGS->DMAC_CHCTRLA = 0;

	/* Connect the peripheral trigger */
	if (config->channel_direction == MEMORY_TO_MEMORY) {
		/*
		 * A single software trigger will start the
		 * transfer
		 */
		DMA_REGS->DMAC_CHCTRLB = DMAC_CHCTRLB_TRIGACT_TRANSACTION |
				    DMAC_CHCTRLB_TRIGSRC(config->dma_slot);
	} else {
		/* One peripheral trigger per beat */
		DMA_REGS->DMAC_CHCTRLB = DMAC_CHCTRLB_TRIGACT_BEAT |
				    DMAC_CHCTRLB_TRIGSRC(config->dma_slot);
	}

	/* Set the priority */
	if (config->channel_priority >= DMAC_LVL_NUM) {
		LOG_ERR("Invalid priority");
		goto inval;
	}

	DMA_REGS->DMAC_CHCTRLB = (DMA_REGS->DMAC_CHCTRLB & ~DMAC_CHCTRLB_LVL_Msk) | DMAC_CHCTRLB_LVL(config->channel_priority);

	/* Enable the interrupts */
	DMA_REGS->DMAC_CHINTENSET = DMAC_CHINTENSET_TCMPL_Msk;
	if (!config->error_callback_dis) {
		DMA_REGS->DMAC_CHINTENSET = DMAC_CHINTENSET_TERR_Msk;
	} else {
		DMA_REGS->DMAC_CHINTENCLR = DMAC_CHINTENSET_TERR_Msk;
	}

	DMA_REGS->DMAC_CHINTFLAG = DMAC_CHINTFLAG_TERR_Msk | DMAC_CHINTFLAG_TCMPL_Msk;
#else
	/* Channels have separate configuration registers */
	DmacChannel * chcfg = &DMA_REGS->Channel[channel];

	if (config->channel_direction == MEMORY_TO_MEMORY) {
		/*
		 * A single software trigger will start the
		 * transfer
		 */
		chcfg->DMAC_CHCTRLA = DMAC_CHCTRLA_TRIGACT_TRANSACTION |
				     DMAC_CHCTRLA_TRIGSRC(config->dma_slot);
	} else if ((config->channel_direction == MEMORY_TO_PERIPHERAL) ||
		(config->channel_direction == PERIPHERAL_TO_MEMORY)) {
		/* One peripheral trigger per beat */
		chcfg->DMAC_CHCTRLA = DMAC_CHCTRLA_TRIGACT_BURST |
				     DMAC_CHCTRLA_TRIGSRC(config->dma_slot);
	} else {
		LOG_ERR("Direction error. %d", config->channel_direction);
		goto inval;
	}

	/* Set the priority */
	if (config->channel_priority >= DMAC_LVL_NUM) {
		LOG_ERR("Invalid priority");
		goto inval;
	}

	chcfg->CHPRILVL = (chcfg->CHPRILVL & ~DMAC_CHPRILVL_PRILVL_Msk) | DMAC_CHPRILVL_PRILVL(config->channel_priority);

	/* Set the burst length */
	if (config->source_burst_length != config->dest_burst_length) {
		LOG_ERR("Source and destination burst lengths must be equal");
		goto inval;
	}

	if (config->source_burst_length > 16U) {
		LOG_ERR("Invalid burst length");
		goto inval;
	}

	if (config->source_burst_length > 0U) {
		chcfg->DMAC_CHCTRLA |= DMAC_CHCTRLA_BURSTLEN(
			config->source_burst_length - 1U);
	}

	/* Enable the interrupts */
	chcfg->DMAC_CHINTENSET = DMAC_CHINTENSET_TCMPL_Msk;
	if (!config->error_callback_dis) {
		chcfg->DMAC_CHINTENSET = DMAC_CHINTENSET_TERR_Msk;
	} else {
		chcfg->DMAC_CHINTENCLR = DMAC_CHINTENSET_TERR_Msk;
	}

	chcfg->DMAC_CHINTFLAG = DMAC_CHINTFLAG_TERR_Msk | DMAC_CHINTFLAG_TCMPL_Msk;
#endif

	/* Set the beat (single transfer) size */
	if (config->source_data_size != config->dest_data_size) {
		LOG_ERR("Source and destination data sizes must be equal");
		goto inval;
	}

	switch (config->source_data_size) {
	case 1:
		btctrl = (btctrl & ~DMAC_BTCTRL_BEATSIZE_Msk) | DMAC_BTCTRL_BEATSIZE_BYTE;
		break;
	case 2:
		btctrl = (btctrl & ~DMAC_BTCTRL_BEATSIZE_Msk) | DMAC_BTCTRL_BEATSIZE_HWORD;
		break;
	case 4:
		btctrl = (btctrl & ~DMAC_BTCTRL_BEATSIZE_Msk) | DMAC_BTCTRL_BEATSIZE_WORD;
		break;
	default:
		LOG_ERR("Invalid data size");
		goto inval;
	}

	/* Set up the one and only block */
	desc->DMAC_BTCNT = block->block_size / config->source_data_size;
	desc->DMAC_DESCADDR = 0;

	/* Set the automatic source / dest increment */
	switch (block->source_addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		desc->DMAC_SRCADDR = block->source_address + block->block_size;
		btctrl |= DMAC_BTCTRL_SRCINC_Msk;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		desc->DMAC_SRCADDR = block->source_address;
		break;
	default:
		LOG_ERR("Invalid source increment");
		goto inval;
	}

	switch (block->dest_addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		desc->DMAC_DSTADDR = block->dest_address + block->block_size;
		btctrl |= DMAC_BTCTRL_DSTINC_Msk;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		desc->DMAC_DSTADDR = block->dest_address;
		break;
	default:
		LOG_ERR("Invalid destination increment");
		goto inval;
	}

	btctrl |= DMAC_BTCTRL_VALID_Msk;
	desc->DMAC_BTCTRL = btctrl;

	channel_control = &data->channels[channel];
	channel_control->cb = config->dma_callback;
	channel_control->user_data = config->user_data;

	LOG_DBG("Configured channel %d for %08X to %08X (%u)",
		channel,
		block->source_address,
		block->dest_address,
		block->block_size);

	irq_unlock(key);
	return 0;

inval:
	irq_unlock(key);
	return -EINVAL;
}

static int dma_u2223start(const struct device *dev, uint32_t channel)
{
	unsigned int key = irq_lock();

	ARG_UNUSED(dev);

#ifdef DMAC_CHID_ID_Msk
	DMA_REGS->DMAC_CHID = channel;
	DMA_REGS->DMAC_CHCTRLA = DMAC_CHCTRLA_ENABLE_Msk;

	if ((DMA_REGS->DMAC_CHCTRLB & DMAC_CHCTRLB_TRIGSRC_Msk) == 0) {
		/* Trigger via software */
		DMA_REGS->DMAC_SWTRIGCTRL = 1U << channel;
	}

#else
	DmacChannel * chcfg = &DMA_REGS->Channel[channel];

	chcfg->CHCTRLA |= DMAC_CHCTRLA_ENABLE_Msk;

	if ((chcfg->CHCTRLA & DMAC_CHCTRLA_TRIGSRC_Msk) == 0) {
		/* Trigger via software */
		DMA_REGS->DMAC_SWTRIGCTRL = 1U << channel;
	}
#endif

	irq_unlock(key);

	return 0;
}

static int dma_u2223stop(const struct device *dev, uint32_t channel)
{
	unsigned int key = irq_lock();

	ARG_UNUSED(dev);

#ifdef DMAC_CHID_ID_Msk
	DMA_REGS->DMAC_CHID = channel;
	DMA_REGS->DMAC_CHCTRLA = 0;
#else
	DmacChannel * chcfg = &DMA_REGS->Channel[channel];

	chcfg->DMAC_CHCTRLA = (chcfg->DMAC_CHCTRLA & ~DMAC_CHCTRLA_ENABLE_Msk);
#endif

	irq_unlock(key);

	return 0;
}

static int dma_u2223reload(const struct device *dev, uint32_t channel,
			   uint32_t src, uint32_t dst, size_t size)
{
	struct dma_u2223data *data = dev->data;
	dmac_descriptor_registers_t *desc = &data->descriptors[channel];
	unsigned int key = irq_lock();

	switch ((desc->DMAC_BTCTRL & DMAC_BTCTRL_BEATSIZE_Msk)>>DMAC_BTCTRL_BEATSIZE_Pos)  {
	case DMAC_BTCTRL_BEATSIZE_BYTE_Val:
		desc->DMAC_BTCNT = size;
		break;
	case DMAC_BTCTRL_BEATSIZE_HWORD_Val:
		desc->DMAC_BTCNT = size / 2U;
		break;
	case DMAC_BTCTRL_BEATSIZE_WORD_Val:
		desc->DMAC_BTCNT = size / 4U;
		break;
	default:
		goto inval;
	}

	if ((desc->DMAC_BTCTRL & DMAC_BTCTRL_SRCINC_Msk)>>DMAC_BTCTRL_SRCINC_Pos) {
		desc->DMAC_SRCADDR = src + size;
	} else {
		desc->DMAC_SRCADDR = src;
	}

	if ((desc->DMAC_BTCTRL & DMAC_BTCTRL_DSTINC_Msk)>>DMAC_BTCTRL_DSTINC_Pos) {
		desc->DMAC_DSTADDR = dst + size;
	} else {
		desc->DMAC_DSTADDR = dst;
	}

	LOG_DBG("Reloaded channel %d for %08X to %08X (%u)",
		channel, src, dst, size);

	irq_unlock(key);
	return 0;

inval:
	irq_unlock(key);
	return -EINVAL;
}

static int dma_u2223get_status(const struct device *dev, uint32_t channel,
			       struct dma_status *stat)
{
	struct dma_u2223data *data = dev->data;
	uint32_t act;

	if (channel >= DMAC_CH_NUM || stat == NULL) {
		return -EINVAL;
	}

	act = DMA_REGS->DMAC_ACTIVE;
	if ((act & DMAC_ACTIVE_ABUSY_Msk) &&
	    ((act & DMAC_ACTIVE_ID_Msk) >> DMAC_ACTIVE_ID_Pos) == channel) {
		stat->busy = true;
		stat->pending_length = (act & DMAC_ACTIVE_BTCNT_Msk) >>
				       DMAC_ACTIVE_BTCNT_Pos;
	} else {
		stat->busy = false;
		stat->pending_length = data->descriptors_wb[channel].DMAC_BTCNT;
	}

	switch ((data->descriptors[channel].DMAC_BTCTRL & DMAC_BTCTRL_BEATSIZE_Msk)>>DMAC_BTCTRL_BEATSIZE_Pos) {
	case DMAC_BTCTRL_BEATSIZE_BYTE_Val:
		break;
	case DMAC_BTCTRL_BEATSIZE_HWORD_Val:
		stat->pending_length *= 2U;
		break;
	case DMAC_BTCTRL_BEATSIZE_WORD_Val:
		stat->pending_length *= 4U;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define dma_u2223IRQ_CONNECT(n)						 \
	do {								 \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, n, irq),		 \
			    DT_INST_IRQ_BY_IDX(0, n, priority),		 \
			    dma_u2223isr, DEVICE_DT_INST_GET(0), 0);	 \
		irq_enable(DT_INST_IRQ_BY_IDX(0, n, irq));		 \
	} while (false)

static int dma_u2223init(const struct device *dev)
{
	struct dma_u2223data *data = dev->data;

	/* Enable clocks. */
#ifdef MCLK_REGS
	MCLK_REGS->MCLK_AHBMASK |= MCLK_AHBMASK_DMAC_Msk;
#else
	PM_REGS->PM_AHBMASK |= PM_AHBMASK_DMAC;
	PM_REGS->PM_APBBMASK |= PM_APBBMASK_DMAC;
#endif

	/* Set up the descriptor and write back addresses */
	DMA_REGS->DMAC_BASEADDR = (uintptr_t)&data->descriptors;
	DMA_REGS->DMAC_WRBADDR = (uintptr_t)&data->descriptors_wb;

#if DMAC_PRICTRL0_Msk
	/* Statically map each level to the same numeric priority */
	DMA_REGS->DMAC_PRICTRL0 =
		DMAC_PRICTRL0_LVLPRI0(0) | DMAC_PRICTRL0_LVLPRI1(1) |
		DMAC_PRICTRL0_LVLPRI2(2) | DMAC_PRICTRL0_LVLPRI3(3);
#else
	/* Statically map each level to the same numeric priority */
	DMA_REGS->DMAC_PRICTRL =
		DMAC_PRICTRL_LVLPRI0(0) | DMAC_PRICTRL_LVLPRI1(1) |
		DMAC_PRICTRL_LVLPRI2(2) | DMAC_PRICTRL_LVLPRI3(3);
#endif

	/* Enable the unit and enable all priorities */
	DMA_REGS->DMAC_CTRL = DMAC_CTRL_DMAENABLE_Msk | DMAC_CTRL_LVLEN(0x0F);

#if DT_INST_IRQ_HAS_CELL(0, irq)
	dma_u2223IRQ_CONNECT(0);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 1)
	dma_u2223IRQ_CONNECT(1);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 2)
	dma_u2223IRQ_CONNECT(2);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 3)
	dma_u2223IRQ_CONNECT(3);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 4)
	dma_u2223IRQ_CONNECT(4);
#endif

	return 0;
}

static struct dma_u2223data dmac_data;

static const struct dma_driver_api dma_u2223api = {
	.config = dma_u2223config,
	.start = dma_u2223start,
	.stop = dma_u2223stop,
	.reload = dma_u2223reload,
	.get_status = dma_u2223get_status,
};

DEVICE_DT_INST_DEFINE(0, &dma_u2223init, NULL,
		    &dmac_data, NULL, PRE_KERNEL_1,
		    CONFIG_DMA_INIT_PRIORITY, &dma_u2223api);
