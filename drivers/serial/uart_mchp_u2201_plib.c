#include "uart_mchp_u2201_plib.h"



int sercom_usart_init(sercom_usart_int_registers_t *const usart, const struct usart_init_data *data)
{
   int retval;
	
	const struct uart_config *const cfg = data->config;
		
    /*
     * Configures USART Clock Mode
     * Configures TXPO and RXPO
     * Configures Data Order
     * Configures IBON
     */
    usart->SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK | data->pads | SERCOM_USART_INT_CTRLA_DORD_Msk | SERCOM_USART_INT_CTRLA_IBON_Msk;

	/* Configure SERCOM */
	retval=sercom_usart_config(usart,cfg,data->clkFrequency);
    if (retval < 0) {
        return retval;
    }

    /* Disable the USART before configurations */
    usart->SERCOM_CTRLA &= ~SERCOM_USART_INT_CTRLA_ENABLE_Msk;

    /* Wait for sync */
    while((usart->SERCOM_SYNCBUSY) != 0U)
    {
        /* Do nothing */
    }
	
		
    /*
     * Configures RXEN
     * Configures TXEN
     */
    usart->SERCOM_CTRLB =  SERCOM_USART_INT_CTRLB_RXEN_Msk | SERCOM_USART_INT_CTRLB_TXEN_Msk;

    /* Wait for sync */
    while((usart->SERCOM_SYNCBUSY) != 0U)
    {
        /* Do nothing */
    }


    /* Enable the UART after the configurations */
    usart->SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;

    /* Wait for sync */
    while((usart->SERCOM_SYNCBUSY) != 0U)
    {
        /* Do nothing */
    }
    return 0;
}



int sercom_usart_config(sercom_usart_int_registers_t *const usart, const struct uart_config *config,
                  uint32_t clk_freq_hz)
{
    bool setupStatus       = false;
    uint32_t baudValue     = 0U;
    uint32_t sampleRate    = 0U;
    uint32_t CTRLA_temp = usart->SERCOM_CTRLA;
    uint32_t CTRLB_temp = usart->SERCOM_CTRLB;

    if((config != NULL) && (config->baudrate != 0U))
    {

        if(clk_freq_hz >= (16U * config->baudrate))
        {
            baudValue = 65536U - (uint32_t)(((uint64_t)65536U * 16U * config->baudrate) / clk_freq_hz);
            sampleRate = 0U;
        }
        else if(clk_freq_hz >= (8U * config->baudrate))
        {
            baudValue = 65536U - (uint32_t)(((uint64_t)65536U * 8U * config->baudrate) / clk_freq_hz);
            sampleRate = 2U;
        }
        else if(clk_freq_hz >= (3U * config->baudrate))
        {
            baudValue = 65536U - (uint32_t)(((uint64_t)65536U * 3U * config->baudrate) / clk_freq_hz);
            sampleRate = 4U;
        }
        else
        {
            /* Do nothing */
        }

        CTRLA_temp =  (CTRLA_temp & ~SERCOM_USART_INT_CTRLA_SAMPR_Msk) | SERCOM_USART_INT_CTRLA_SAMPR((uint32_t)sampleRate); 

		switch (config->parity) {
			case UART_CFG_PARITY_NONE:
				CTRLA_temp = (CTRLA_temp & ~SERCOM_USART_INT_CTRLA_FORM_Msk) | SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_NO_PARITY;
				break;
			case UART_CFG_PARITY_ODD:
				CTRLA_temp = (CTRLA_temp & ~SERCOM_USART_INT_CTRLA_FORM_Msk) | SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_WITH_PARITY;
				CTRLB_temp = (CTRLB_temp & ~SERCOM_USART_INT_CTRLB_PMODE_Msk) | SERCOM_USART_INT_CTRLB_PMODE_ODD;
				break;
			case UART_CFG_PARITY_EVEN:
				CTRLA_temp = (CTRLA_temp & ~SERCOM_USART_INT_CTRLA_FORM_Msk) | SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_WITH_PARITY;
				CTRLB_temp = (CTRLB_temp & ~SERCOM_USART_INT_CTRLB_PMODE_Msk) | SERCOM_USART_INT_CTRLB_PMODE_EVEN;
				break;
			default:
				return -ENOTSUP;
		}


		switch (config->stop_bits) {
			case UART_CFG_STOP_BITS_1:
				CTRLB_temp = (CTRLB_temp & ~SERCOM_USART_INT_CTRLB_SBMODE_Msk);
				break;
			case UART_CFG_STOP_BITS_2:
				CTRLB_temp = (CTRLB_temp & ~SERCOM_USART_INT_CTRLB_SBMODE_Msk) | SERCOM_USART_INT_CTRLB_SBMODE_Msk;
				break;
			default:
				return -ENOTSUP;
		}


		switch (config->data_bits) {
			case UART_CFG_DATA_BITS_5:
				CTRLB_temp = (CTRLB_temp & ~SERCOM_USART_INT_CTRLB_CHSIZE_Msk) | SERCOM_USART_INT_CTRLB_CHSIZE(0x5);
				break;
			case UART_CFG_DATA_BITS_6:
				CTRLB_temp = (CTRLB_temp & ~SERCOM_USART_INT_CTRLB_CHSIZE_Msk) | SERCOM_USART_INT_CTRLB_CHSIZE(0x6);
				break;
			case UART_CFG_DATA_BITS_7:
				CTRLB_temp = (CTRLB_temp & ~SERCOM_USART_INT_CTRLB_CHSIZE_Msk) | SERCOM_USART_INT_CTRLB_CHSIZE(0x7);
				break;
			case UART_CFG_DATA_BITS_8:
				CTRLB_temp = (CTRLB_temp & ~SERCOM_USART_INT_CTRLB_CHSIZE_Msk) | SERCOM_USART_INT_CTRLB_CHSIZE(0x0);
				break;
			case UART_CFG_DATA_BITS_9:
				CTRLB_temp = (CTRLB_temp & ~SERCOM_USART_INT_CTRLB_CHSIZE_Msk) | SERCOM_USART_INT_CTRLB_CHSIZE(0x1);
				break;
			default:
				return -ENOTSUP;
		}


        /* Disable the USART before configurations */
        usart->SERCOM_CTRLA &= ~SERCOM_USART_INT_CTRLA_ENABLE_Msk;

        /* Wait for sync */
        while((usart->SERCOM_SYNCBUSY) != 0U)
        {
            /* Do nothing */
        }

        /* Configure Baud Rate */
        usart->SERCOM_BAUD = (uint16_t)SERCOM_USART_INT_BAUD_BAUD(baudValue);

		usart->SERCOM_CTRLA = CTRLA_temp;
		usart->SERCOM_CTRLB = CTRLB_temp;

        /* Wait for sync */
        while((usart->SERCOM_SYNCBUSY) != 0U)
        {
            /* Do nothing */
        }

        /* Enable the USART after the configurations */
        usart->SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;

        /* Wait for sync */
        while((usart->SERCOM_SYNCBUSY) != 0U)
        {
            /* Do nothing */
        }

        setupStatus = true;
    }

    return setupStatus;
}

int sercom_err_check(sercom_usart_int_registers_t *const usart)
{
    uint32_t err = 0U;

    if (usart->SERCOM_STATUS & SERCOM_USART_INT_STATUS_BUFOVF_Msk) {
        err |= UART_ERROR_OVERRUN;
    }

    if (usart->SERCOM_STATUS & SERCOM_USART_INT_STATUS_FERR_Msk) {
        err |= UART_ERROR_PARITY;
    }

    if (usart->SERCOM_STATUS & SERCOM_USART_INT_STATUS_PERR_Msk) {
        err |= UART_ERROR_FRAMING;
    }

    if (usart->SERCOM_STATUS & SERCOM_USART_INT_STATUS_ISF_Msk) {
        err |= UART_BREAK;
    }

    if (usart->SERCOM_STATUS & SERCOM_USART_INT_STATUS_COLL_Msk) {
        err |= UART_ERROR_COLLISION;
    }

    usart->SERCOM_STATUS |=  SERCOM_USART_INT_STATUS_BUFOVF_Msk
             |  SERCOM_USART_INT_STATUS_FERR_Msk
             |  SERCOM_USART_INT_STATUS_PERR_Msk
             |  SERCOM_USART_INT_STATUS_COLL_Msk
             |  SERCOM_USART_INT_STATUS_ISF_Msk;

    return err;	
}




//*********************************
// Polling UART APIs
//*********************************
int sercom_usart_poll_in(sercom_usart_int_registers_t *const usart, unsigned char *c)
{

    if (!(usart->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk)) {
        return -EBUSY;
    }

    *c = (unsigned char)usart->SERCOM_DATA;
    return 0;
}

void sercom_usart_poll_out(sercom_usart_int_registers_t *const usart, unsigned char c)
{
    while (!(usart->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk)) {
    }

    /* send a character */
    usart->SERCOM_DATA = c;
}

//*********************************
// Interrupt Drivern UART APIs
//*********************************
int sercom_usart_fifo_fill(sercom_usart_int_registers_t *const usart,
                   const uint8_t *tx_data, int len)
{
    if ((usart->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk) && len >= 1) {
        usart->SERCOM_DATA = tx_data[0];
        return 1;
    } else {
        return 0;
    }
}

int sercom_usart_fifo_read(sercom_usart_int_registers_t *const usart, uint8_t *rx_data,
                   const int size)
{

    if ((usart->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk)) {
        uint8_t ch = usart->SERCOM_DATA;

        if (size >= 1) {
            *rx_data = ch;
            return 1;
        } else {
            return -EINVAL;
        }
    }
    return 0;
}

void sercom_usart_irq_tx_enable(sercom_usart_int_registers_t *const usart)
{
    usart->SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_DRE_Msk
               | SERCOM_USART_INT_INTENSET_TXC_Msk;
}

void sercom_usart_irq_tx_disable(sercom_usart_int_registers_t *const usart)
{
    usart->SERCOM_INTENCLR = SERCOM_USART_INT_INTENCLR_DRE_Msk
               | SERCOM_USART_INT_INTENCLR_TXC_Msk;
}

int sercom_usart_irq_tx_ready(sercom_usart_int_registers_t *const usart)
{
    return ((usart->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk) != 0) &&
    ((usart->SERCOM_INTENSET & SERCOM_USART_INT_INTENSET_DRE_Msk) != 0);
}

int sercom_usart_irq_tx_complete(sercom_usart_int_registers_t *const usart)
{
    return ((usart->SERCOM_INTENSET & SERCOM_USART_INT_INTENSET_TXC_Msk) != 0);
}

void sercom_usart_irq_rx_enable(sercom_usart_int_registers_t *const usart)
{
    usart->SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_RXC_Msk;
}

void sercom_usart_irq_rx_disable(sercom_usart_int_registers_t *const usart)
{
    usart->SERCOM_INTENCLR = SERCOM_USART_INT_INTENCLR_RXC_Msk;
}

int sercom_usart_irq_rx_ready(sercom_usart_int_registers_t *const usart)
{
    return (usart->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) != 0;
}

int sercom_usart_irq_is_pending(sercom_usart_int_registers_t *const usart)
{
    return (usart->SERCOM_INTENSET & usart->SERCOM_INTFLAG) != 0;
}

void sercom_usart_irq_err_enable(sercom_usart_int_registers_t *const usart)
{
    usart->SERCOM_INTENSET |= SERCOM_USART_INT_INTENSET_ERROR_Msk;
}

void sercom_usart_irq_err_disable(sercom_usart_int_registers_t *const usart)
{
    usart->SERCOM_INTENCLR |= SERCOM_USART_INT_INTENCLR_ERROR_Msk;

}

//*********************************
// Async UART APIs
//*********************************