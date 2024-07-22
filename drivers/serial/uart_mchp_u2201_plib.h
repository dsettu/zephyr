
#include <soc.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <zephyr/drivers/uart.h>

struct usart_init_data {
	struct uart_config *config;
    uint32_t pads;
	uint32_t clkFrequency;
};

int sercom_usart_init(sercom_usart_int_registers_t *const usart, const struct usart_init_data *data);
int sercom_usart_config(sercom_usart_int_registers_t *const usart, const struct uart_config *cfg, uint32_t clk_freq_hz);
int sercom_err_check(sercom_usart_int_registers_t *const usart);

//*********************************
// Polling UART APIs
//*********************************
int sercom_usart_poll_in(sercom_usart_int_registers_t *const usart, unsigned char *c);
void sercom_usart_poll_out(sercom_usart_int_registers_t *const usart, unsigned char c);

//*********************************
// Interrupt Drivern UART APIs
//*********************************
int sercom_usart_fifo_fill(sercom_usart_int_registers_t *const usart,
                   const uint8_t *tx_data, int len);
int sercom_usart_fifo_read(sercom_usart_int_registers_t *const usart, uint8_t *rx_data,
                   const int size);
				   
void sercom_usart_irq_tx_enable(sercom_usart_int_registers_t *const usart);
void sercom_usart_irq_tx_disable(sercom_usart_int_registers_t *const usart);
int sercom_usart_irq_tx_ready(sercom_usart_int_registers_t *const usart);
int sercom_usart_irq_tx_complete(sercom_usart_int_registers_t *const usart);

void sercom_usart_irq_rx_enable(sercom_usart_int_registers_t *const usart);
void sercom_usart_irq_rx_disable(sercom_usart_int_registers_t *const usart);
int sercom_usart_irq_rx_ready(sercom_usart_int_registers_t *const usart);

int sercom_usart_irq_is_pending(sercom_usart_int_registers_t *const usart);
					   
void sercom_usart_irq_err_enable(sercom_usart_int_registers_t *const usart);
void sercom_usart_irq_err_disable(sercom_usart_int_registers_t *const usart);


//*********************************
// Async UART APIs
//*********************************