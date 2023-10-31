#include "usart2_init.h"

volatile bool ready_to_send = false;

char data_to_send[BUFFER_SIZE];

struct USART_buffer husart2;

void default_packet_parcer(char *buffer, uint32_t size);

void (*packet_parcer)(char *, uint32_t) = default_packet_parcer;

uint8_t st_pos = -1;
volatile uint8_t st_send_pos = 0;

void add_to_circle_buffer(uint8_t *buffer, uint8_t data, int16_t *counter);
void extract_string_from_buffer(uint8_t *buffer, char *extract_to, int16_t st_pos);


void write_data(char *result)
{
	strncpy(husart2.tx_buffer, result, BUFFER_SIZE);
	husart2.tx_len = strlen(result);
	husart2.tx_counter = 0;

	CLEAR_BIT(TX_DMA_S->CR, DMA_SxCR_EN);
	while (TX_DMA_S->CR & DMA_SxCR_EN)
	{
	}
	SET_BIT(TX_DMA_S->NDTR, husart2.tx_len);
	SET_BIT(TX_DMA_S->CR, DMA_SxCR_EN);
}

char *get_data()
{
	if (ready_to_send)
	{
		ready_to_send = false;
		extract_string_from_circle_buffer(husart2.rx_process_buffer, data_to_send, st_send_pos);
		return data_to_send;
	}
	return NULL;
}

void add_to_circle_buffer(uint8_t *buffer, uint8_t data, int16_t *counter)
{
	buffer[*counter] = data;

	if (++*counter == BUFFER_SIZE)
	{
		*counter = 0;
	}
}

void extract_string_from_circle_buffer(uint8_t *buffer, char *extract_to, int16_t st_pos) 
{
	uint8_t res_pos = 0;

	do {
		extract_to[res_pos++] = buffer[st_pos++];

		if (st_pos == BUFFER_SIZE && buffer[st_pos-1] != '\0') {
			st_pos = 0;
		}
	} while (buffer[st_pos-1] != '\0');
	
}



void DMA1_Stream6_IRQHandler(void)
{
	if ((DMA1->HISR & DMA_HISR_TCIF6) == DMA_HISR_TCIF6)
	{
		CLEAR_BIT(TX_DMA_S->CR, DMA_SxCR_EN);

		DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
	}
}

uint32_t last_end_point = 0;
uint32_t packet_size;

void DMA1_Stream5_IRQHandler(void)
{
	if (READ_BIT(DMA1->HISR, DMA_HISR_TCIF5))
	{
		SET_BIT(DMA1->HIFCR, DMA_HIFCR_CTCIF5);
		// CLEAR_BIT(RX_DMA_S->CR, DMA_SxCR_EN);

		packet_parcer(&husart2.rx_buffer[last_end_point], BUFFER_SIZE - last_end_point);

		last_end_point = 0;
	}
}

void USART2_IRQHandler(void)
{
	if (READ_BIT(USART2->SR, USART_SR_IDLE))
	{
		USART2->DR;
		packet_size = -last_end_point + (BUFFER_SIZE - RX_DMA_S->NDTR);

		packet_parcer(&husart2.rx_buffer[last_end_point], packet_size);

		last_end_point += packet_size;
	}
}

void usart2_init()
{
	SystemCoreClockUpdate();
	usart_init();
	DMA_init();
}

void default_packet_parcer(char *buffer, uint32_t size)
{

	static bool transaction_started = false;

	for (int ch_pos = 0; ch_pos < size; ++ch_pos)
	{
		char char_digit = buffer[ch_pos];
		if (char_digit == START_TR_CHAR)
		{
			st_pos = husart2.rx_process_counter;
			transaction_started = true;
			continue;
			;
		}

		if (char_digit == END_TR_CHAR && transaction_started)
		{
			st_send_pos = st_pos;
			ready_to_send = true;
			add_to_circle_buffer(husart2.rx_process_buffer, 0, &husart2.rx_process_counter);
			// husart2.proccess_buffer[husart2.rx_process_counter++] = 0;
			transaction_started = false;
			continue;
		}

		if (char_digit > '9' || char_digit < '0')
		{
			transaction_started = false;
			continue;
			;
		}

		if (transaction_started)
		{
			add_to_circle_buffer(husart2.rx_process_buffer, char_digit, &husart2.rx_process_counter);
		}
	}
}

void set_packet_parcer(void (*parcer)(char *, uint32_t)) {
	packet_parcer = parcer;
}
