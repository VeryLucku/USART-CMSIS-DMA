#include "cmsis_init.h"
// TODO buffer cycle
#define START_TR_CHAR 'S'
#define END_TR_CHAR 'E'

volatile bool ready_to_send = false;

char data_to_send[BUFFER_SIZE];

struct USART_buffer husart2;

uint8_t st_pos = -1;
volatile uint8_t st_send_pos = 0;

void write_data(char *result)
{
	strncpy(husart2.tx_buffer, result, BUFFER_SIZE);
	husart2.tx_len = strlen(result);
	husart2.tx_counter = 0;

	CLEAR_BIT(TX_DMA_S->CR, DMA_SxCR_EN);
	while (TX_DMA_S->CR & DMA_SxCR_EN) {}
	SET_BIT(TX_DMA_S->NDTR, husart2.tx_len);
	SET_BIT(TX_DMA_S->CR, DMA_SxCR_EN);
}

char *get_data()
{
	if (ready_to_send)
	{
		strncpy(data_to_send, &husart2.proccess_buffer[st_send_pos], BUFFER_SIZE);
		ready_to_send = false;
		return data_to_send;
	}
	return NULL;
}

void process_user_input(char *buffer, uint32_t size)
{

	static bool transaction_started = false;

	for (int ch_pos = 0; ch_pos < size; ++ch_pos)
	{
		char char_digit = buffer[ch_pos];
		if (char_digit == START_TR_CHAR)
		{
			st_pos = husart2.rx_process_counter;
			transaction_started = true;
			continue;;
		}

		if (char_digit == END_TR_CHAR && transaction_started)
		{
			st_send_pos = st_pos;
			ready_to_send = true;
			husart2.proccess_buffer[husart2.rx_process_counter++] = 0;
			transaction_started = false;
			continue;;
		}

		if (char_digit > '9' || char_digit < '0')
		{
			transaction_started = false;
			continue;;
		}

		if (transaction_started)
		{
			husart2.proccess_buffer[husart2.rx_process_counter++] = char_digit;
		}
	}
}

void DMA1_Stream6_IRQHandler(void)
{
	if ((DMA1->HISR & DMA_HISR_TCIF6) == DMA_HISR_TCIF6)
	{
		CLEAR_BIT(TX_DMA_S->CR, DMA_SxCR_EN);

		DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
	}
}

void DMA1_Stream5_IRQHandler(void)
{
	if ((DMA1->HISR & DMA_HISR_TCIF5) == DMA_HISR_TCIF5)
	{
		CLEAR_BIT(RX_DMA_S->CR, DMA_SxCR_EN);

		DMA1->HIFCR |= DMA_HIFCR_CTCIF5;
	}
}

uint32_t last_end_point = BUFFER_SIZE;
uint32_t packet_size;
void USART2_IRQHandler(void)
{
	if (READ_BIT(USART2->SR, USART_SR_IDLE) == USART_SR_IDLE)
	{
		USART2->DR;
		packet_size = last_end_point - RX_DMA_S->NDTR;
	
		process_user_input(&husart2.rx_buffer[BUFFER_SIZE - last_end_point], packet_size);

		last_end_point -= packet_size;
	}
}

void usart2_init()
{
	SystemCoreClockUpdate();
	usart_init();
	set_husart_buffer(&husart2);
	DMA_init();
}
