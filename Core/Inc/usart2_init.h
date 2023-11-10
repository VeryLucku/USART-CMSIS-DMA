#include "stm32f4xx.h"
#include "usart2.h"

#define RX_DMA_S DMA1_Stream5
#define TX_DMA_S DMA1_Stream6

#define RX_DMA_CH 4
#define TX_DMA_CH 4

#define BUFFER_SIZE 4

#define START_TR_CHAR 'S'
#define END_TR_CHAR 'E'

#define DEF_PACKET_PARCER

struct USART_buffer {
	uint8_t tx_buffer[BUFFER_SIZE];
	uint8_t rx_buffer[BUFFER_SIZE];
	uint8_t rx_process_buffer[BUFFER_SIZE];
	int16_t rx_counter;
  	int16_t rx_process_counter;
	int16_t tx_counter;
	int tx_len;
};

extern struct USART_buffer husart2;

void usart_init();
void DMA_init();