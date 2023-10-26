#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define RX_DMA_S DMA1_Stream5
#define TX_DMA_S DMA1_Stream6

#define RX_DMA_CH 4
#define TX_DMA_CH 4

#define BUFFER_SIZE 128

struct USART_buffer {
	uint8_t tx_buffer[BUFFER_SIZE];
	uint8_t rx_buffer[BUFFER_SIZE];
	uint8_t proccess_buffer[BUFFER_SIZE];
	int16_t rx_counter;
  	int16_t rx_process_counter;
	int16_t tx_counter;
	int tx_len;
};

void write_data(char *result);

char* get_data();

void usart2_init();
