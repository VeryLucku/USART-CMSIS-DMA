#include "stm32f4xx.h"
#include "usart2.h"

void usart_init();
void DMA_init();
void set_husart_buffer(struct USART_buffer *buffer);