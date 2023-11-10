#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

void write_data(char *result);

char* get_data();

void usart2_init();

void set_packet_parcer(void (*parcer)(char *, uint32_t));

void UART2_Packet_Recieved_Callback();
