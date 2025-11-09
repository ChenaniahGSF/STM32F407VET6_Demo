/*
 * logger.c
 *
 *  Created on: Oct 4, 2025
 *      Author: Diamo
 */
#include "logger.h"

#ifdef LOG_ENABLE

#ifdef LWPRINTF_ENABLE

#define UART_TX_BUFFER_SIZE (64)

struct uart_tx_buffer {
    uint8_t buffer[UART_TX_BUFFER_SIZE];
    int size;
};
struct uart_tx_buffer uart_buffer;

int lwprintf_self_func(int ch, lwprintf_t* p) {
    uint8_t c = (uint8_t)ch;

    /* Don't print zero */
    if (c == '\0') {
        HAL_UART_Transmit(&huart1, uart_buffer.buffer, uart_buffer.size, 100);
        uart_buffer.size = 0;
        return ch;
    }
    if(uart_buffer.size >= (UART_TX_BUFFER_SIZE - 1)) {
        uart_buffer.buffer[uart_buffer.size++] = ch;
        HAL_UART_Transmit(&huart1, uart_buffer.buffer, uart_buffer.size, 100);
        uart_buffer.size = 0;
    } else {
      uart_buffer.buffer[uart_buffer.size++] = ch;
    }

    return ch;
}

void logger_init(void) {
  lwprintf_init(lwprintf_self_func);
}

void logger_hex(uint8_t* in, size_t ilen) {
  size_t i;
  for(i=0; i<ilen; i++) {
    lwprintf("%02x", in[i]);
    if((i+1)%16 == 0) {
      lwprintf("\r\n");
    }
  }
  if(i%16 != 0) {
    lwprintf("\r\n");
  }
}

#else
int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart1, (const uint8_t*)ptr, len, 100);
  return len;
}

void logger_init(void) {
  //do nothing
};

void logger_hex(uint8_t* in, size_t ilen) {
  size_t i;
  for(i=0; i<ilen; i++) {
    printf("%02x", in[i]);
    if((i+1)%16 == 0) {
      printf("\r\n");
    }
  }
  if(i%16 != 0) {
    printf("\r\n");
  }
}
#endif

#else
void logger_init(void) {
  //do nothing
};

void logger_hex(uint8_t* in, size_t ilen) {
  (void)in;
  (void)ilen;
  //do nothing
}
#endif

