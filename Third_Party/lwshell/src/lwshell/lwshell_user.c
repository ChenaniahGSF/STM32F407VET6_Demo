/*
 * lwshell_user.c
 *
 *  Created on: Sep 29, 2025
 *      Author: Diamo
 */
#include "lwshell/lwshell.h"
#include "lwprintf/lwprintf.h"
#include "usart.h"

void lwshell_user_output(const char* str, struct lwshell* lwobj) {
  (void)lwobj;
  //lwprintf("%s", str);
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

static int32_t hello_cmd(int32_t argc, char** argv) {
  lwshell_user_output("Hello, LwSHELL!\r\n", NULL);
  for (int32_t i = 0; i < argc; i++) {
    lwshell_user_output("Arg ", NULL);
    lwshell_user_output(argv[i], NULL);
    lwshell_user_output("\r\n", NULL);
  }
  return 0;
}

void lwshell_user_init(void) {
  lwshell_init();
  lwshell_set_output_fn(lwshell_user_output);
  lwshell_register_cmd("hello", hello_cmd, "Print hello message with args");
}

