/*
 * logger.h
 *
 *  Created on: Oct 4, 2025
 *      Author: Diamo
 */

#ifndef LOGGER_INC_LOGGER_H_
#define LOGGER_INC_LOGGER_H_

#define LOG_ENABLE
#define LWPRINTF_ENABLE

#ifdef LOG_ENABLE

#include "usart.h"
#ifdef LWPRINTF_ENABLE
#include "lwprintf/lwprintf.h"
#else
#include <stdio.h>
#endif

#define LOG_TAG  ("STM32F407")
#ifdef LWPRINTF_ENABLE
#define logger_debug(format, ...) {lwprintf("[%s][%s][%d]:"format, LOG_TAG, __func__, __LINE__, ##__VA_ARGS__);lwprintf("\r\n");}
#define logger_info(format, ...)  {lwprintf("[%s][%s][%d]:"format, LOG_TAG, __func__, __LINE__, ##__VA_ARGS__);lwprintf("\r\n");}
#define logger_warn(format, ...)  {lwprintf("[%s][%s][%d]:"format, LOG_TAG, __func__, __LINE__, ##__VA_ARGS__);lwprintf("\r\n");}
#define logger_error(format, ...) {lwprintf("[%s][%s][%d]:"format, LOG_TAG, __func__, __LINE__, ##__VA_ARGS__);lwprintf("\r\n");}

#define logger_printf  lwprintf
#else
#define logger_debug(format, ...) {printf("[%s][%s][%d]:"format, LOG_TAG, __func__, __LINE__, ##__VA_ARGS__);printf("\r\n");}
#define logger_info(format, ...)  {printf("[%s][%s][%d]:"format, LOG_TAG, __func__, __LINE__, ##__VA_ARGS__);printf("\r\n");}
#define logger_warn(format, ...)  {printf("[%s][%s][%d]:"format, LOG_TAG, __func__, __LINE__, ##__VA_ARGS__);printf("\r\n");}
#define logger_error(format, ...) {printf("[%s][%s][%d]:"format, LOG_TAG, __func__, __LINE__, ##__VA_ARGS__);printf("\r\n");}

#define logger_printf  printf
#endif

#else
#include <stdint.h>
#include <stddef.h>

#define logger_debug(format, ...)
#define logger_info(format, ...)
#define logger_warn(format, ...)
#define logger_error(format, ...)
#endif

void logger_init(void);
void logger_hex(uint8_t* in, size_t ilen);



#endif /* LOGGER_INC_LOGGER_H_ */
