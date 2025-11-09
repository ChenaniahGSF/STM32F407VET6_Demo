/*
 * lwshell_user.c
 *
 *  Created on: Sep 29, 2025
 *      Author: Diamo
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "getopt_long.h"
#include "lwshell/lwshell.h"
#include "lwprintf/lwprintf.h"
#include "logger.h"
#include "rtc.h"

void lwshell_user_output(const char* str, struct lwshell* lwobj) {
  (void)lwobj;
  //HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
  logger_printf("%s", str);
}

////////////////////hello_cmd//////////////////////////////////////////
static int32_t hello_cmd(int32_t argc, char** argv) {
  logger_printf("Hello, LwSHELL!\r\n");
  for (int32_t i = 0; i < argc; i++) {
	  logger_printf("Arg ");
	  logger_printf(argv[i]);
	  logger_printf("\r\n");
  }
  return 0;
}

////////////////////rtc_date_command//////////////////////////////////////////
// Command line options definition
static struct option date_command_long_options[] = {
    {"set",    required_argument, 0, 's'},  // Set time
    {"show",   no_argument,       0, 'S'},  // Show time
    {"help",   no_argument,       0, 'h'},  // Help information
    {0, 0, 0, 0}
};

// Display help information
static void date_command_print_help(void) {
	logger_printf("Usage: date [options] [time string]\r\n");
	logger_printf("Control RTC real-time clock date and time\r\n");
	logger_printf("Options:\r\n");
	logger_printf("  -s, --set \"YYYY-MM-DD HH:MM:SS\"  Set RTC time and date\r\n");
	logger_printf("  -S, --show                       Display current RTC time and date\r\n");
	logger_printf("  -h, --help                       Display this help information\r\n");
	logger_printf("Examples:\r\n");
	logger_printf("  date -s \"2024-10-15 14:30:00\"  Set time to October 15, 2024 14:30:00\r\n");
	logger_printf("  date -S                          Display current RTC time\r\n");
}

// Display current RTC time
static void show_rtc_time(void) {
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    const char *weekday_str[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

    // Read RTC time and date (must read time first, then date)
    if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
    	logger_error("Failed to read RTC time");
        return;
    }

    if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
    	logger_error("Failed to read RTC date");
        return;
    }

    // Format output
    logger_printf("Current RTC time: %04d-%02d-%02d %s %02d:%02d:%02d\r\n",
           2000 + sDate.Year,
           sDate.Month,
           sDate.Date,
           weekday_str[sDate.WeekDay - 1],  // RTC weekday starts from 1
           sTime.Hours,
           sTime.Minutes,
           sTime.Seconds);
}

// Calculate weekday (1-7, 1=Sunday, 7=Saturday)
int get_weekday(int year, int month, int day) {
    int q, m, K, J, h;

    if (month < 3) {
        month += 12;
        year--;
    }

    q = day;
    m = month;
    K = year % 100;
    J = year / 100;

    h = (q + (13 * (m + 1)) / 5 + K + K / 4 + J / 4 + 5 * J) % 7;

    // Convert to RTC format (1=Sunday, 2=Monday, ..., 7=Saturday)
    return h == 0 ? 7 : h;
}

// Parse time string and set RTC
static int set_rtc_time(const char *time_str) {
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    int year, month, day, hour, minute, second;

    // Parse format: YYYY-MM-DD HH:MM:SS
    if (sscanf(time_str, "%d-%d-%d %d:%d:%d",
              &year, &month, &day, &hour, &minute, &second) != 6) {
    	logger_error("Time format error, please use \"YYYY-MM-DD HH:MM:SS\"");
        return -1;
    }

    // Validate time values
    if (year < 2000 || year > 2100 ||
        month < 1 || month > 12 ||
        day < 1 || day > 31 ||
        hour < 0 || hour >= 24 ||
        minute < 0 || minute >= 60 ||
        second < 0 || second >= 60) {
        logger_error("Invalid time values");
        return -1;
    }

    // Set date
    sDate.Year = year - 2000;  // RTC year is offset from 2000
    sDate.Month = month;
    sDate.Date = day;
    sDate.WeekDay = get_weekday(year, month, day);  // Call weekday calculation function

    // Set time
    sTime.Hours = hour;
    sTime.Minutes = minute;
    sTime.Seconds = second;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    // Set time first, then date (HAL library requirement)
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        logger_error("Failed to set RTC time");
        return -1;
    }

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        logger_error("Failed to set RTC date");
        return -1;
    }

    logger_printf("RTC time set to: %04d-%02d-%02d %02d:%02d:%02d\r\n",
           year, month, day, hour, minute, second);
    return 0;
}

// Main function: parse command line arguments and execute operations
int32_t rtc_date_command(int32_t argc, char *argv[]) {
    int opt;
    int option_index = 0;
    int show_time = 0;
    char *set_time_str = NULL;

    // Default to showing time when no arguments
    if (argc == 1) {
        show_rtc_time();
        return 0;
    }

    // Parse command line options
    while ((opt = getopt_long(argc, argv, "s:Sh", date_command_long_options, &option_index)) != -1) {
        switch (opt) {
            case 's':
                set_time_str = optarg;
                break;
            case 'S':
                show_time = 1;
                break;
            case 'h':
            	date_command_print_help();
                return 0;
            default:
            	date_command_print_help();
                return -1;
        }
    }
    getopt_reset();

    // Execute corresponding operations
    if (set_time_str) {
        return set_rtc_time(set_time_str);
    }

    if (show_time) {
        show_rtc_time();
    }

    return 0;
}

void lwshell_user_init(void) {
  lwshell_init();
  lwshell_set_output_fn(lwshell_user_output);
  lwshell_register_cmd("hello", hello_cmd, "Print hello message with args");
  lwshell_register_cmd("date", rtc_date_command, "date command");
}

