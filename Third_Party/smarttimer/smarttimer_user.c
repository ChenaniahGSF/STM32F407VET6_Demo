/*
 * smarttimer_user.c
 *
 *  Created on: Oct 12, 2025
 *      Author: Diamo
 */
#include <stdlib.h>
#include "smarttimer_user.h"
#include "logger.h"
#include "rtc.h"
#include "can.h"

struct date m_date;
const char datestr[] = "2025:10:12#13:02:00";

static uint8_t isLeap(uint16_t year)
{
  if(year % 4 == 0 &&
      year %100 != 0 &&
      year % 400 == 0){
    return 1;
  }else{
    return 0;
  }
}

static void increaseday(uint8_t limit)
{
  if(++m_date.day > limit){
    m_date.day = 1;
    if(++m_date.month > 12){
      m_date.month = 1;
      m_date.year++;
    }
  }
}

static void changeday ( void )
{
  switch(m_date.month){
    case 1:
    case 3:
    case 5:
    case 7:
    case 8:
    case 10:
    case 12:
      increaseday(31);
      break;
    case 4:
    case 6:
    case 9:
    case 11:
      increaseday(30);
      break;
    case 2:
      if(isLeap(m_date.year) == 1){
        increaseday(29);
      }else{
        increaseday(28);
      }
      break;
  }
}

static void simulation_rtc(void)
{
  if(++m_date.second == 60){
    m_date.second = 0;
    if(++m_date.minute == 60){
      m_date.minute = 0;
      if(++m_date.hour == 24){
        m_date.hour = 0;
        changeday();
      }
    }
  }
  logger_info("time------>[%04d-%02d-%02d %02d:%02d:%02d]",m_date.year, m_date.month, m_date.day, m_date.hour, m_date.minute, m_date.second);
}

void set_timetick (void)
{
    //xxxx:xx:xx#xx:xx:xx
    m_date.year = atoi(datestr);
    m_date.month = atoi(datestr + 5);
    m_date.day = atoi(datestr + 8);
    m_date.hour = atoi(datestr + 11);
    m_date.minute = atoi(datestr + 14);
    m_date.second = atoi(datestr + 17);

    stim_loop(1000, simulation_rtc, STIM_LOOP_FOREVER);
}

void print_data_time(void) {
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  logger_info("Date: 20%02d-%02d-%02d Weekday: %d",
         sDate.Year, sDate.Month, sDate.Date, sDate.WeekDay);
  logger_info("Time: %02d:%02d:%02d",
         sTime.Hours, sTime.Minutes, sTime.Seconds);
}

uint32_t can_id = 0x113;
uint8_t can_tx_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

void send_can_message(void) {
  if(Can_Send_Message(&hcan1, can_id, can_tx_data, sizeof(can_tx_data)) != 0) {
    logger_error("can send failed");
  }
}

void set_can_message(void) {
  stim_loop(1000, send_can_message, STIM_LOOP_FOREVER);
}

