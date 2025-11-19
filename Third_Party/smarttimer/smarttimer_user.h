/*
 * smarttimer_user.h
 *
 *  Created on: Oct 12, 2025
 *      Author: Diamo
 */

#ifndef SMARTTIMER_SMARTTIMER_USER_H_
#define SMARTTIMER_SMARTTIMER_USER_H_
#include "smarttimer.h"

struct date{
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

void set_timetick (void);
void print_data_time(void);

#endif /* SMARTTIMER_SMARTTIMER_USER_H_ */
