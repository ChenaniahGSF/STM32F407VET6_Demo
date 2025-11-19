////////////////////////////////////////////////////////////////////////
//                  ___ ___  ___ _____ ___      ___                   //
//                 |_ _/ __|/ _ \_   _| _ \___ / __|                  //
//                  | |\__ \ (_) || | |  _/___| (__                   //
//                 |___|___/\___/ |_| |_|      \___|                  //
//                                                                    //
////////////////////////////////////////////////////////////////////////

#ifndef ISOTPC_USER_H
#define ISOTPC_USER_H

#include <stdint.h>
#include "logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief user implemented, print debug message */
//void isotp_user_debug(const char* message, ...);
#define isotp_user_debug(...) logger_debug(__VA_ARGS__)

/**
 * @brief user implemented, send can message. should return ISOTP_RET_OK when success.
 *
 * @return may return ISOTP_RET_NOSPACE if the CAN transfer should be retried later
 * or ISOTP_RET_ERROR if transmission couldn't be completed
 */
int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t* data, const uint8_t size
#ifdef ISO_TP_USER_SEND_CAN_ARG
                        , void* arg
#endif
);

/**
 * @brief user implemented, gets the amount of time passed since the last call in microseconds
 */
uint32_t isotp_user_get_us(void);

void isotp_init(void);
void isotp_polling(void);
int isotp_receiving(uint8_t* payload, const uint32_t payload_size, uint32_t* out_size);
int isotp_sending(const uint8_t payload[], uint32_t size);
void isotp_on_can_rx_message(const uint8_t* data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // ISOTPC_USER_H
