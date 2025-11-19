/*
 * iostp_user.c
 *
 *  Created on: Nov 18, 2025
 *      Author: Diamo
 */
#include "can.h"
#include "isotp.h"
#include "isotp_user.h"

static IsoTpLink g_phylink;

static uint8_t g_isotpPhyRecvBuf[4095];
static uint8_t g_isotpPhySendBuf[4095];

int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t* data, const uint8_t size
#ifdef ISO_TP_USER_SEND_CAN_ARG
                        , void* arg
#endif
)
{
  return Can_Send_Message(&hcan1, arbitration_id, data, size);
}

uint32_t isotp_user_get_us(void)
{
  return HAL_GetTick();
}

void isotp_init(void)
{
  /* Initialize link, 0x7E8 is the CAN ID send with */
  isotp_init_link(&g_phylink, 0x7E8, g_isotpPhySendBuf, sizeof(g_isotpPhySendBuf), g_isotpPhyRecvBuf, sizeof(g_isotpPhyRecvBuf));
}

void isotp_polling(void)
{
  isotp_poll(&g_phylink);
}

int isotp_receiving(uint8_t* payload, const uint32_t payload_size, uint32_t* out_size)
{
  return isotp_receive(&g_phylink, payload, payload_size, out_size);
}

int isotp_sending(const uint8_t payload[], uint32_t size)
{
  return isotp_send(&g_phylink, payload, size);
}

void isotp_on_can_rx_message(const uint8_t* data, uint8_t len)
{
  isotp_on_can_message(&g_phylink, data, len);
}
