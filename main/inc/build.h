#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"


#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "esp_log.h"

#include "nvs_flash.h"
#include "sdkconfig.h"

#define MountButtonPIN GPIO_NUM_0
#define LogButtonPIN GPIO_NUM_19
#define CardDetectPIN GPIO_NUM_13
#define CardDetectLED GPIO_NUM_25
#define LogLED GPIO_NUM_18
///          FUNCTION Definitions           ///
extern "C" { void app_main(void); }

///     CAN module     ///
/*void CAN_Init(twai_timing_config_t baudrate);
void CAN_RX_Callback(twai_message_t message);   /// User defined callback
void CAN_send(uint32_t ID, uint8_t* CAN_Data, uint16_t msg_size);
void CAN_Pause();
void CAN_Play();
bool isCANenabled();*/