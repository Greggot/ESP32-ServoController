#include <build.h>   ///PWM
#include <Bluetooth_GAP.hpp>
#include <Bluetooth.hpp>

ServerDevice BLE_Kit;
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    BLE_Kit.HandleEvent(event, gatts_if, param);
}

void LEDThread(void* pvParameter)
{
    gpio_pad_select_gpio(GPIO_NUM_25);
    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
    while(true)
    {
        gpio_set_level(GPIO_NUM_25, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(GPIO_NUM_25, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static uint8_t ServoModeOmega;
void Button180DegreeThreadOmega(void)
{
    ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
        ledc_timer.timer_num        = LEDC_TIMER_0;
        ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT;
        ledc_timer.freq_hz          = 50;  // Set output frequency at 50 Hz
        ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel;
        ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
        ledc_channel.channel        = LEDC_CHANNEL_0;
        ledc_channel.timer_sel      = LEDC_TIMER_0;
        ledc_channel.intr_type      = LEDC_INTR_DISABLE;
        ledc_channel.gpio_num       = GPIO_NUM_18;
        ledc_channel.duty           = 78; // Set 90 degree
        ledc_channel.hpoint         = 0;
    ledc_channel_config(&ledc_channel);
}

static uint8_t ServoModeTheta;
void Button180DegreeThreadTheta(void)
{
    ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
        ledc_timer.timer_num        = LEDC_TIMER_1;
        ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT;
        ledc_timer.freq_hz          = 50;  // Set output frequency at 50 Hz
        ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel;
        ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
        ledc_channel.channel        = LEDC_CHANNEL_1;
        ledc_channel.timer_sel      = LEDC_TIMER_1;
        ledc_channel.intr_type      = LEDC_INTR_DISABLE;
        ledc_channel.gpio_num       = GPIO_NUM_19;
        ledc_channel.duty           = 78; // Set 90 degree
        ledc_channel.hpoint         = 0;
    ledc_channel_config(&ledc_channel);
}

static Characteristic OmegaAngleChar = Characteristic(0xA0111, ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE);
static Characteristic ThetaAngleChar = Characteristic(0xA111E, ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE);

static byte OmegaAngle;
static byte ThetaAngle;
void OmegaAngleCallback(Characteristic* ch, esp_ble_gatts_cb_param_t* param)
{
    OmegaAngle = param->write.value[0];
    if(OmegaAngle > 180)
        OmegaAngle -= 180;
    
    OmegaAngle /= 2;
    OmegaAngle += 30;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, OmegaAngle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}
void ThetaAngleCallback(Characteristic* ch, esp_ble_gatts_cb_param_t* param)
{
    ThetaAngle = param->write.value[0];
    if(ThetaAngle > 180)
        ThetaAngle -= 180;
    
    ThetaAngle /= 2;
    ThetaAngle += 30;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ThetaAngle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void UARTreadThread(void* pvParamter)
{
    byte *data = new byte[20];
    int len = 0;
    while(true)
    {
        len = uart_read_bytes(UART_NUM_1, data, 20, 5 / portTICK_RATE_MS);
        if(len > 2)
        {
            if(data[2])
            {
                ThetaAngle = data[1];
                if(ThetaAngle > 180)
                    ThetaAngle -= 180;
                
                ThetaAngle /= 2;
                ThetaAngle += 30;

                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ThetaAngle);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            }
            else
            {
                OmegaAngle = data[1];
                if(OmegaAngle > 180)
                    OmegaAngle -= 180;
                
                OmegaAngle /= 2;
                OmegaAngle += 30;

                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, OmegaAngle);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void app_main(void)
{
    nvs_flash_init();

    uart_config_t uart_config = {
        .baud_rate = 56000,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 25, 26, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(LEDThread, "LEDThread", 1024, NULL, 0, NULL);
    xTaskCreate(UARTreadThread, "UARTreadThread", 2048, NULL, 0, NULL);
    Button180DegreeThreadOmega();
    Button180DegreeThreadTheta();
    
    std::vector <Characteristic*> OmegaChars {&OmegaAngleChar};
    OmegaAngleChar.setWritehandler(OmegaAngleCallback);
    Service* OmegaService = new Service(0xC0111E7A, OmegaChars);

    std::vector <Characteristic*> ThetaChars {&ThetaAngleChar};
    ThetaAngleChar.setWritehandler(ThetaAngleCallback);
    Service* ThetaService = new Service(0xC111E1A, ThetaChars);

    
    std::vector <Service*> Services {OmegaService, ThetaService};
    BLE_Kit = ServerDevice("BTservoController", adv_data, scan_rsp_data, adv_params, Services);
    
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(0);
}
