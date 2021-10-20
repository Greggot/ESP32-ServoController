#include "esp_event.h"

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "esp_gap_ble_api.h"

#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"

#include <vector>
#include <map>

#ifndef INC_BLUETOOTH_HPP
#define INC_BLUETOOTH_HPP

#define BLE_SERVICES_PRINTF
#define BLE_CHARACTERISTICS_PRINTF
//#define BLE_INPUT_PRINTF

typedef __uint8_t byte;

class Characteristic
{
    private:
        typedef void BLEcallbackType(Characteristic*, esp_ble_gatts_cb_param_t*);

        esp_bt_uuid_t UUID;
        esp_attr_value_t  Char_Data;

        esp_gatt_perm_t Permition;
        esp_gatt_char_prop_t Property;

        esp_attr_control_t Control;

        uint16_t Handler;
        esp_gatt_if_t GATTinterface;

        byte* Data;
        size_t DataSize;
        
        static void DefaultReadCallback(Characteristic*, esp_ble_gatts_cb_param_t*);
        BLEcallbackType* ReadHandler = &DefaultReadCallback;
        static void DefaultWriteCallback(Characteristic*, esp_ble_gatts_cb_param_t*);
        BLEcallbackType* WriteHandler = &DefaultWriteCallback;

        void UUID_Init(uint32_t _UUID);
        bool sendMutex = false;
    public:
        void ConsoleInfoOut(); 

        /* SETters */
        void setReadhandler(BLEcallbackType* callback) { this->ReadHandler = callback; }
        void setWritehandler(BLEcallbackType* callback) { this->WriteHandler = callback; }
        void setHandler(uint16_t Handler) { this->Handler = Handler; }
        void setGATTinterface(esp_gatt_if_t GATTinterface) { this->GATTinterface = GATTinterface; }
        void setPermition(esp_gatt_perm_t Permition) { this->Permition = Permition; }
        void setProperty(esp_gatt_char_prop_t Property) {this->Property = Property; }
        void setData(const byte* Data, size_t DataSize);

        /* GETters */
        esp_gatt_if_t getGATTinterface() { return this->GATTinterface; }
        uint16_t getHandler() { return this->Handler; }
        byte* getData() { return this->Data; }
        size_t getDataSize() { return this->DataSize; }

        /* CALLers */
        void callReadHandler(esp_ble_gatts_cb_param_t *param);
        void callWriteHandler(esp_ble_gatts_cb_param_t *param);

        esp_err_t AttachToService(uint16_t ServiceHandler);

        void Notify(byte* Data, size_t Data_Length, uint16_t connected_device_id = 0);

        Characteristic();
        Characteristic(uint32_t _UUID);
        Characteristic(uint8_t _UUID[ESP_UUID_LEN_128]);

        Characteristic(uint32_t _UUID, esp_gatt_perm_t, esp_gatt_char_prop_t);
        Characteristic(uint8_t _UUID[ESP_UUID_LEN_128], esp_gatt_perm_t, esp_gatt_char_prop_t);
};

class Service
{
    private:
        esp_bt_uuid_t UUID;
        void UUID_Init(uint32_t _UUID);
    
        esp_gatt_if_t GATTinterface;
        esp_gatt_srvc_id_t service_id;
        uint16_t num_handle;
        uint16_t Handler;
        std::vector<Characteristic*> Characteristics;
        //Characteristic** Characteristics;
        size_t CharacteristicsSize;
    public:
        //std::map <uint16_t, Characteristic*> Chars;
        uint16_t CharCounter = 0;

        void ConsoleInfoOut();
        /* SETters */
        void setHandler(uint16_t Handler) { this->Handler = Handler; }
        void setGATTinterface(esp_gatt_if_t GATTinterface);
        /* GETters */
        std::vector<Characteristic*> getCharacteristics() { return this->Characteristics; }
        size_t getCharacteristicsSize() { return this->CharacteristicsSize; }
        uint16_t getHandler() { return this->Handler; }

        Service();
        Service(uint32_t _UUID, std::vector<Characteristic*> Characteristics);
        //Service(uint32_t _UUID, Characteristic** Characteristics, size_t CharacteristicsSize);

        void Start();
        void Create();
        void CharacteristicInitialization();
};

class ServerDevice
{
    private:
        typedef void BLEcallbackType(ServerDevice*, esp_ble_gatts_cb_param_t*);

        esp_ble_adv_data_t AdvertisingData;
        esp_ble_adv_data_t ScanResponceData;
        esp_ble_adv_params_t AdvertisingParameters;
        
        esp_gatt_if_t GATT_Interface;

        std::vector<Service*> SERVICE;
            std::map <uint16_t, Characteristic*> AllCharacteristics;
            std::map <uint16_t, Service*> AllServices;

        std::vector<int> AmountOfChars;
        uint16_t ServiceInitializeCounter = 0;
        uint16_t CharacteristicInitializeCount = 0;
        uint16_t CharacteristicInitializeCountMax;
        const char* Name;

        void DeviceCallback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
        BLEcallbackType* GATTeventsCallbacks[24];
        std::vector<uint32_t>callbackIndexes;
    public:
        ServerDevice();
        ServerDevice(const char* Name);
        ServerDevice(const char* Name, esp_ble_adv_data_t AdvertisingData, esp_ble_adv_data_t ScanResponceData, 
                    esp_ble_adv_params_t AdvertisingParameters, std::vector<Service*> SERVICE);

        /*      GETters     */
        std::vector<Service*> getServices() { return this->SERVICE; }
        esp_gatt_if_t getGATTinterface() { return GATT_Interface; }

        /*      SETters     */
        void setAdvertisingData(esp_ble_adv_data_t, esp_ble_adv_data_t, esp_ble_adv_params_t);
        void setDeviceGATTeventCallback(esp_gatts_cb_event_t Event, BLEcallbackType* Callback);

        void HandleEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) 
            { DeviceCallback(event, gatts_if, param); }

        void Start(esp_gatt_if_t GATT_Interface);
        void AddService(Service* service) { SERVICE.push_back(service); }
};

#endif