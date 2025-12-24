#include <iostream>
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_softap.h"
#include "nvs_flash.h"
#include "qrcode.h"
#include "freertos/task.h"
#include "cstring"
#include "json.hpp"
#define MAX_RETRY_NUM 5
static const char* TAG = "main";
void print_qr(void)
{
    nlohmann::json qr_string = 
    {
        {"ver","v1"},
        {"name","PROV_ESP"},
        {"pop","abcd1234"},
        {"transport","softap"}
    };
    esp_qrcode_config_t qr = ESP_QRCODE_CONFIG_DEFAULT();
    esp_qrcode_generate(&qr,qr_string.dump().c_str());
}
static void wifi_prov_handler(void *user_data, wifi_prov_cb_event_t event, void *event_data)
{
    switch(event)
    {
        case WIFI_PROV_START :
        ESP_LOGI(TAG,"[WIFI_PROV_START]");
        break;
        case WIFI_PROV_CRED_RECV :
        ESP_LOGI(TAG ,"cred : ssid : %s pass : %s",((wifi_sta_config_t*)event_data)->ssid,((wifi_sta_config_t*)event_data)->password);
        break;
        case WIFI_PROV_CRED_SUCCESS :
        ESP_LOGI(TAG,"prov success");
        wifi_prov_mgr_stop_provisioning();
        break;
        case WIFI_PROV_CRED_FAIL :
        ESP_LOGE(TAG,"credentials worng");
        wifi_prov_mgr_reset_sm_state_on_failure();
        print_qr();
        break;
        case WIFI_PROV_END: 
        ESP_LOGI(TAG,"prov ended");
        wifi_prov_mgr_deinit();
        break;
        default : break;
    }
}
static void wifi_event_handler(void* event_handler_arg,
                                        esp_event_base_t event_base,
                                        int32_t event_id,
                                        void* event_data)
{
    static int retry_cnt =0 ;
    if(event_base == WIFI_EVENT)
    {
        switch(event_id)
        {
            case WIFI_EVENT_STA_START :
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED :
                ESP_LOGE(TAG,"ESP32 Disconnected, retrying");
                retry_cnt ++;
                if(retry_cnt < MAX_RETRY_NUM)
                {
                    esp_wifi_connect();
                }
                else ESP_LOGE(TAG,"Connection error");
                break;
            default : break;
        }
    }
    else if(event_base == IP_EVENT)
    {
        if(event_id == IP_EVENT_STA_GOT_IP)
        {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            ESP_LOGI(TAG,"station ip :"IPSTR,IP2STR(&event->ip_info.ip));
        }
    }
}
void wifi_hw_init(void)
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&wifi_event_handler,NULL,&instance_any_id);
    esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&wifi_event_handler,NULL,&instance_got_ip);
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_start();
 

}
static void prov_start(void)
{
    wifi_prov_mgr_config_t cfg = 
    {
        .scheme =wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
        .app_event_handler = wifi_prov_handler,
        
      
    };
    wifi_prov_mgr_init(cfg);
    bool is_provisioned = 0;
    wifi_prov_mgr_is_provisioned(&is_provisioned);
    wifi_prov_mgr_disable_auto_stop(100);
    if(is_provisioned)
    {
        ESP_LOGI(TAG,"Already provisioned");
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
    }
    else
    {
        wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_1,"abcd1234","PROV_ESP",NULL);
        print_qr();

    }

}
extern "C" void app_main(void)
{
    wifi_hw_init();
    prov_start();
}