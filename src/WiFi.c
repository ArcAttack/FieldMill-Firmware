#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "FieldMill.h"
#include "ConfigManager.h"

static const char *TAG = "WiFi";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static esp_err_t WIFI_connectAP();
static esp_err_t WIFI_initSoftAP();
unsigned WIFI_inited = 0;

esp_err_t WIFI_init(){

    ESP_LOGI(TAG, "WIFI is starting");

    //Load configuration
    SettingsItem* clientEnabled = CFM_getSetting("WIFI_clientEnabled");
    unsigned mode = 0;
    if(clientEnabled != 0){
        mode = memcmp(clientEnabled->value, "true", strlen("true")) == 0;
        ESP_LOGI(TAG, "WIFI_clientEnabled is \"%s\"", clientEnabled->value);
    }

    /*if(WIFI_inited){
        if(mode){
            wifi_config_t wifi_config = { .sta = { .threshold.authmode = WIFI_AUTH_WPA2_PSK, .pmf_cfg = { .capable = true, .required = false }, }, };
            SettingsItem * ssid = CFM_getSetting("WIFI_ssid");
            if(ssid == 0) return ESP_FAIL;
            strlcpy((char*) wifi_config.sta.ssid, ssid->value, 32);
            SettingsItem * pass = CFM_getSetting("WIFI_password");
            if(pass == 0) return ESP_FAIL;
            strlcpy((char*) wifi_config.sta.password, pass->value, 32);
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
        }else{
            wifi_config_t wifi_config = { .ap = { .ssid = "Field Mill", .ssid_len = strlen("Field Mill"), .channel = 5, .max_connection = 4, .authmode = WIFI_AUTH_OPEN }, };
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
        }
        return ESP_OK;
    }*/

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if(mode){   //client is enabled
        ESP_LOGI(TAG, "attempting to connect to ap");
        if(WIFI_connectAP() == ESP_OK){
            WIFI_inited = 1;
            return ESP_OK;
        }
        ESP_LOGI(TAG, "failed! creating AP");
    }

    WIFI_inited = 1;
    return WIFI_initSoftAP();
}

static void event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

static esp_err_t WIFI_connectAP(){
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = { .sta = { .threshold.authmode = WIFI_AUTH_WPA2_PSK, .pmf_cfg = { .capable = true, .required = false }, }, };

    SettingsItem * ssid = CFM_getSetting("WIFI_ssid");
    if(ssid == 0) return ESP_FAIL;
    strlcpy((char*) wifi_config.sta.ssid, ssid->value, 32);
    SettingsItem * pass = CFM_getSetting("WIFI_password");
    if(pass == 0) return ESP_FAIL;
    strlcpy((char*) wifi_config.sta.password, pass->value, 32);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wifi connected to %s", wifi_config.sta.ssid);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to %s", wifi_config.sta.ssid);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);

    return (bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_FAIL;
}

static esp_err_t WIFI_initSoftAP(){
    esp_netif_create_default_wifi_ap();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "Field Mill",
            .ssid_len = strlen("Field Mill"),
            .channel = 5,
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "created softAP \"Field Mill\"");
    return ESP_OK;
}