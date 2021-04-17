#include <stdint.h>
#include "esp_err.h"
#include "esp_http_server.h"

#ifndef CFM_INC
#define CFM_INC

typedef struct{
    float appliedField;
    int32_t sensorReading;
    float inclination;
} CalPoint;

typedef struct{
    char* key;
    char* value;
    unsigned skip;
} SettingsItem;

esp_err_t CFM_init();
SettingsItem* CFM_getSetting(char* key);
float CFM_scaleMeasurement(int32_t reading);
char* CFM_parseJSON(char* data, char* propertyToFind);
esp_err_t CFM_processNewCalData(httpd_req_t *req);
esp_err_t CFM_processNewSettingsData(httpd_req_t *req);
void CFM_loadAllSettings(char* data);

#endif