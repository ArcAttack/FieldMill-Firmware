#include <stdint.h>

#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "esp_err.h"
#include "ConfigManager.h"
#include "FieldMill.h"
#include "esp_http_server.h"
#include "WiFi.h"

static const char *TAG = "config_manager";

CalPoint * calData = NULL;
uint32_t calDataCount = 0;

SettingsItem * settings = NULL;
uint32_t settingsCount = 0;

SettingsItem* CFM_getSetting(char* key){
    for (uint32_t i = 0; i < settingsCount; i++){
        if(settings[i].key == 0) continue;
        if(strcmp(settings[i].key, key) == 0 && strlen(key) == strlen(settings[i].key)) return &settings[i];
    }
    return 0;
}

float CFM_scaleMeasurement(int32_t reading){
    if(calDataCount < 2) return -1;

    if(reading < calData[0].sensorReading){
        int32_t base = reading - calData[0].sensorReading;
        float ret = calData[0].appliedField + (float) base * calData[0].inclination;
        //ESP_LOGI(TAG, "apply cal: %d => %f (i=%d SR=%d AF=%f incl=%f)", reading, ret, i, calData[i].sensorReading, calData[i].appliedField, calData[i].inclination);
        return ret;
    }

    uint32_t i = calDataCount-1;
    for (; i >= 0; i--){
        if(reading > calData[i].sensorReading){
            int32_t base = reading - calData[i].sensorReading;
            float ret = calData[i].appliedField + (float) base * calData[(i == calDataCount - 1) ? (i-1) : i].inclination;
            //ESP_LOGI(TAG, "apply cal: %d => %f (i=%d SR=%d AF=%f incl=%f)", reading, ret, i, calData[i].sensorReading, calData[i].appliedField, calData[i].inclination);
            return ret;
        }
    }

    return -1;
}

static void CFM_loadCal(char * data){
    if(data == 0) return;
    ESP_LOGI(TAG, "loading");

    char * values = CFM_parseJSON(data, "Datapoints");
    if(values == 0) return;

    if(calData != 0) free(calData);
    calData = malloc(sizeof(CalPoint) * 100);
    memset(calData, 0, sizeof(CalPoint) * 100);
    uint32_t currData = 0;

    char * start = strchr(values, '[') + 1;
    if(start == 0){
        free(values);
        return;
    } 

    while(1){
        start = strchr(start, '[');
        if(start == 0){
            free(values);
            return;
        } 
        start++;
        char * end = strchr(start, ']');
        char * sep = strchr(start, ',');

        if(sep > end || end == 0 || sep == 0){
            ESP_LOGI(TAG, "got weird data for calbration! %s", start);
            free(values);
            return;
        }

        uint32_t yLen = end - 1 - sep;
        char * yChar = malloc(yLen + 1);
        memcpy(yChar, sep + 1, yLen);
        yChar[yLen] = 0;

        uint32_t xLen = sep - start;
        char * xChar = malloc(xLen + 1);
        memcpy(xChar, start, xLen);
        xChar[xLen] = 0;

        calData[currData].appliedField = atof(yChar);
        calData[currData].sensorReading = (uint32_t) atof(xChar);

        if(currData > 0){
            int32_t dx = (calData[currData].sensorReading - calData[currData - 1].sensorReading);
            float dy = (calData[currData].appliedField - calData[currData - 1].appliedField);
            calData[currData-1].inclination = dy / (float) dx;
            ESP_LOGI(TAG, "Got new calibration data: %f %d incl %f (dX = %d, dY = %f)", calData[currData-1].appliedField, calData[currData-1].sensorReading, calData[currData-1].inclination, dx, dy);
        }
        currData++;

        free(xChar); free(yChar);

        if(strchr(end, ',') == 0) break;
    } 
    
    free(values);
    calDataCount = currData;
    ESP_LOGI(TAG, "loaded %d datapoints", calDataCount);
}

esp_err_t CFM_init(){
    ESP_LOGI(TAG, "Starting CFM");

    uint32_t settingsSize, calSize;

    FILE * settings = fopen("/spiffs/settings.json", "r");
    if(settings == NULL){
        ESP_LOGI(TAG, "settings.json could not be found!");
        return ESP_FAIL;
    }

    fseek(settings, 0, SEEK_END);
    settingsSize = ftell(settings);
    fseek(settings, 0, SEEK_SET);
    ESP_LOGI(TAG, "found settings.json with size %d", (uint32_t) settingsSize);

    if(settingsSize > 0){
        char * settingsString = malloc(settingsSize+1);
        memset(settingsString, 0, settingsSize + 1);
        fread(settingsString, 1, settingsSize, settings);
        CFM_loadAllSettings(settingsString);
        fclose(settings);
        free(settingsString);
    }

    FILE * cal = fopen("/spiffs/cal.json", "r");
    if(cal == NULL){
        ESP_LOGI(TAG, "cal.json could not be found!");
        return ESP_FAIL;
    }
    
    fseek(cal, 0, SEEK_END);
    calSize = ftell(cal);
    fseek(cal, 0, SEEK_SET);
    ESP_LOGI(TAG, "found cal.json with size %d", (uint32_t) calSize);

    if(calSize > 0){
        char * calString = malloc(calSize+1);
        memset(calString, 0, calSize + 1);
        fread(calString, 1, calSize, cal);
        ESP_LOGI(TAG, "%s", calString);
        CFM_loadCal(calString);
        free(calString);
        fclose(cal);
    }

    return ESP_OK;
}

char* CFM_parseJSON(char* data, char* propertyToFind){
    if(data == 0 || propertyToFind == 0) return 0;

    char * currPos = strchr(data, '{')+1;
    if(currPos == NULL) return 0;
    unsigned fileDone = 0;

    char * keyStart = currPos;

    while(!fileDone){
        char * keyEnd = strchr(keyStart, ':');
        if(currPos == NULL) break;
        char * propertyStart = keyEnd;
        while(*(++propertyStart) == ' ');
        char * propertyEnd = propertyStart;

        if(*propertyEnd == '{'){
            uint32_t currLevel = 1;
            while(currLevel){
                propertyEnd ++;
                if(*propertyEnd == 0){
                    fileDone = 1;
                    break;
                }else if(*propertyEnd == '{'){
                    currLevel ++;
                }else if(*propertyEnd == '}'){
                    currLevel --;
                }
            }
        }else if(*propertyEnd == '['){
            uint32_t currLevel = 1;
            while(currLevel){
                propertyEnd ++;
                if(*propertyEnd == 0){
                    fileDone = 1;
                    break;
                }else if(*propertyEnd == '['){
                    currLevel ++;
                }else if(*propertyEnd == ']'){
                    currLevel --;
                }
            }
        }else if(*propertyEnd == ','){
            //handle empty property
        }

        while(1){
            propertyEnd++;
            if(*propertyEnd == ',') break;
            if(*propertyEnd == '}' || *propertyEnd == 0){
                fileDone = 1;
                break;
            }
        }

        keyStart = strchr(keyStart, '"');
        if(keyStart == 0) return 0;
        int32_t keyLength = keyEnd - keyStart - 1;
        int32_t propLength = propertyEnd - propertyStart;
        if(keyLength > 0 && propLength > 0){
            char * key = malloc(keyLength);
            memcpy(key, keyStart + 1, keyLength - 1);
            key[keyLength-1] = 0;

            char * property = malloc(propLength + 1);
            memcpy(property, propertyStart, propLength);
            property[propLength-1] = 0;

            if(strcmp(key, propertyToFind) == 0){
                free(key);
                return property;
            }

            free(key); free(property);
        }
        keyStart = propertyEnd + 1;
    }

    return 0;
}

void CFM_loadAllSettings(char* data){
    if(data == 0) return;

    char * currPos = strchr(data, '{')+1;
    if(currPos == NULL) return;
    unsigned fileDone = 0;

    char * keyStart = currPos;

    if(settings != NULL){
        for(int32_t i = 0; i < settingsCount; i ++){
            free(settings[i].key);
            free(settings[i].value);
        }
        free(settings);
    } 
    settings = malloc(sizeof(SettingsItem) * 100);
    if(settings == 0) return;
    settingsCount = 0;
    memset(settings, 0, sizeof(SettingsItem) * 100);        

    while(!fileDone){
        char * keyEnd = strchr(keyStart, ':');
        if(keyEnd == NULL) break;
        char * propertyStart = keyEnd;
        while(*(++propertyStart) == ' ');
        char * propertyEnd = propertyStart;

        if(*propertyEnd == '{'){
            uint32_t currLevel = 1;
            while(currLevel){
                propertyEnd ++;
                if(*propertyEnd == 0){
                    fileDone = 1;
                    break;
                }else if(*propertyEnd == '{'){
                    currLevel ++;
                }else if(*propertyEnd == '}'){
                    currLevel --;
                }
            }
        }else if(*propertyEnd == '['){
            uint32_t currLevel = 1;
            while(currLevel){
                propertyEnd ++;
                if(*propertyEnd == 0){
                    fileDone = 1;
                    break;
                }else if(*propertyEnd == '['){
                    currLevel ++;
                }else if(*propertyEnd == ']'){
                    currLevel --;
                }
            }
        }else if(*propertyEnd == ','){
            //handle empty property
        }

        while(1){
            propertyEnd++;
            if(*propertyEnd == ',') break;
            if(*propertyEnd == '}' || *propertyEnd == 0){
                fileDone = 1;
                break;
            }
        }

        keyStart = strchr(keyStart, '"');
        if(keyStart == 0) return;
        int32_t keyLength = keyEnd - keyStart - 1;
        int32_t propLength = propertyEnd - propertyStart;
        if(keyLength > 0 && propLength > 0){
            char * key = malloc(keyLength);
            memcpy(key, keyStart + 1, keyLength - 1);
            key[keyLength-1 ] = 0;

            char * property = malloc(propLength + 1);
            memcpy(property, propertyStart, propLength);
            property[propLength] = 0;

            char * start = strchr(property, '"');
            if(start != 0){
                start++;
                char * end = strchr(start, '"');
                propLength = end - start;
                char * temp = malloc(propLength + 1);
                memcpy(temp, start, propLength);
                temp[propLength] = 0;
                free(property);
                property = temp;
            }

            settings[settingsCount].key = key;
            settings[settingsCount].value = property;
            settingsCount ++;
        }
        keyStart = propertyEnd + 1;
    }

    return;
}

static void CFM_saveCalFile(){
    unlink("/spiffs/cal.json");
    FILE* cf = fopen("/spiffs/cal.json", "w");
    if(cf == 0) return;

    fprintf(cf, "{\r\n");
    fprintf(cf, "\t\"Datapoints\":[\r\n");

    for(int32_t i = 0; i < calDataCount; i ++){
        fprintf(cf, "\t\t[%d,%f]%c\r\n", calData[i].sensorReading, calData[i].appliedField, ((i+1) < calDataCount) ? ',' : ' ');
    }

    fprintf(cf, "\t]\r\n}");
    fclose(cf);
}

esp_err_t CFM_processNewCalData(httpd_req_t *req){
    char * data = malloc(2048);
    memset(data, 0, 2048);
    httpd_req_recv(req, data, 2048);

    CFM_loadCal(data);
    CFM_saveCalFile();

    free(data);

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_sendstr(req, "yeah man");
    return ESP_OK;
}

static void CFM_saveSettingsFile(){
    unlink("/spiffs/settings.json");
    FILE* sf = fopen("/spiffs/settings.json", "w");
    if(sf == 0) return;

    fprintf(sf, "{");

    for(int32_t i = 0; i < settingsCount; i ++){
        if(settings[i].skip) continue;
        fprintf(sf, "\t%c\r\n\"%s\":\"%s\"", ((i > 0) ? ',' : ' '), settings[i].key, settings[i].value);
    }

    fprintf(sf, "\r\n}");
    fclose(sf);
}

esp_err_t CFM_processNewSettingsData(httpd_req_t *req){
    char * data = malloc(2048);
    memset(data, 0, 2048);
    httpd_req_recv(req, data, 2048);

    CFM_loadAllSettings(data);

    unsigned wifiRestartRequired = 0;
    SettingsItem * wfc = CFM_getSetting("WIFICHANGED");
    if(wfc != 0){
        wifiRestartRequired = 1;
        ESP_LOGI(TAG, "wifiRestartString = %s", wfc->value);
        wfc->skip = 1;
    } 

    unsigned mqttRestartRequired = 0;
    SettingsItem * mqc = CFM_getSetting("MQTTCHANGED");
    if(mqc != 0) {
        mqttRestartRequired = 1;
        ESP_LOGI(TAG, "mqttRestartString = %s", mqc->value);
        mqc->skip = 1;
    }

    ESP_LOGI(TAG, "wifiRestart = %d mqttRestart = %d", wifiRestartRequired, mqttRestartRequired);


    CFM_saveSettingsFile();

    free(data);

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_sendstr(req, "yeah man");

    FM_loadSettings();
    if(wifiRestartRequired) WIFI_init();
    //if(mqttRestartRequired) MQTT_init();
    return ESP_OK;
}

