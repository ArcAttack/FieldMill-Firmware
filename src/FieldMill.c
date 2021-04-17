#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "ConfigManager.h"
#include "driver/gpio.h"
#include "FieldMill.h"
#include "driver/timer.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "mqtt_client.h"

#include "MCP3301.h"

static void FM_motorCountTask(void * taskData);
static esp_err_t FM_getMeasurementHandler(httpd_req_t *req);
static void FM_valueTask(void * taskData);
static void FM_initMotorSubSystem();

static uint32_t FM_currRpm = 0;
static uint32_t FM_motorSensorValid = 0;
static float FM_motorPower = 0;
static unsigned FM_motorEnabled = 1;
static uint64_t FM_lastZC = 0;
static uint64_t FM_lastPeriod = 0;
static float FM_currAVG = 0;
static float FM_currAVGField = 0;

static const char *TAG = "FieldMill";

static xQueueHandle FM_Motor_ISR_queue = NULL;

char * FM_mqttTopic = NULL;
uint32_t FM_mqttPeriod = 0;

float FM_motorTuneP = 0.001f;
float FM_motorTuneD = -0.001f;
uint32_t FM_motorTargetRPM = 3600;
int32_t FM_lastRPMError = 0;
unsigned FM_rotorPos = 0;

void FM_initMQTT();

void FM_init(){
    httpd_handle_t server = SERVER_getServer();

    httpd_uri_t measuredData = {
        .uri       = "/measure.json",  // Match all URIs of type /path/to/file
        .method    = HTTP_GET,
        .handler   = FM_getMeasurementHandler
    };
    httpd_register_uri_handler(server, &measuredData);

    FM_loadSettings();

    xQueueHandle adcQueue = ADC_init(1000);
    xTaskCreate(FM_valueTask, "fm value task", configMINIMAL_STACK_SIZE + 4000, adcQueue, tskIDLE_PRIORITY + 1, 0);

    FM_initMotorSubSystem();
}

void FM_loadSettings(){
    SettingsItem * cs = CFM_getSetting("FM_targetRPM");
    if(cs != 0) FM_motorTargetRPM = atoi(cs->value);
    if(cs != 0) ESP_LOGI(TAG, "set target RPM to %d (%s)", FM_motorTargetRPM, cs->value);
    
    cs = CFM_getSetting("FM_motorTuneP");
    if(cs != 0) FM_motorTuneP = atof(cs->value);
    if(cs != 0) ESP_LOGI(TAG, "set tune i to %.5f (%s)", FM_motorTuneP, cs->value);
    
    cs = CFM_getSetting("FM_motorTuneD");
    if(cs != 0) FM_motorTuneD = atof(cs->value);
    if(cs != 0) ESP_LOGI(TAG, "set tune d to %.5f (%s)", FM_motorTuneD, cs->value);
    
    cs = CFM_getSetting("MQTT_period");
    if(cs != 0) FM_mqttPeriod = atoi(cs->value);
    
    cs = CFM_getSetting("MQTT_topic");
    if(cs != 0) FM_mqttTopic = cs->value;

    FM_initMQTT();
}

unsigned FM_isSampleUsable(uint64_t time){
    if(FM_lastZC == 0) return 0;
    uint32_t phaseTime = time % (FM_lastZC>>2);
    //ESP_LOGI(TAG, "time = %d, segment = %d, comp = %d", (uint32_t) time, phaseTime, (uint32_t) (FM_lastZC>>2));
    return (phaseTime > FM_CONF_DEADTIME) && (phaseTime < (FM_lastZC>>2) - FM_CONF_DEADTIME);
}

static int32_t scaleForMotorSpeed(int32_t value){
    return value;
    int32_t error = 3600 - FM_getMotorRPM();
    if(abs(error) < 3) return value;

    //out@n0 = out * 1/(1 + dn*y)
    return (int32_t) ((float) value * (1.0f / (1.0f + (float) error * FM_CONF_MOTORSPEED_GAIN)));
}

static void FM_valueTask(void * taskData){
    xQueueHandle adcQueue = (xQueueHandle) taskData;
    ADC_Sample_t currSample;
    uint16_t count = 0;
    while(1){
        if(xQueueReceive(adcQueue, &currSample, 1000/portTICK_PERIOD_MS)){
            /*
            why the deadtime anyway?
                due to field fringing at the edge of the rotor the output voltage is more similar to a sine wave that the expected square. 
                During these slow falling edges the data is not valid and needs to be ignored.
            */
            int32_t reading = (int32_t) (currSample.rotorPos ? currSample.value : -currSample.value);
            //int32_t readingMotorCal = scaleForMotorSpeed(reading);
            FM_currAVG = (FM_currAVG * 9999.0f + (float) reading) / 10000.0f;
            FM_currAVGField = CFM_scaleMeasurement(FM_currAVG);
            if(count++ == 1000){
                count = 0;
                ESP_LOGI(TAG, "currAvg = %+5.5f -> field %.2f", FM_currAVG, FM_currAVGField);
            }
        }else{
            ESP_LOGI(TAG, "ADC is too slow :(");
        }
    }
}

uint32_t FM_edgeCount = 0;
uint64_t FM_lt = 0;
static void IRAM_ATTR FM_motorISR(void* arg){
    FM_rotorPos = gpio_get_level(FM_INTERRUPTER_PIN);
    uint64_t buff = 0;
    timer_get_counter_value(TIMER_GROUP_0, 0, &buff);
    if((buff - FM_lt) < (FM_lastZC >> 4)) return;
    if(FM_edgeCount == 3){
        FM_edgeCount = 0;
        FM_lastZC = buff;
        timer_set_counter_value(TIMER_GROUP_0, 0, 0);
        xQueueSendFromISR(FM_Motor_ISR_queue, &buff, NULL);
        buff = 0;
    }else{
        FM_edgeCount ++;
    }
    gpio_set_level(5, FM_rotorPos);
    FM_lt = buff;
}

static void FM_motorCountTask(void * taskData){
    uint64_t data = 0;
    while(1){
        if(xQueueReceive(FM_Motor_ISR_queue, &data, 1000/portTICK_PERIOD_MS)){
            uint64_t dT = data;
            uint32_t rpm = (uint32_t) (4800000000 / dT);
            FM_currRpm = rpm;
            FM_lastPeriod = dT;
            FM_motorSensorValid = 1;
            gpio_set_level(2, 1);
        }else{
            if(FM_motorSensorValid == 0){
                FM_currRpm = 0;
                gpio_set_level(2, 0);
            }else{
                FM_motorSensorValid = 0;
            }
        }
    }
}

static void FM_motorCtrlTask(void * taskData){
    while(1){
        if(FM_motorSensorValid && FM_motorEnabled){
            int32_t error = FM_motorTargetRPM - FM_getMotorRPM();
            if(abs(error) < 2) error = 0;

            if(error != 0){
                FM_motorPower += (float) error * FM_motorTuneP + (float) (error - FM_lastRPMError) * FM_motorTuneD;
                FM_lastRPMError = error;
                if(FM_motorPower < 0.0f) FM_motorPower = 0.0f;

                if(FM_motorPower >= 100.0){
                    FM_motorPower = 100.0f;
                    ESP_LOGW(TAG, "Motor power range exauhsted! (100%% ; error = %d rpm (is %d rpm))", error, FM_getMotorRPM());
                }else if(FM_motorPower > 93.0){
                    ESP_LOGW(TAG, "Motor is getting close to its power limit! (%.2f%%)", FM_motorPower);
                }/*else{
                    ESP_LOGW(TAG, "Motor power = %.2f%% error = %d rpm (is %d rpm)", FM_motorPower, error, FM_getMotorRPM());
                }*/
                mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, FM_motorPower);  //turn off motor if error occours
            }
        }else{
            if(FM_currRpm == 0){
                mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 50.0);  //turn off motor if error occours
            }
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

float FM_getField(){
    return FM_currAVGField;
}

uint32_t FM_getMotorRPM(){
    return FM_currRpm;
}

int32_t FM_getRaw(){
    return FM_currAVG;
}

static void FM_initMotorSubSystem(){

    FM_Motor_ISR_queue = xQueueCreate(10, sizeof(uint64_t));  
    
    //initialize the counter timer used to measure time between interrupter pulses
    timer_config_t config = {
        .divider = 2,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_START,
        .alarm_en = TIMER_ALARM_DIS,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    timer_init(TIMER_GROUP_0, 0, &config);
    timer_start(TIMER_GROUP_0, 0);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL<<FM_INTERRUPTER_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0; //pullup is done in hardware to allow for faster and more consistent edges
    gpio_config(&io_conf);
    gpio_set_intr_type(FM_INTERRUPTER_PIN, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);
    uint64_t * timerBuffer = malloc(sizeof(uint64_t)); 
    gpio_isr_handler_add(FM_INTERRUPTER_PIN, FM_motorISR, timerBuffer);

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, FM_Motor_PIN);
    mcpwm_config_t pwm_config = {.frequency = 1000, .cmpr_b = 0, .counter_mode = MCPWM_UP_COUNTER, .duty_mode = MCPWM_DUTY_MODE_0};
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0);

    ESP_LOGI(TAG, "motor ready! 0x%08x", (uint32_t) FM_Motor_ISR_queue);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    gpio_set_direction(5, GPIO_MODE_OUTPUT);
    gpio_set_direction(22, GPIO_MODE_OUTPUT);
    gpio_set_direction(23, GPIO_MODE_OUTPUT);
    xTaskCreate(FM_motorCountTask, "MotorCountTask", configMINIMAL_STACK_SIZE + 4000, 0, tskIDLE_PRIORITY + 3, 0);
    xTaskCreate(FM_motorCtrlTask, "MotorCtrlTask", configMINIMAL_STACK_SIZE + 4000, 0, tskIDLE_PRIORITY + 3, 0);
}

static esp_err_t FM_getMeasurementHandler(httpd_req_t *req){
    char * buff = malloc(128);
    sprintf(buff, "{\"measuredField\": %f,\r\n\"sensorReading\": %d,\r\n\"motorRPM\": %d\r\n}", FM_getField(), FM_getRaw(), FM_getMotorRPM());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, buff);
    free(buff);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}


static void FM_MQTTTask(void * param){
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t) param;
    char buff[1024];
    char fieldChannel[512];
    sprintf(fieldChannel, "%s/reading", FM_mqttTopic);
    while(1){ 
        uint32_t len = sprintf(buff, "{\"measuredField\": %f,\r\n\"sensorReading\": %d,\r\n\"motorRPM\": %d\r\n}", FM_getField(), FM_getRaw(), FM_getMotorRPM());
        esp_mqtt_client_publish(client, fieldChannel, buff, len, 1, 0);
        vTaskDelay(FM_mqttPeriod / portTICK_PERIOD_MS);
    }
}

TaskHandle_t FM_MqttTaskHandle = NULL;
static void FM_MQTTHandler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        if(FM_MqttTaskHandle != NULL) vTaskDelete(FM_MqttTaskHandle);
        xTaskCreate(FM_MQTTTask, "MQTT task", configMINIMAL_STACK_SIZE + 4000, client, tskIDLE_PRIORITY + 1, &FM_MqttTaskHandle);
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        if(FM_MqttTaskHandle != NULL) vTaskDelete(FM_MqttTaskHandle);
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    default:
        break;
    }
}

void FM_initMQTT(){
    SettingsItem * enabled = CFM_getSetting("MQTT_clientEnabled");
    if(enabled == 0) return;
    if(strcmp(enabled->value, "true") != 0) return;

    char * bUri = CFM_getSetting("MQTT_brokerURI")->value;
    char * user = CFM_getSetting("MQTT_user")->value;
    char * password = CFM_getSetting("MQTT_password")->value;
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = bUri,
        .username = user,
        .password = password,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, FM_MQTTHandler, NULL);
    esp_mqtt_client_start(client);
}