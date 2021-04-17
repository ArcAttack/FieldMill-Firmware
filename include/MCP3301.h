#ifndef MCP_include
#define MCP_include
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

xQueueHandle ADC_init();
void ADC_setSampingRate(uint32_t samplingRate);
uint16_t ADC_read();

typedef struct{
    int32_t value;
    unsigned rotorPos;
    uint64_t sampleTime;
} ADC_Sample_t;

#endif