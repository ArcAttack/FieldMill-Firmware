#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/timer.h"

#include "FieldMill.h"
#include "MCP3301.h"

static void ADC_task(void * harambe);

static xQueueHandle ADC_sampleQue = NULL;
static xQueueHandle ADC_triggerQue = NULL;
static spi_device_handle_t ADC_devHandle;

static const char *TAG = "ADC";

unsigned state2;
void ADC_timerISR(void *para){
	timer_spinlock_take(TIMER_GROUP_0);
	timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, 1);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, 1);
    timer_spinlock_give(TIMER_GROUP_0);
	BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR(ADC_triggerQue, ADC_triggerQue, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

xQueueHandle ADC_init(uint32_t samplingRate){
	spi_bus_config_t buscfg={
		.miso_io_num	=	FM_ADC_DIN_PIN,
		.sclk_io_num	=	FM_ADC_CLK_PIN,
		.mosi_io_num 	= 	-1,
		.quadwp_io_num	=	-1,
		.quadhd_io_num	=	-1,
		.max_transfer_sz=	4
	};

	spi_device_interface_config_t devcfg={
		.clock_speed_hz	=	1700000,
		.mode			=	0,
		.command_bits 	= 	0,
		.address_bits 	= 	0,
		.dummy_bits 	= 	0,
		.spics_io_num	=	FM_ADC_CS_PIN,
		.queue_size		=	1,
        .flags = SPI_DEVICE_HALFDUPLEX,
	};

	esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0);
	ESP_ERROR_CHECK(ret);
	ret=spi_bus_add_device(HSPI_HOST, &devcfg, &ADC_devHandle);
	ESP_ERROR_CHECK(ret);

	timer_config_t config = {
        .divider = 2,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_START,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    timer_init(TIMER_GROUP_0, 1, &config);
    timer_start(TIMER_GROUP_0, 1);

    ADC_sampleQue = xQueueCreate(10, sizeof(ADC_Sample_t)); 
    ADC_triggerQue = xQueueCreate(10, sizeof(ADC_triggerQue));  
	
    ADC_setSampingRate(2400);
    timer_enable_intr(TIMER_GROUP_0, 1);
    timer_isr_register(TIMER_GROUP_0, 1, ADC_timerISR, 1, 0, NULL);

    xTaskCreate(ADC_task, "adc task", configMINIMAL_STACK_SIZE + 4000, 0, tskIDLE_PRIORITY + 10, 0);
    return ADC_sampleQue;
}

unsigned state;
static void ADC_sample(ADC_Sample_t * ret){
    spi_transaction_t transaction = {.rxlength = 16, .rx_buffer=&(ret->value)};
    timer_get_counter_value(TIMER_GROUP_0, 0, &(ret->sampleTime));
	unsigned use = FM_isSampleUsable(ret->sampleTime);
	gpio_set_level(22, use);
	if(!use) return;
    ret->rotorPos = FM_rotorPos;
	spi_device_polling_transmit(ADC_devHandle, &transaction);
	ret->value = SPI_SWAP_DATA_RX(ret->value, 16);
    if(ret->value & 0x1000) ret->value |= 0xfffff000; else ret->value &= 0xfff;  //sign extend the value
	//ESP_LOGI(TAG, "got new ADC reading: %d at motorPos %d", ret->value, ret->rotorPos);
	gpio_set_level(23, (state = !state));
}

static void ADC_task(void * harambe){
    //kill(harambe);

    while(1){
		QueueHandle_t * data;
		if(xQueueReceive(ADC_triggerQue, &data, 1000/portTICK_PERIOD_MS)){
			ADC_Sample_t sample;
			ADC_sample(&sample);
			xQueueSend(ADC_sampleQue, &sample, 0);
		}else{
			ESP_LOGI(TAG, "where samples??");
		}
    }
}

void ADC_setSampingRate(uint32_t samplingRate){
	uint32_t targetCount = TIMER_BASE_CLK / (samplingRate * 4);
	ESP_LOGI(TAG, "target count = %d", targetCount);
    timer_set_alarm_value(TIMER_GROUP_0, 1, targetCount);
    timer_set_counter_value(TIMER_GROUP_0, 1, 0);
}