#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_partition.h"


void emu_user_init(void);

char *osd_getromdata() {
	char* romdata;
	const esp_partition_t* part;
	spi_flash_mmap_handle_t hrom;
	esp_err_t err;
	nvs_flash_init();
	part=esp_partition_find_first(0x40, 1, NULL);
	if (part==0) printf("Couldn't find rom part!\n");
	err=esp_partition_mmap(part, 0, 3*1024*1024, SPI_FLASH_MMAP_DATA, (const void**)&romdata, &hrom);
	if (err!=ESP_OK) printf("Couldn't map rom part!\n");
	printf("Initialized. ROM@%p\n", romdata);
    return (char*)romdata;
}

void flic_led(){
    nvs_flash_init();

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    int level = 0;
    for(int i=0; i<3; i++){
        gpio_set_level(GPIO_NUM_2, level);
        level = !level;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

int app_main(void)
{
    flic_led();      // For Hard Test
    emu_user_init(); // SMS Emu Main
    return 0;
}

