/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "sdkconfig.h"
#include "psxcontroller.h"
#include "driver/i2s.h"
#include "esp_partition.h"
#include <esp_spi_flash.h>
#include "types.h"
#include "spi_lcd.h"

#define  ZERO_LENGTH 0

extern t_bitmap bitmap;
extern char vidram[160*144*2];
//unsigned char *patpix; //[2048][8][8];

static uint32_t start_f;
static uint32_t end_f;
static spi_flash_mmap_handle_t handle1;
char* romfptr;

QueueHandle_t vidQueue;
void gbemu();

//Read an unaligned byte.
char unalChar(const char *adr) {
    //printf("%4x|",(int)adr-0x3ffb1110);
	//See if the byte is in memory that can be read unaligned anyway.
	if (!(((int)adr)&0x40000000)) return *adr;
	//Nope: grab a word and distill the byte.
	int *p=(int *)((int)adr&0xfffffffc);
	int v=*p;
	int w=((int)adr&3);
	if (w==0) return ((v>>0)&0xff);
	if (w==1) return ((v>>8)&0xff);
	if (w==2) return ((v>>16)&0xff);
	if (w==3) return ((v>>24)&0xff);
}

//This runs on core 1.
static void videoTask(void *arg) {
	int x, y,tickCnt,qdata;

    ili9341_init();
    ili9341_write_frame(0, 0, 320, 240, NULL);

	x = (320-DEFAULT_WIDTH)/2;
    y = ((240-DEFAULT_HEIGHT)/2);
    while(1) {
		//xQueueReceive(vidQueue, &qdata, portMAX_DELAY);
        tickCnt=xTaskGetTickCount();
        ili9341_write_frame(x, y, DEFAULT_WIDTH, DEFAULT_HEIGHT, vidram);
        //printf("[%d]",(xTaskGetTickCount()-tickCnt)*1);
	}
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void emu_user_init(void)
{
    init_sound();
   	const esp_partition_t* part = esp_partition_find_first(0x40, 1, NULL);
	esp_err_t err=esp_partition_mmap(part, 0, 1*1024*1024, SPI_FLASH_MMAP_DATA, (const void**)&romfptr, &handle1);
    if (err==ESP_OK) printf("Mapping handle=%d ptr=%p\n", handle1, romfptr);
    
    printf("Heap left %d\n", system_get_free_heap_size()); 
   	vidQueue=xQueueCreate(1, sizeof(uint32_t));
	xTaskCreatePinnedToCore(&videoTask, "videoTask", 2048, NULL, 5, NULL, 1);

	xTaskCreate(gbemu, "gbemu"  , 8192, NULL, 3, NULL);
	psxcontrollerInit();
}

