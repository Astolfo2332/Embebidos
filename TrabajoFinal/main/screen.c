#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ssd1306.h"
#include "font8x8_basic.h"
#include "screen.h"

#define tag "SSD1306"

SSD1306_t screen_init(uint8_t SDA_pin,uint8_t SCL_pin){
	SSD1306_t dev;
	ESP_LOGI(tag, "INTERFACE is i2c");
	ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d",SDA_pin);
	ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d",SCL_pin);
	ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	i2c_master_init(&dev, SDA_pin, SCL_pin, CONFIG_RESET_GPIO);
	ESP_LOGI(tag, "Panel is 128x64");
	ssd1306_init(&dev, 128, 64);
	return dev;
}

void func_screen(SSD1306_t dev,float temp_data)
{
	char lineChar[5];
	

	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text_x3(&dev, 0, "Temp", 4, false);
    sprintf(lineChar,"%2.2f",temp_data);
    ssd1306_display_text_x3(&dev,3,lineChar,5,false);

}