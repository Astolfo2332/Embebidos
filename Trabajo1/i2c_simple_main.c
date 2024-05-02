/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "lm75.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#define GPIO_LED 35 //Se define el pin

void app_main(void)
{
uint8_t dataon=0;
gpio_set_direction(GPIO_LED,GPIO_MODE_OUTPUT); //Se inicializa el pin con el modo

xTaskCreate(&tem_app,"temp",2048,NULL,5,NULL); //appname, identificador, ram, handle, prioridad,argumentos 
while (1)
{
    dataon=!dataon;
    ESP_ERROR_CHECK(gpio_set_level(GPIO_LED,dataon));
    vTaskDelay(1500/portTICK_PERIOD_MS);

    /* code */
}

}
