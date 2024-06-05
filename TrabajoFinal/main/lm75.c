#include "esp_log.h"
#include <stdlib.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "lm75.h"
#include "freertos/queue.h"

extern QueueHandle_t lmqueue;

static const char *TAG = "TEMP_READER";

#define I2C_MASTER_SCL_IO           39      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           38      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define LM75_SENSOR_ADDR                 0x48        /*!< Slave address of the LM75 sensor */
#define LM75_TEMP_READ           0x00        /*!< Register addresses of the "who am I" register */


i2c_master_dev_handle_t dev_handle2;


/* static esp_err_t LM75_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, LM75_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
} */




/* static esp_err_t temp_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
} */
void temp_init(void){
    ESP_LOGI(TAG,"Iniciando configuración el I2C");
    i2c_master_bus_config_t conf={
        .clk_source=I2C_CLK_SRC_DEFAULT,
        .i2c_port= I2C_NUM_1,
        .scl_io_num=I2C_MASTER_SCL_IO,
        .sda_io_num=I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt=7,
        .flags.enable_internal_pullup=true,
    };
    i2c_master_bus_handle_t bus_handle2;
    ESP_ERROR_CHECK (i2c_new_master_bus(&conf,&bus_handle2));
    ESP_LOGI(TAG,"Iniciando configuración LM");
    i2c_device_config_t lmconf={
        .dev_addr_length=I2C_ADDR_BIT_LEN_7,
        .device_address=LM75_SENSOR_ADDR,
        .scl_speed_hz=I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK( i2c_master_bus_add_device(bus_handle2,&lmconf,&dev_handle2));
}

void swap(uint16_t* a,uint16_t* b){
    uint16_t t= *a;
    *a=*b;
    *b=t;
}

int partition(uint16_t array[],int low,int high){
    uint16_t pivot=array[high];
    int j=(low-1);
    for (int k=low; k<=high-1; k++) {
        if (array[k]<pivot) {
            j++;
            swap(&array[j],&array[k]);
        }
    
    }
    swap(&array[j+1],&array[high]);
    return (j+1);
}



void quickshort(uint16_t array[],int low, int high){
    if (low<high) {
        int pi = partition(array,low,high);
        quickshort(array,low,pi-1);
        quickshort(array,pi+1,high);
    }
}

void tem_app(void *)
{
    
    uint8_t data[3];
    uint16_t temp;
    uint16_t vec[11];
    int n= 11;
    float temp_data;
    temp_init();
    ESP_LOGI(TAG, "I2C initialized successfully");
        while (true)
    {
    /* Read the LM75 WHO_AM_I register, on power up the register should have the value 0x71 */
    for (uint8_t i=0;i<n;i++){
        ESP_ERROR_CHECK(i2c_master_receive(dev_handle2,data,16,100));
        temp=data[0]<<8;
        temp|=data[1];
        temp=(temp>>5)*12.5;
        vec[i]=temp;
        vTaskDelay(250/portTICK_PERIOD_MS);
    }

    /* for (int i = 0; i < n; i++)
    {
        for (int j = i; j < n; j++)
        {
            if (vec[j]<vec[i])
            {
                uint8_t a=vec[i];
                vec[i]= vec[j];
                vec[j]=a;
            }
            
        }
        
    } */
    
    quickshort(vec,0,n);
    for (uint8_t i = 0; i < 11; i++)
    {
        ESP_LOGI(TAG,"%d",vec[i]);
    }

    temp_data=(float) vec[5]/1000;
    


    ESP_LOGI(TAG, "Temp = %.2f", temp_data);
    if (xQueueSend(lmqueue,&temp_data,portMAX_DELAY)!=pdPASS)
    {
        ESP_LOGE("lm75_task","Fallo");
    }
    

    }

}