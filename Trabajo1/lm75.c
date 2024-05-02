#include "esp_log.h"
#include <stdlib.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "lm75.h"

static const char *TAG = "TEMP_READER";

#define I2C_MASTER_SCL_IO           15      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           16      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define LM75_SENSOR_ADDR                 0x48        /*!< Slave address of the LM75 sensor */
#define LM75_TEMP_READ           0x00        /*!< Register addresses of the "who am I" register */



/**
 * @brief Read a sequence of bytes from a LM75 sensor registers
 */
static esp_err_t LM75_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, LM75_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}



/**
 * @brief i2c master initialization
 */
static esp_err_t temp_init(void)
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
}


void tem_app(void)
{
    
    uint8_t data[3];
    uint16_t temp;
    float vec[]={3,5,4,2,1};
    int n= sizeof(vec)/sizeof(vec[0]);
    float tempsi;
    ESP_ERROR_CHECK(temp_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    while (true)
    {
    

    /* Read the LM75 WHO_AM_I register, on power up the register should have the value 0x71 */
    ESP_ERROR_CHECK(LM75_register_read(LM75_TEMP_READ, data, 2));
    temp=data[0]<<8;
    temp|=data[1];
    tempsi=(temp>>5)*0.125;

    for (int i=0;i<n;i++){
        vec[i]=tempsi;
        vTaskDelay(500/portTICK_PERIOD_MS);
    }

    for (int i = 0; i < n; i++)
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
        
    }
    ESP_LOGI(TAG, "Temp = %.2f", vec[2]);
    }
    
    


    /* Demonstrate writing by reseting the LM75 */

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}