/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_sleep.h"
#include "bme280.h"

#define I2C_MASTER_NUM      0
#define I2C_MASTER_SCL_IO   14
#define I2C_MASTER_SDA_IO   12

#define BOARD_LED           2

#define ACK_CHECK_EN        0x1              /*!< I2C master will check ack from slave*/
#define LAST_NACK_VAL       0x2              /*!< I2C last_nack value */

#define ERROR_CHECK(x) ({ esp_err_t res = (x); \
                          if(res != ESP_OK) \
                            printf("%s %d: Error 0x%X\n", __FUNCTION__, __LINE__, res); \
                        })

#define SENS_ABS(x) ((x) < 0 ? -(x) : (x))

static esp_err_t i2c_init(i2c_port_t i2c_num)
{
    i2c_config_t conf = { 0, };
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode));
    ERROR_CHECK(i2c_param_config(i2c_num, &conf));

    return ESP_OK;
}

static int8_t i2c_read_func(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t *bmeAddr = intf_ptr;

    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (*bmeAddr) << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (*bmeAddr) << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, reg_data, len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static int8_t i2c_write_func(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t *bmeAddr = intf_ptr;
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (*bmeAddr) << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, (uint8_t*)reg_data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void delay_us(uint32_t period, void *intf_ptr)
{
    vTaskDelay((period / 1000 + portTICK_PERIOD_MS / 2) / portTICK_PERIOD_MS);
}

static int8_t configure_sensor(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, dev);

    return rslt;
}

static void init_led(int led_io_num)
{
    gpio_config_t io_conf;

    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL << led_io_num);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    ERROR_CHECK(gpio_config(&io_conf));
}

static void set_led(int led_io_num, bool on)
{
    ERROR_CHECK(gpio_set_level(led_io_num, on ? 0 : 1));
}

void app_main()
{
    static uint8_t bmeAddr = 0x76;
    struct bme280_dev bmedev = {
        .intf_ptr = &bmeAddr,
        .intf = BME280_I2C_INTF,
        .read = i2c_read_func,
        .write = i2c_write_func,
        .delay_us = delay_us,
    };
    struct bme280_data comp_data;
    int status;

    init_led(BOARD_LED);
    set_led(BOARD_LED, true);

    if(i2c_init(I2C_MASTER_NUM) == ESP_OK)
    {
        status = bme280_init(&bmedev);
        if(status >= 0)
        {
            if(configure_sensor(&bmedev) == 0)
            {
                while(1)
                {
                    if(bme280_set_sensor_mode(BME280_FORCED_MODE, &bmedev) == 0)
                    {
                        vTaskDelay((bme280_cal_meas_delay(&bmedev.settings) + portTICK_PERIOD_MS / 2) / portTICK_PERIOD_MS);
                        if(bme280_get_sensor_data(BME280_ALL, &comp_data, &bmedev) == 0)
                        {
                            printf("Temp %d.%d, Presure %d, Humidity %d.%d\n", comp_data.temperature / 100, SENS_ABS(comp_data.temperature) % 100, comp_data.pressure, comp_data.humidity >> 10, comp_data.humidity & 0x3FF);
                        }
                        else
                        {
                            printf("%s %d: Failed get data from BME\n", __FUNCTION__, __LINE__);
                        }
                    }
                    else
                    {
                        printf("%s %d: Failed set Forced mode for BME\n", __FUNCTION__, __LINE__);
                    }
                    fflush(stdout);
                    set_led(BOARD_LED, false);
                    ERROR_CHECK(esp_sleep_enable_timer_wakeup(10 * 1000000)); // 10s
                    ERROR_CHECK(esp_light_sleep_start());
                    set_led(BOARD_LED, true);
                }
            }
            else
            {
                printf("%s %d: Failed configure BME\n", __FUNCTION__, __LINE__);
            }
        }
        else
        {
            printf("%s %d: Failed init BME: %d\n", __FUNCTION__, __LINE__, status);
        }

        ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    }
    else
    {
        printf("%s %d: Failed init %d I2C\n", __FUNCTION__, __LINE__, I2C_MASTER_NUM);
    }
    set_led(BOARD_LED, false);
}
