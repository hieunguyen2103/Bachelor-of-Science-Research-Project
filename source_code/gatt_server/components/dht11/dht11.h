#pragma once

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"



void dht11_init(gpio_num_t pin);

/******************************** Hàm này để chờ pin ở mức điện áp mong muốn trong thời gian timeout_us ************************************/
static int wait_for_level(gpio_num_t pin, int level, uint32_t timeout_us);


/************************************* Hàm đọc dữ liệu nhiêt độ và độ ẩm từ DHT11 ************************************/
esp_err_t dht11_read(gpio_num_t pin, int *temp, int *humidity);