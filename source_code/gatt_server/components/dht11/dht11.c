#include "dht11.h"

static const char *TAG = "DHT11";

/* Lưu chân được truyền vào từ init */
static gpio_num_t dht_pin;

void dht11_init(gpio_num_t pin) {
    dht_pin = pin;
    gpio_set_direction(dht_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dht_pin, 1);
}

/******************************** Hàm này để chờ pin ở mức điện áp mong muốn trong thời gian timeout_us ************************************/
int wait_for_level(gpio_num_t pin, int level, uint32_t timeout_us)
{
    while(gpio_get_level(pin) != level) // Nếu mức điện áp hiện tại khác mức điện áp mong muốn
    {
        if(timeout_us-- == 0)   // Hết thời gian chờ, chưa hết thì giảm đi
        {
            return -1;
        }
        esp_rom_delay_us(1);    // Chờ 1 micro giây. Phải chờ nếu không thì vòng lặp while nó tiếp tục cực nhanh, sẽ chiếm toàn bộ CPU, giảm số lần kiểm tra GPIO
    }   
    return 0;   // Chờ thành công rồi thì trả về 0
}


/************************************* Hàm đọc dữ liệu nhiêt độ và độ ẩm từ DHT11 ************************************/
esp_err_t dht11_read(gpio_num_t pin, int *temp, int *humidity)
{
    int bits[40] = {0}; // Mảng này để lưu từng bit đọc được. DHT11 sẽ gửi về 16 bit đầu là độ ẩm, 16 bit tiếp là nhiệt độ, 8 bit cuối là checksum
    uint8_t data[5] = {0};  // Dữ liệu trả về gồm 5 byte chính là chuyển đổi từ 40 bit kia

    // Gửi tín hiệu bắt đầu đến DHT11. DHT11 chỉ gửi dữ liệu khi nhận được tín hiệu khởi động này
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);  // Đặt đối số pin truyền vào là OUTPUT để gửi tín hiệu đi
    gpio_set_level(pin, 0);     // Kéo mức điện áp xuống thấp
    vTaskDelay(pdMS_TO_TICKS(20));  //  Giữ mức điện áp thấp trong khoảng 20ms

    gpio_set_level(pin, 1);     // Kéo mức điện áp lên cao
    esp_rom_delay_us(30);   // Giữ trong khoảng 30us. Đây là khoảng thời gian để dht11 biết là tín hiệu khởi động đã kết thúc
    gpio_set_direction(pin, GPIO_MODE_INPUT);   // Chuyển sang là Input để nhận dữ liệu vào
    gpio_pullup_en(pin);

    // DHT11 gửi phản hồi. DHT11 sẽ kéo chân xuống thấp rồi kéo lên cao để phàn hồi là nó sẵn sàng gửi dữ liệu
    if(wait_for_level(pin, 0, 80) < 0) return ESP_FAIL;
    if(wait_for_level(pin, 1, 80) < 0) return ESP_FAIL;


    // Đọc 40 bit data
    for(int i = 0; i < 40; i++)
    {
        if(wait_for_level(pin, 0, 50) < 0)  // Mỗi bit bắt đầu bằng mức thấp
            return ESP_FAIL;
        if(wait_for_level(pin, 1, 70) < 0)  // Rồi kết thúc bằng mức cao
            return ESP_FAIL;

        esp_rom_delay_us(40);   // Nếu sau 40us điện áp vẫn là mức cao thì là bit = 1
        // bits[i] = gpio_get_level(pin);
        bits[i] = (gpio_get_level(pin) == 1) ? 1 : 0;
    }

    // Ghép 40 bit thành 5 byte data
    for(int i = 0; i < 40; i++)
    {
        data[i / 8] <<= 1;
        data[i / 8] |= bits[i];
    }

    // Kiểm tra checksum
    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if(checksum != data[4])
    {
        ESP_LOGE(TAG, "Checksum fail: %d != %d", checksum, data[4]);
        return ESP_FAIL;
    }

    // Gán giá trị trả về
    *humidity = data[0];
    *temp = data[2];
    return ESP_OK;
}