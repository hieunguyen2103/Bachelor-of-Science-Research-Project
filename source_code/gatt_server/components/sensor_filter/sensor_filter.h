#pragma once

#include <stdint.h>

#define FILTER_MQ2_SAMPLE_COUNT 10
#define FILTER_MQ7_SAMPLE_COUNT 5

// -------- MQ2 --------
void filter_mq2_init(void);
int filter_mq2_apply(int new_value, int threshold);

// -------- MQ7 --------
void filter_mq7_init(void);
int filter_mq7_apply_median(int new_value);
int filter_mq7_apply_ema(int new_value, float alpha);

// -------- DHT11 --------
void filter_dht11_init(void);
float filter_dht11_temperature(float raw_temp);
int filter_dht11_humidity(int raw_hum);
