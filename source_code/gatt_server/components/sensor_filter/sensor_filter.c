#include "sensor_filter.h"
#include <stdlib.h>

static int mq2_samples[FILTER_MQ2_SAMPLE_COUNT];
static int mq2_index = 0;

static int mq7_samples[FILTER_MQ7_SAMPLE_COUNT];
static int mq7_index = 0;

static int last_dht_temp = -1000;
static int last_dht_hum = -1000;

void filter_mq2_init(void) {
    for (int i = 0; i < FILTER_MQ2_SAMPLE_COUNT; ++i) mq2_samples[i] = 0;
    mq2_index = 0;
}

int filter_mq2_apply(int new_value, int threshold) {
    mq2_samples[mq2_index++] = new_value;
    if (mq2_index >= FILTER_MQ2_SAMPLE_COUNT) mq2_index = 0;

    int sum = 0;
    for (int i = 0; i < FILTER_MQ2_SAMPLE_COUNT; ++i) sum += mq2_samples[i];
    int avg = sum / FILTER_MQ2_SAMPLE_COUNT;

    return (abs(avg - new_value) <= threshold) ? avg : new_value;
}

// -------- MQ7 --------

void filter_mq7_init(void) {
    for (int i = 0; i < FILTER_MQ7_SAMPLE_COUNT; ++i) mq7_samples[i] = 0;
    mq7_index = 0;
}

static int compare_int(const void *a, const void *b) {
    return (*(int *)a - *(int *)b);
}

int filter_mq7_apply_median(int new_value) {
    mq7_samples[mq7_index++] = new_value;
    if (mq7_index >= FILTER_MQ7_SAMPLE_COUNT) mq7_index = 0;

    int sorted[FILTER_MQ7_SAMPLE_COUNT];
    for (int i = 0; i < FILTER_MQ7_SAMPLE_COUNT; ++i) sorted[i] = mq7_samples[i];
    qsort(sorted, FILTER_MQ7_SAMPLE_COUNT, sizeof(int), compare_int);

    return sorted[FILTER_MQ7_SAMPLE_COUNT / 2];
}

float filter_mq7_apply_ema(int new_value, float alpha) {
    static float ema = 0;
    static int initialized = 0;
    if (!initialized) {
        ema = new_value;
        initialized = 1;
    } else {
        ema = alpha * new_value + (1.0f - alpha) * ema;
    }
    return ema;
}

// -------- DHT11 --------

void filter_dht11_init(void) {
    last_dht_temp = -1000;
    last_dht_hum = -1000;
}

float filter_dht11_temperature(float raw_temp) {
    if (abs(raw_temp - last_dht_temp) >= 1) {
        last_dht_temp = raw_temp;
    }
    return last_dht_temp;
}

int filter_dht11_humidity(int raw_hum) {
    if (abs(raw_hum - last_dht_hum) >= 1) {
        last_dht_hum = raw_hum;
    }
    return last_dht_hum;
}
