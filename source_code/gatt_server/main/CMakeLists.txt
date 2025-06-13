idf_component_register(SRCS "gatts_demo.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_client cjson nvs_flash bt
                    PRIV_REQUIRES driver
                    REQUIRES dht esp_idf_lib_helpers sensor_filter)
