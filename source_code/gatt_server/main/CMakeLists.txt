idf_component_register(SRCS "gatts_demo.c"
                    INCLUDE_DIRS "."
                    REQUIRES dht11 esp_http_client cjson nvs_flash bt
                    PRIV_REQUIRES driver)
