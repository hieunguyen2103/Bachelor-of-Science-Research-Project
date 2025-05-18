idf_component_register(SRCS "gatts_demo.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_client cJSON nvs_flash bt
                    PRIV_REQUIRES driver)
