idf_component_register(
    SRCS "main.c" "board.c" "display.c" "touch.c" "imu.c" "settings.c"
         "view_ctrl.c" "render.c" "lvgl_port.c" "ui_settings.c" "model_store.c" "diag.c"
    INCLUDE_DIRS "."
    REQUIRES r3d esp_lcd esp_driver_spi esp_driver_gpio esp_driver_i2c
             esp_partition nvs_flash esp_timer
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format-truncation)
