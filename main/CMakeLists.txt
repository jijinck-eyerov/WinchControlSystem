idf_component_register(
    SRCS "i2c-lcd.c" "main.c"
    SRCS "joystick_buttons.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES    "esp32-button" "driver" "ds18x20" "nvs_flash"
    )
