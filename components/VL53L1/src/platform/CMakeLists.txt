# Definisci i file sorgente platform
list(APPEND PLATFORM_SOURCES 
    "${CMAKE_CURRENT_LIST_DIR}/vl53l1_platform.c"
    "${CMAKE_CURRENT_LIST_DIR}/i2c_platform_esp.c"
    "${CMAKE_CURRENT_LIST_DIR}/gpio_platform_esp.c"
)

