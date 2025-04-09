# Definisci liste vuote per raccogliere i sorgenti
set(CORE_SOURCES "")
set(PLATFORM_SOURCES "")

# Includi le definizioni dai sottomoduli
include(${CMAKE_CURRENT_LIST_DIR}/src/core/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/src/platform/CMakeLists.txt)

# Registra il componente con tutti i sorgenti
idf_component_register(SRCS 
    ${CORE_SOURCES}
    ${PLATFORM_SOURCES}
    INCLUDE_DIRS "include/public" "include/platform"
    REQUIRES mbedtls log driver esp_timer freertos
)

