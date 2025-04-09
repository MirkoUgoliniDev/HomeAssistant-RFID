# Definisci lista vuota per i sorgenti
set(SOURCES "")

# Aggiungi i file sorgente
list(APPEND SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/DFPlayerMiniFast.c")

# Registra il componente
idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_timer)
