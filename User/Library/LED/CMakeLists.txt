add_library(led STATIC)

target_sources(led
        PRIVATE
        Src/LED.c
        )

target_include_directories(led
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        )

target_link_libraries(motor
        PRIVATE
            hal
        )