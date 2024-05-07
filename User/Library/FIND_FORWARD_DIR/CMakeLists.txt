add_library(find_forward_dir STATIC)

target_sources(find_forward_dir
        PRIVATE
        Src/FindForwardDir.c
        )

target_include_directories(find_forward_dir
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}
        )

target_link_libraries(find_forward_dir
        PRIVATE
            hal
            motor
            oled
            sw
            led
        )