add_library(rs485 STATIC)

target_sources(rs485
        PRIVATE
        Src/RS485.c
        )

target_include_directories(rs485
        INTERFACE
        ./Inc
        )

target_link_libraries(rs485
        PRIVATE
            hal
        )