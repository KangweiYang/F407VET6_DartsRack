add_library(motor STATIC)

target_sources(motor
        PRIVATE
        Src/MotorUnit.c
        )

target_include_directories(motor
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}
        )

target_link_libraries(motor
        PRIVATE
            hal
            inc_pi
        )