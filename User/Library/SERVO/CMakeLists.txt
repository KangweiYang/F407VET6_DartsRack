add_library(servo STATIC)

target_sources(servo
        PRIVATE
        Src/Servo.c
        )

target_include_directories(servo
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        )

target_link_libraries(servo
        PRIVATE
            hal
        )