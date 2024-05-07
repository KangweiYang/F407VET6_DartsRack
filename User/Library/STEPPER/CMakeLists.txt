add_library(stepper STATIC)

target_sources(stepper
        PRIVATE
        Src/Stepper.c
        )

target_include_directories(stepper
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        )

target_link_libraries(stepper
        PRIVATE
            hal
        )