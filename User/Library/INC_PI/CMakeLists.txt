add_library(inc_pi STATIC)

target_sources(inc_pi
        PRIVATE
        Src/Inc_PI.c
        )

target_include_directories(inc_pi
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        )

target_link_libraries(inc_pi
        PRIVATE
            hal
            motor
        )