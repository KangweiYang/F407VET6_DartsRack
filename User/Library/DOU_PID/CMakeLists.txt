add_library(dou_pid STATIC)

target_sources(dou_pid
        PRIVATE
        Src/Double_PID.c
        )

target_include_directories(dou_pid
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        )

target_link_libraries(dou_pid
        PRIVATE
            hal
            inc_pi
        )