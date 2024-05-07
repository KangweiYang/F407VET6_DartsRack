add_library(shoot_process STATIC)

target_sources(shoot_process
        PRIVATE
        Src/ShootProcess.c
        )

target_include_directories(shoot_process
        INTERFACE
        ./Inc
        )

target_link_libraries(shoot_process
        PRIVATE
            hal
        )