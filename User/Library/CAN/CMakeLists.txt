add_library(can STATIC)

target_sources(can
        PRIVATE
        Src/Can.c
        )

target_include_directories(can
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        )

target_link_libraries(can
        PRIVATE
            hal
        )