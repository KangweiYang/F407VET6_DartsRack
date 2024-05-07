add_library(sw STATIC Src/SW.c Inc/SW.h)

target_sources(sw
        PRIVATE
        Src/SW.c
        )

target_include_directories(sw
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        )

target_link_libraries(sw
        PRIVATE
            hal
        )