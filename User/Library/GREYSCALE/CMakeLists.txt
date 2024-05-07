add_library(greyscale STATIC)

target_sources(greyscale
        PRIVATE
        Src/Greyscale.c
        )

target_include_directories(greyscale
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}
        )

target_link_libraries(greyscale
        PRIVATE
            hal
        )