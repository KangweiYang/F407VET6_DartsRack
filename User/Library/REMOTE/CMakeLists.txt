add_library(remote STATIC)

target_sources(remote
        PRIVATE
        Src/remote.c
        )

target_include_directories(remote
        INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}
        )

target_link_libraries(remote
        PRIVATE
            hal
        )