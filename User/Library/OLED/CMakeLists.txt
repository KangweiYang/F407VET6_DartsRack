add_library(oled STATIC Src/UserOLED.c Inc/UserOLED.h)

target_sources(oled
        PRIVATE
        Src/OLED.c
        Src/OLED_Font.c
        Src/UserOLED.c
        )

target_include_directories(oled
        INTERFACE
        ./Inc
        )

target_link_libraries(motor
        PRIVATE
            hal
            find_forward_dir
        )