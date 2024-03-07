add_executable(${PROJECT_NAME}.elf ${LINKER_SCRIPT} Library/SERVO/Src/Servo.c Library/SERVO/Inc/Servo.h Library/STEPPER/Src/Stepper.c Library/STEPPER/Inc/Stepper.h)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Library/MOTOR/Inc)
add_subdirectory(Library)
add_subdirectory(Application)
target_sources(${PROJECT_NAME}.elf
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/main.c
        )

target_link_libraries(${PROJECT_NAME}.elf
        PRIVATE
        hal
        )

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")