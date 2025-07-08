add_executable(${PROJECT_NAME}.elf ${LINKER_SCRIPT} Library/DOU_PID/Inc/Double_PID.h Library/DOU_PID/Src/Double_PID.c Library/INC_PI/Inc/Inc_PI.h Library/INC_PI/Src/Inc_PI.c Library/MOTOR/Inc/MotorUnit.h Library/MOTOR/Src/MotorUnit.c Library/SERVO/Src/Servo.c Library/SERVO/Inc/Servo.h Library/STEPPER/Src/Stepper.c Library/STEPPER/Inc/Stepper.h Library/CAN/Src/Can.c Library/CAN/Inc/Can.h Library/RS485/Src/RS485.c Library/RS485/Inc/RS485.h
        Library/SHOOT_PROCESS/Src/ShootProcess.c
        Library/SHOOT_PROCESS/Inc/ShootProcess.h)
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
        can
        adc_dma
        dou_pid
        motor
        servo
        stepper
        shoot_process
        remote
        )

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")