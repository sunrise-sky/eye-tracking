cmake_minimum_required(VERSION 3.16)

# =========================================================================
# 工具链（交叉编译：ARM Cortex-M4）
# =========================================================================
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# 若在主机上进行单元测试编译，可通过 -DHOST_TEST=ON 跳过工具链设置
if(NOT HOST_TEST)
    set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
    set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
    set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
    set(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
    set(CMAKE_SIZE         arm-none-eabi-size)

    set(CMAKE_EXE_LINKER_FLAGS
        "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
         -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections \
         -Wl,-Map=${PROJECT_NAME}.map,--cref"
        CACHE INTERNAL ""
    )
endif()

# =========================================================================
# 项目定义
# =========================================================================
project(eye_tracking_fw
    VERSION 1.0.0
    DESCRIPTION "嵌入式眼动追踪系统固件 (STM32F407)"
    LANGUAGES C ASM
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# =========================================================================
# 编译选项
# =========================================================================
if(NOT HOST_TEST)
    set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
    set(COMMON_FLAGS
        "${MCU_FLAGS} \
         -Wall -Wextra -Wpedantic \
         -Wno-unused-parameter \
         -fdata-sections -ffunction-sections \
         -fno-common"
    )
    set(CMAKE_C_FLAGS_DEBUG   "${COMMON_FLAGS} -Og -g3 -DDEBUG")
    set(CMAKE_C_FLAGS_RELEASE "${COMMON_FLAGS} -O2 -DNDEBUG")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# =========================================================================
# 源文件列表
# =========================================================================
set(FW_SOURCES
    src/main.c
    src/camera/ov7670.c
    src/image_proc/image_proc.c
    src/eye_tracking/eye_tracking.c
    src/hal/uart.c
)

# =========================================================================
# 包含目录
# =========================================================================
set(FW_INCLUDES
    include
    src
    src/camera
    src/image_proc
    src/eye_tracking
    src/hal
)

# =========================================================================
# 主固件目标
# =========================================================================
if(NOT HOST_TEST)
    add_executable(${PROJECT_NAME}.elf ${FW_SOURCES})

    target_include_directories(${PROJECT_NAME}.elf PRIVATE ${FW_INCLUDES})

    target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
        STM32F407xx
        USE_HAL_DRIVER
    )

    target_link_libraries(${PROJECT_NAME}.elf PRIVATE m)

    # 生成 .bin 和 .hex 烧录文件
    add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary
                ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex
                ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
        COMMAND ${CMAKE_SIZE}   ${PROJECT_NAME}.elf
        COMMENT "生成烧录文件 (.bin / .hex)"
    )
endif()

# =========================================================================
# 主机单元测试目标（-DHOST_TEST=ON）
# =========================================================================
if(HOST_TEST)
    enable_testing()

    # 图像处理模块测试
    add_executable(test_image_proc
        tests/test_image_proc.c
        src/image_proc/image_proc.c
    )
    target_include_directories(test_image_proc PRIVATE ${FW_INCLUDES} tests)
    target_compile_definitions(test_image_proc PRIVATE HOST_TEST)
    target_compile_options(test_image_proc PRIVATE -Wall -Wextra -O0 -g)
    target_link_libraries(test_image_proc PRIVATE m)
    add_test(NAME image_proc_tests COMMAND test_image_proc)

    # 眼动追踪算法模块测试
    add_executable(test_eye_tracking
        tests/test_eye_tracking.c
        src/image_proc/image_proc.c
        src/eye_tracking/eye_tracking.c
    )
    target_include_directories(test_eye_tracking PRIVATE ${FW_INCLUDES} tests)
    target_compile_definitions(test_eye_tracking PRIVATE HOST_TEST)
    target_compile_options(test_eye_tracking PRIVATE -Wall -Wextra -O0 -g)
    target_link_libraries(test_eye_tracking PRIVATE m)
    add_test(NAME eye_tracking_tests COMMAND test_eye_tracking)
endif()
