cmake_minimum_required(VERSION 3.13)
project(tinycandle C ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(
    -std=gnu11 -mthumb
    -Wall -Wextra -Werror
    -Wstrict-prototypes
    -fmessage-length=0
    -fsigned-char
    -ffunction-sections -fdata-sections
    -ffreestanding
    -fno-move-loop-invariants
    -Os -g3
    -flto -ffat-lto-objects
    --specs=nano.specs
    --specs=nosys.specs
)

#need these later, per-platform
set(CPUFLAGS_F0 -mcpu=cortex-m0)
set(CPUFLAGS_F4 -mcpu=cortex-m4)

add_link_options(
    -mthumb -Wall -Wextra -g3
    -Xlinker --gc-sections
    --specs=nano.specs
    --specs=nosys.specs
)

add_subdirectory(libs/STM32_HAL)


SET(TINY_USB
    libs/tinyusb/src/tusb_option.h
    libs/tinyusb/src/tusb.c
    libs/tinyusb/src/tusb.h

    libs/tinyusb/src/common/tusb_common.h
    libs/tinyusb/src/common/tusb_compiler.h
    libs/tinyusb/src/common/tusb_debug.h
    libs/tinyusb/src/common/tusb_fifo.c
    libs/tinyusb/src/common/tusb_fifo.h
    libs/tinyusb/src/common/tusb_mcu.h
    libs/tinyusb/src/common/tusb_private.h
    libs/tinyusb/src/common/tusb_timeout.h
    libs/tinyusb/src/common/tusb_types.h
    libs/tinyusb/src/common/tusb_verify.h

    libs/tinyusb/src/device/dcd.h
    libs/tinyusb/src/device/usbd_control.c
    libs/tinyusb/src/device/usbd_pvt.h
    libs/tinyusb/src/device/usbd.c
    libs/tinyusb/src/device/usbd.h

    libs/tinyusb/src/class/cdc/cdc_device.c
    libs/tinyusb/src/class/cdc/cdc_device.h
    libs/tinyusb/src/class/cdc/cdc_host.c
    libs/tinyusb/src/class/cdc/cdc_host.h
    libs/tinyusb/src/class/cdc/cdc.h

    libs/tinyusb/src/class/vendor/vendor_device.c
    libs/tinyusb/src/class/vendor/vendor_device.h
)

SET(TINY_USB_F0
    libs/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev_pvt_st.h
    libs/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
)

SET(TINY_USB_F4
    libs/tinyusb/src/portable/st/synopsys/dcd_synopsys.c
    libs/tinyusb/src/portable/st/synopsys/synopsys_common.h
)

add_library(tinyusb_f0 ${TINY_USB} ${TINY_USB_F0})
target_compile_definitions(tinyusb_f0 PUBLIC CFG_TUSB_MCU=OPT_MCU_STM32F0)
target_link_libraries(tinyusb_f0 PRIVATE STM32_HAL_STM32F072xB)
target_include_directories(tinyusb_f0 PUBLIC libs/tinyusb/src include)
target_compile_options(tinyusb_f0 PRIVATE ${CPUFLAGS_F0})

add_library(tinyusb_f4 ${TINY_USB} ${TINY_USB_F4})
target_compile_definitions(tinyusb_f4 PUBLIC CFG_TUSB_MCU=OPT_MCU_STM32F4)
target_link_libraries(tinyusb_f4 PRIVATE STM32_HAL_STM32F407xE)
target_include_directories(tinyusb_f4 PUBLIC libs/tinyusb/src include)
target_compile_options(tinyusb_f4 PRIVATE ${CPUFLAGS_F4})

# Add a custom target that produces version.h, plus
# a dummy output that's not actually produced, in order
# to force version.hmake to always be re-run before the build


add_custom_target(version_h BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/version.h"
	COMMAND ${CMAKE_COMMAND}
		-D SRCDIR="${CMAKE_CURRENT_SOURCE_DIR}"
		-P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gitversion.cmake
)

set(
    SOURCE_FILES
        include/config.h
        include/tusb_config.h src/usb_descriptors.c

        include/gs_usb.h src/gs_usb.c

        include/can.h src/can.c
        include/dfu.h src/dfu.c
        include/flash.h src/flash.c
        include/gpio.h src/gpio.c
        include/led.h src/led.c
        include/queue.h src/queue.c
        include/timer.h src/timer.c
        include/util.h src/util.c

        include/neo_pixel.h src/neo_pixel.c

        src/startup.c
        src/main.c
        src/interrupts.c

        ${CMAKE_CURRENT_BINARY_DIR}/version.h
)


####### some helpers to generate targets

## objcopy to produce .bin file
function(make_bin_file target)
    add_custom_command(
        TARGET ${target} POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        BYPRODUCTS ${target}.bin
        COMMAND ${CMAKE_OBJCOPY} -O binary ${target} ${target}.bin
    )
endfunction()

## report size
function(show_object_size target)
    string(REPLACE "objcopy" "size" CMAKE_OBJSIZE "${CMAKE_OBJCOPY}")
    add_custom_command(
        TARGET ${target} POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_OBJSIZE} ${target}
    )
endfunction()

find_package(DFUSuffix)
## run dfu-suffix to append DFU stuff and signature; generate relevant flash-* target
# TODO ? : run with execute_proces(... OUTPUT_QUIET ) instead of '... 1>/dev/null'

function(dfu_flash target)
	if (DFU_SUFFIX_EXECUTABLE)
		add_custom_command( TARGET ${target}
			BYPRODUCTS ${target}.dfu
			COMMAND ${CMAKE_OBJCOPY} -O binary ${target} ${target}.dfu
			COMMAND ${DFU_SUFFIX_EXECUTABLE} --add ${target}.dfu --vid 1d50 --pid 606f 1>/dev/null
			COMMENT "create and sign dfu bin file: ${TGTNAME}_fw"
		)
		add_custom_target( flash-${target}
			dfu-util -a 0 -R -s 0x08000000 -D ${target}.dfu
		)
	else()
		add_custom_target( flash-${target}
			dfu-util -d 1d50:606f -a 0 -R -s 0x08000000 -D ${target}.bin
			)
	endif()
endfunction()


######### commands for adding each target have a lot in common: make helper func.
# Split into two categories, F042-based and F072-based.

function(add_f042_target TGTNAME)
	add_executable(${TGTNAME}_fw ${SOURCE_FILES})
	add_dependencies(${TGTNAME}_fw version_h)
	target_include_directories(${TGTNAME}_fw PRIVATE libs/STM32_HAL/config/ include/ ${CMAKE_CURRENT_BINARY_DIR})
    target_compile_options(${TGTNAME}_fw BEFORE PRIVATE ${CPUFLAGS_F0})
    target_compile_definitions(${TGTNAME}_fw PRIVATE BOARD_${TGTNAME} STM32F0)
    target_link_options(${TGTNAME}_fw BEFORE PRIVATE ${CPUFLAGS_F0})
	target_link_options(${TGTNAME}_fw PRIVATE -T ${CMAKE_SOURCE_DIR}/ldscripts/STM32F042X6.ld)
	target_link_libraries(${TGTNAME}_fw PRIVATE STM32_HAL_STM32F042x6 tinyusb_f0)
    make_bin_file(${TGTNAME}_fw)
    dfu_flash(${TGTNAME}_fw)
    show_object_size(${TGTNAME}_fw)
endfunction()

function(add_f072_target TGTNAME)
	add_executable(${TGTNAME}_fw ${SOURCE_FILES})
	add_dependencies(${TGTNAME}_fw version_h)
	target_include_directories(${TGTNAME}_fw PRIVATE libs/STM32_HAL/config/ include/ ${CMAKE_CURRENT_BINARY_DIR})
    target_compile_options(${TGTNAME}_fw BEFORE PRIVATE ${CPUFLAGS_F0})
    target_compile_definitions(${TGTNAME}_fw PRIVATE BOARD_${TGTNAME} STM32F0)
    target_link_options(${TGTNAME}_fw BEFORE PRIVATE ${CPUFLAGS_F0})
	target_link_options(${TGTNAME}_fw PRIVATE -T ${CMAKE_SOURCE_DIR}/ldscripts/STM32F072XB.ld)
	target_link_libraries(${TGTNAME}_fw PRIVATE STM32_HAL_STM32F072xB tinyusb_f0)
    make_bin_file(${TGTNAME}_fw)
    dfu_flash(${TGTNAME}_fw)
    show_object_size(${TGTNAME}_fw)
endfunction()

function(add_f407_target TGTNAME)
	add_executable(${TGTNAME}_fw ${SOURCE_FILES})
	add_dependencies(${TGTNAME}_fw version_h)
	target_include_directories(${TGTNAME}_fw PRIVATE libs/STM32_HAL/config/ include/ ${CMAKE_CURRENT_BINARY_DIR})
    target_compile_definitions(${TGTNAME}_fw PRIVATE BOARD_${TGTNAME} STM32F4)
    target_compile_options(${TGTNAME}_fw BEFORE PRIVATE ${CPUFLAGS_F4})
    target_link_options(${TGTNAME}_fw BEFORE PRIVATE ${CPUFLAGS_F4})
    target_link_options(${TGTNAME}_fw PRIVATE -T ${CMAKE_SOURCE_DIR}/ldscripts/STM32F407XE.ld)
    target_link_libraries(${TGTNAME}_fw PRIVATE STM32_HAL_STM32F407xE tinyusb_f4)
    make_bin_file(${TGTNAME}_fw)
    dfu_flash(${TGTNAME}_fw)
    show_object_size(${TGTNAME}_fw)
endfunction()

########## generate list of targets.
# the "_fw" part is appended automatically
#set(TGT042_LIST "cantact" "canalyze" "canable" "usb2can" "cannette")
#set(TGT072_LIST "candleLight" "CANable_MKS")
set(TGT407_LIST "STM32F4_DevBoard")
set(TGT072_LIST "wmc_usb_can" "wmc_usb_can_standalone")

foreach (TGTNAME IN LISTS TGT042_LIST)
	option(BUILD_${TGTNAME} "Build firmware for \"${TGTNAME}\" (default=yes)" ON)
	if (BUILD_${TGTNAME})
		add_f042_target(${TGTNAME})
	endif()
endforeach()

foreach (TGTNAME IN LISTS TGT072_LIST)
	option(BUILD_${TGTNAME} "Build firmware for \"${TGTNAME}\" (default=yes)" ON)
	if (BUILD_${TGTNAME})
		add_f072_target(${TGTNAME})
	endif()
endforeach()

foreach (TGTNAME IN LISTS TGT407_LIST)
	option(BUILD_${TGTNAME} "Build firmware for \"${TGTNAME}\" (default=yes)" ON)
	if (BUILD_${TGTNAME})
		add_f407_target(${TGTNAME})
	endif()
endforeach()

message("*******************")
message("You may now:\n\t-compile all targets ('make')\n\t-compile a single target (e.g. 'make cantact_fw'")
message("\t-flash a device (e.g. 'make flash-cantact_fw'")
