# Example of cmake script of your project root,
# copy this file to project root folder, change as necessary.
# Expected that it is stm32f1-ll subfolder of git submodule

cmake_minimum_required(VERSION 3.0)
project(max7219_ll)

# change this to your stm32f1-ll subfolder location, usually stm32f1-ll
set(STM32F1_LL ${PROJECT_SOURCE_DIR}/stm32f1-ll)
#set(STM32F1_LL ${CMAKE_CURRENT_SOURCE_DIR}/..)
message("STM32F1_LL: ${STM32F1_LL}")

set(ELF ${PROJECT_NAME}.elf)

enable_language(ASM)

add_subdirectory(${STM32F1_LL} out_ll)

add_executable(${ELF} 
  ${STM32F1_LL}/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103c8tx.s
  ${STM32F1_LL}/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c
  ${STM32F1_LL}/CMSIS/Device/ST/STM32F1xx/Source/Templates/sysmem.c
  MD_MAX72XX/src/MAX72xx_buf.c
  MD_MAX72XX/src/MAX72xx.c
  MD_MAX72XX/src/MAX72xx_font.c
  MD_MAX72XX/src/MAX72xx_pix.c
  MD_MAX72XX/src/main.c)

target_link_libraries(${ELF} ll)

target_include_directories(${ELF} PUBLIC MD_MAX72XX/src)

set_target_properties(
  ${ELF}
  PROPERTIES
  LINK_FLAGS
  "-T${STM32F1_LL}/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/linker/STM32F103C8TX_FLASH.ld \
   -mthumb -mcpu=cortex-m3 \
   -Wl,--gc-sections \
   -Wl,-Map=${PROJECT_NAME}.map")

target_compile_options(${ELF} PUBLIC
  -Wall 
  -g 
  -std=gnu99 
  -Os
  -mthumb
  -mcpu=cortex-m3
  -mfloat-abi=soft
  -mlittle-endian
  -ffunction-sections 
  -fdata-sections
  -Werror 
  -Wstrict-prototypes 
  -Warray-bounds 
  -fno-strict-aliasing 
  -Wno-unused-const-variable 
  -specs=nano.specs 
  -specs=nosys.specs)

file(CONFIGURE OUTPUT openocd.cfg CONTENT "
source [find interface/stlink.cfg]
transport select hla_swd
source [find target/stm32f1x.cfg]
program ${PROJECT_NAME}.elf verify reset exit")

