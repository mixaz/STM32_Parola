# Example of cmake script of your project root,
# copy this file to project root folder, change as necessary.
# Expected that it is stm32f1-ll subfolder of git submodule

cmake_minimum_required(VERSION 3.0)
project(stm32parola)

# change this to your stm32f1-ll subfolder location, usually stm32f1-ll
set(STM32F1_LL ${PROJECT_SOURCE_DIR}/stm32f1-ll)
#set(STM32F1_LL ${CMAKE_CURRENT_SOURCE_DIR}/..)
message("STM32F1_LL: ${STM32F1_LL}")

set(ELF ${PROJECT_NAME}.elf)

enable_language(ASM)

set(MAX72XXSRC1 src/main.c)

# stm32 CMIS files
set(CMIS_SOURCE_FILES
  ${STM32F1_LL}/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103c8tx.s
  ${STM32F1_LL}/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c
  ${STM32F1_LL}/CMSIS/Device/ST/STM32F1xx/Source/Templates/sysmem.c)

#  ${STM32F1_LL}/CMSIS/Device/ST/STM32F1xx/Source/Templates/stm32f1xx_it.c)

# MD_MAX77XX files
set(MAX77XX_SOURCE_FILES
  MD_MAX72XX/src/MD_MAX72xx_buf.c
  MD_MAX72XX/src/MD_MAX72xx.c
  MD_MAX72XX/src/MD_MAX72xx_1.c
  MD_MAX72XX/src/MD_MAX72XX_platform.c
  MD_MAX72XX/src/MD_MAX72xx_font.c
  MD_MAX72XX/src/MD_MAX72xx_pix.c)

# MD_Parola files
set(PAROLA_SOURCE_FILES
        MD_Parola/src/MD_Parola.c
        MD_Parola/src/MD_Parola_1.c
        MD_Parola/src/MD_Parola_Blinds.c
        MD_Parola/src/MD_Parola_Close.c
        MD_Parola/src/MD_Parola_Diag.c
        MD_Parola/src/MD_Parola_Dissolve.c
        MD_Parola/src/MD_Parola_Fade.c
        MD_Parola/src/MD_Parola_Grow.c
        MD_Parola/src/MD_Parola_HScroll.c
        MD_Parola/src/MD_Parola_Mesh.c
        MD_Parola/src/MD_Parola_Open.c
        MD_Parola/src/MD_Parola_Print.c
        MD_Parola/src/MD_Parola_Random.c
        MD_Parola/src/MD_Parola_Scan.c
        MD_Parola/src/MD_Parola_Slice.c
        MD_Parola/src/MD_Parola_Sprite.c
        MD_Parola/src/MD_Parola_VScroll.c
        MD_Parola/src/MD_Parola_Wipe.c
        MD_Parola/src/MD_PZone.c
        MD_Parola/src/MD_Parola_platform.c)

message("MAX77XX_SOURCE_FILES: ${MAX77XX_SOURCE_FILES}")
message("ELF: ${ELF}")

add_executable(${ELF}
        ${CMIS_SOURCE_FILES}
        ${PAROLA_SOURCE_FILES}
        ${MAX77XX_SOURCE_FILES}
        src/main_parola.c)

#        ${STM32F1_LL}/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103c8tx.s
#        ${STM32F1_LL}/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c
#        ${STM32F1_LL}/CMSIS/Device/ST/STM32F1xx/Source/Templates/sysmem.c
#        src/main.c)

target_include_directories(${ELF} PUBLIC MD_MAX72XX/src)
target_include_directories(${ELF} PUBLIC ./src)
target_include_directories(${ELF} PUBLIC MD_Parola/src)
target_include_directories(${ELF} PUBLIC ${STM32F1_LL}/inc)

# stm32f1-ll library
add_subdirectory(${STM32F1_LL} out_ll)
target_link_libraries(${ELF} ll)

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
  -Og
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

#file(CONFIGURE OUTPUT openocd.cfg CONTENT "
#source [find interface/stlink.cfg]
#transport select hla_swd
#source [find target/stm32f1x.cfg]
#program ${PROJECT_NAME}.elf verify reset exit")

