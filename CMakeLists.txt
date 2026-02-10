# --- MANDATORY MAC CROSS-COMPILE OVERRIDE ---
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# Force skip the compiler checks that trigger the -arch arm64 error
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_ASM_COMPILER_WORKS 1)

# Prevent Mac SDK injection
set(CMAKE_OSX_SYSROOT "")
set(CMAKE_OSX_ARCHITECTURES "")
# --------------------------------------------

cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME H5_Cache_Benchmark)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})

# --- GLOBAL SETTINGS ---
# Force Chip Identity Globally
add_compile_definitions(
    STM32H533xx
    USE_HAL_DRIVER
    USE_NUCLEO_64
)

# Force CPU Architecture Globally
add_compile_options(
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -Wall
    -fdata-sections
    -ffunction-sections
)

# Linker Options
add_link_options(
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    -Wl,--gc-sections
    -lc
    -lm
    # REMOVED: -lnosys (This caused the multiple definition error)
    
    # LINKER SCRIPT: Matches your confirmed filename
    -T "${CMAKE_CURRENT_SOURCE_DIR}/STM32H533xx_FLASH.ld" 
)
# -----------------------

message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
# We ONLY link the CubeMX sources, libc, and math lib.
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    -lc
    -lm
)