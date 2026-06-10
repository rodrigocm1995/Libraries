cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME NUCLEO-F303_HDC1080)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# ==============================================================================
# CONFIGURACIÓN DE SELECCIÓN DE ARCHIVO MAIN
# ==============================================================================
# Cambia esta variable según lo que quieras ejecutar:
# - "MAIN"  -> Compila el código original (Core/Src/main.c)
# - "TEST1" -> Compila tu código de pruebas (Core/Src/main_1.c)
set(ACTIVE_MAIN "TEST1")
if(ACTIVE_MAIN STREQUAL "MAIN")
    set(MAIN_SOURCE "Core/Src/main.c")
    message(STATUS "Compilando: MAIN PRINCIPAL (Core/Src/main.c)")
elseif(ACTIVE_MAIN STREQUAL "TEST1")
    set(MAIN_SOURCE "Core/Src/main_1.c")
    message(STATUS "Compilando: PRUEBA 1 (Core/Src/main_1.c)")
elseif(ACTIVE_MAIN STREQUAL "TEST2")
    set(MAIN_SOURCE "Core/Src/main_2.c")
    message(STATUS "Compilando: PRUEBA 2 (Core/Src/main_2.c)")
else()
    message(FATAL_ERROR "Selección de ACTIVE_MAIN inválida. Usa 'MAIN' o 'TEST1' o 'TEST2'.")
endif()
# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    ${MAIN_SOURCE})
# ==============================================================================

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/HDC1080/HDC1080.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/HDC1080
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Habilitar el soporte de punto flotante para printf/sprintf (newlib-nano)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)

# Registrar las fuentes inactivas en CMake solo para que IntelliSense las indexe
add_library(inactive_mains EXCLUDE_FROM_ALL
    Core/Src/main.c
    Core/Src/main_1.c
    Core/Src/main_2.c
)
target_link_libraries(inactive_mains PRIVATE stm32cubemx)
target_include_directories(inactive_mains PRIVATE Core/HDC1080)

