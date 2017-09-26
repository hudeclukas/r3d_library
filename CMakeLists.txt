cmake_minimum_required(VERSION 3.0)

project(R3DSolution)

set(R3D_source_dir ${CMAKE_SOURCE_DIR})
set(R3D_binary_dir ${CMAKE_BINARY_DIR})

set(R3DLibraryDir "${R3D_source_dir}3DLibrary")
set(DataHandlingDir "${R3D_source_dir}DataHandling")
set(ExporterDir "${R3D_source_dir}Exporter")
set(R3DGuiDir "${R3D_source_dir}R3DGui")
set(VisualizationDir "${R3D_source_dir}Visualization")

add_subdirectory(${R3DLibraryDir})
add_subdirectory(${DataHandlingDir})
add_subdirectory(${ExporterDir})
add_subdirectory(${R3DGuiDir})
add_subdirectory(${VisualizationDir})

if(CMAKE_CONFIGURATION_TYPES)
    message("Preset configuration - Debug, Release - only.")
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Build configuration types" FORCE)
endif()
