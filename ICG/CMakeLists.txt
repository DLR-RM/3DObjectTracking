cmake_minimum_required(VERSION 3.5)

# =============================================================================
project(ICG)


# Options
# =============================================================================
option(USE_AZURE_KINECT "Use Azure Kinect" ON)
option(USE_REALSENSE "Use RealSense D435" ON)


# Libraries
# =============================================================================
if (WIN32)
    set(GLEW_STATIC true)
    set(OpenCV_STATIC true)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif ()

find_package(Eigen3 3.3.2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.1.2 REQUIRED)
find_package(OpenCV 4.0.0 REQUIRED)
if(USE_AZURE_KINECT)
    find_package(K4A 1.3.0 REQUIRED)
endif()
if (USE_REALSENSE)
    find_package(realsense2 REQUIRED)
endif ()

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    if (WIN32)
        set(CMAKE_CXX_FLAGS_RELEASE
                "${CMAKE_C_FLAGS_RELEASE} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS_RELEASE
                "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS
                "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    else ()
        set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS
                "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endIf ()
endif ()


# Compiler Settings
# =============================================================================

# Default settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-march HAS_MARCH)
check_cxx_compiler_flag(-mtune HAS_MTUNE)
if (HAS_MARCH)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif ()
if (HAS_MTUNE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native")
endif ()

# Debug settings
check_cxx_compiler_flag(-ggdb HAS_GGDB)
check_cxx_compiler_flag(-Z7 HAS_Z7)
check_cxx_compiler_flag(-ftrapv HAS_FTRAPV)
check_cxx_compiler_flag(-Od HAS_OD)
if (HAS_GGDB)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb")
elseif (HAS_Z7)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Z7")
endif ()
if (HAS_FTRAPV)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ftrapv")
endif ()
if (HAS_OD)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Od")
endif ()
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

# Release settings
check_cxx_compiler_flag(-Ob3 HAS_OB3)
check_cxx_compiler_flag(-O3 HAS_O3)
check_cxx_compiler_flag(-Ob2 HAS_OB2)
check_cxx_compiler_flag(-O2 HAS_O2)
if (HAS_OB3)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ob3")
elseif (HAS_O3)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
elseif (HAS_OB2)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ob2")
elseif (HAS_O2)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")

# OS settings
if (WIN32)
    if(OPENMP_FOUND)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -openmp:experimental")
    endif()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -MTd")
endif()


# Definitions
# =============================================================================
if (USE_AZURE_KINECT)
    add_definitions( -DUSE_AZURE_KINECT=${USE_AZURE_KINECT} )
endif ()
if (USE_REALSENSE)
    add_definitions( -DUSE_REALSENSE=${USE_REALSENSE} )
endif ()


# Directories
# =============================================================================

# Include Directories
include_directories("include")
include_directories("third_party")

# Sub Directories
add_subdirectory(src)
add_subdirectory(examples)
add_subdirectory(doc)
