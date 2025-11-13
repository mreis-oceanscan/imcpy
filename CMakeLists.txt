cmake_minimum_required(VERSION 3.8)

# Set compatibility policy to prevent warnings about older CMake versions in submodules
if(POLICY CMP0114)
  cmake_policy(SET CMP0114 NEW)
endif()

SET(PROJ_NAME _imcpy)
project(${PROJ_NAME})

# Set CXX standard to 17
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(pybind11)

#######################################################
##                     DUNE                          ##
#######################################################

# Embed DUNE into build, but only build dune-core
add_subdirectory(dune EXCLUDE_FROM_ALL)
include_directories(dune/src/)
include_directories(${CMAKE_BINARY_DIR}/DUNEGeneratedFiles/src)  # Contains generated Config.hpp/Version.cpp files

# Add DUNE vendor library includes (especially for json.hpp)
include_directories(dune/vendor/libraries/json)
include_directories(dune/vendor/libraries/bzip2)
include_directories(dune/vendor/libraries/date)
include_directories(dune/vendor/libraries/md5)
include_directories(dune/vendor/libraries/pstream)
include_directories(dune/vendor/libraries/rapidxml)
include_directories(dune/vendor/libraries/sqlite3)
include_directories(dune/vendor/libraries/stb)
include_directories(dune/vendor/libraries/wmm2015)
include_directories(dune/vendor/libraries/zlib)
include_directories(dune/vendor/libraries/API500)
include_directories(dune/vendor/libraries/API500/asio/include)
include_directories(dune/vendor/libraries/API500/rapidjson/include)

if(MSVC)
    include_directories(dune/vendor/libraries/pthreads-win32)
endif()

set_property(TARGET dune-core PROPERTY POSITION_INDEPENDENT_CODE ON)  # Enable -fPIC on dune-core build

#######################################################
##                      IMC                          ##
#######################################################

# Generate IMC definitions from spec
add_dependencies(dune-core imc)


#######################################################
##                   Source/Libs                     ##
#######################################################

# Find all source files starting with pb
FILE(GLOB_RECURSE CPP_SRC src/imcpy_bindings pb*.cpp)

# Use THIN_LTO if available (Clang/LLVM)
pybind11_add_module(${PROJ_NAME} MODULE THIN_LTO src/imcpy_bindings/imcpy.cpp ${CPP_SRC})

# Link with dune-core
if(MSVC)
    target_link_libraries(${PROJ_NAME} PRIVATE pybind11::module dune-core ws2_32)
else()
    target_link_libraries(${PROJ_NAME} PRIVATE pybind11::module dune-core)
endif()
