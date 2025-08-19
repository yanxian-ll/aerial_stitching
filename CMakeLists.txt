cmake_minimum_required(VERSION 3.15)
project(fastcorrector CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
string(APPEND CMAKE_CXX_FLAGS_RELEASE " -O3")

# 添加开关，默认开启 debug 日志
option(ENABLE_DEBUG_LOG "Enable debug log messages" ON)

if(ENABLE_DEBUG_LOG)
    add_definitions(-DENABLE_DEBUG_LOG)
endif()


add_executable(fastcorrector
    main.cpp
    seamfinder.cpp
    functions.cpp
    utils.cpp
    metadata.cpp
    debug_utils.cpp
)

# ---- 依赖 ----
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED IMPORTED_TARGET opencv4)
pkg_check_modules(EXIV2  REQUIRED IMPORTED_TARGET exiv2)
pkg_check_modules(PROJ   REQUIRED IMPORTED_TARGET proj)
pkg_check_modules(GDAL   REQUIRED IMPORTED_TARGET gdal)

# AMQP-CPP（很多环境是装到 /usr/local）
find_path(AMQPCPP_INCLUDE_DIR amqpcpp.h
    PATHS /usr/local/include /usr/include
)
find_library(AMQPCPP_LIBRARY amqpcpp
    PATHS /usr/local/lib /usr/lib
)

if(NOT AMQPCPP_INCLUDE_DIR OR NOT AMQPCPP_LIBRARY)
    message(FATAL_ERROR "AMQP-CPP not found. Please install it to /usr/local or set AMQPCPP_INCLUDE_DIR/AMQPCPP_LIBRARY.")
endif()

target_include_directories(fastcorrector PRIVATE
    ${AMQPCPP_INCLUDE_DIR}
    /usr/local/include          
)

target_link_directories(fastcorrector PRIVATE
    /usr/local/lib              
)

target_link_libraries(fastcorrector PRIVATE
    ${AMQPCPP_LIBRARY}
    Boost::system
    Threads::Threads
    PkgConfig::OPENCV
    PkgConfig::EXIV2
    PkgConfig::PROJ
    PkgConfig::GDAL
)

install(TARGETS fastcorrector RUNTIME DESTINATION bin)
