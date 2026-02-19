cmake_minimum_required(VERSION 3.5)
project(server)

# Установка путей для MSYS2
set(MSYS2_PREFIX "H:/msys2/mingw64")  # или "C:/msys64/ucrt64" в зависимости от вашего окружения

# Настройка компилятора
set(CMAKE_C_COMPILER "${MSYS2_PREFIX}/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "${MSYS2_PREFIX}/bin/g++.exe")

set(CMAKE_GENERATOR "MinGW Makefiles" CACHE INTERNAL "")

set(CMAKE_CXX_STANDARD 17)

# Пути для pkg-config
set(ENV{PKG_CONFIG_PATH} "${MSYS2_PREFIX}/lib/pkgconfig")
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 gstreamer-rtp-1.0 gstreamer-plugins-bad-1.0)

add_executable(server 
    # ${CMAKE_SOURCE_DIR}/src/server.cpp
    # ${CMAKE_SOURCE_DIR}/main.cpp
    ${CMAKE_SOURCE_DIR}/server.c
)
target_link_libraries(server ${GSTREAMER_LIBRARIES})
target_include_directories(server 
    PRIVATE 
    ${GSTREAMER_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

# Добавление путей к библиотекам
target_compile_options(server PRIVATE -I"${MSYS2_PREFIX}/include")
target_link_directories(server PRIVATE "${MSYS2_PREFIX}/lib")
