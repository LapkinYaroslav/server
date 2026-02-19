cmake_minimum_required(VERSION 3.10)
project(client)

# Установка путей для MSYS2
set(MSYS2_PREFIX "H:/msys2/mingw64")  # или "C:/msys64/ucrt64" в зависимости от вашего окружения

# Настройка компилятора
set(CMAKE_C_COMPILER "${MSYS2_PREFIX}/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "${MSYS2_PREFIX}/bin/g++.exe")

set(CMAKE_GENERATOR "MSYS Makefiles" CACHE INTERNAL "")

# Настройка компилятора (MSVC)
set(CMAKE_CXX_STANDARD 17)  # GStreamer и Qt 5/6 требуют C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Поиск Qt6
find_package(Qt5 COMPONENTS Core Widgets OpenGL REQUIRED)

# Поиск GStreamer через pkg-config
# Пути для pkg-config
set(ENV{PKG_CONFIG_PATH} "${MSYS2_PREFIX}/lib/pkgconfig")
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-rtp-1.0 gstreamer-app-1.0)

# Исполняемый файл
add_executable(client
    ${CMAKE_SOURCE_DIR}/main.cpp
    ${CMAKE_SOURCE_DIR}/src/client.cpp
    ${CMAKE_SOURCE_DIR}/src/video_widget.cpp
    ${CMAKE_SOURCE_DIR}/src/main_window.cpp
    ${CMAKE_SOURCE_DIR}/include/video_widget.h
    ${CMAKE_SOURCE_DIR}/include/main_window.h
    ${CMAKE_SOURCE_DIR}/include/client.h
)

# Подключение библиотек
target_link_libraries(client
    PRIVATE
    Qt5::Core
    Qt5::Widgets
    Qt5::OpenGL
    ${GSTREAMER_LIBRARIES}
)

# Подключение include-директорий
target_include_directories(client PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${GSTREAMER_INCLUDE_DIRS}
    ${Qt5_INCLUDE_DIRS}
)

# Флаги компиляции из pkg-config
target_compile_options(client PRIVATE ${GSTREAMER_CFLAGS_OTHER})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)