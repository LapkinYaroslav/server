cmake_minimum_required(VERSION 3.5)
project(robot_control)

# Установка стандартов языка
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# Поиск необходимых пакетов
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 gstreamer-rtp-1.0 gstreamer-plugins-bad-1.0)

# Поиск OpenCV
find_package(OpenCV REQUIRED)

# Поиск Qt5 или Qt6
find_package(Qt6 COMPONENTS Core Widgets Network REQUIRED)
if(NOT Qt6_FOUND)
    find_package(Qt5 REQUIRED COMPONENTS Core Widgets Network)
endif()

# Включение автоматической обработки MOC, UIC и RCC файлов
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Создание исполняемых файлов
add_executable(server 
    ${CMAKE_SOURCE_DIR}/server.c
)

add_executable(client 
    ${CMAKE_SOURCE_DIR}/client.cpp
)

# Линковка библиотек для сервера
target_link_libraries(server 
    ${GSTREAMER_LIBRARIES}
    pthread
    m
)

target_include_directories(server 
    PRIVATE 
    ${GSTREAMER_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

# Линковка библиотек для клиента
if(Qt6_FOUND)
    target_link_libraries(client 
        Qt6::Core 
        Qt6::Widgets 
        Qt6::Network
        ${OpenCV_LIBS}
    )
else()
    target_link_libraries(client 
        Qt5::Core 
        Qt5::Widgets 
        Qt5::Network
        ${OpenCV_LIBS}
    )
endif()

target_include_directories(client 
    PRIVATE 
    ${OpenCV_INCLUDE_DIRS}
)

# Добавление опций компиляции
target_compile_options(server PRIVATE ${GSTREAMER_CFLAGS_OTHER})
target_compile_options(client PRIVATE)

# Установка путей к библиотекам
target_link_directories(server PRIVATE ${GSTREAMER_LIBRARY_DIRS})
