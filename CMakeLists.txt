cmake_minimum_required(VERSION 3.10)
project(FrequencyTrackedHarmonicSuppression C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(include)

file(GLOB SOURCES "src/*.c")

add_executable(harmonic_suppressor ${SOURCES})

if(UNIX)
    target_link_libraries(harmonic_suppressor m)
endif()
