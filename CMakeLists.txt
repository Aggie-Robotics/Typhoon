project(Worlds)
cmake_minimum_required(VERSION 3.11)

add_library(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
