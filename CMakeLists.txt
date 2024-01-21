cmake_minimum_required(VERSION 3.0.2)

project(LeggedKinModule)

set(CMAKE_CXX_STANDARD 11)

find_package(pinocchio REQUIRED)
# pkg_check_modules(pinocchio REQUIRED pinocchio)

file(GLOB_RECURSE SOURCES src/*.cpp)

set(IKFAST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config/ikfast_leg_lib)

add_subdirectory(${IKFAST_DIR})

add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC inc ${pinocchio_INCLUDE_DIRS} ${IKFAST_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio ikfast_leg)