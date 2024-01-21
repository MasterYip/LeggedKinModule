cmake_minimum_required(VERSION 3.0.2)

project(LeggedKinModule)

set(CMAKE_CXX_STANDARD 11)

find_package(pinocchio REQUIRED)
# pkg_check_modules(pinocchio REQUIRED pinocchio)
set(IKFAST_LIB_NAME ikfast_leg_lib)
set(IKFAST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config/${IKFAST_LIB_NAME})
set(BUILD_GTESTING OFF)

file(GLOB_RECURSE SOURCES src/*.cpp)

add_subdirectory(${IKFAST_DIR})

add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC inc ${pinocchio_INCLUDE_DIRS} ${IKFAST_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio ${IKFAST_LIB_NAME})

# Tests
file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_include_directories(${TEST_NAME} PUBLIC inc ${pinocchio_INCLUDE_DIRS} ${IKFAST_DIR})
    target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME} ${IKFAST_LIB_NAME})
endforeach()


if(BUILD_GTESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  add_executable(${PROJECT_NAME}_test ${TEST_SOURCES})
  target_include_directories(${PROJECT_NAME}_test PUBLIC inc ${pinocchio_INCLUDE_DIRS} ${IKFAST_DIR})
  target_link_libraries(${PROJECT_NAME}_test PUBLIC ${PROJECT_NAME} GTest::GTest GTest::Main ${IKFAST_LIB_NAME})
  gtest_discover_tests(${PROJECT_NAME}_test)
endif()