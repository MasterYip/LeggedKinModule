cmake_minimum_required(VERSION 3.0.2)

project(LeggedKinModule 
    LANGUAGES CXX
    VERSION 1.0.0)

# Settings
set(SHARED_VERSION ${PROJECT_VERSION_MAJOR}) # Only major will affect the shared library version
set(LIB_TYPE SHARED) # SHARED or STATIC
set(BUILD_GTESTING OFF)

set(CMAKE_CXX_STANDARD 11)

find_package(pinocchio 2.5.1 REQUIRED)
message(STATUS "Pinocchio version: ${pinocchio_VERSION}")

# get folder name of ikfast library (in config/)
file(GLOB CONFIG_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/config/*)
foreach(CONFIG_DIR ${CONFIG_DIRS})
    get_filename_component(CONFIG_DIR_EXT ${CONFIG_DIR} EXT)
    if (CONFIG_DIR_EXT STREQUAL ".urdf")
        set(URDF_PATH ${CONFIG_DIR})
    endif()
    if (CONFIG_DIR_EXT STREQUAL "")
        get_filename_component(IKFAST_LIB_NAME ${CONFIG_DIR} NAME)
    endif()
endforeach()

message(STATUS "IKFAST_LIB_NAME: ${IKFAST_LIB_NAME} | URDF_PATH: ${URDF_PATH}")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/core/config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/core/config.h)
set(IKFAST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config/${IKFAST_LIB_NAME})
file(GLOB_RECURSE SOURCES core/*.cpp)
list(APPEND INCLUDES 
    ${CMAKE_CURRENT_SOURCE_DIR}/core 
    ${CMAKE_CURRENT_SOURCE_DIR}/config 
    ${pinocchio_INCLUDE_DIRS} 
    ${IKFAST_DIR})

add_subdirectory(${IKFAST_DIR})
add_library(${PROJECT_NAME} ${LIB_TYPE} ${SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${SHARED_VERSION}
)
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${INCLUDES}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/config>
)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio ${IKFAST_LIB_NAME})

# install(TARGETS ${PROJECT_NAME}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Tests
file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_include_directories(${TEST_NAME} PUBLIC ${INCLUDES})
    target_link_libraries(${TEST_NAME} PUBLIC ${PROJECT_NAME})
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
