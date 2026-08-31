
# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Set target name
get_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_NAME)
get_alias_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_ALIAS_NAME)

# Set file collection
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB_RECURSE souce_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE test_files  ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cc ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)
list(REMOVE_ITEM souce_files ${test_files})
list(FILTER souce_files EXCLUDE REGEX ".*\\(copy\\).*")

# Add target
add_library(${CURRENT_TARGET_NAME} SHARED)
add_library(${CURRENT_TARGET_ALIAS_NAME} ALIAS ${CURRENT_TARGET_NAME})

# Set source file of target
target_sources(${CURRENT_TARGET_NAME} PRIVATE ${souce_files})

# Set head files of target
target_sources(${CURRENT_TARGET_NAME} INTERFACE ${head_files})

# Set include path of target
target_include_directories(${CURRENT_TARGET_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

find_path(ENGINEAI_MNN_INCLUDE_DIR
  NAMES MNN/Interpreter.hpp
  PATHS ${ENGINEAI_ROBOTICS_THIRD_PARTY}
  PATH_SUFFIXES include
  NO_DEFAULT_PATH)
find_library(ENGINEAI_MNN_LIBRARY
  NAMES MNN
  PATHS ${ENGINEAI_ROBOTICS_THIRD_PARTY}
  PATH_SUFFIXES lib
  NO_DEFAULT_PATH)
if(NOT ENGINEAI_MNN_INCLUDE_DIR OR NOT ENGINEAI_MNN_LIBRARY)
  message(FATAL_ERROR
    "LeoLab runner requires the SDK-provided MNN headers and library under "
    "${ENGINEAI_ROBOTICS_THIRD_PARTY}; build inside the official SDK environment")
endif()
target_include_directories(${CURRENT_TARGET_NAME} PRIVATE ${ENGINEAI_MNN_INCLUDE_DIR})

# Set link libraries of target
target_link_libraries(${CURRENT_TARGET_NAME}
  PUBLIC
    yaml-cpp::yaml-cpp
    glog::glog
    fmt::fmt
    Eigen3::Eigen
    src::runner::basic
    src::runner::motion_transition
    src::common::base
    src::core::model
    ${ENGINEAI_MNN_LIBRARY}
)

# Set installation of target
install(TARGETS ${CURRENT_TARGET_NAME}
  EXPORT ${CURRENT_TARGET_ALIAS_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Set tests
if(BUILD_TESTS AND test_files)
add_googletest(test_${CURRENT_TARGET_NAME}
  SRCS
    ${test_files}
  DEPS
    GTest::GTest
    GTest::Main
    pthread
    src::core::math
    ${CURRENT_TARGET_NAME}
)
endif()
