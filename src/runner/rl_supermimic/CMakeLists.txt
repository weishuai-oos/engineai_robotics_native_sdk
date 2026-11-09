get_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_NAME)
get_alias_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_ALIAS_NAME)

file(GLOB_RECURSE header_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE source_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)
file(GLOB_RECURSE test_files ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cc)
list(REMOVE_ITEM source_files ${test_files})

add_library(${CURRENT_TARGET_NAME} SHARED ${source_files})
add_library(${CURRENT_TARGET_ALIAS_NAME} ALIAS ${CURRENT_TARGET_NAME})
target_sources(${CURRENT_TARGET_NAME} INTERFACE ${header_files})
target_include_directories(${CURRENT_TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../rl_dance_example/include/cnpy>
)

find_path(ENGINEAI_MNN_INCLUDE_DIR NAMES MNN/Interpreter.hpp
  PATHS ${ENGINEAI_ROBOTICS_THIRD_PARTY} PATH_SUFFIXES include NO_DEFAULT_PATH)
find_library(ENGINEAI_MNN_LIBRARY NAMES MNN
  PATHS ${ENGINEAI_ROBOTICS_THIRD_PARTY} PATH_SUFFIXES lib NO_DEFAULT_PATH)
if(NOT ENGINEAI_MNN_INCLUDE_DIR OR NOT ENGINEAI_MNN_LIBRARY)
  message(FATAL_ERROR "SuperMimic requires the existing SDK MNN headers/library")
endif()
if(NOT TARGET cnpy)
  message(FATAL_ERROR "SuperMimic requires the existing cnpy target from rl_dance_example")
endif()
target_include_directories(${CURRENT_TARGET_NAME} PRIVATE ${ENGINEAI_MNN_INCLUDE_DIR})
target_link_libraries(${CURRENT_TARGET_NAME} PUBLIC
  Eigen3::Eigen yaml-cpp::yaml-cpp glog::glog fmt::fmt
  src::runner::basic src::runner::motion_transition cnpy ${ENGINEAI_MNN_LIBRARY}
)
if(TARGET src_data_supplement)
  target_link_libraries(${CURRENT_TARGET_NAME} PUBLIC src_data_supplement)
endif()
install(TARGETS ${CURRENT_TARGET_NAME} EXPORT ${CURRENT_TARGET_ALIAS_NAME}
  RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

if(BUILD_TESTS AND test_files)
  add_googletest(test_${CURRENT_TARGET_NAME} SRCS ${test_files}
    DEPS GTest::GTest GTest::Main pthread ${CURRENT_TARGET_NAME})
endif()
