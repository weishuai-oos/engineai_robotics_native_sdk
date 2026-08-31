# Get target names from the repository path.
get_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_NAME)
get_alias_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_ALIAS_NAME)

file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB_RECURSE source_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE test_files ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cc ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)
list(REMOVE_ITEM source_files ${test_files})

add_library(${CURRENT_TARGET_NAME} SHARED)
add_library(${CURRENT_TARGET_ALIAS_NAME} ALIAS ${CURRENT_TARGET_NAME})
target_sources(${CURRENT_TARGET_NAME} PRIVATE ${source_files})
target_sources(${CURRENT_TARGET_NAME} INTERFACE ${head_files})
target_include_directories(${CURRENT_TARGET_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${CURRENT_TARGET_NAME}
  PUBLIC
    Eigen3::Eigen
    glog::glog
    src::runner::basic
)

install(TARGETS ${CURRENT_TARGET_NAME}
  EXPORT ${CURRENT_TARGET_ALIAS_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

if(BUILD_TESTS AND test_files)
  add_googletest(test_${CURRENT_TARGET_NAME}
    SRCS
      ${test_files}
    DEPS
      GTest::GTest
      GTest::Main
      pthread
      ${CURRENT_TARGET_NAME}
  )
endif()
