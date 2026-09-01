get_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_NAME)
get_alias_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_ALIAS_NAME)

file(GLOB_RECURSE header_files ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
file(GLOB_RECURSE source_files ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)
file(GLOB_RECURSE test_files ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cc)
list(REMOVE_ITEM source_files ${test_files})

add_library(${CURRENT_TARGET_NAME} SHARED ${source_files})
add_library(${CURRENT_TARGET_ALIAS_NAME} ALIAS ${CURRENT_TARGET_NAME})
target_sources(${CURRENT_TARGET_NAME} INTERFACE ${header_files})
target_include_directories(${CURRENT_TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${CURRENT_TARGET_NAME}
  PUBLIC
    Eigen3::Eigen
    glog::glog
    yaml-cpp::yaml-cpp
    src::runner::basic
    src::data
    src::core::model
    src::common::base
)
install(TARGETS ${CURRENT_TARGET_NAME} EXPORT ${CURRENT_TARGET_ALIAS_NAME}
        RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

if(BUILD_TESTS AND test_files)
  add_googletest(test_${CURRENT_TARGET_NAME} SRCS ${test_files}
    DEPS GTest::GTest GTest::Main pthread yaml-cpp::yaml-cpp ${CURRENT_TARGET_NAME})
endif()
