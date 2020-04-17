option(INSTALL_GTEST "Option if to install gtest" OFF) #不要安装 GTest
add_subdirectory(googletest)
include_directories(${GTest_SRC_DIR} ${GTest_SRC_DIR}/include)

enable_testing()

file(GLOB TESTS *.c*)
message("To build tests ... ")
foreach(TEST ${TESTS})
  get_filename_component(TEST_TARGET "${TEST}" NAME_WE)
  get_filename_component(TEST_FILE "${TEST}" NAME)
  add_executable(${TEST_TARGET} ${TEST_FILE})
  message("  - ${TEST_FILE} -> ${TEST_TARGET}")
  target_link_libraries(${TEST_TARGET} 
    ${PROJECT_NAME}
    gtest gtest_main
  )
  add_test(${TEST_TARGET} ${TEST_TARGET})
endforeach()
