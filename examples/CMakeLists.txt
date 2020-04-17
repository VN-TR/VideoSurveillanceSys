file(GLOB EXAMPLES *.c*)
message("To build examples ... ")
foreach(EXAMPLE ${EXAMPLES}) 
  get_filename_component(EXAMPLE_TARGET "${EXAMPLE}" NAME_WE)
  get_filename_component(EXAMPLE_FILE "${EXAMPLE}" NAME)
  add_executable(${EXAMPLE_TARGET} ${EXAMPLE_FILE})
  message("  - ${EXAMPLE_FILE} -> ${EXAMPLE_TARGET}")
  target_link_libraries(${EXAMPLE_TARGET}
    ${PROJECT_NAME}
    )
  # 安装示例文件，供使用者参考
  install(FILES ${EXAMPLE_FILE}
    DESTINATION examples/${PROJECT_NAME}   # 目标地址：${CMAKE_INSTALL_PREFIX}/examples/${PROJECT_NAME}
    )
endforeach()
