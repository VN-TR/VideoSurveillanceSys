file(GLOB APPS *.c*)
message("To build apps ... ")

foreach(APP ${APPS})
  get_filename_component(APP_TARGET "${APP}" NAME_WE)
  get_filename_component(APP_FILE "${APP}" NAME)
  add_executable(${APP_TARGET} ${APP_FILE})
  message("  - ${APP_FILE} -> ${APP_TARGET}")
  target_link_libraries(${APP_TARGET}
    ${PROJECT_NAME}
  )
  #  安装
  install(TARGETS ${APP_TARGET}
    RUNTIME DESTINATION bin        # 可执行文件/dll目标地址： ${CMAKE_INSTALL_PREFIX}/bin
    )
endforeach()
