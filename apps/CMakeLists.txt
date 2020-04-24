file(GLOB APPS *.c*)
message("To build apps ... ")
include(generate_product_version)

foreach(APP ${APPS})
  get_filename_component(APP_TARGET "${APP}" NAME_WE)
  get_filename_component(APP_FILE "${APP}" NAME)

  generate_product_version(${APP_TARGET}ProductVersionFiles
    NAME ${APP_TARGET}
    VERSION_MAJOR ${PROJECT_VERSION_MAJOR}
    VERSION_MINOR ${PROJECT_VERSION_MINOR}
    VERSION_PATCH ${PROJECT_VERSION_PATCH}
    COMPANY_NAME "VisionNav Robotics")

  add_executable(${APP_TARGET} ${APP_FILE} ${${APP_TARGET}ProductVersionFiles})
  message("  - ${APP_FILE} -> ${APP_TARGET}")
  target_link_libraries(${APP_TARGET}
    ${PROJECT_NAME}
  )
  #  安装
  install(TARGETS ${APP_TARGET}
    RUNTIME DESTINATION bin        # 可执行文件/dll目标地址： ${CMAKE_INSTALL_PREFIX}/bin
    )
endforeach()
