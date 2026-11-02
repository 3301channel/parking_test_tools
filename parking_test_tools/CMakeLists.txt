cmake_minimum_required(VERSION 3.8)
project(parking_test_tools)

find_package(ament_cmake REQUIRED)

# launch / config / scripts 原样安装到 share/parking_test_tools 下，
# 保留可执行权限，这样 $SCRIPT_DIR 相对定位在 install 后依然成立。
# docs 一并装上，README 里引用的示例图在 install 后才不会断链。
install(DIRECTORY launch config scripts docs
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# README 也放进 share，装到车端后能直接翻
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
