cmake_minimum_required(VERSION 3.10)
project(walker_c1_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch config mujoco usd 关键参数
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES script/view_mujoco.py DESTINATION share/${PROJECT_NAME}/script)
install(FILES "docs/1、URDF-C1.png" "docs/2、Mujoco-C1.png" "docs/3、USD-C1.png"
  DESTINATION share/${PROJECT_NAME}/docs)
install(FILES README.md NOTICE.md DESTINATION share/${PROJECT_NAME})
ament_package()
