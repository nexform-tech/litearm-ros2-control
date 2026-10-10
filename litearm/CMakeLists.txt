cmake_minimum_required(VERSION 3.8)

project(litearm)

find_package(ament_cmake REQUIRED)

# Vendored model, not hand-written: see README.md. Every directory is installed
# as-is so `package://litearm/...` in the URDF, RViz config and launch file
# resolves in the installed tree the same way it does in the source tree.
#
# `textures/` is deliberately absent. The vendor export ships it empty, git
# cannot carry an empty directory, and installing a directory that is not there
# fails the install step.
foreach(dir config launch meshes urdf rviz)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

ament_package()
