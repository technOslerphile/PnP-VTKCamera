cmake_minimum_required(VERSION 2.8)
 
PROJECT(PnP-VtkCamera)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
add_executable(PnP-VtkCamera MACOSX_BUNDLE PnP-VtkCamera)
 
if(VTK_LIBRARIES)
  target_link_libraries(PnP-VtkCamera ${VTK_LIBRARIES})
else()
  target_link_libraries(PnP-VtkCamera vtkHybrid vtkWidgets)
endif()