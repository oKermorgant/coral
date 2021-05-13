cmake_minimum_required(VERSION 3.5)
project(coral)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


set(OSGOCEAN_SRC ${PROJECT_SOURCE_DIR}/osgocean/osgocean)

OPTION(OSGOCEAN_EXTERNAL "Compile osgOcean as external lib" ON)

# get osgocean if not here
if(NOT EXISTS ${OSGOCEAN_SRC})
message("getting osgocean submodule...")
execute_process(COMMAND git submodule update --init
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()

if(OSGOCEAN_EXTERNAL)

    # manual compile 
    set(OSGOCEAN_LIBRARIES ${PROJECT_SOURCE_DIR}/osgocean/build/libosgOcean.so)

    SET(OSGOCEAN_BUILD_DIR ${PROJECT_SOURCE_DIR}/osgocean/build)
        if(NOT EXISTS ${OSGOCEAN_LIBRARIES})
        file(MAKE_DIRECTORY ${OSGOCEAN_BUILD_DIR})
        execute_process(COMMAND cmake .. 
        WORKING_DIRECTORY ${OSGOCEAN_BUILD_DIR})
        execute_process(COMMAND make
        WORKING_DIRECTORY ${OSGOCEAN_BUILD_DIR})
    endif()

    # manual install 
    if(NOT EXISTS ${CMAKE_INSTALL_PREFIX}/lib/libosgOcean.so)
        file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX})
        file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)
        file(COPY ${OSGOCEAN_LIBRARIES} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)
    endif()
    
    # register lib for dependency
    add_library(osgOcean SHARED IMPORTED)
    set_target_properties(osgOcean PROPERTIES IMPORTED_LOCATION ${CMAKE_INSTALL_PREFIX}/lib/libosgOcean.so)

else()

add_subdirectory(osgocean)

endif()

set(OSGOCEAN_INCLUDE_DIRS ${OSGOCEAN_SRC}/include)

find_package(OpenSceneGraph COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osg osgParticle osgManipulator)
set(OpenGL_GL_PREFERENCE GLVND)
find_package (OpenGL REQUIRED)

add_executable(coral
src/coral_main.cpp
src/coral_node.cpp
src/scene_interface.cpp
src/visual_link.cpp
src/resource_helpers.cpp
src/viewer.cpp
src/OceanScene.cpp
src/Scene.cpp
src/SkyDome.cpp
src/SphereSegment.cpp
)
target_include_directories(coral SYSTEM PRIVATE include ${OPENSCENEGRAPH_INCLUDE_DIR} ${OSGOCEAN_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
target_link_libraries(coral osgOcean ${OPENSCENEGRAPH_LIBRARIES} ${OPENGL_LIBRARY})
ament_target_dependencies(coral rclcpp sensor_msgs geometry_msgs nav_msgs urdf ament_index_cpp tf2_ros)

#add_executable(listener
#tests/listener.cpp
#src/coral_node.cpp
#src/visual_link.cpp
#src/resource_helpers.cpp
#)
#target_include_directories(listener SYSTEM PRIVATE include ${OPENSCENEGRAPH_INCLUDE_DIR} ${OSGOCEAN_INCLUDE_DIRS})
#target_link_libraries(listener osgOcean ${OPENSCENEGRAPH_LIBRARIES} ${OPENGL_LIBRARY})
#ament_target_dependencies(listener rclcpp sensor_msgs geometry_msgs nav_msgs urdf ament_index_cpp tf2_ros)


## osgOcean example
#add_executable(example
#example/application.cpp example/Scene.cpp  example/SceneEventHandler.h  example/Scene.h  example/ScopedTimer.h  example/SkyDome.cpp  example/SkyDome.h  example/SphereSegment.cpp  example/SphereSegment.h  example/TextHUD.h)
#target_include_directories(example PRIVATE include ${OPENSCENEGRAPH_INCLUDE_DIR} ${OSGOCEAN_INCLUDE_DIRS} -lGLU)
#target_link_libraries(example osgOcean ${OPENSCENEGRAPH_LIBRARIES} -lGLU)
#ament_target_dependencies(example ament_index_cpp)


install(TARGETS 
coral
#listener
DESTINATION lib/${PROJECT_NAME})

if(NOT OSGOCEAN_EXTERNAL)
install(TARGETS osgOcean
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
endif()

# common osgocean resources
install(DIRECTORY ${OSGOCEAN_SRC}/resources DESTINATION share/${PROJECT_NAME}) 
install(DIRECTORY ${OSGOCEAN_SRC}/demo/island ${OSGOCEAN_SRC}/demo/textures DESTINATION share/${PROJECT_NAME}/resources)

ament_export_libraries(osgOcean)
ament_package()
