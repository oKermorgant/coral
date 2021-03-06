cmake_minimum_required(VERSION 3.5)
project(coral)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/Spawn.srv" "srv/Surface.srv")

if("$ENV{ROS_DISTRO}" STREQUAL "galactic" OR "$ENV{ROS_DISTRO}" STREQUAL "foxy")
    set(LEGACY_IDL TRUE)
else()
    set(LEGACY_IDL FALSE)
endif()
if(NOT ${LEGACY_IDL})
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
endif()

set(OSGOCEAN_SRC ${PROJECT_SOURCE_DIR}/osgocean/osgocean)

OPTION(OSGOCEAN_EXTERNAL "Compile osgOcean as external lib" ON)

# get osgocean if not here
if(NOT EXISTS ${OSGOCEAN_SRC}/CMakeLists.txt)
message("getting osgocean submodule...")
execute_process(COMMAND git submodule update --init
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()

set(CMAKE_BUILD_TYPE Debug)

if(OSGOCEAN_EXTERNAL)

    # manual compile 
    SET(OSGOCEAN_BUILD_DIR ${PROJECT_SOURCE_DIR}/osgocean/build)
    set(OSGOCEAN_LIBRARIES ${OSGOCEAN_BUILD_DIR}/libosgOcean.so)
        
    if(NOT EXISTS ${OSGOCEAN_LIBRARIES})
        file(MAKE_DIRECTORY ${OSGOCEAN_BUILD_DIR})
        execute_process(COMMAND cmake .. WORKING_DIRECTORY ${OSGOCEAN_BUILD_DIR})        
    endif()
    
    execute_process(COMMAND make WORKING_DIRECTORY ${OSGOCEAN_BUILD_DIR})
    
    # manual install, ament-wise
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

add_executable(coral_sim
src/coral_main.cpp
src/coral_node.cpp
src/event_handler.cpp
src/camera.cpp
src/link.cpp
src/link_visual.cpp
src/urdf_parser.cpp
src/resource_helpers.cpp
src/viewer.cpp
# from osgOcean + example
src/OceanScene.cpp
src/Scene.cpp
src/SkyDome.cpp
src/SphereSegment.cpp
)

target_include_directories(coral_sim SYSTEM PRIVATE include ${OPENSCENEGRAPH_INCLUDE_DIR} ${OSGOCEAN_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
target_include_directories(coral_sim PRIVATE include)
target_link_libraries(coral_sim osgOcean ${OPENSCENEGRAPH_LIBRARIES} ${OPENGL_LIBRARY})
ament_target_dependencies(coral_sim rclcpp sensor_msgs geometry_msgs nav_msgs urdf ament_index_cpp tf2_ros image_transport std_srvs)
if(${LEGACY_IDL})
    rosidl_target_interfaces(coral_sim ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    target_link_libraries(coral_sim "${cpp_typesupport_target}")
endif()



# spawn service wrapper
add_executable(spawn src/spawn.cpp)
ament_target_dependencies(spawn rclcpp)
if(${LEGACY_IDL})
    rosidl_target_interfaces(spawn ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    target_link_libraries(spawn "${cpp_typesupport_target}")
endif()




install(TARGETS coral_sim spawn
DESTINATION lib/${PROJECT_NAME})

FILE(GLOB SCRIPTS scripts/*.py)
install(PROGRAMS ${SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

if(NOT OSGOCEAN_EXTERNAL)
install(TARGETS osgOcean
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}) 

# common osgocean resources
install(DIRECTORY ${OSGOCEAN_SRC}/resources/shaders DESTINATION share/${PROJECT_NAME}/resources)
install(DIRECTORY ${OSGOCEAN_SRC}/demo/island DESTINATION share/${PROJECT_NAME}/resources)

# override some shaders if needed
install(DIRECTORY osgocean/resources DESTINATION share/${PROJECT_NAME}) 

ament_export_libraries(osgOcean)
ament_package()
