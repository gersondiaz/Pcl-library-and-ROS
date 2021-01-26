cmake_minimum_required(VERSION 2.8.3)
project(procesamiento_nubes)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  message_filters
  pcl_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES procesamiento_nubes
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/procesamiento_nubes.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/procesamiento_nubes_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_procesamiento_nubes.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(Leer_Nube src/Leer_Nube.cpp)
target_link_libraries(Leer_Nube ${catkin_LIBRARIES})

add_executable(Filtro_SOR src/Filtro_SOR.cpp)
target_link_libraries(Filtro_SOR ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(Submuestreo src/Submuestreo.cpp)
target_link_libraries(Submuestreo ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(Pass_Through src/Pass_Through.cpp)
target_link_libraries(Pass_Through ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(Planar_Segmentation src/Planar_Segmentation.cpp)
target_link_libraries(Planar_Segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(Molde_malo src/Molde_malo.cpp)
target_link_libraries(Molde_malo ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(msgs_filters src/msgs_filters.cpp)
target_link_libraries(msgs_filters ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(projecting src/projecting.cpp)
target_link_libraries(projecting ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_transfor src/pcl_transfor.cpp)
target_link_libraries(pcl_transfor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_submuestreo_final src/pcl_submuestreo_final.cpp)
target_link_libraries(pcl_submuestreo_final ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_escalar src/pcl_escalar.cpp)
target_link_libraries(pcl_escalar ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_filtro_sor_final src/pcl_filtro_sor_final.cpp)
target_link_libraries(pcl_filtro_sor_final ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_cropbox src/pcl_cropbox.cpp)
target_link_libraries(pcl_cropbox ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(Molde_comparar src/Molde_comparar.cpp)
target_link_libraries(Molde_comparar ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(transformacion_plato src/transformacion_plato.cpp)
target_link_libraries(transformacion_plato ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_modelcoef src/pcl_modelcoef.cpp)
target_link_libraries(pcl_modelcoef ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(msgs_filters_getcoef src/msgs_filters_getcoef.cpp)
target_link_libraries(msgs_filters_getcoef ${catkin_LIBRARIES} ${PCL_LIBRARIES})








add_executable(projection_1 src/projection_1.cpp)
target_link_libraries(projection_1 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(projection_2 src/projection_2.cpp)
target_link_libraries(projection_2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(projection_3 src/projection_3.cpp)
target_link_libraries(projection_3 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(projection_4 src/projection_4.cpp)
target_link_libraries(projection_4 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(projection_5 src/projection_5.cpp)
target_link_libraries(projection_5 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(projection_6 src/projection_6.cpp)
target_link_libraries(projection_6 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(projection_7 src/projection_7.cpp)
target_link_libraries(projection_7 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(projection_8 src/projection_8.cpp)
target_link_libraries(projection_8 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(projection_9 src/projection_9.cpp)
target_link_libraries(projection_9 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(sumas src/sumas.cpp)
target_link_libraries(sumas ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(kdtree_search src/kdtree_search.cpp)
target_link_libraries(kdtree_search ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(Comparacion src/Comparacion.cpp)
target_link_libraries(Comparacion ${catkin_LIBRARIES} ${PCL_LIBRARIES})





