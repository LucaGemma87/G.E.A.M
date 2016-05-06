cmake_minimum_required(VERSION 2.8.3)
project(grasp_estimator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs  message_generation tf  gencpp  geneus  genlisp  genpy
  sensor_msgs geometry_msgs shape_msgs visualization_msgs urdf moveit_core moveit_ros_planning 
  moveit_ros_planning_interface pluginlib moveit_msgs control_msgs gazebo_msgs  cmake_modules tf_conversions eigen_conversions
  #opencv
)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

#find_package(OpenCV 3.1 REQUIRED)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

#find_package(PkgConfig REQUIRED)

#pkg_check_modules(GSL REQUIRED /home/pacman/Projects/gsl)
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
add_message_files(
  FILES
   Goal.msg
   DataAcquired.msg
   WeightEstimated.msg
   GraspShape.msg
   VolumeEstimated.msg
   ObjectRecognition.msg
 )



## Generate services in the 'srv' folder
 add_service_files(
    FILES
    ArmPlanning.srv
    DataAcquiring.srv
#   Service2.srv
 )
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
 geometry_msgs
 std_msgs
  sensor_msgs
  gazebo_msgs
)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grasp_estimator
  CATKIN_DEPENDS roscpp rospy std_msgs  message_runtime sensor_msgs tf geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface eigen tf_conversions eigen_conversions
  DEPENDS system_lib grasp_estimator gazebo eigen
)



#find_package(PCL 1.8 REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(GSL 2.1 REQUIRED)
include_directories(${GSL_INCLUDE_DIRS})
link_directories(${GSL_LIBRARY_DIRS})
add_definitions(${GSL_DEFINITIONS})


find_package(dlib REQUIRED)
include_directories(${DLIB_INCLUDE_DIRS})
link_directories(${DLIB_LIBRARY_DIRS})
add_definitions(${DLIB_DEFINITIONS})



# include_directories(${FLANN_INCLUDE_DIRS})
# link_directories(${FLANN_LIBRARY_DIRS})
# add_definitions(${FLANN_DEFINITIONS})

find_package(PkgConfig)
pkg_check_modules(PC_FLANN flann)
set(FLANN_DEFINITIONS ${PC_FLANN_CFLAGS_OTHER})

find_path(FLANN_INCLUDE_DIR flann/flann.hpp
    HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS})

find_library(FLANN_LIBRARY flann
    HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS})

set(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})
set(FLANN_LIBRARIES ${FLANN_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Flann DEFAULT_MSG
    FLANN_LIBRARY FLANN_INCLUDE_DIR)

mark_as_advanced(FLANN_LIBRARY FLANN_INCLUDE_DIR)

find_package(HDF5 REQUIRED)
include_directories(${HDF5_INCLUDE_DIRS})



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

#   include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}  
#    ${EIGEN_INCLUDE_DIRS} gazebo gsl-2.1 dlib LAPACK BLAS) #for my pc 
  include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}   ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS} gazebo /home/pacman/Projects/gsl/include dlib LAPACK BLAS) # for pacman head pc 



#link_directories(${GSL_LIBRARY_DIRS})
# for pacman head pc : 
link_directories(BEFORE /home/pacman/Projects/gsl/lib ) #for pacman head
#link_directories(BEFORE /usr/local/lib  ) #for my pc 
## Declare a C++ library
# # add_library(grasp_estimator
#   src/${PROJECT_NAME}/grasp_estimator.cpp
# )

#add_library(gazebo_ros_ft_sensor src/gazebo_ros_ft_sensor.cpp)
#add_library(gazebo_ros_ft_sensor src/gazebo_ros_ft_sensor.cpp)
#target_link_libraries(gazebo_ros_ft_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} gazebo)
#add_library(GSL)

#install(TARGETS
#gazebo_ros_ft_sensor
#)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(grasp_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_dependencies(grasp_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(grasp_estimator_node src/grasp_estimator_node.cpp)

#add_executable(right_arm_planning_node src/right_arm_planning.cpp)
add_executable(left_arm_planning_node src/left_arm_planning.cpp)
#add_executable(test_object_in_hand_right_arm_planning_node src/test_object_in_hand_right_arm_planning.cpp)
add_executable(test_object_in_hand_left_arm_planning_node src/test_object_in_hand_left_arm_planning.cpp)

#add_executable(data_acquiring_node src/data_acquiring.cpp)
# add_executable(left_hand_arm_data_acquiring_node src/left_hand_arm_data_acquiring.cpp)
# add_executable(test_data_acquiring src/test_data_acquiring.cpp)
# add_executable(grasp_state_data_server src/grasp_state_data_server.cpp)
add_executable(grasp_state_data_server_2 src/grasp_state_data_server_2.cpp)
#add_executable(weight_estimator_server src/weight_estimation.cpp)
add_executable(weight_estimator_server src/hand_control.cpp)
add_executable(grasp_shape_estimator_server src/grasp_shape_estimation.cpp)
add_executable(volume_estimator_server src/volume_estimation.cpp)
add_executable(wrench_estimator_server src/wrench_estimation.cpp)
add_executable(twist_estimator_server src/twist_estimation.cpp)
add_executable(object_clustering_recognition_server src/object_clustering_recognition.cpp)
add_executable(offline_object_clustering_recognition_server src/offline_object_clustering_recognition.cpp)
add_executable(multi_test_offline_object_clustering_recognition_server src/multi_test_offline_object_clustering_recognition.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(grasp_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_dependencies(right_arm_planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
add_dependencies(left_arm_planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)

#add_dependencies(test_object_in_hand_right_arm_planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
add_dependencies(test_object_in_hand_left_arm_planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)

#add_dependencies(data_acquiring_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
# add_dependencies(left_hand_arm_data_acquiring_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
# add_dependencies(test_data_acquiring ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
# add_dependencies(grasp_state_data_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
add_dependencies(grasp_state_data_server_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
add_dependencies(weight_estimator_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
add_dependencies(grasp_shape_estimator_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp gls)
add_dependencies(volume_estimator_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
add_dependencies(wrench_estimator_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
add_dependencies(twist_estimator_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
add_dependencies(object_clustering_recognition_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp ${BLAS_EXPORTED_TARGETS} ${LAPACK_EXPORTED_TARGETS} })
add_dependencies(offline_object_clustering_recognition_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp ${BLAS_EXPORTED_TARGETS} ${LAPACK_EXPORTED_TARGETS} })
add_dependencies(multi_test_offline_object_clustering_recognition_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp ${BLAS_EXPORTED_TARGETS} ${LAPACK_EXPORTED_TARGETS} })
## Specify libraries to link a library or executable target against
# target_link_libraries(right_arm_planning_node    ${catkin_LIBRARIES}   )
 target_link_libraries(left_arm_planning_node    ${catkin_LIBRARIES}   )
 

 # target_link_libraries(test_object_in_hand_right_arm_planning_node    ${catkin_LIBRARIES})
  target_link_libraries(test_object_in_hand_left_arm_planning_node    ${catkin_LIBRARIES})
 # target_link_libraries(data_acquiring_node  ${catkin_LIBRARIES})
  #  target_link_libraries(left_hand_arm_data_acquiring_node  ${catkin_LIBRARIES})
  # target_link_libraries(test_data_acquiring  ${catkin_LIBRARIES})
  #   target_link_libraries(grasp_state_data_server  ${catkin_LIBRARIES})
    target_link_libraries(grasp_state_data_server_2  ${catkin_LIBRARIES})
    target_link_libraries(weight_estimator_server ${catkin_LIBRARIES} gsl gslcblas)
 target_link_libraries(grasp_shape_estimator_server ${catkin_LIBRARIES} gsl gslcblas)
 target_link_libraries(volume_estimator_server ${catkin_LIBRARIES} ${PCL_LIBRARIES}) 
  target_link_libraries(wrench_estimator_server ${catkin_LIBRARIES}) 
    target_link_libraries(twist_estimator_server ${catkin_LIBRARIES}) 
    target_link_libraries(object_clustering_recognition_server ${catkin_LIBRARIES} ${dlib_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES}  ${FLANN_LIBRARIES} ${HDF5_LIBRARIES}) 
     target_link_libraries(offline_object_clustering_recognition_server ${catkin_LIBRARIES} ${dlib_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_LIBRARIES}) 
       target_link_libraries(multi_test_offline_object_clustering_recognition_server ${catkin_LIBRARIES} ${dlib_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_LIBRARIES})    

############# ${GSL_LIBRARIES}
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


## Mark executables and/or libraries for installation
# install(TARGETS grasp_estimator grasp_estimator_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grasp_estimator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

## for COG estimation server



add_executable(cog_estimator_server src/ft_sensor_calib.cpp src/cog_estimation.cpp)
add_dependencies(cog_estimator_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} grasp_estimator_genlisp grasp_estimator_gen_cpp)
target_link_libraries(cog_estimator_server ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS cog_estimator_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config include launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

