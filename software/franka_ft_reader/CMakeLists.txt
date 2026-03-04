cmake_minimum_required(VERSION 3.10)
project(franka_ft_reader)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    message_generation
    geometry_msgs
)


# add_message_files(
#   FILES
#   xyz.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )



###################################
## catkin specific configuration ##
###################################


catkin_python_setup()

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    message_runtime
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  # Set manually because Eigen sets a non standard INCLUDE DIR
)



# ## Declare a cpp library
# add_library(${PROJECT_NAME}_core
# )
# target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)

# ## Declare cpp executables
# add_executable(${PROJECT_NAME}
#   src/${PROJECT_NAME}_node.cpp
#   src/PsyonicSdkRos.cpp
# )
# target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

# ## Add dependencies to exported targets, like ROS msgs or srvs
# add_dependencies(${PROJECT_NAME}_core
#   ${catkin_EXPORTED_TARGETS}
# )

# add_dependencies(${PROJECT_NAME}
#   ${catkin_EXPORTED_TARGETS}
# )

## Specify libraries to link executable targets against
# target_link_libraries(${PROJECT_NAME}_core
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(${PROJECT_NAME}
#   ${PROJECT_NAME}_core
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############
# List all executable python and shell scripts for installation
install(PROGRAMS
   scripts/*
   nodes/*
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables and/or libraries for installation
# install(
#   TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
# )
