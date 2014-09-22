cmake_minimum_required(VERSION 2.8.3)
project(davinci_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  hardware_interface
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system regex thread)

add_service_files(
  FILES
  EnableMotor.srv
)

generate_messages(
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES davinci_driver_node
    CATKIN_DEPENDS 
        roscpp
        controller_manager
        hardware_interface
        message_runtime
)

###########
## Build ##
###########

include_directories(
    "include"
    "libjson"
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

set(libjson_files
    libjson/_internal/Source/JSONAllocator.cpp
    libjson/_internal/Source/internalJSONNode.cpp
    libjson/_internal/Source/JSONChildren.cpp
    libjson/_internal/Source/JSONDebug.cpp
    libjson/_internal/Source/JSONIterators.cpp
    libjson/_internal/Source/JSONMemory.cpp
    libjson/_internal/Source/JSONNode.cpp
    libjson/_internal/Source/JSONNode_Mutex.cpp
    libjson/_internal/Source/JSONPreparse.cpp
    libjson/_internal/Source/JSONStream.cpp
    libjson/_internal/Source/JSONValidator.cpp
    libjson/_internal/Source/JSONWorker.cpp
    libjson/_internal/Source/JSONWriter.cpp
    libjson/_internal/Source/libjson.cpp
)

add_executable(davinci_driver_node
    src/ros_driver.cpp
    src/davinci_driver.cpp
    src/sbrio_driver.cpp
    ${libjson_files}
)
target_link_libraries(davinci_driver_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

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

## Mark executables and/or libraries for installation
# install(TARGETS davinci_driver davinci_driver_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_davinci_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
