cmake_minimum_required(VERSION 2.8)
project(marble_virtual_comms)
set(CMAKE_CXX_STANDARD 17)

set(PACKAGE_DEPENDENCIES
  roscpp
  std_msgs
  subt_ign
  subt_communication_broker
  subt_msgs
  marble_multi_agent
  marble_artifact_detection_msgs
  marble_mapping
  message_generation
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(Boost REQUIRED)

set(IGN_MSGS_VER 6)
find_package(ignition-msgs${IGN_MSGS_VER} REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${ignition-msgs6_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_service_files(DIRECTORY srv
  FILES CreatePeer.srv
  )
generate_messages()

catkin_package()

add_library(${PROJECT_NAME} src/MarbleVirtualComms.cpp src/MVCPeer.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(marble_virtual_comms_node src/marble_virtual_comms_node.cpp)
target_link_libraries(marble_virtual_comms_node ${PROJECT_NAME} ${catkin_LIBRARIES} ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER})

# install targets:
install(TARGETS ${PROJECT_NAME}
  marble_virtual_comms
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
