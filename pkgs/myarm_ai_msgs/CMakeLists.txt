cmake_minimum_required(VERSION 3.5)
project(myarm_ai_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

######################################################################
########## NOTE TO TEMPLATE USERS: When adding messages that require other messages,
########## add the following line below to ensure that the other messages are found.
########## Also make sure new messages are added to the rosidl_generate_interfaces list.
# find_package(my_required_msgs_package REQUIRED)
######################################################################


rosidl_generate_interfaces(
  ${PROJECT_NAME}

  ######################################################################
  ########## NOTE TO TEMPLATE USERS: Add new messages here, like so:
  # msg/ANewMessageFile.msg
  ######################################################################

  "msg/ExampleMessage.msg"

  "action/ExampleAction.action"

  "srv/ExampleService.srv"

  ######################################################################
  ########## NOTE TO TEMPLATE USERS: Add your dependency packages here, like so:
  # DEPENDENCIES my_required_msgs_package
  ######################################################################

)

ament_package()
