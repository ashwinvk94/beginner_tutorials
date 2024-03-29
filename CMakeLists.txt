cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation tf)


## Declare the message files to be built
add_message_files(
  FILES
  customString.msg
)

## DEclare the service files to be built
add_service_files(
  FILES
  editText.srv  
)

add_compile_options(-std=c++11)

# Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

## Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  add_rostest_gtest(
    talkerTest
    test/talkerTest.launch
    test/main.cpp
    test/talkerTest.cpp
  )

  target_link_libraries(
    talkerTest
    ${catkin_LIBRARIES}
  )
  add_dependencies(
    talker
    talkerTest
    ${catkin_EXPORTED_TARGETS}
  )
endif()
