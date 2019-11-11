/**
 @file talker.cpp
 @author Ashwin Varghese Kuruttukulam
 @copyright MIT
 @brief ROS subscriber publishes to  to chatter topic
 @brief ROS publisher that publishes to chatter topic
 ROS publisher from the ros tutorials page
 */

#pragma once

#include <iostream>
#include <sstream>
#include <string>

/**
  * Structure to store default message of type string
  */
struct DefaultMessage {
	  std::string defaultMessage = "this is the default message";
};
