/**
 @file talker.cpp
 @author Ashwin Varghese Kuruttukulam
 @copyright MIT
 @brief ROS subscriber publishes to  to chatter topic
 @brief ROS publisher that publishes to chatter topic
 ROS publisher from the ros tutorials page
 */
#include <sstream>
#include "tf/transform_broadcaster.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/customString.h"
#include "beginner_tutorials/editText.h"

std::string defaultMessage = "This is the defult message";

bool changeMessage(beginner_tutorials::editText::Request &req\
,
                   beginner_tutorials::editText::Response &res) {
  defaultMessage = req.inputString;
  res.outputString = req.inputString;
  ROS_WARN_STREAM("Service call has been used to change the default mesage");
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  // tf transform broadcaster object declaration
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise < beginner_tutorials::customString
      > ("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  ros::ServiceServer server = n.advertiseService("editText", changeMessage);
  int count = 0;
  ROS_DEBUG_STREAM("Starting string publisher");
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    beginner_tutorials::customString msg;

    std::stringstream ss;
    ss << defaultMessage << count;
    msg.data.data = ss.str();
    ROS_INFO("%s", msg.data.data.c_str());
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    // Setting the position of the transformation
    transform.setOrigin(tf::Vector3(0, 0, 1));

    // Declaring quaternion object
    tf::Quaternion q;
    q.setRPY(0, 0, 1);

    // Setting the orientation of the transformation
    transform.setRotation(q);

    // broadcasting the transformation
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  ROS_FATAL_STREAM("ROS Node has died");
  return 0;
}

