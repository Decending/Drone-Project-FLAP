#include "ros/ros.h"
#include <iostream>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/default_topics.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish

    //Topic you want to subscribe
    rc_sub = n.subscribe("/rc", 1, &SubscribeAndPublish::SetThrust, this);
    //imu_sub = n.subscribe("/imu", 1, &SubscribeAndPublish::SetFlaps, this);
    thrust = 0.0;
    flap1 = 0.5;
    flap2 = 0.5;
  }

  void SetThrust(sensor_msgs::Joy msg)
  {
    thrust = msg.axes[2];
    return;
  }

  void SetFlaps(sensor_msgs::Imu msg)
  {
    //Flap1 = (-msg.axes[1]+msg.axes[0])
    //Flap2 = ((msg.axes[1]+msg.axes[0])
    //out.normalized[4] = ((-msg.axes[1]+msg.axes[0]) + 1) / 2; Flap1
    //out.normalized[5] = ((msg.axes[1]+msg.axes[0]) + 1) / 2; Flap2
    return;
  }

  float returnThrust(){
      return thrust;
  }

  float returnFlap1(){
      return flap1;
  }

  float returnFlap2(){
      return flap2;
  }

private:
  ros::NodeHandle n;
  ros::Subscriber rc_sub;
  ros::Subscriber imu_sub;
  float thrust;
  float flap1;
  float flap2;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
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
  ros::NodeHandle n;
  ros::init(argc, argv, "control");
  SubscribeAndPublish myObject;
  ros::Publisher rc_pub = n.advertise< mav_msgs::Actuators >( mav_msgs::default_topics::COMMAND_ACTUATORS, 1);


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

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
  int counter = 0;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
   if(counter % 2){
   mav_msgs::Actuators out;
   out.normalized.resize(8);
   out.normalized[0] = myObject.returnThrust();
   out.normalized[1] = myObject.returnThrust();
   out.normalized[2] = 0.0;
   out.normalized[3] = 0.0;
   out.normalized[4] = myObject.returnFlap1();
   out.normalized[5] = myObject.returnFlap2();
   out.normalized[6] = 0.0;
   out.normalized[7] = 0.0;
   }
   else{
   mav_msgs::Actuators out;
   out.normalized.resize(8);
   out.normalized[0] = myObject.returnThrust();
   out.normalized[1] = myObject.returnThrust();
   out.normalized[2] = 0.0;
   out.normalized[3] = 0.0;
   out.normalized[4] = myObject.returnFlap2();
   out.normalized[5] = myObject.returnFlap1();
   out.normalized[6] = 0.0;
   out.normalized[7] = 0.0;
   }
   ++counter;
   rc_pub.publish(out);

   ros::spinOnce();

   loop_rate.sleep();
  }

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */


  return 0;
}
