#include "ros/ros.h"
#include <iostream>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/default_topics.h>
#include <std_msgs/Float64.h>
#include <test4/SetVariables.h>
#include <sensor_msgs/Joy.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
      rc_sub = n.subscribe("/rc", 1, &SubscribeAndPublish::SetThrust, this);
      myFrequency = 3;
      myMotorOffset = 0.0;
  }

  void SetThrust(sensor_msgs::Joy msg)
  {
    myThrust = msg.axes[2];
    return;
  }

  bool setVariables(test4::SetVariables::Request& req, test4::SetVariables::Response& res){
    myFrequency = req.Frequency;
    myMotorOffset = req.MotorOffset;
    return true;
  }

  float returnFrequency(){
      return myFrequency;
  }

  double returnThrust(){
      return myThrust;
  }

  double returnMotorOffset(){
      return myMotorOffset;
  }

private:
  float myFrequency;
  double myMotorOffset;
  double myThrust;
  ros::NodeHandle n;
  ros::Subscriber rc_sub;


};

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
  ros::init(argc, argv, "control");

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
  SubscribeAndPublish myObject;
  ros::ServiceServer service = n.advertiseService("SetVariables", &SubscribeAndPublish::setVariables, &myObject);
  ros::Publisher control_pub = n.advertise< mav_msgs::Actuators >( mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    mav_msgs::Actuators out;
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    out.normalized.resize(8);
    if(count % 2 == 0){
    out.normalized[0] = myObject.returnThrust() * 0.952;
    out.normalized[1] = myObject.returnThrust();
    out.normalized[2] = 0.0;
    out.normalized[3] = 0.0;
    out.normalized[4] = 0.5;
    out.normalized[5] = 0.5;
    out.normalized[6] = 0.0;
    out.normalized[7] = 0.0;
    }
    else{
    out.normalized[0] = (myObject.returnThrust() + myObject.returnMotorOffset() )* 0.952;
    out.normalized[1] = myObject.returnThrust() + myObject.returnMotorOffset();
    out.normalized[2] = 0.0;
    out.normalized[3] = 0.0;
    out.normalized[4] = 0.5;
    out.normalized[5] = 0.5;
    out.normalized[6] = 0.0;
    out.normalized[7] = 0.0;
    }
    control_pub.publish(out);

    ros::spinOnce();
    ros::Rate loop_rate(myObject.returnFrequency());
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
