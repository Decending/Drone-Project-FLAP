#include "ros/ros.h"
#include <iostream>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/default_topics.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <biquad.h>
#include <biquad.cpp>
#include <vector>
#include <control/SetGains.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    rc_sub = n.subscribe("/rc", 1, &SubscribeAndPublish::SetThrust, this);
    imu_sub = n.subscribe("/imu", 1, &SubscribeAndPublish::SetFlaps, this);
    imu_pub = n.advertise<sensor_msgs::Imu>("/filtered_imu", 1);
    rc_pub = n.advertise< mav_msgs::Actuators >( mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
    //service = n.advertiseService("Set gains", &SubscribeAndPublish::SubscribeAndPublish::setGains, this);
    SENSOR_ACCGYRO_HZ = 100;
    ACCGYRO_BIQUAD_CUT_HZ = 40;
    ACCGYRO_BUTTERWORTH_Q = 0.707;
    for(int i = 0; i < 3; i++){
	    BiquadInitStateDF2T(&gyro_lpf_biquad[i].state);
	    BiquadUpdateCoeffs(&gyro_lpf_biquad[i].coeffs, SENSOR_ACCGYRO_HZ, ACCGYRO_BIQUAD_CUT_HZ, ACCGYRO_BUTTERWORTH_Q, BIQUAD_TYPE_LPF);
    }
    thrust = 0.0;
    flap1 = 0.5;
    flap2 = 0.5;
    k1 = 0.2;
    k2 = 0.24;
    k3 = 0.05;
    //SENSOR_ACCGYRO_HZ = 25;
    //ACCGYRO_BIQUAD_CUT_HZ = 20;
    //ACCGYRO_BUTTERWORTH_Q = 0.707;
    gyro_u = {0.0, 0.0, 0.0};
    gyro_f = {0.0, 0.0, 0.0};
  }

  void SetThrust(sensor_msgs::Joy msg)
  {
    thrust = msg.axes[2];
    return;
  }

  bool setGains(control::SetGains::Request& req, control::SetGains::Response& res){
    k1 = req.yaw;
    k2 = req.pitch;
    k3 = req.roll;
    return true;
  }

  void SetFlaps(sensor_msgs::Imu msg)
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    gyro_u.at(0) = msg.angular_velocity.x;
    gyro_u.at(1) = msg.angular_velocity.y;
    gyro_u.at(2) = msg.angular_velocity.z;
    for(int i = 0; i < 3; i++){
        gyro_f.at(i) = BiquadDF2TApply(&gyro_lpf_biquad[i], gyro_u.at(i));
	//std::cout.precision(4);
	//std::cout << std::setw(10) << BiquadDF2TApply(&gyro_lpf_biquad[i], gyro_u.at(i)) << "\n";
    }
    sensor_msgs::Imu out;
    out.header.stamp = ros::Time::now();
    out.angular_velocity.x = gyro_f.at(0);
    out.angular_velocity.y = gyro_f.at(1);
    out.angular_velocity.z = gyro_f.at(2);
    flap1 = k1 * -gyro_f.at(1) + k2 * -gyro_f.at(0) + 0.5;
    flap2 = k1 * -gyro_f.at(1) + k2 * gyro_f.at(0) + 0.5;
    thrust_roll = k3 * gyro_f.at(2);
    if(thrust_roll > 0.1){
	 thrust_roll = 0.1;
    }
    if(thrust_roll < -0.1){
	 thrust_roll = -0.1;
    }
    //out.normalized[4] = ((-msg.axes[1]+msg.axes[0]) + 1) / 2; Flap1
    //out.normalized[5] = ((msg.axes[1]+msg.axes[0]) + 1) / 2; Flap2
    imu_pub.publish(out);




   mav_msgs::Actuators command;
   command.normalized.resize(8);
   //if(!counter%1){
   command.normalized[0] = 1.05 * (returnThrust() - returnThrust_roll());
   command.normalized[1] = returnThrust() + returnThrust_roll();
   command.normalized[2] = 0.0;
   command.normalized[3] = 0.0;
   command.normalized[4] = returnFlap1();
   command.normalized[5] = returnFlap2();
   command.normalized[6] = 0.0;
   command.normalized[7] = 0.0;
   if(command.normalized[0] > 1){
	command.normalized[0] = 1;
   }
   if(command.normalized[1] > 1){
	command.normalized[1] = 1;
   }
   if(command.normalized[0] < 0){
	command.normalized[0] = 0;
   }
   if(command.normalized[1] < 0){
	command.normalized[1] = 0;
   }
   //}
   //else{
   //out.normalized[0] = myObject.returnThrust() + 0.1;
   //out.normalized[1] = myObject.returnThrust() - 0.1;
   //out.normalized[2] = 0.0;
   //out.normalized[3] = 0.0;
   //out.normalized[4] = myObject.returnFlap2();
   //out.normalized[5] = myObject.returnFlap2();
   //out.normalized[6] = 0.0;
   //out.normalized[7] = 0.0;
   //if(out.normalized[0] > 1){
	//out.normalized[0] = 1;
   //}
   //if(out.normalized[1] > 1){
	//out.normalized[1] = 1;
   //}
   //if(out.normalized[0] < 0){
	//out.normalized[0] = 0;
   //}
   //if(out.normalized[1] < 0){
	//out.normalized[1] = 0;
   //}
   //}
   rc_pub.publish(command);

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
  float returnThrust_roll(){
      return thrust_roll;
  }

private:
  ros::NodeHandle n;
  ros::Subscriber rc_sub;
  ros::Subscriber imu_sub;
  ros::Publisher imu_pub;
  ros::Publisher rc_pub;
  ros::ServiceServer service;
  float thrust;
  float flap1;
  float flap2;
  float thrust_roll;
  float k1;
  float k2;
  float k3;
  biquad_df2t_t gyro_lpf_biquad[3];
  biquad_type_t BIQUAD_TYPE_LPF;
  float SENSOR_ACCGYRO_HZ;
  float ACCGYRO_BIQUAD_CUT_HZ;
  float ACCGYRO_BUTTERWORTH_Q;
  std::vector<float> gyro_u;
  std::vector<float> gyro_f;
  

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
  ros::init(argc, argv, "control");
  ros::NodeHandle n;
  SubscribeAndPublish myObject;
  ros::ServiceServer service = n.advertiseService("SetGains", &SubscribeAndPublish::setGains, &myObject);
   /* The first NodeHandle constructed will fully initialize this node, and the last
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
   ros::spin();

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */


  return 0;
}
