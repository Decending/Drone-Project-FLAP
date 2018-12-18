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
#include <nav_msgs/Odometry.h>

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
    vicon_sub = n.subscribe("/vicon/flap/flap/odom", 1, &SubscribeAndPublish::SetEulerAngles, this);
    SENSOR_ACCGYRO_HZ = 100;
    ACCGYRO_BIQUAD_CUT_HZ = 40;
    ACCGYRO_BUTTERWORTH_Q = 0.707;
    for(int i = 0; i < 3; i++){
	    BiquadInitStateDF2T(&gyro_lpf_biquad[i].state);
	    BiquadUpdateCoeffs(&gyro_lpf_biquad[i].coeffs, SENSOR_ACCGYRO_HZ, ACCGYRO_BIQUAD_CUT_HZ, ACCGYRO_BUTTERWORTH_Q, BIQUAD_TYPE_LPF);
            BiquadInitStateDF2T(&velocity_lpf_biquad[i].state);
            BiquadUpdateCoeffs(&velocity_lpf_biquad[i].coeffs, SENSOR_ACCGYRO_HZ, ACCGYRO_BIQUAD_CUT_HZ, ACCGYRO_BUTTERWORTH_Q, BIQUAD_TYPE_LPF);
    }
    //Data holders for actuators
    thrust = 0.0;
    flap1 = 0.5;
    flap2 = 0.5;
    flapOffSet = 0.13;

    //Controller gains
    k1 = 0.2;
    k2 = 0.24;
    k3 = 0.05;
    k4 = 10;
    k5 = 10;
    k6 = 10;
    k7 = 0.3;
    k8 = 0.3;
    k9 = 1;
    k10 = 1;

    //Holders for filter data
    gyro_u = {0.0, 0.0, 0.0};
    gyro_f = {0.0, 0.0, 0.0};
    lin_speed_vicon = {0.0, 0.0};
    lin_speed_vicon_f = {0.0, 0.0};

    //Initial data for vicon angles
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;

    //Reference values
    roll_ref = 0.0;
    pitch_ref = 0.0;
    yaw_ref = 0.0;
    ang_speed_ref_x = 0.0;
    ang_speed_ref_y = 0.0;
    ang_speed_ref_z = 0.0;
    lin_speed_ref_x = 0.0;
    lin_speed_ref_y = 0.0;
    x_ref = 0.0;
    y_ref = 4.0;
  }

  void SetThrust(sensor_msgs::Joy msg)
  {
    thrust = msg.axes[2];
    return;
  }

  void SetEulerAngles(nav_msgs::Odometry msg){
      lin_speed_vicon.at(0) = msg.twist.twist.linear.x;
      lin_speed_vicon.at(1) = msg.twist.twist.linear.y;
      for(int i = 0; i < 2; i++){
          lin_speed_vicon_f.at(i) = BiquadDF2TApply(&velocity_lpf_biquad[i], lin_speed_vicon.at(i));
      }
      std::cout.precision(4);
      std::cout << std::setw(10) << lin_speed_vicon_f.at(0) << "\n";
      std::cout << std::setw(10) << lin_speed_vicon_f.at(1) << "\n";
      std::cout << std::setw(10) << "------------------------------" << "\n";

      position_vicon_x = msg.pose.pose.position.x;
      position_vicon_y = msg.pose.pose.position.y;

      double qx = msg.pose.pose.orientation.x;
      double qy = msg.pose.pose.orientation.y;
      double qz = msg.pose.pose.orientation.z;
      double qw = msg.pose.pose.orientation.w;

      // roll (x-axis rotation)
      double sinr_cosp = +2.0 * (qw * qx + qy * qz);
      double cosr_cosp = +1.0 - 2.0 * (qx * qx + qy * qy);
      roll = atan2(sinr_cosp, cosr_cosp);

      // pitch (y-axis rotation)
      double sinp = +2.0 * (qw * qy - qz * qx);
      if (fabs(sinp) >= 1)
              pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      else
              pitch = asin(sinp);

      // yaw (z-axis rotation)
      double siny_cosp = +2.0 * (qw * qz + qx * qy);
      double cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);
      yaw = atan2(siny_cosp, cosy_cosp);
  }

  bool setGains(control::SetGains::Request& req, control::SetGains::Response& res){
    k1 = req.k1;
    k2 = req.k2;
    k3 = req.k3;
    k4 = req.k4;
    k5 = req.k5;
    k6 = req.k6;
    k7 = req.k7;
    k8 = req.k8;
    k9 = req.k9;
    k10 = req.k10;
    x_ref = req.x_ref;
    y_ref = req.y_ref;
    flapOffSet = req.flapOffSet;
    return true;
  }

  void SetFlaps(sensor_msgs::Imu msg)
  {
    lin_speed_ref_x = k9*(x_ref - position_vicon_x);
    lin_speed_ref_y = k10*(y_ref - position_vicon_y);

    roll_ref = k7*(lin_speed_ref_y - lin_speed_vicon_f.at(1));
    pitch_ref = k8*(lin_speed_ref_x - lin_speed_vicon_f.at(0));

    if(roll_ref > 0.1){
        roll_ref = 0.1;
    }
    if(roll_ref < -0.1){
        roll_ref = -0.1;
    }
    if(pitch_ref > 0.1){
        pitch_ref = 0.1;
    }
    if(pitch_ref < -0.1){
        pitch_ref = -0.1;
    }

    ang_speed_ref_x = k4*(roll_ref - roll);
    ang_speed_ref_y = k5*(pitch_ref - pitch);
    ang_speed_ref_z = k6*(yaw_ref - yaw);

    //defined so we can print it
    gyro_u.at(0) = msg.angular_velocity.x;
    gyro_u.at(1) = msg.angular_velocity.y;
    gyro_u.at(2) = msg.angular_velocity.z;

    for(int i = 0; i < 3; i++){
        gyro_f.at(i) = BiquadDF2TApply(&gyro_lpf_biquad[i], gyro_u.at(i));
    }
    sensor_msgs::Imu out;
    out.header.stamp = ros::Time::now();
    out.angular_velocity.x = gyro_f.at(0);
    out.angular_velocity.y = gyro_f.at(1);
    out.angular_velocity.z = gyro_f.at(2);
    flap1 = k1 * (-ang_speed_ref_z-gyro_f.at(1)) + k2 * (ang_speed_ref_y-gyro_f.at(0)) + 0.5 + flapOffSet;
    flap2 = k1 * (-ang_speed_ref_z-gyro_f.at(1)) + k2 * (gyro_f.at(0)-ang_speed_ref_y) + 0.5 - flapOffSet;
    thrust_roll = k3 *(ang_speed_ref_x + gyro_f.at(2));
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
   command.normalized[0] = returnThrust() - returnThrust_roll();
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
   if(command.normalized[4] < 0){
	   command.normalized[4] = 0;
   }
   if(command.normalized[4] > 1){
        command.normalized[4] = 1;
   }
   if(command.normalized[5] < 0){
	   command.normalized[5] = 0;
   }
   if(command.normalized[5] > 1){
	   command.normalized[5] = 1;
   }
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
  ros::Subscriber vicon_sub;
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
  float k4;
  float k5;
  float k6;
  float k7;
  float k8;
  float k9;
  float k10;
  float x_ref;
  float y_ref;
  float flapOffSet;
  float roll;
  float pitch;
  float yaw;
  float roll_ref;
  float pitch_ref;
  float yaw_ref;
  float ang_speed_ref_x;
  float ang_speed_ref_y;
  float ang_speed_ref_z;
  float lin_speed_ref_x;
  float lin_speed_ref_y;
  float position_vicon_x;
  float position_vicon_y;
  biquad_df2t_t gyro_lpf_biquad[3];
  biquad_df2t_t velocity_lpf_biquad[3];
  biquad_type_t BIQUAD_TYPE_LPF;
  float SENSOR_ACCGYRO_HZ;
  float ACCGYRO_BIQUAD_CUT_HZ;
  float ACCGYRO_BUTTERWORTH_Q;
  std::vector<float> gyro_u;
  std::vector<float> gyro_f;
  std::vector<float> lin_speed_vicon;
  std::vector<float> lin_speed_vicon_f;
  

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
