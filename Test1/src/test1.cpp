#include "ros/ros.h"
#include <iostream>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/default_topics.h>
#include <std_msgs/Float64.h>
#include <test1/SetVariables.h>

//The testfile for rolling, done by altering the input between the two engines

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
      //Default values for variables
      myFrequency = 10;
      myMotorSpeed = 0.5;
  }
  
  //The service response function to reset the variables
  bool setVariables(test1::SetVariables::Request& req, test1::SetVariables::Response& res){
    myFrequency = req.Frequency;
    myMotorSpeed = req.MotorSpeed;
    return true;
  }

  //Functions to return variables outside of the test node
  float returnFrequency(){
      return myFrequency;
  }
  double returnMotorSpeed(){
      return myMotorSpeed;
  }

private:
  float myFrequency;
  double myMotorSpeed;
};

//The main function to set up the test node
int main(int argc, char **argv)
{ 
  
  ros::init(argc, argv, "control");
  ros::NodeHandle n;
  SubscribeAndPublish myObject;
  ros::ServiceServer service = n.advertiseService("SetVariables", &SubscribeAndPublish::setVariables, &myObject);
  ros::Publisher control_pub = n.advertise< mav_msgs::Actuators >( mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
  ros::Rate loop_rate(10);
  int count = 0;
  
  //Main loop
  while (ros::ok())
  {
    
    /* Creating and filling the command with data
     *  - Index 0 and 1 are for the engines on the drone
     *  - Index 4 and 5 are for the flaps on the drone
     * The flaps are mirrored on the drone and they take inputs from 0 to 1
     * hence input 0.2 to one flap corresponds to input 0.8 to the other
     * The engines have different RPM with the same input and is corrected by multiplication
    */
    
    mav_msgs::Actuators out;
    out.normalized.resize(8);
    if(count % 2 == 0){
    out.normalized[0] = myObject.returnMotorSpeed() * 0.952;
    out.normalized[1] = 0.0;
    out.normalized[2] = 0.0;
    out.normalized[3] = 0.0;
    out.normalized[4] = 0.5;
    out.normalized[5] = 0.5;
    out.normalized[6] = 0.0;
    out.normalized[7] = 0.0;
    }
    else{
    out.normalized[0] = 0.0;
    out.normalized[1] = myObject.returnMotorSpeed();
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
