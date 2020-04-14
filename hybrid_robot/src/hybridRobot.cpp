#include "ros/ros.h"
#include "hybrid_robot/pointcloud_cmd.h"
#include "hybrid_robot/mp_service.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/JointState.h>

// #include <geometry_msgs/Twist.h>
#include <serial/serial.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
// #include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
// #include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16.h>
// #include <stdlib.h>     /* atoi */
#include "hybrid_robot/DynamixelCommand.h"
// #include <string>
#include <Eigen/Dense>
// #include <map>


int32_t mobConf_j1 = 0;       // joint 6
int32_t mobConf_j2 = 0;       // joint 5
int32_t mobConf_j3 = 276000;  // joint 4
int32_t mobConf_j4 = -135000; // joint 3
int32_t mobConf_j5 = 0;       // joint 2
int32_t mobConf_j6 = 85000;   // joint 1

std::vector<std::vector<double>> list_trajectories;

serial::Serial serialWheel1;
serial::Serial serialWheel2;
serial::Serial serialUpDown1;
serial::Serial serialUpDown2;
std::string  wheel1_port="/dev/ttyUSB2";
std::string  wheel2_port="/dev/ttyUSB4";
std::string  updown1_port="/dev/ttyUSB1";
std::string  updown2_port="/dev/ttyUSB3";

bool start = true;

void traj_Callback(const sensor_msgs::JointState::ConstPtr& msg) {
  std::vector<double> jointstate = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // std::cout << "receiving data from the fake_controller " << std::endl;
  for (int i=0; i<6; i++) {
    jointstate[i] = double(msg->position[i]);
  }
  list_trajectories.push_back(jointstate);
}

int setupserial() {
  try {
    serialUpDown1.setPort(updown1_port);
    serialUpDown1.setBaudrate(57600);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serialUpDown1.setTimeout(to);
    serialUpDown1.open();
  }
  catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open port updown1 ");
    return -1;
  }
  if(serialUpDown1.isOpen()){
    ROS_INFO_STREAM("Serial Port initialized");
  }
  else {
      return -1;
  }
  try {
      serialUpDown2.setPort(updown2_port);
      serialUpDown2.setBaudrate(57600);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      serialUpDown2.setTimeout(to);
      serialUpDown2.open();
  }
  catch (serial::IOException& e) {
      ROS_ERROR_STREAM("Unable to open port updown2 ");
      return -1;
  }
  if(serialUpDown2.isOpen()){
      ROS_INFO_STREAM("Serial Port initialized");
  }
  else {
      return -1;
  }
  try {
      serialWheel1.setPort(wheel1_port);
      serialWheel1.setBaudrate(57600);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      serialWheel1.setTimeout(to);
      serialWheel1.open();
  }
  catch (serial::IOException& e) {
      ROS_ERROR_STREAM("Unable to open port wheel1_port");
      return -1;
  }
  if (serialWheel1.isOpen()){
    ROS_INFO_STREAM("Serial Port initialized");
  }
  else {
    return -1;
  }
  try {
    serialWheel2.setPort(wheel2_port);
    serialWheel2.setBaudrate(57600);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serialWheel2.setTimeout(to);
    serialWheel2.open();
  }
  catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open port wheel2_port");
    return -1;
  }
  if (serialWheel2.isOpen()) {
      ROS_INFO_STREAM("Serial Port initialized");
  }
  else {
    return -1;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hybridRobot");
  ros::NodeHandle n;
  ros::NodeHandle n_para("~");

  // service to generate a trajectory
  ros::ServiceClient client_traj = n.serviceClient<hybrid_robot::mp_service>("mpServer/trajectory");
  // service to control the motor
  ros::ServiceClient client_motor = n.serviceClient<hybrid_robot::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  
  // service to check plane and area
  ros::ServiceClient client_planeArea = n.serviceClient<hybrid_robot::plane_area>("/pcl_server/plane_area");

  // service to check the height
  ros::ServiceClient client_height = n.serviceClient<hybrid_robot::height_check>("/pcl_server/height_check");
  // service to estimate the pose
  ros::ServiceClient client_pose = n.serviceClient<hybrid_robot::pointcloud_cmd>("pointcloud_cmd");

  // service to control the magnetic array
  ros::ServiceClient client_magArr = n.serviceClient<hybrid_robot::>("magArray/updown");


  // subcribe  the trajectory topic
  ros::Subscriber sub_traj = n.subscribe("/move_group/fake_controller_joint_states", 50, traj_Callback);

  hybrid_robot::pointcloud_cmd srv_pose;
  hybrid_robot::mp_service srv_traj;
  hybrid_robot::DynamixelCommand srv_motor;
  ros::Rate loop_rate(50);

  setupserial();
  // arguments for pointcloud processing
  std::string addressToSave;
  n_para.param("addressToSave", addressToSave, std::string("/home/buivn/bui_ws/src/pahap/pointcloud/"));  
  bool requestTopic;
  n_para.param("requestTopic", requestTopic, true);

  // parameter for worming transformation
  bool convPose_state, move_1leg, move_2leg;
  nh.param("convPose_state", convPose_state, false);
  nh.param("move_1leg", move_1leg, false);
  nh.param("move_2leg", move_2leg, false);

  int  distance1, distance2;
  nh.param("distance1", distance1, 51);
  nh.param("distance2", distance2, 51);

  bool worming_mode, mobile_mode;
  nh.param("worming_mode", worming_mode, false);
  nh.param("mobile_mode", mobile_mode, false);
  bool switching_controller = true;
  bool area_check = false, plane_check = false, height = false;
  std::vector<double> oneTraj = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  int i = 0;
  while(ros::ok()) {
    



    if (i<1) {
      // set for the service request
      srv_pose.request.num_name = std::to_string(i);
      srv_pose.request.cmd = true;
      srv_pose.request.path = addressToSave; 
      srv_pose.request.topic = requestTopic;
      client_pose.call(srv_pose);
      // if (client.call(srv_pose))
      if ((srv_pose.response.planePose.position.x == 0) and (srv_pose.response.planePose.position.y == 0) and (srv_pose.response.planePose.position.z == 0)) {
        ROS_INFO("The is no pose detected");
      }
      else {
        ROS_INFO("One Pose is detected: ");
        // call the trajectories service
        srv_traj.request.cmd = true;
        srv_traj.request.targetPose = srv_pose.response.planePose;
        if (client_traj.call(srv_traj))
          if (srv_traj.response.result) 
            ROS_INFO("The trajectory is generated");
        // ROS_INFO("Position: z = [%f]", srv_pose.response.planePose.position.z);
      }
    }
    i += 1;

    if (not (list_trajectories.size() == 0)) {
      for (int k=0; k < 6; k++) {        
        // get the first data of the list_trajectories
        // std::cout << "Dimension of list before:" << list_trajectories.size() << std::endl;
        oneTraj[k] = list_trajectories[0][k];
        // oneTraj[k] += 0.052; 
      }
      // erase the first data on the list_trajectories
      list_trajectories.erase(list_trajectories.begin());


    }

    


    ros::spinOnce();
    loop_rate.sleep();
  } 


  return 0;
}