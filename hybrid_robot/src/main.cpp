#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16.h>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atoi */
#include "climb3/DynamixelCommand.h"
#include <string>
#include <Eigen/Dense>
#include <map>
using namespace std;

serial::Serial serialWheel1;
serial::Serial serialWheel2;
serial::Serial serialUpDown1;
serial::Serial serialUpDown2;
string  wheel1_port="/dev/ttyUSB2";
string  wheel2_port="/dev/ttyUSB4";
string  updown1_port="/dev/ttyUSB1";
string  updown2_port="/dev/ttyUSB3";

// Init variables

char key(' ');
double speed_robot=20;

int32_t servo1_val1=0;  // joint 6
int32_t servo1_val2=0;  // joint 5
int32_t servo1_val3=276000; // joint 4
int32_t servo1_val4=-135000;  // joint 3
int32_t servo1_val5=0;      // joint 2
int32_t servo1_val6=85000;  // joint 1

int32_t targetJointPos1=0;
int32_t targetJointPos2=0;
int32_t targetJointPos3=0;
int32_t targetJointPos4=0;
int32_t targetJointPos5=0;
int32_t targetJointPos6=0;

bool change = true;

// For non-blocking keyboard inputs - get a char from keyboard
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);
  // Get the current character
  ch = getchar();
  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

// send Data to a motor
void sendData(int serialport, string A,double x){
    string Data= string(A+to_string(x)+'?');
    int n = Data.length(); 
    char data[n + 1]; 
    strcpy(data, Data.c_str()); 
    if(serialport==1) serialWheel1.write(data);
    else if(serialport==2) serialWheel2.write(data);
    else if(serialport==3) serialUpDown1.write(data);
    else if(serialport==4) serialUpDown2.write(data);
}

void move_lui()
{
  sendData(1,"L",speed_robot);
  sendData(1,"R",speed_robot);
  sendData(2,"L",speed_robot);
  sendData(2,"R",speed_robot);
}
void move_toi()
{
  sendData(1,"L",-speed_robot);
  sendData(1,"R",-speed_robot);
  sendData(2,"L",-speed_robot);
  sendData(2,"R",-speed_robot);
}
void move_phai()
{
  sendData(1,"L",-speed_robot);
  sendData(1,"R",speed_robot);
  sendData(2,"L",-speed_robot);
  sendData(2,"R",speed_robot);
}
void move_trai()
{
  sendData(1,"L",speed_robot);
  sendData(1,"R",-speed_robot);
  sendData(2,"L",speed_robot);
  sendData(2,"R",-speed_robot);
}
void stop()
{
  sendData(1,"L",0);
  sendData(1,"R",0);
  sendData(2,"L",0);
  sendData(2,"R",0);
}

int setupserial()
{
    try
    {
        serialUpDown1.setPort(updown1_port);
        serialUpDown1.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialUpDown1.setTimeout(to);
        serialUpDown1.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port updown1 ");
        return -1;
    }

    if(serialUpDown1.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    try
    {
        serialUpDown2.setPort(updown2_port);
        serialUpDown2.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialUpDown2.setTimeout(to);
        serialUpDown2.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port updown2 ");
        return -1;
    }

    if(serialUpDown2.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    try
    {
        serialWheel1.setPort(wheel1_port);
        serialWheel1.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialWheel1.setTimeout(to);
        serialWheel1.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port wheel1_port");
        return -1;
    }

    if(serialWheel1.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    try
    {
        serialWheel2.setPort(wheel2_port);
        serialWheel2.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialWheel2.setTimeout(to);
        serialWheel2.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port wheel2_port");
        return -1;
    }

    if(serialWheel2.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
}
void jointState_wormingCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // Eigen::VectorXd jointstate;
  std::cout << "receiving the joint position - outside the loop" << std::endl;
  for (int i=0; i<6;i++)
  {
    if (i==0) targetJointPos1 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==1) targetJointPos2 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==2) targetJointPos3 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==3) targetJointPos4 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==4) targetJointPos5 = int32_t(msg->data[i]*500000/3.141592653);
    if (i==5) targetJointPos6 = int32_t(msg->data[i]*500000/3.141592653);
    // targetJointPos3 = 5000;
    // targetJointPos4 = -5000;
    std::cout << "receiving the joint position" << std::endl;
  }
  // targetJointPos2 = 0;
  // targetJointPos3 = 0;
  // targetJointPos5 = 0;
  // targetJointPos6 = 0;
  // targetJointPos1 = 0;
  change = true;
}


int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "Robot_control");
  ros::NodeHandle nh("~");
  
  // Init cmd_vel publisher
  // ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  // geometry_msgs::Twist twist;
  ros::ServiceClient client = nh.serviceClient<climb3::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  climb3::DynamixelCommand srv;
  setupserial();
  ros::Rate loop_rate(1);
  // int mode=0;

  bool convPose_state, move_1leg, move_2leg;
  nh.param("convPose_state", convPose_state, false);
  nh.param("move_1leg", move_1leg, false);
  nh.param("move_2leg", move_2leg, false);

  
  int  distance1, distance2;
  nh.param("distance1", distance1, 51);
  nh.param("distance2", distance2, 51);

  ros::Subscriber tra_sub = nh.subscribe("/trajectory_processing/jointState_worming", 5, jointState_wormingCallback);
  
  ros::Publisher jp_pub1 = nh.advertise<std_msgs::UInt16>("/setpoint1", 1);
  ros::Publisher jp_pub2 = nh.advertise<std_msgs::UInt16>("/setpoint2", 1);

  std_msgs::UInt16 Pos1, Pos2; 

  bool worming_mode, mobile_mode;

  nh.param("worming_mode", worming_mode, false);
  nh.param("mobile_mode", mobile_mode, false);
  bool switching_controller = true;
  bool area_check = false, plane_check = false, height = false;

  while(ros::ok())
  {

    // the switching controller
    // if (switching_controller)
    // {
    //   // asking service to process the point cloud to return area_check, plane_check, height_check
    //   bool switching = false;
    //   bool keep_working = false;
    //   keep_working = area_check*plane_check;
    //   if (keep_working)
    //   {
    //     switching = height;
    //     if (switching)
    //       mobile_mode = true;
    //     else
    //       worming_mode = true;
    //   }
    // }

    if (mobile_mode)
    {
      std::cout << "The mobile mode is turned on " << std::endl;
      switching_controller = true;
      // asking a service to take a point cloud data
      srv.request.command="";
      srv.request.addr_name="Goal_Position";
      
      // moving the joint 6 the base joint - first
      srv.request.id=1;
      srv.request.value = 0;
      client.call(srv);
      ros::Duration(1.0).sleep();

      //moving the joint 4 - the last
      srv.request.id=3;
      srv.request.value = servo1_val3;
      client.call(srv);
      ros::Duration(2.0).sleep();
      
     //moving the joint 5 - the last
      srv.request.id=2;
      srv.request.value = 0;
      client.call(srv);

     //moving the joint 2 - the last
      srv.request.id=5;
      srv.request.value = 0;
      client.call(srv);

      //moving the joint 3 - the second
      srv.request.id=4;
      srv.request.value = servo1_val4;
      client.call(srv);
      ros::Duration(2.0).sleep();

      // moving the joint 1 the base joint - first
      srv.request.id=6;
      srv.request.value = servo1_val6;
      client.call(srv);
      mobile_mode = false;

    }
    if (worming_mode)
    {
      // asking a service to have a pointcloud  --------------------------------------------
      // asking another service to process a point cloud to have Pose ---------------------------------
      
      Eigen::MatrixXd trajectory1(10,6);

      trajectory1(0,0) = -2340;
      trajectory1(0,1) = -25;
      trajectory1(0,2) = -849;
      trajectory1(0,3) = 2533;
      trajectory1(0,4) = 46;
      trajectory1(0,5) = -2635;
      
      trajectory1(1,0) = -7016;
      trajectory1(1,1) = -75;
      trajectory1(1,2) = -2546;
      trajectory1(1,3) = 7953;
      trajectory1(1,4) = 140;
      trajectory1(1,5) = -7900;

      trajectory1(2,0) = -17271;
      trajectory1(2,1) = -184;
      trajectory1(2,2) = -6251;
      trajectory1(2,3) = 18638;
      trajectory1(2,4) = 344;
      trajectory1(2,5) = -19392;

      trajectory1(3,0) = -31357;
      trajectory1(3,1) = -336;
      trajectory1(3,2) = -11381;
      trajectory1(3,3) = 33935;
      trajectory1(3,4) = 628;
      trajectory1(3,5) = -35307;

      trajectory1(4,0) = -45491;
      trajectory1(4,1) = -488;
      trajectory1(4,2) = -16512;
      trajectory1(4,3) = 49232;
      trajectory1(4,4) = 911;
      trajectory1(4,5) = -51223;

      trajectory1(5,0) = -59626;
      trajectory1(5,1) = -640;
      trajectory1(5,2) = -21642;
      trajectory1(5,3) = 64530;
      trajectory1(5,4) = 1194;
      trajectory1(5,5) = -67139;

      trajectory1(6,0) = -73471;
      trajectory1(6,1) = -789;
      trajectory1(6,2) = -26668;
      trajectory1(6,3) = 79514;
      trajectory1(6,4) = 1471;
      trajectory1(6,5) = -82728;

      trajectory1(7,0) = -82186;
      trajectory1(7,1) = 882;
      trajectory1(7,2) = -29831;
      trajectory1(7,3) = 88945;
      trajectory1(7,4) = 1645;
      trajectory1(7,5) = -92541;

      trajectory1(8,0) = -86861;
      trajectory1(8,1) = -932;
      trajectory1(8,2) = -31528;
      trajectory1(8,3) = 94004;
      trajectory1(8,4) = 1739;
      trajectory1(8,5) = -97805;

      trajectory1(9,0) = -88393;
      trajectory1(9,1) = -949;
      trajectory1(9,2) = -32084;
      trajectory1(9,3) = 95663;
      trajectory1(9,4) = 1770;
      trajectory1(9,5) = -99530;

      // trajectory1(5,0) = -4367;
      // trajectory1(5,1) = 98;
      // trajectory1(5,2) = -32663;
      // trajectory1(5,3) = 35127;
      // trajectory1(5,4) = 406;
      // trajectory1(5,5) = -30596;
      int32_t convPos_joint1=-75000, convPos_joint3=-50000, convPos_joint4=140000;
      int32_t convInter_j1=-150000, convInter_j3=-130000, convInter_j4=272000;
      int32_t convInter1_j3=-165000, convInter1_j4=285000;

      
      // move down permanent magnet of the second leg ---------------------------------------
      // move up permanent magnet of the first leg -------------------------------------------

      // moving to the convenient Pose
      if (convPose_state)
      {
        ros::Duration(3.0).sleep();
        Pos1.data = distance1;
        jp_pub1.publish(Pos1);
        ros::Duration(15.0).sleep();
        Pos1.data = 200;
        jp_pub1.publish(Pos1);
        Pos2.data = 200;
        jp_pub2.publish(Pos2);

        srv.request.command="";
        srv.request.addr_name="Goal_Position";
        //moving the joint 3 - the second
        // srv.request.id=4;
        // // srv.request.value = convPos_joint3;
        // srv.request.value = convInter_j3;
        // client.call(srv);
        // ros::Duration(2.0).sleep();
        // moving the joint 6 - keep the safe distance between two feet
        srv.request.id=1;
        // srv.request.value = convPos_joint4;
        srv.request.value = -7000;
        client.call(srv);
        // // if (check)
        // //   std::cout << "check is done successfully ----------------------- " << std::endl;
        // ros::Duration(2.0).sleep();
        // moving the joint 1 the base joint - first
        srv.request.id=6;
        // srv.request.value = convPos_joint1;
        srv.request.value = convPos_joint1;
        client.call(srv);
        ros::Duration(3.0).sleep();
        //moving the joint 3 - the second
        srv.request.id=4;
        // srv.request.value = convPos_joint3;
        srv.request.value = convInter1_j3;
        client.call(srv);
        
        //moving the joint 4 - keep the safe distance between two feet
        srv.request.id=3;
        // srv.request.value = convPos_joint4;
        srv.request.value = convInter1_j4;
        client.call(srv);
        ros::Duration(7.0).sleep();


        //moving the joint 4 - keep the safe distance between two feet
        srv.request.id=3;
        srv.request.value = convPos_joint4;
        // srv.request.value = convInter_j4;
        client.call(srv);
        ros::Duration(9.0).sleep();

        srv.request.id=4;
        srv.request.value = convPos_joint3;
        // srv.request.value = convInter_j3;
        client.call(srv);
        ros::Duration(10.0).sleep();

        // if (check)
        //   std::cout << "check is done successfully ----------------------- " << std::endl;
        // ros::Duration(20.0).sleep();

        move_1leg = true;
        convPose_state = false;
      }    

      if (move_1leg)
      {
        for (int k = 0; k<10; k++)
          for (int j = 0; j < 6; j++)
          {
            srv.request.command="";
            srv.request.id=j+1;
            srv.request.addr_name="Goal_Position";

            if(j==0) 
            {
              // safety condition for joints
              if (trajectory1(k,5) > 167120)
              {
                ROS_ERROR("Joint 6 go over upper limit");
                srv.request.value = 167120;
              }
              else if (trajectory1(k,5) < -167120)
              {
                ROS_ERROR("Joint 6 go over lower limit");
                srv.request.value = -167120;
              }
              else
                srv.request.value = trajectory1(k,5);  // joint 6   - counter clockwise - positive
              client.call(srv);
              ros::Duration(0.5).sleep();
            }
            if(j==1) 
            {
              // safety condition for joints
              if (trajectory1(k,4) > 445640)
              { 
                srv.request.value = 445640;
                ROS_ERROR("Joint 5 go over upper limit");
              }
              else if (trajectory1(k,4) < -445640)
              {
                ROS_ERROR("Joint 5 go over lower limit");
                srv.request.value = -445640;
              }  
              else
                srv.request.value = trajectory1(k,4);  // joint 5  - counter clockwise - positive
              client.call(srv);
              ros::Duration(0.5).sleep();
            }
            if(j==2) 
            {
              int32_t convert4 = trajectory1(k,3) - 139807;
              // safety condition for joints
              // if (trajectory1(k,3) > -7970) 
              if (convert4 > -7970) 
              {
                srv.request.value = 7970;
                ROS_ERROR("Joint 4 go over lower limit");
              }
              else if (convert4 < -238740)
              {
                srv.request.value = 238740;
                ROS_ERROR("Joint 4 go over upper limit");
              }  
              else
                srv.request.value = -convert4; // joint 4   - clockwise - positive
              client.call(srv);
              ros::Duration(0.5).sleep();
            }
            if(j==3) 
            {
              int32_t convert3 = trajectory1(k,2) - 50000;
              // safety condition for joints
              // if (trajectory1(k,2) > -4785) 
              if (convert3 > -4785) 
              {
                srv.request.value = -4785;
                ROS_ERROR("Joint 3 go over upper limit");
              }
              else if (convert3 < -238740)
              {
                srv.request.value = -238740;
                ROS_ERROR("Joint 3 go over lower limit");
              }
              else              
                srv.request.value = convert3;  // joint 3   - counter clockwise - positive
              client.call(srv);
              ros::Duration(0.5).sleep();
            }
            if(j==4) 
            {
              // safety condition for joints
              if (trajectory1(k,1) > 445640) 
              {
                srv.request.value = 445640;
                ROS_ERROR("Joint 2 go over upper limit");
              }
              else if (trajectory1(k,1) < -445640)
              {
                srv.request.value = -445640;
                ROS_ERROR("Joint 2 go over lower limit");
              }
              else
                srv.request.value = trajectory1(k,1);  // joint 2    - counter clockwise - positive
              client.call(srv);
              ros::Duration(0.5).sleep();
            }
            if(j==5) 
            {
              int32_t convert1 = trajectory1(k,0) + 64909;
              // if (trajectory1(k,0) > 167120) 
              if (convert1 > 167120) 
              {
                ROS_ERROR("Joint 1 go over lower limit");
                srv.request.value = -167120;
              }
              else if (convert1 < -167120)
              {
                ROS_ERROR("Joint 1 go over upper limit");
                srv.request.value = 167120;
              }
              else
                   srv.request.value = -convert1; // joint 1    - clockwise - positive
              client.call(srv);
              ros::Duration(0.5).sleep();
            }
            
          }
        ros::Duration(3.0).sleep();
        // move down the permanent magnet of the first leg ---
        Pos2.data = distance2;
        jp_pub2.publish(Pos2);
        ros::Duration(15.0).sleep();
        Pos2.data = 200;
        jp_pub2.publish(Pos2);

        // move up the permanent magnet of the second leg ----
        Pos1.data = distance1+10;
        jp_pub1.publish(Pos1);
        ros::Duration(13.0).sleep();
        Pos1.data = 200;
        jp_pub1.publish(Pos1);

        move_2leg = true;
        move_1leg = false;       
      }
      
      
      

      // moving to the initial Pose
      if (move_2leg)
      {
        srv.request.command="";
        srv.request.addr_name="Goal_Position";
        
        // moving the joint 1 the base joint - first
        srv.request.id=1;
        srv.request.value = 0;
        client.call(srv);


        // srv.request.id=6;
        // srv.request.value = convInter_j1;
        // client.call(srv);
        ros::Duration(4.0).sleep();


        //moving the joint 4 - the last
        srv.request.id=3;
        srv.request.value = servo1_val3;
        client.call(srv);
        ros::Duration(17.0).sleep();
        


        srv.request.id=2;
        srv.request.value = 0;
        client.call(srv);

        srv.request.id=5;
        srv.request.value = 0;
        client.call(srv);

        //moving the joint 3 - the second
        srv.request.id=4;
        srv.request.value = servo1_val4;
        client.call(srv);
        ros::Duration(5.0).sleep();


        // moving the joint 1 the base joint - first
        srv.request.id=6;
        srv.request.value = servo1_val6;
        client.call(srv);
        ros::Duration(6.0).sleep();

        Pos1.data = distance1;
        jp_pub1.publish(Pos1);
        ros::Duration(11.0).sleep();
        Pos1.data = 200;
        jp_pub1.publish(Pos1);

        move_2leg = false;
      }
      // switching_controller = true;
      worming_mode = false;

      // move down the permanent magnet of the second leg ---------------------------------------------------
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
  // ros::spinOnce();

  return 0;
}
