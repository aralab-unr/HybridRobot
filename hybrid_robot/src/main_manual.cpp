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
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atoi */
#include "hybridRobot/DynamixelCommand.h"
#include <string>
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
// For non-blocking keyboard inputs
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
void sendData(int serialport,string A,double x){
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
//int32_t servo1_val1=0;
//int32_t servo1_val2=0;
//int32_t servo1_val3=276000;
//int32_t servo1_val4=-135000;
//int32_t servo1_val5=0;
//int32_t servo1_val6=85000;

int32_t servo1_val1=0;
int32_t servo1_val2=0;
int32_t servo1_val3=0;
int32_t servo1_val4=0;
int32_t servo1_val5=0;
int32_t servo1_val6=0;

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

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "Robot_control");
  ros::NodeHandle nh;
  //ros::ServiceClient client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("dynamixel_workbench_msgs/DynamixelCommand");
  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  geometry_msgs::Twist twist;
  ros::ServiceClient client = nh.serviceClient<hybridRobot::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  hybridRobot::DynamixelCommand srv;
  setupserial();
  int mode=0;
  // for(int l=1;l<=6;l++){
  //   srv.request.command="";
  //     srv.request.id=l;
  //     srv.request.addr_name="Goal_Position";
  //     if(l==1) srv.request.value=servo1_val1;
  //     else if(l==2) srv.request.value=servo1_val2;
  //     else if(l==3) srv.request.value=servo1_val3;
  //     else if(l==4) srv.request.value=servo1_val4;
  //     else if(l==5) srv.request.value=servo1_val5;
  //     else if(l==6) srv.request.value=servo1_val6;
  //     client.call(srv);
  // }
  while(ros::ok()){

    // Get the pressed key
    key = getch();
    cout<<key<<endl;
    if(key=='1') {sendData(3,"M",200);cout<<"send 200 to updown1"<<endl;}
    if(key=='2') {sendData(3,"M",-200);cout<<"send -200 to updown1"<<endl;}
    if(key=='3') {sendData(3,"M",0);cout<<"send 0 to updown1"<<endl;}
    if(key=='4') {sendData(4,"M",200);cout<<"send 200 to updown2"<<endl;}
    if(key=='5') {sendData(4,"M",-200);cout<<"send -200 to updown2"<<endl;}
    if(key=='6') {sendData(4,"M",0);cout<<"send 0 to updown2"<<endl;}

    if(key=='+') {cout<<"tang speed"<<speed_robot<<endl;speed_robot++;}
    if(key=='-') {cout<<"giam speed"<<speed_robot<<endl;speed_robot--;}
    if(key=='o') {move_toi();cout<<"toi"<<endl;}
    else if(key=='l') {move_lui();cout<<"lui"<<endl;}
    else if(key=='k') {move_trai();cout<<"trai"<<endl;}
    else if(key==';') {move_phai();cout<<"phai"<<endl;}
    else {stop();cout<<"stop"<<endl;}
    
    int x=1000;
    if(key=='q')      {mode=1;servo1_val1=servo1_val1+x; if(servo1_val1>500000)  servo1_val1=500000;cout<<servo1_val1<<endl;}
    else if(key=='a') {mode=1;servo1_val1=servo1_val1-x; if(servo1_val1<-500000) servo1_val1=-500000;cout<<servo1_val1<<endl;}
    if(key=='w')      {mode=2;servo1_val2=servo1_val2+x; if(servo1_val2>500000)  servo1_val2=500000;cout<<servo1_val2<<endl;}
    else if(key=='s') {mode=2;servo1_val2=servo1_val2-x; if(servo1_val2<-500000) servo1_val2=-500000;cout<<servo1_val2<<endl;}
    if(key=='e')      {mode=3;servo1_val3=servo1_val3+x; if(servo1_val3>500000)  servo1_val3=500000;cout<<servo1_val3<<endl;}
    else if(key=='d') {mode=3;servo1_val3=servo1_val3-x; if(servo1_val3<-500000) servo1_val3=-500000;cout<<servo1_val3<<endl;}
    if(key=='r')      {mode=4;servo1_val4=servo1_val4+x; if(servo1_val4>500000)  servo1_val4=500000;cout<<servo1_val4<<endl;}
    else if(key=='f') {mode=4;servo1_val4=servo1_val4-x; if(servo1_val4<-500000) servo1_val4=-500000;cout<<servo1_val4<<endl;}
    if(key=='t')      {mode=5;servo1_val5=servo1_val5+x; if(servo1_val5>500000)  servo1_val5=500000;cout<<servo1_val5<<endl;}
    else if(key=='g') {mode=5;servo1_val5=servo1_val5-x; if(servo1_val5<-500000) servo1_val5=-500000;cout<<servo1_val5<<endl;}
    if(key=='y')      {mode=6;servo1_val6=servo1_val6+x; if(servo1_val6>500000)  servo1_val6=500000;cout<<servo1_val6<<endl;}
    else if(key=='h') {mode=6;servo1_val6=servo1_val6-x; if(servo1_val6<-500000) servo1_val6=-500000;cout<<servo1_val6<<endl;}
    cout<<"mode "<<mode<<endl;
    if(mode!=0){
      srv.request.command="";
      srv.request.id=mode;
      srv.request.addr_name="Goal_Position";
      if(mode==1) srv.request.value=servo1_val1;
      else if(mode==2) srv.request.value=servo1_val2;
      else if(mode==3) srv.request.value=servo1_val3;
      else if(mode==4) srv.request.value=servo1_val4;
      else if(mode==5) srv.request.value=servo1_val5;
      else if(mode==6) srv.request.value=servo1_val6;
      client.call(srv);
    }
    mode=0;
    if (key == '\x03')
      {
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }
    // Publish it and resolve any remaining callbacks
    pub.publish(twist);
    ros::spinOnce();
  }

  return 0;
}
