#ifndef MP_SERVER_H_
#define MP_SERVER_H_

// ROS
#include <ros/ros.h>
// #include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/transform_broadcaster.h>

#include "hybrid_robot/mp_service.h"

using movGroInterface = moveit::planning_interface::MoveGroupInterface;
using rs_JMG = robot_state::JointModelGroup*;
using movVisTools = moveit_visual_tools::MoveItVisualTools;

namespace mp_server
{
  class mpServer
  {
    public:     // public function
      // constructor with a point cloud topic
      mpServer(std::string planningGroup, std::string baseLink);
      // mpServer(std::string PLANNING_GROUP, movGroInterface movGroup, movVisTools visTools);
      // ~mpServer();
      
      // initialize function
      // void initPublisher();
      // void initSubscriber();
      void initServer();

      // list of propressing functions

      // list of Callbackfunction
      // void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
      bool mp_processing(hybridRobot::mp_service::Request &req, hybridRobot::mp_service::Response &res);


    private:
      ros::NodeHandle nh_; // general ROS NodeHandle - used for pub, sub, advertise ...
      ros::NodeHandle nh_private_; // private ROS NodeHandle - used to get parameter from server
      // ros::Publisher mpServer_pose_pub_;
      // ros::Subscriber mpServer_pclCam_sub_;
      // ROS Service Server
      ros::ServiceServer mpServer_traj_;

      // Robot name
      movGroInterface *move_group_;
      const robot_state::JointModelGroup* joint_model_group_;
      //visualizing objects, robots, & trajectories in RViz, debugging tools by step-by-step introspection of a script
      // namespace rvt_ = rviz_visual_tools;
      movVisTools *visual_tools_;
  };
}

#endif /* MP_SERVER_ */
