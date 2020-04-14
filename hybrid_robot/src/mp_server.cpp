/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Bui, Hoang-Dung
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Bui, Hoang-Dung */
#include <ros/ros.h>
#include <hybrid_robot/mp_server.h>


namespace mp_server
{
  //constructor
  mpServer::mpServer(std::string planningGroup, std::string baseLink):
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
  { 
    move_group_ = new movGroInterface(planningGroup);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.01);
    joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup(planningGroup);
    visual_tools_ = new movVisTools(baseLink);
    // visual_tools_.deleteAllMarkers();
    // Remote control: an introspection tool to step through a high level script via buttons & keyboard shortcuts in RViz
    visual_tools_->loadRemoteControl();
    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools_->trigger();
    // print the name of the reference frame and end-effector link for this robot.
    ROS_INFO_NAMED("wormRobot", "Reference frame: %s", move_group_->getPlanningFrame().c_str());
    ROS_INFO_NAMED("wormRobot", "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    // initialize other element
    // initPublisher();
    // initSubscriber();
    initServer();
  }

  /********************************************************************************
  ** Init Functions
  ********************************************************************************/
  void mpServer::initServer()
  {
    mpServer_traj_ = nh_.advertiseService("mpServer/trajectory", &mpServer::mp_processing, this);
  }

  bool mpServer::mp_processing(hybrid_robot::mp_service::Request &req,
                              hybrid_robot::mp_service::Response &res){
    if (req.cmd) {
      moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group_, joint_group_positions);

      move_group_->setPoseTarget(req.targetPose);

      // Now, we call the planner to compute the plan and visualize it.
      // Note that we are just planning, not asking move_group to actually move the robot.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ROS_INFO_NAMED("wormingRobot", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
      // std::cout << my_plan.trajectory_.joint_trajectory.joint_names[0] << std::endl;
      namespace rvt = rviz_visual_tools;
      // Visualizing plans as a line with markers in RViz.
      ROS_INFO_NAMED("wormingRobot", "Visualizing plan 1 as trajectory line");
      Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
      text_pose.translation().z() = 1.0;
      visual_tools_->publishAxisLabeled(req.targetPose, "pose1");
      visual_tools_->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools_->publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);
      visual_tools_->trigger();
      move_group_->move();
      // visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
      // joint_group_positions[0] = 2.0;  // radians
      // move_group_->setJointValueTarget(joint_group_positions);  
      
      res.result = true;
      return true;
    }
    else return false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motionPlanning/move_group_interface");
  ros::NodeHandle node_handle("~");
  ros::Rate loop_rate(100);
  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  
  std::string robot_name;
  std::string base_link;
  // double x, z, qua_x,qua_y,qua_z,qua_w;
  node_handle.param("robot_name", robot_name, std::string("wormRobot"));
  node_handle.param("base_link", base_link, std::string("base_link"));
  // node_handle.param("x", x, -0.6);
  // node_handle.param("z", z, 0.5);
  // node_handle.param("qua_x", qua_x, 0.0);
  // node_handle.param("qua_y", qua_y, -0.6);
  // node_handle.param("qua_z", qua_z, 0.0);
  // node_handle.param("qua_w", qua_w, 0.8);

  static const std::string PLANNING_GROUP = robot_name;
  // movGroInterface move_group(PLANNING_GROUP); // create a class: move_group
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  // const rs_JMG joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // visualizing objects, robots, & trajectories in RViz, debugging tools by step-by-step introspection of a script
  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  // movVisTools visual_tools(base_link);
  // declare a class for motion planning
  mp_server::mpServer motionPlanning(PLANNING_GROUP, base_link);
  // mp_server::mpServer motionPlanning(PLANNING_GROUP);

  ros::spin();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  return 0;
}
