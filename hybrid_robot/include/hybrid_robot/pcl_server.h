#ifndef PCL_SERVER_H_
#define PCL_SERVER_H_

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/don.h>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <Eigen/Dense>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include "geometry_msgs/Pose.h"

#include "hybrid_robot/pointcloud_cmd.h"

using pclXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using pclXYZ = pcl::PointCloud<pcl::PointXYZ>;
using pclNormal = pcl::PointCloud<pcl::PointNormal>;
using SvPclIndices = std::vector<pcl::PointIndices>;
using EvXf = Eigen::VectorXf;
using Ev4f = Eigen::Vector4f;
using Ev3f = Eigen::Vector3f;
using EmXf = Eigen::MatrixXf;
using Em4f = Eigen::Matrix4f;
using SvEv3f = std::vector<Eigen::Vector3f>;

namespace pcl_server
{
  class pclServer
  {
    public:     // public function
      // constructor with a point cloud topic
      pclServer(std::string t_Name, std::string f_Name, double s_Fact, double tol, bool s_Boun, bool s_Rect, bool s_Pose, double f_W, double f_L);
      // ~PahaP();
      
      // list of displaying functions
      void display_pcd(const pclXYZRGB::Ptr cloud, int clus_order, const std::string& titlename);
      void display_centroid_multCoor(pclXYZRGB::Ptr cloud, EmXf& centroid3D);

      void display_planeNormals(pclXYZRGB::Ptr cloud, Ev3f normals, Ev4f centroid);
      void display_clofarPair(pclXYZRGB::Ptr cloud, EvXf closfart, Ev4f centroid);
      void display_targetCoordinate(pclXYZRGB::Ptr cloud, Em4f RotMatrix);
      void display_pointSet_asPointCloud(SvEv3f setOfPoint);
      void draw_Rectangles(EmXf fiRect, pclXYZRGB::Ptr cloudp);
      

      // initialize function
      void initPublisher();
      void initSubscriber();
      void initServer();

      // list of propressing functions
      pclXYZRGB::Ptr passthrough_filter(pclXYZRGB::Ptr cloud); // passThrough filter
      pclXYZRGB::Ptr downsampling(pclXYZRGB::Ptr cloud);
      pclXYZRGB::Ptr est_plane(pclXYZRGB::Ptr cloud);
      
      Ev3f est_planeNormal(pclXYZRGB::Ptr cloud);  // estimate the normal vector of a plane
      EvXf det_MaxMinxyz(pclXYZRGB::Ptr cloudp);  // det max & min coordinate of point cloud according to xyz
      SvEv3f est_BounPoint(pclXYZRGB::Ptr cloudp, EvXf MaxMinxyz,float slicing_factor); // est the boundary point set
      // float cal_PlaneArea(const EvXf clos_fart);  // approximate the area by closest and farthest point pair
      Em4f convert_PosetoRotationMatrix(geometry_msgs::Pose robot_pose);       // convert Quaternion to heterousgeneous matrix
      EmXf est_5closPoints_toCentroid(SvEv3f B_Point, Ev4f centroid);
      // determine 5 rectangles with five closest point to the centroid
      EmXf det_5rectangle(EmXf fiPoint, Ev4f centroid, Ev3f normal, float l, float w);
      EmXf select_Rectangle(EmXf fiRect, Ev4f centroid, SvEv3f B_Point, double tolerance);
      geometry_msgs::Pose est_Pose2(EmXf rectangle, Ev3f ave_normal); // estimate the Quaternions by five closest points

      // list of Callbackfunction
      void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
      bool pclProcess(hybridRobot::pointcloud_cmd::Request &req, hybridRobot::pointcloud_cmd::Response &res);


    private:
      
      ros::NodeHandle nh_; // general ROS NodeHandle - used for pub, sub, advertise ...
      ros::NodeHandle nh_private_; // private ROS NodeHandle - used to get parameter from server
      // ROS Publisher
      ros::Publisher pclServer_pose_pub_;
      // ROS Subscribers
      ros::Subscriber pclServer_pclCam_sub_;
      // ROS Service Server
      ros::ServiceServer pclServer_getPcl_ser_;

      // Topic name
      std::string pclTopic_;
      // Create a container for the input point cloud data.
      pclXYZRGB::Ptr pclTopic_input_;
      pclXYZRGB::Ptr pclFile_input_;
      pclXYZRGB::Ptr pcl_input_;

      double slicingFactor_;
      double tolerance_;
      double rFWidth_;
      double rFLength_;
      bool showBound_;
      bool showSelRect_;
      bool showPose_;
  };
}

#endif /* PCL_SERVER_ */
