#include <sstream>
#include <hybrid_robot/pcl_server.h>

namespace pcl_server
{

// constructor
pclServer::pclServer(std::string t_Name, std::string f_Name, double s_Fact, double tol, bool s_Boun, bool s_Rect, bool s_Pose, double f_W, double f_L): 
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~")),
  pcl_input_(new pclXYZRGB),
  pclFile_input_(new pclXYZRGB),
  pclTopic_input_(new pclXYZRGB)
{
  pclTopic_ = t_Name;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (f_Name, *pclFile_input_)) {
      PCL_ERROR ("Couldn't read the point cloud data file \n");
  }
  slicingFactor_ = s_Fact;
  tolerance_ = tol;
  showBound_ = s_Boun;
  showSelRect_ = s_Rect;
  showPose_ = s_Pose;
  rFWidth_ = f_W;
  rFLength_ = f_L;  
  // initialize other element
  // initPublisher();
  initSubscriber();
  initServer();
}

/********************************************************************************
** Init Functions
********************************************************************************/
// void PahaP::initPublisher()
// {
  // ros message publisher
  // pahap_pose_pub_ = nh_.advertise<open_manipulator_msgs::OpenManipulatorState>("states", 10);
// }
void pclServer::initSubscriber()
{
  // ros message subscriber
  pclServer_pclCam_sub_ = nh_.subscribe (pclTopic_, 1, &pclServer::pointcloudCallback, this);
}

void pclServer::initServer()
{
  pclServer_getPcl_ser_ = nh_.advertiseService("pointcloud_cmd", &pclServer::pclProcess, this);
}

//callback to get camera data through "image_pub" topic
void pclServer::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){   
  // Convert to PCL data type
  pcl::fromROSMsg(*msg, *pclTopic_input_);
}

bool pclServer::pclProcess(hybridRobot::pointcloud_cmd::Request &req,
                            hybridRobot::pointcloud_cmd::Response &res){
  if (req.cmd) {
    if (req.topic) copyPointCloud(*pclTopic_input_, *pcl_input_);
    else copyPointCloud(*pclFile_input_, *pcl_input_); 
    // display_pcd(pcl_input_, 0, "Original Plane");
    pclXYZRGB::Ptr pcl_pass (new pclXYZRGB);
    pcl_pass = passthrough_filter(pcl_input_);
    // pcl_pass = passthrough_filter(pclTopic_input_);
    pcl_pass = downsampling(pcl_pass);
    pcl_pass = est_plane(pcl_pass);
    // display_pcd(pcl_pass, 0, "Plane estimation");
    Ev3f normal;
    normal = est_planeNormal(pcl_pass);
    Ev4f cen;
    // calculate the centroid of the plane
    pcl::compute3DCentroid(*pcl_pass, cen);
    // display_planeNormals(pcl_pass, normal, cen);

    EvXf maxmin_xyz;
    maxmin_xyz = det_MaxMinxyz(pcl_pass);
    SvEv3f bound;
    bound = est_BounPoint(pcl_pass, maxmin_xyz, slicingFactor_);
    if (showBound_){
      display_pointSet_asPointCloud(bound);
    }

    EmXf fiPoints;
    fiPoints = est_5closPoints_toCentroid(bound, cen);
    // std::cout << fiPoints << std::endl;
    EmXf fiveRect, selRect;
    fiveRect = det_5rectangle(fiPoints, cen, normal, rFLength_, rFWidth_);
    // draw_Rectangles(fiveRect, pcl_pass);
    selRect = select_Rectangle(fiveRect, cen, bound, tolerance_);
    if (showSelRect_) draw_Rectangles(selRect, pcl_pass);

    geometry_msgs::Pose pose2;
    pose2 = est_Pose2(selRect, normal);

    
    Em4f  RotMatrix;
    RotMatrix = convert_PosetoRotationMatrix(pose2);
    if (showPose_) display_targetCoordinate(pcl_pass, RotMatrix);
    // res.planePose = pose2;
    res.planePose.position.x = pose2.position.x;
    res.planePose.position.y = pose2.position.y;
    res.planePose.position.z = pose2.position.z;
    res.planePose.orientation.w = pose2.orientation.w;
    res.planePose.orientation.x = pose2.orientation.x;
    res.planePose.orientation.y = pose2.orientation.y;
    res.planePose.orientation.z = pose2.orientation.z;
    return true;
  }
  else return false;
}


//passThrough filter
pclXYZRGB::Ptr pclServer::passthrough_filter(pclXYZRGB::Ptr cloud) {
  pclXYZRGB::Ptr cloud_filtered (new pclXYZRGB);
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> distance_pass;
  distance_pass.setInputCloud (cloud);
  distance_pass.setFilterFieldName ("z");
  distance_pass.setFilterLimits (0.5, 0.72);
  //pass.setFilterLimitsNegative (true);
  distance_pass.filter (*cloud_filtered);
  // std::cerr << "Cloud after passthrough filtering: " << cloud_filtered->size() << std::endl;
  return cloud_filtered;
}

pclXYZRGB::Ptr pclServer::downsampling(pclXYZRGB::Ptr cloud) {
    pclXYZRGB::Ptr cloud_filtered (new pclXYZRGB);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    // define the size of the the leaf (in meter) = voxel size
    sor.setLeafSize (0.005f, 0.005f, 0.005f);
    // perform the downsampling filter then save the new data to cloud_filtered
    sor.filter (*cloud_filtered);
    return cloud_filtered;
}

pclXYZRGB::Ptr pclServer::est_plane(pclXYZRGB::Ptr cloud) {
    pclXYZRGB::Ptr cloud_p (new pclXYZRGB);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());  // variable to store the point cloud belong to the desired object
    // Create the segmentation object -> filter the desired point belong to the object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true); // Optional> set this function using refine optimize Coefficient
    // Mandatory - set parameter for the planar filter
    seg.setModelType (pcl::SACMODEL_PLANE); // -> segment the plane in the point cloud
    // -> RANSAC = Random Sample Consensus = estimator method
    seg.setMethodType (pcl::SAC_RANSAC);    //-> use 2 step to determine the outliers or inliers
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);    // -> tolerate range to define the inliers & outlier
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    int nr_points = (int) cloud->points.size ();
    int cloudsize = 0;
    // While 30% of the original cloud is still there
    while (cloud->points.size() > 0.3*nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud);
      // seg.segment() result in the inliers point and coefficients
      seg.segment (*inliers, *coefficients); //save all points belong to the plane to inliners
      // outlier -> mau ngoai lai, out of observation range -> skew the estimation result
      // inlier -> sample lie inside the range -> fit with the model 
      if (inliers->indices.size () == 0) // -> if there is no inliers -> no point fit with the object
      {
          std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
      }
      // input the pointcloud into Extract() function
      extract.setInputCloud (cloud);
      // input a pointer to vector of indices (inliers)->represent the input data
      extract.setIndices(inliers);
      // set whether the indices should be returned, or all points_except_the indices
      extract.setNegative (false); //false -> return the indices
      // save the result to the cloud_p
      extract.filter (*cloud_p);
      // if no change in the indices -> stop
      if (cloudsize == cloud_p->width*cloud_p->height)
          break;
      // std::cerr <<"PointCloud representing the planar component: "<< cloud_p->width*cloud_p->height <<" data points."<< std::endl;
      cloud.swap (cloud_p);
      cloudsize = cloud_p->width*cloud_p->height;
    }
    return cloud;
}

// estimate the normal vector of a plane
Ev3f pclServer::est_planeNormal(pclXYZRGB::Ptr cloud) {
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
  ne.setInputCloud (cloud);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  tree->setInputCloud(cloud);
  ne.setSearchMethod (tree);
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
  // Output datasets
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.02);
  // Compute the features - all normal vector
  ne.compute (*cloud_normals);
  float sum_x=0.0, sum_y=0.0, sum_z=0.0;
  for (int i=0; i<cloud_normals->size(); i++) {
    sum_x += cloud_normals->points[i].normal_x;
    sum_y += cloud_normals->points[i].normal_y;
    sum_z += cloud_normals->points[i].normal_z;
  }
  int number = cloud_normals->size();
  //get average normal vectors
  Eigen::Vector3f aver_nor;
  aver_nor[0] = sum_x/number;
  aver_nor[1] = sum_y/number;
  aver_nor[2] = sum_z/number; 
  return aver_nor;
}

// get the maximum and minimum coordinate of point cloud according to xyz
EvXf pclServer::det_MaxMinxyz(pclXYZRGB::Ptr cloudp) {
    Eigen::VectorXf max_min(6);
    float max_x, min_x, max_y, min_y, max_z, min_z;
    for (int i = 0; i < cloudp->size(); i++) {    
        if (i ==0) {
            max_x = cloudp->points[i].x;
            min_x = cloudp->points[i].x;
            max_y = cloudp->points[i].y;
            min_y = cloudp->points[i].y;
            max_z = cloudp->points[i].z;
            min_z = cloudp->points[i].z;
        }
        // For x
        if (cloudp->points[i].x > max_x)
            max_x = cloudp->points[i].x;
        if (cloudp->points[i].x < min_x)
            min_x = cloudp->points[i].x;
        // For y
        if (cloudp->points[i].y > max_y)
            max_y = cloudp->points[i].y;
        if (cloudp->points[i].y < min_y)
            min_y = cloudp->points[i].y;
        // For z
        if (cloudp->points[i].z > max_z)
            max_z = cloudp->points[i].z;
        if (cloudp->points[i].z < min_z)
            min_z = cloudp->points[i].z;
    }
    max_min[0] = min_x;
    max_min[1] = max_x;
    max_min[2] = min_y;
    max_min[3] = max_y;
    max_min[4] = min_z;
    max_min[5] = max_z;
    return max_min;
}
// get the set of boundary points
SvEv3f pclServer::est_BounPoint(pclXYZRGB::Ptr cloudp, EvXf MaxMinxyz, float slicing_factor)  // slicing_factor =0.02
{
    
  std::vector<Eigen::Vector3f> boundaryPoint;
  Eigen::Vector3f point1(0,0,0), point2(0,0,0);
  float start, distance1, distance2, max_distance;
  Eigen::VectorXf maxmin(6);
  for (int i =0; i < 6; i++)
      maxmin[i] = MaxMinxyz[i];
  int count_check = 0;

  // slicing for each axis direction find the farthest point couples in each slicing axis
  for (int axis = 0; axis < 3; axis++)
  {
    start = maxmin[2*axis];
    // find the point cloud belong to one slicing
    while (start < maxmin[2*axis+1])
    {   
      for (int i = 0; i< cloudp->size(); i++)
      {
        // select the right distance
        float dis_xyz=0;
        if (axis == 0) dis_xyz = cloudp->points[i].x - start;
        if (axis == 1) dis_xyz = cloudp->points[i].y - start;
        if (axis == 2) dis_xyz = cloudp->points[i].z - start;

        // if the point belong to the slicing, check whether it set a furthest distance pair
        if ((dis_xyz < slicing_factor/2) and (dis_xyz > -slicing_factor/2))
        {
          if (count_check ==0)  // this is the first point
          {
            point1[0] = cloudp->points[i].x;
            point1[1] = cloudp->points[i].y;
            point1[2] = cloudp->points[i].z;
          }
          else if (count_check ==1) // this is the second point
          {
            point2[0] = cloudp->points[i].x;
            point2[1] = cloudp->points[i].y;
            point2[2] = cloudp->points[i].z;
            max_distance = pow((point2[0]-point1[0]),2)+pow((point2[1]-point1[1]),2)+pow((point2[2]-point1[2]),2);
          }
          else {
            distance1 = pow((point1[0]-cloudp->points[i].x),2)+pow((point1[1]-cloudp->points[i].y),2)+pow((point1[2]-cloudp->points[i].z),2);
            distance2 = pow((point2[0]-cloudp->points[i].x),2)+pow((point2[1]-cloudp->points[i].y),2)+pow((point2[2]-cloudp->points[i].z),2);
            if (distance2 < distance1)
              if (distance1 > max_distance) {
                max_distance = distance1;
                point2[0] = cloudp->points[i].x;
                point2[1] = cloudp->points[i].y;
                point2[2] = cloudp->points[i].z;
              }    
            if (distance2 > distance1)
              if (distance2 > max_distance) {
                max_distance = distance2;
                point1[0] = cloudp->points[i].x;
                point1[1] = cloudp->points[i].y;
                point1[2] = cloudp->points[i].z;
              }    
          }
          count_check += 1;
        }
      }
      count_check =0;  
      // check if there is points with common coordinate
      bool commonCoor1 = false, commonCoor2 = false;
      for (int j = 0; j < boundaryPoint.size(); j++) {
        if ((point1[0] == boundaryPoint[j][0]) and (point1[1] == boundaryPoint[j][1]) and (point1[2] == boundaryPoint[j][2]))
        {  
          commonCoor1 = true;
          // std::cout << "Point 1 has the same coordinate with a point in boundary set" << std::endl;
        }
        if ((point2[0] == boundaryPoint[j][0]) and (point2[1] == boundaryPoint[j][1]) and (point2[2] == boundaryPoint[j][2]))
        {
          commonCoor2 = true;
          // std::cout << "Point 2 has the same coordinate with a point in boundary set" << std::endl;
        }
      }
      if (not commonCoor1) boundaryPoint.push_back(point1);
      if (not commonCoor2) boundaryPoint.push_back(point2);
      start += slicing_factor/2;
      point1[0] = 0.0;
      point1[1] = 0.0;
      point1[2] = 0.0;
      point2[0] = 0.0;
      point2[1] = 0.0;
      point2[2] = 0.0;
    }
  }
  return boundaryPoint;
}

// convert Quaternion to heterousgeneous matrix
Em4f pclServer::convert_PosetoRotationMatrix(geometry_msgs::Pose robot_pose)
{
    // geometry_msgs::Pose robot_pose;
    Eigen::Matrix4f hete_matrix;
    
    // for translation
    hete_matrix(0,3) = robot_pose.position.x;
    hete_matrix(1,3) = robot_pose.position.y;
    hete_matrix(2,3) = robot_pose.position.z;
    hete_matrix(3,3) = 1;
    // the last row
    hete_matrix(3,0) = 0;
    hete_matrix(3,1) = 0;
    hete_matrix(3,2) = 0;
    
    // for rotation part
    hete_matrix(0,0) = 1 - 2*pow(robot_pose.orientation.y,2) - 2*pow(robot_pose.orientation.z,2);
    hete_matrix(1,0) = 2*(robot_pose.orientation.x*robot_pose.orientation.y + robot_pose.orientation.z*robot_pose.orientation.w);
    hete_matrix(2,0) = 2*(robot_pose.orientation.x*robot_pose.orientation.z - robot_pose.orientation.y*robot_pose.orientation.w);

    hete_matrix(0,1) = 2*(robot_pose.orientation.x*robot_pose.orientation.y - robot_pose.orientation.z*robot_pose.orientation.w);
    hete_matrix(1,1) = 1 - 2*pow(robot_pose.orientation.x,2) - 2*pow(robot_pose.orientation.z,2);
    hete_matrix(2,1) = 2*(robot_pose.orientation.x*robot_pose.orientation.w + robot_pose.orientation.y*robot_pose.orientation.z);

    hete_matrix(0,2) = 2*(robot_pose.orientation.x*robot_pose.orientation.z + robot_pose.orientation.y*robot_pose.orientation.w);
    hete_matrix(1,2) = 2*(robot_pose.orientation.y*robot_pose.orientation.z - robot_pose.orientation.x*robot_pose.orientation.w);
    hete_matrix(2,2) = 1 - 2*pow(robot_pose.orientation.x,2) - 2*pow(robot_pose.orientation.y,2);

    return hete_matrix;
}

// est 5 closest points to the centroid
EmXf pclServer::est_5closPoints_toCentroid(SvEv3f B_Point, Ev4f centroid)
{
  Eigen::MatrixXf fiveClosest(5,3);
  Eigen::VectorXf fiveDistance(5);
  float distance = 0.0, d_max=0;
  int max_index = 0;
  for (int i = 0; i < B_Point.size(); i++) {
    distance = pow((centroid[0]-B_Point[i][0]),2)+pow((centroid[1]-B_Point[i][1]),2)+pow((centroid[2]-B_Point[i][2]),2);
    if (i < 5) {
        if (i==0) {
          // d_max = distance;
          max_index = 0;
        }
        fiveClosest(i,0) = B_Point[i][0];
        fiveClosest(i,1) = B_Point[i][1];
        fiveClosest(i,2) = B_Point[i][2];
        fiveDistance(i) = distance;
        if (fiveDistance(i) > fiveDistance(max_index))
          max_index = i;
        // if (i == 4){
        //   std::cout << fiveClosest << std::endl;
        //   std::cout << fiveDistance << std::endl;
        //   std::cout << max_index << std::endl;
        // }
    }
    else {
      // using the max_index to manage farthest point in the array
      if (distance < fiveDistance(max_index)) {
        fiveClosest(max_index,0) = B_Point[i][0];
        fiveClosest(max_index,1) = B_Point[i][1];
        fiveClosest(max_index,2) = B_Point[i][2];
        fiveDistance(max_index) = distance;
        for (int j =0; j<5; j++) {
          if (fiveDistance(j) > fiveDistance(max_index))
            max_index = j;
        }
      }
    }
  }
  return fiveClosest;
}

// determine 5 rectangles with five closest point to the centroid
EmXf pclServer::det_5rectangle(EmXf fiPoint, Ev4f centroid, Ev3f normal, float l, float w)
{
    Eigen::MatrixXf fiveRectangle(5,24);
    Eigen::Vector3f e_x(0,0,0), e_y(0,0,0);
    // create rectangle based on the closest points and its centroid
    for (int i = 0; i < 5; i++)
    { 
      e_x(0) = fiPoint(i,0) - centroid(0);
      e_x(1) = fiPoint(i,1) - centroid(1);
      e_x(2) = fiPoint(i,2) - centroid(2);
      e_y(0) = normal(1)*e_x(2) - normal(2)*e_x(1);
      e_y(1) = normal(2)*e_x(0) - normal(0)*e_x(2);
      e_y(2) = normal(0)*e_x(1) - normal(1)*e_x(0);
      // w is the width of the robot foot (w=0.28)
      float factor_x = w/sqrt(pow(e_x(0),2)+pow(e_x(1),2)+pow(e_x(2),2));
      // point 1
      fiveRectangle(i,0) = fiPoint(i,0);
      fiveRectangle(i,1) = fiPoint(i,1);
      fiveRectangle(i,2) = fiPoint(i,2);
      // point 2
      fiveRectangle(i,3) = fiveRectangle(i,0) - factor_x*e_x(0);
      fiveRectangle(i,4) = fiveRectangle(i,1) - factor_x*e_x(1);
      fiveRectangle(i,5) = fiveRectangle(i,2) - factor_x*e_x(2);
      // other points on y-axis 
      // l is the length of robot feet (l=0.24)
      float factor_y = l/sqrt(pow(e_y(0),2)+pow(e_y(1),2)+pow(e_y(2),2));
      // point 3
      fiveRectangle(i,6) = fiveRectangle(i,0) + factor_y*e_y(0);
      fiveRectangle(i,7) = fiveRectangle(i,1) + factor_y*e_y(1);
      fiveRectangle(i,8) = fiveRectangle(i,2) + factor_y*e_y(2);
      // point 4
      fiveRectangle(i,9) = fiveRectangle(i,0) - factor_y*e_y(0);
      fiveRectangle(i,10) = fiveRectangle(i,1) - factor_y*e_y(1);
      fiveRectangle(i,11) = fiveRectangle(i,2) - factor_y*e_y(2);

      // point 5
      fiveRectangle(i,12) = (fiveRectangle(i,0) + fiveRectangle(i,3))/2 + factor_y*e_y(0);
      fiveRectangle(i,13) = (fiveRectangle(i,1) + fiveRectangle(i,4))/2 + factor_y*e_y(1);
      fiveRectangle(i,14) = (fiveRectangle(i,2) + fiveRectangle(i,5))/2 + factor_y*e_y(2);

      // point 6
      fiveRectangle(i,15) = (fiveRectangle(i,0) + fiveRectangle(i,3))/2 - factor_y*e_y(0);
      fiveRectangle(i,16) = (fiveRectangle(i,1) + fiveRectangle(i,4))/2 - factor_y*e_y(1);
      fiveRectangle(i,17) = (fiveRectangle(i,2) + fiveRectangle(i,5))/2 - factor_y*e_y(2);

      // point 7
      fiveRectangle(i,18) = fiveRectangle(i,3) + factor_y*e_y(0);
      fiveRectangle(i,19) = fiveRectangle(i,4) + factor_y*e_y(1);
      fiveRectangle(i,20) = fiveRectangle(i,5) + factor_y*e_y(2);
      // point 8
      fiveRectangle(i,21) = fiveRectangle(i,3) - factor_y*e_y(0);
      fiveRectangle(i,22) = fiveRectangle(i,4) - factor_y*e_y(1);
      fiveRectangle(i,23) = fiveRectangle(i,5) - factor_y*e_y(2);
    }

    return fiveRectangle;
}

EmXf pclServer::select_Rectangle(EmXf fiRect, Ev4f centroid, SvEv3f B_Point, double tolerance)
{
  Eigen::MatrixXf threeClosest = Eigen::MatrixXf::Zero(3,3);
  Eigen::Vector3f threeDistance(0,0,0);
  Eigen::MatrixXf EightPoints = Eigen::MatrixXf::Zero(8,3);
  float distance =0.0, d_1 = 0.0, d_2 = 0.0, d_3 = 0.0, d_c = 0.0, error_sum =0, min_error;
  int Inside = 0, selRect =0;
  bool firstTime = true, cond1=false, cond2=false, cond3=false;
  int max_index = 0;
  for (int i =0; i<5; i++)
  {
    // std::cout << "print out eight point on the rectangle" << std::endl;
    for (int k =0; k<8; k++)
    {
      EightPoints(k,0) = fiRect(i,3*k);
      EightPoints(k,1) = fiRect(i,3*k+1);
      EightPoints(k,2) = fiRect(i,3*k+2);

      // std::cout << EightPoints.row(k) << std::endl;
    }
    for (int m =0; m<8; m++)
    {
      for (int j =0; j <B_Point.size(); j++)
      {
        distance = pow((EightPoints(m,0)-B_Point[j][0]),2)+pow((EightPoints(m,1)-B_Point[j][1]),2)+pow((EightPoints(m,2)-B_Point[j][2]),2);
        // std::cout << distance << std::endl;
        if (j<3)
        {
          if (j == 0)
          {
            max_index =0;
            // d_max = distance;
          }
          threeClosest(j,0) = B_Point[j][0];
          threeClosest(j,1) = B_Point[j][1];
          threeClosest(j,2) = B_Point[j][2];
          threeDistance(j) = distance;
          if (not (max_index == j))  
            if (threeDistance(j) > threeDistance(max_index))
              max_index = j;
        }
        else
        {
          if (distance < threeDistance(max_index))
          {
            threeClosest(max_index,0) = B_Point[j][0];
            threeClosest(max_index,1) = B_Point[j][1];
            threeClosest(max_index,2) = B_Point[j][2];
            threeDistance(max_index) = distance;
            // check the value for max_index
            for (int n =0; n<3; n++)
              if (not (max_index == n)) 
                if (threeDistance(max_index) < threeDistance(n))
                  max_index = n;      
          }
        }
      }
      // std::cout << "Three distance to point %d" << m  << std::endl;
      // std::cout << threeDistance << std::endl;
      d_1 = sqrt(pow((threeClosest(0,0)-centroid(0)),2)+pow((threeClosest(0,1)-centroid(1)),2)+pow((threeClosest(0,2)-centroid(2)),2));
      d_2 = sqrt(pow((threeClosest(1,0)-centroid(0)),2)+pow((threeClosest(1,1)-centroid(1)),2)+pow((threeClosest(1,2)-centroid(2)),2));
      d_3 = sqrt(pow((threeClosest(2,0)-centroid(0)),2)+pow((threeClosest(2,1)-centroid(1)),2)+pow((threeClosest(2,2)-centroid(2)),2));
      d_c = sqrt(pow((EightPoints(m,0)-centroid(0)),2)+pow((EightPoints(m,1)-centroid(1)),2)+pow((EightPoints(m,2)-centroid(2)),2));

      float check1=0.0, check2=0.0, check3=0.0;
      check1 = (d_c-d_1)/d_c;
      check2 = (d_c-d_2)/d_c;
      check3 = (d_c-d_3)/d_c;

      if (check1 < 0) check1 = -check1;
      if (check2 < 0) check2 = -check2;
      if (check3 < 0) check3 = -check3;
      error_sum = error_sum + check1 + check2 + check3;
      if ((d_c < d_1) or (check1 < tolerance))
        cond1 = true;
      if ((d_c < d_2) or (check2 < tolerance))
        cond2 = true;
      if ((d_c < d_3) or (check3 < tolerance))
        cond3 = true;
      // check condition the point lies inside or on the boundary of pointcloud
      if (cond1 and cond2 and cond3)   // error of 5% = 0.05
        Inside += 1;
      cond1 = false;
      cond2 = false;
      cond3 = false;
    }
    if (Inside == 8) {
      if (firstTime){
        min_error = error_sum;
        selRect = i;
        firstTime = false;
      }
      else if (error_sum < min_error){
        min_error = error_sum;
        selRect = i;
      }
    }
    error_sum =0;
    Inside = 0;
  }
  if (not firstTime)
    return fiRect.row(selRect);
  else {
    std::cout << "Cannot find any good rectangle" << std::endl;
    return fiRect.row(0).setZero();
  }
}

// estimate the Quaternions by five closest points
geometry_msgs::Pose pclServer::est_Pose2(EmXf rectangle, Ev3f ave_normal) {
    geometry_msgs::Pose robot_pose;
    Eigen::Matrix3f orie_matrix;
    Eigen::Vector3f origin(0,0,0), pose_posistion1, pose_posistion2;
    float norm=0.0;
    // calculate the origin of coordinate frame
    origin(0) = (rectangle(0,0)+rectangle(0,3))/2;
    origin(1) = (rectangle(0,1)+rectangle(0,4))/2;
    origin(2) = (rectangle(0,2)+rectangle(0,5))/2;
    
    // z- axis
    orie_matrix(0,2) = ave_normal(0);
    orie_matrix(1,2) = ave_normal(1);
    orie_matrix(2,2) = ave_normal(2);
    // normalize
    norm = sqrt(pow(orie_matrix(0,2), 2) + pow(orie_matrix(1,2), 2) + pow(orie_matrix(2,2), 2));
    orie_matrix(0,2) = orie_matrix(0,2)/norm;
    orie_matrix(1,2) = orie_matrix(1,2)/norm;
    orie_matrix(2,2) = orie_matrix(2,2)/norm;
    
    // x axis
    orie_matrix(0,0) = rectangle(0,0) - origin(0);
    orie_matrix(1,0) = rectangle(0,1) - origin(1);
    orie_matrix(2,0) = rectangle(0,2) - origin(2);
    // normalize
    norm = sqrt(pow(orie_matrix(0,0), 2) + pow(orie_matrix(1,0), 2) + pow(orie_matrix(2,0), 2));
    orie_matrix(0,0) = orie_matrix(0,0)/norm;
    orie_matrix(1,0) = orie_matrix(1,0)/norm;
    orie_matrix(2,0) = orie_matrix(2,0)/norm;
    
    // y azis - using cross product
    orie_matrix(0,1) = orie_matrix(1,2)*orie_matrix(2,0) - orie_matrix(2,2)*orie_matrix(1,0);
    orie_matrix(1,1) = orie_matrix(2,2)*orie_matrix(0,0) - orie_matrix(0,2)*orie_matrix(2,0);
    orie_matrix(2,1) = orie_matrix(0,2)*orie_matrix(1,0) - orie_matrix(1,2)*orie_matrix(0,0);
    // normalize
    norm = sqrt(pow(orie_matrix(0,1), 2) + pow(orie_matrix(1,1), 2) + pow(orie_matrix(2,1), 2));
    orie_matrix(0,1) = orie_matrix(0,1)/norm;
    orie_matrix(1,1) = orie_matrix(1,1)/norm;
    orie_matrix(2,1) = orie_matrix(2,1)/norm;
    // std::cout << orie_matrix(0,0) << " " << orie_matrix(1,0) << " " << orie_matrix(2,0) << std::endl;
    // std::cout << orie_matrix(0,1) << " " << orie_matrix(1,1) << " " << orie_matrix(2,1) << std::endl;
    // std::cout << orie_matrix(0,2) << " " << orie_matrix(1,2) << " " << orie_matrix(2,2) << std::endl;
    // calculate the quaternion
    robot_pose.orientation.w = 0.5*sqrt(1+orie_matrix(0,0)+orie_matrix(1,1)+orie_matrix(2,2));
    robot_pose.orientation.x = 0.25*(orie_matrix(2,1)-orie_matrix(1,2))/robot_pose.orientation.w;
    robot_pose.orientation.y = 0.25*(orie_matrix(0,2)-orie_matrix(2,0))/robot_pose.orientation.w;
    robot_pose.orientation.z = 0.25*(orie_matrix(1,0)-orie_matrix(0,1))/robot_pose.orientation.w;
    // pose position
    pose_posistion1[0] = (origin[0] + rectangle(0,12))/2;
    pose_posistion1[1] = (origin[1] + rectangle(0,13))/2;
    pose_posistion1[2] = (origin[2] + rectangle(0,14))/2;
    pose_posistion2[0] = (origin[0] + rectangle(0,15))/2;
    pose_posistion2[1] = (origin[1] + rectangle(0,16))/2;
    pose_posistion2[2] = (origin[2] + rectangle(0,17))/2;
    
    // be sure that the first pose is always higher than the second one
    if (pose_posistion1(1) < pose_posistion2(1))
    {  
      robot_pose.position.x = pose_posistion1(0);
      robot_pose.position.y = pose_posistion1(1);
      robot_pose.position.z = pose_posistion1(2);
    }
    else
    { 
      robot_pose.position.x = pose_posistion2(0);
      robot_pose.position.y = pose_posistion2(1);
      robot_pose.position.z = pose_posistion2(2);

    }
    return robot_pose;
}

// possible to display a set of point cloud
void pclServer::display_pcd(const pclXYZRGB::Ptr cloud, int clus_order, const std::string& titlename) {
    int v1(0);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (titlename));
    viewer->setBackgroundColor (0, 0, 0);
    if (clus_order != 0) {   
      std::stringstream ss1;
      // convert clus_order to string
      ss1 << clus_order;       
      std::string ss = "Cluster " + ss1.str();
      viewer->addText(ss, 80, 50, 20, 120, 120, 200, "v1 text", v1);
    }   
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (0.25);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped()) {
      viewer->spinOnce(10);
    }
}

void pclServer::display_centroid_multCoor(pclXYZRGB::Ptr cloud, EmXf& centroid3D)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Showing Centroid"));
  viewer->setBackgroundColor (0, 0, 0);
  // Add text to the screen
  //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.25);
  
  Eigen::Affine3f trans_matrix;
  Eigen::Quaternion<float> q;
  q.x()= 0.683013;
  q.y()= 0.683013;
  q.z()= -0.183013;
  q.w()= 0.183013;
  
  trans_matrix = Eigen::Translation3f(0.0, 0.551812, 0.161767) * Eigen::AngleAxis<float>(q);


  // viewer->addCoordinateSystem (0.25, 0.3,0.3,0.3, "base_frame");
  viewer->addCoordinateSystem (0.25, trans_matrix, "base_frame");
  viewer->initCameraParameters ();

  int clusterNumber1 = centroid3D.rows();
  std::string id[clusterNumber1];
  pcl::ModelCoefficients line_coeff[clusterNumber1]; //, line_coeff2;

  for (int i=0; i < clusterNumber1; ++i)
  {
      line_coeff[i].values.resize(6);    // We need 6 values
      line_coeff[i].values[0] = 0;
      line_coeff[i].values[1] = 0;
      line_coeff[i].values[2] = 0; 
      line_coeff[i].values[3] = centroid3D(i,0);
      line_coeff[i].values[4] = centroid3D(i,1);
      line_coeff[i].values[5] = centroid3D(i,2);
  
      id[i] = "line"+i;
      viewer->addLine(line_coeff[i], id[i], 0);

      line_coeff[i].values[0] = 0;
      line_coeff[i].values[1] = 0.551812;
      line_coeff[i].values[2] = 0.161767; 
      line_coeff[i].values[3] = centroid3D(i,0);
      line_coeff[i].values[4] = centroid3D(i,1)-0.551812;
      line_coeff[i].values[5] = centroid3D(i,2)-0.161767;
      id[i] = "line"+i+1;
      viewer->addLine(line_coeff[i], id[i], 0);

  }
  while (!viewer->wasStopped()) {
    viewer->spinOnce (100);
  }
}

void pclServer::display_targetCoordinate(pclXYZRGB::Ptr cloud, Em4f RotMatrix) {
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Target Coordinate Frame"));
  viewer->setBackgroundColor (0, 0, 0);
  // Add text to the screen
  //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.3);

  std::vector<pcl::PointXYZ> pointSet;
  pcl::PointXYZ point; 
  std::string id[3];
  for (int i=0; i < 4; ++i)
  {
      if (i==0) {
        point.x = RotMatrix(0,3);
        point.y = RotMatrix(1,3);
        point.z = RotMatrix(2,3);
        pointSet.push_back(point);
      }
      if (i==1) {
        point.x = RotMatrix(0,3)+RotMatrix(0,0)/2;
        point.y = RotMatrix(1,3)+RotMatrix(1,0)/2;
        point.z = RotMatrix(2,3)+RotMatrix(2,0)/2;
        pointSet.push_back(point);
        id[i-1] = "line"+i;
        viewer->addLine(pointSet[0], pointSet[i], 1, 0, 0, id[i-1], 0);     // select red color
        viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id[i-1]);
      }
      if (i==2) {
        point.x = RotMatrix(0,3)+RotMatrix(0,1)/2;
        point.y = RotMatrix(1,3)+RotMatrix(1,1)/2;
        point.z = RotMatrix(2,3)+RotMatrix(2,1)/2;
        pointSet.push_back(point);
        id[i-1] = "line"+i;
        viewer->addLine(pointSet[0], pointSet[i], 0, 1, 0, id[i-1], 0);  // select green color
        viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id[i-1]);
      }
      if (i==3) {
        point.x = RotMatrix(0,3)+RotMatrix(0,2)/2;
        point.y = RotMatrix(1,3)+RotMatrix(1,2)/2;
        point.z = RotMatrix(2,3)+RotMatrix(2,2)/2;
        pointSet.push_back(point);
        id[i-1] = "line"+i;
        viewer->addLine(pointSet[0], pointSet[i], 0, 0, 1, id[i-1], 0);  // select blue color
        viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id[i-1]);
      }
  }
  while (!viewer->wasStopped()) {
    viewer->spinOnce (100);
  }
}

void pclServer::display_pointSet_asPointCloud(SvEv3f setOfPoint) {
  pclXYZ cloud;

  // Fill in the cloud data
  cloud.width = setOfPoint.size();
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);
  // Eigen::Vector3f getData;
  // getData = setOfPoint[0];

  for (std::size_t i = 0; i < cloud.points.size (); ++i)
  {
  //   getData = setOfPoint[i];
    cloud.points[i].x = setOfPoint[i][0];
    cloud.points[i].y = setOfPoint[i][1];
    cloud.points[i].z = setOfPoint[i][2];
  }
  pcl::visualization::CloudViewer viewer ("BoundaryPoint");
  // viewer.addCoordinateSystem (0.25);
  viewer.showCloud(cloud.makeShared());
  while (!viewer.wasStopped())
  {
  }
}


void pclServer::draw_Rectangles(EmXf fiRect, pclXYZRGB::Ptr cloudp) {
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Rectangles"));
  viewer->setBackgroundColor (0, 0, 0);
  // Add text to the screen
  //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloudp, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloudp, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.25);

  std::vector<pcl::PointXYZ> pointSet;
  pcl::PointXYZ point; 
  std::string id[20];
  EmXf EightPoints = Eigen::MatrixXf::Zero(8,3);
  for (int r =0; r < fiRect.rows(); r++)
  {
    if ((fiRect.rows()==1) and (fiRect(r,0)+fiRect(r,1)+fiRect(r,21)+fiRect(r,22) == 0))
    {
      std::cout << "No rectangle meet the requirement" << std::endl;
      break;
    }
    // get the points of the first rectangle
    for (int k =0; k<8; k++)
    {
      EightPoints(k,0) = fiRect(r,3*k);
      EightPoints(k,1) = fiRect(r,3*k+1);
      EightPoints(k,2) = fiRect(r,3*k+2);
    }
    for (int i=0; i < 8; i++)
    {
        if (i==2)
        {
          point.x = EightPoints(i,0);
          point.y = EightPoints(i,1);
          point.z = EightPoints(i,2);
          pointSet.push_back(point);
        }
        if (i==3)
        {
          point.x = EightPoints(i,0);
          point.y = EightPoints(i,1);
          point.z = EightPoints(i,2);
          pointSet.push_back(point);
          // draw the first edge
          id[4*r] = "line"+4*r;
          if (r==0) viewer->addLine(pointSet[4*r], pointSet[4*r+1], 1, 0, 0, id[4*r], 0);     // select red color
          if (r==1) viewer->addLine(pointSet[4*r], pointSet[4*r+1], 0, 1, 0, id[4*r], 0);     // select green color
          if (r==2) viewer->addLine(pointSet[4*r], pointSet[4*r+1], 0, 0, 1, id[4*r], 0);     // select blue color
          if (r==3) viewer->addLine(pointSet[4*r], pointSet[4*r+1], 1, 1, 0, id[4*r], 0);     // select yellow color
          if (r==4) viewer->addLine(pointSet[4*r], pointSet[4*r+1], 1, 0, 1, id[4*r], 0);     // select purple color
          
          viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, id[4*r]);
        }
        if (i==6)
        {
          point.x = EightPoints(i,0);
          point.y = EightPoints(i,1);
          point.z = EightPoints(i,2);
          pointSet.push_back(point);
          id[4*r+1] = "line"+4*r+1;
          // viewer->addLine(pointSet[4*r], pointSet[4*r+2], 0, 1, 0, id[4*r+1], 0);  // select green color
          if (r==0) viewer->addLine(pointSet[4*r], pointSet[4*r+2], 1, 0, 0, id[4*r+1], 0);     // select red color
          if (r==1) viewer->addLine(pointSet[4*r], pointSet[4*r+2], 0, 1, 0, id[4*r+1], 0);     // select green color
          if (r==2) viewer->addLine(pointSet[4*r], pointSet[4*r+2], 0, 0, 1, id[4*r+1], 0);     // select blue color
          if (r==3) viewer->addLine(pointSet[4*r], pointSet[4*r+2], 1, 1, 0, id[4*r+1], 0);     // select yellow color
          if (r==4) viewer->addLine(pointSet[4*r], pointSet[4*r+2], 1, 0, 1, id[4*r+1], 0);     // select purple color

          viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, id[4*r+1]);
        }
        if (i==7)
        {
          point.x = EightPoints(i,0);
          point.y = EightPoints(i,1);
          point.z = EightPoints(i,2);
          pointSet.push_back(point);
          // draw the third edge
          id[4*r+2] = "line"+4*r+2;
          // viewer->addLine(pointSet[4*r+1], pointSet[4*r+3], 0, 0, 1, id[4*r+2], 0);  // select blue color
          if (r==0) viewer->addLine(pointSet[4*r+1], pointSet[4*r+3], 1, 0, 0, id[4*r+2], 0);     // select red color
          if (r==1) viewer->addLine(pointSet[4*r+1], pointSet[4*r+3], 0, 1, 0, id[4*r+2], 0);     // select green color
          if (r==2) viewer->addLine(pointSet[4*r+1], pointSet[4*r+3], 0, 0, 1, id[4*r+2], 0);     // select blue color
          if (r==3) viewer->addLine(pointSet[4*r+1], pointSet[4*r+3], 1, 1, 0, id[4*r+2], 0);     // select yellow color
          if (r==4) viewer->addLine(pointSet[4*r+1], pointSet[4*r+3], 1, 0, 1, id[4*r+2], 0);     // select purple color
          viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, id[4*r+2]);
          
          // draw the last edge
          id[4*r+3] = "line"+4*r+3;
          // viewer->addLine(pointSet[4*r+2], pointSet[4*r+3], 0, 0, 1, id[4*r+3], 0);  // select blue color
          if (r==0) viewer->addLine(pointSet[4*r+2], pointSet[4*r+3], 1, 0, 0, id[4*r+3], 0);     // select red color
          if (r==1) viewer->addLine(pointSet[4*r+2], pointSet[4*r+3], 0, 1, 0, id[4*r+3], 0);     // select green color
          if (r==2) viewer->addLine(pointSet[4*r+2], pointSet[4*r+3], 0, 0, 1, id[4*r+3], 0);     // select blue color
          if (r==3) viewer->addLine(pointSet[4*r+2], pointSet[4*r+3], 1, 1, 0, id[4*r+3], 0);     // select yellow color
          if (r==4) viewer->addLine(pointSet[4*r+2], pointSet[4*r+3], 1, 0, 1, id[4*r+3], 0);     // select purple color
          viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, id[4*r+3]);
        }
    }
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce (100);
  }
}

}     // end of namespace

int main(int argc, char **argv)
{
  // pointcloudServer pclServer;
  ros::init(argc, argv, "pcl_server");
  ros::NodeHandle nh;
  ros::NodeHandle n_para;
  ros::Rate loop_rate(100);

  double slicingFactor, tolerance, rFL, rFW;
  n_para.param("slicingFactor", slicingFactor, 0.02);
  n_para.param("tolerance", tolerance, 0.05);
  n_para.param("robotFootWidth", rFW, 0.28);
  n_para.param("robotFootLength", rFL, 0.24);
  
  bool showBound, showSelRect, showPose;
  // n_para.param("showBoundary", showBound, false);
  n_para.param("showBoundary", showBound, true);
  n_para.param("showSelectRectange", showSelRect, true);
  n_para.param("showPose", showPose, true);
  // ROS_INFO_STREAM("The value of showBoundary is: [%s]", showBound.toString().c_str());
  // ROS_INFO("The value of showBoundary is: [%s]", showBound);
  
  std::string pclTopic;
  n_para.param("pclTopic", pclTopic, std::string("/camera_remote/depth_registered/points"));
  std::string filename;
  n_para.param("fileAddress", filename, std::string("/home/buivn/bui_ws/src/pahap/src/check1.pcd"));

  pcl_server::pclServer pclServer_process(pclTopic, filename, slicingFactor, tolerance, showBound, showSelRect, showPose, rFW, rFL);

  ros::spin();
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  return 0;
}
