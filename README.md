# Hybrid Steel Bridge Inspection Robot
This repo contains all the functions, libraries which were used in this projects. There are some main functions such as:
(1) Point Cloud Boundary
(2) Area check
(3) Pose Estimation
(4) Plane's Height check
(5) Path Availability in Mobile Transformation 

  To get a stable point cloud data, the data is captured several times, then combine the pointclouds into one. THe processing on the aggregated one is more stable than a single one.

The program works on ROS-Service approach. All functions are in the hybrid_robot.cpp, pcl_server.cpp, mp_server.cpp files, which operates as a server. To trigger services, the program hybrid_robot.cpp, pcl_server.cpp, mp_server.cpp are invoked.


Full implementation video on youtube of Hybrid Steel Bridge Inspection Robot - ARA Lab
<p align="center">
<a href="https://www.youtube.com/watch?v=SHk5IIOBRdA&feature=youtu.be" 
target="_blank"><img src="/images/HybridRobot.gif" width="450" 
alt="Hybrid Steel Bridge Inspection Robot" width="450"/></a>
</p>



