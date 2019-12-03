/**************************************************************

 * ************************************************************/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "sensor_msgs/LaserScan.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/project_inliers.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <map>
#include <algorithm>
#include <pcl/common/impl/angles.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>


typedef pcl::PointXYZ PointT;

const float HORIZONTAL_VIEW_DEGREE = 60;//120.0; //水平视角
const int ULTRASOUND_NUM = HORIZONTAL_VIEW_DEGREE*1;//120; // * frame numbers //一共多少帧
const float HALF_HORIZONTAL_VIEW_DEGREE = HORIZONTAL_VIEW_DEGREE * 0.5;
const float ANGLE_INCREMENT = HORIZONTAL_VIEW_DEGREE / ULTRASOUND_NUM;
float HEIGNT_THRESHOLD = 0.10;
const float RAD_TO_DEGREE = 180.0 / M_PI;
const float SAFE_DISTANCE = 0.1;
const float MAX_DISTANCE = 5;//10;

ros::Publisher cloudForPlane_pub;
ros::Publisher boundaryG_pub;
ros::Publisher boundaryS_pub;
ros::Publisher laserAll_pub;
ros::Publisher allPoints_pub;
ros::Publisher ground_seg_pub;
ros::Publisher space_seg_pub;
ros::Publisher ground_norm_pub;
ros::Publisher space_norm_pub;
ros::Publisher GT_pub;

std::ofstream outfile;
double ray[ULTRASOUND_NUM+1];
boost::mutex mutex;
sensor_msgs::LaserScan scan_msg;
Eigen::Matrix4f mTransform_inv;
// Eigen::Vector3f groundAxis(0,0,-1);
Eigen::Vector3d groundAxis(0,-1,0);

Eigen::Matrix4d transM_depth_optical;
Eigen::Matrix4d transM_;
double trueGroundHeight = -1;
double trueRotateAngle = -1;

void rotate_to_worldFrame(pcl::PointCloud<PointT>::Ptr cloud , pcl::ModelCoefficients::ConstPtr coef , ros::Time time);
void pointsCallback (const sensor_msgs::PointCloud2ConstPtr& input);
pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud_boundary(pcl::PointCloud<PointT>);
void cloud_boundary_to_laser(pcl::PointCloud<pcl::PointXYZ>::Ptr);

int main (int argc, char** argv)
{
  if(argc == 2)
  {
    float tmp = atof(argv[1]);
    if(tmp < 0.2 && tmp > 0.03)
      HEIGNT_THRESHOLD = tmp;
  }

  // std::cout <<"Threshold of obstacle height is " << HEIGNT_THRESHOLD << std::endl;

  // Initialize ROS
  ros::init (argc, argv, "rgbd_to_laser");
  // std::cout << "ros::init done.." << std::endl;
  ros::NodeHandle nh;

  tf::TransformListener listener;
  tf::StampedTransform transformT_depth_optical;

  try{
    listener.waitForTransform(("/camera_depth_frame"), ("/camera_depth_optical_frame"), ros::Time(0), ros::Duration(2.0));
    listener.lookupTransform(("/camera_depth_frame"), ("/camera_depth_optical_frame"),  ros::Time(0), transformT_depth_optical);
  }catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::Matrix3x3 rotM = transformT_depth_optical.getBasis();
  tf::Vector3 origin = transformT_depth_optical.getOrigin();
  
  transM_depth_optical<<rotM[0][0], rotM[0][1], rotM[0][2], origin[0],
                  rotM[1][0], rotM[1][1], rotM[1][2], origin[1],
                  rotM[2][0], rotM[2][1], rotM[2][2], origin[2],
                           0,          0,          0,         1;

  double publishHz = 10;
  scan_msg.angle_max = (HORIZONTAL_VIEW_DEGREE/180.0 * M_PI)/2 - (HORIZONTAL_VIEW_DEGREE/180.0 * M_PI) /ULTRASOUND_NUM/2;
  scan_msg.angle_min = -scan_msg.angle_max;
  scan_msg.angle_increment = (HORIZONTAL_VIEW_DEGREE/180.0 * M_PI) /ULTRASOUND_NUM;
  scan_msg.scan_time = 1.0/publishHz;
  scan_msg.time_increment = scan_msg.scan_time / (double)(ULTRASOUND_NUM-1);
  scan_msg.range_min = SAFE_DISTANCE;
  scan_msg.range_max = MAX_DISTANCE;
  scan_msg.header.frame_id = "/camera_depth_frame";//"world";
  // std::cout << "scan_msg init done.." << std::endl;

  outfile.open("rgbd_to_laser_log.dat");
  // std::cout << "open file done.." << std::endl;

  // Create a ROS publisher for the output point cloud
  cloudForPlane_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_for_plane", 10);
  boundaryG_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_boundaryG", 10);//ground boundary
  boundaryS_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_boundaryS", 10);//space boundary
  allPoints_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_boundaryAll", 10);//all boundary
  laserAll_pub = nh.advertise<sensor_msgs::LaserScan>("/laser_boundaryAll", 10);//laser all
  ground_seg_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_ground_seg", 10);//ground
  space_seg_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_space_seg", 10);//ground
  ground_norm_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_ground_norm", 10);//ground  
  space_norm_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_space_norm", 10);//space  
  GT_pub = nh.advertise<sensor_msgs::PointCloud2>("/GT", 10);  
  // std::cout << "topics advertise done.." << std::endl;

  //ros::Rate loop_rate(10);

  // Create a ROS subscriber for the input point cloud
  // nh.subscribe(topic,quene_num,callback_function)
  ros::Subscriber pointCloudSub = nh.subscribe ("/camera/depth/points", 1, pointsCallback);
  // std::cout << "topics subscribe done.." << std::endl;

  ros::spin();
}


void pointsCallback (const sensor_msgs::PointCloud2ConstPtr& input)
{
  std::cout << "---start--------pointsCallback---------------------" << std::endl;

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<PointT> rawCloud;
  pcl::fromROSMsg (*input, rawCloud);
  // ROS_INFO("size of pointcloud is %d", (int)rawCloud.size());

  pcl::PointCloud<PointT> cloudForPlaneSeg ;
  pcl::ExtractIndices<PointT> extract;
  pcl::IndicesPtr indicesPT (new std::vector <int>);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (rawCloud.makeShared());
  pass.setFilterFieldName ("y");  //x:right y:down z:forward
  pass.setFilterLimits (-2, 1);
  pass.filter (*indicesPT);
  extract.setInputCloud (rawCloud.makeShared ());
  extract.setIndices (indicesPT);
  extract.setNegative (false);
  extract.filter (cloudForPlaneSeg);  // extract the inliers

  // downsample
  pcl::VoxelGrid<PointT> sor; //创建滤波对象
  sor.setDownsampleAllData(false); 
  sor.setInputCloud (cloudForPlaneSeg.makeShared());
  sor.setLeafSize (0.05f, 0.05f, 0.05f); //设置滤波时创建的体素大小为5cm立方体
  sor.filter (cloudForPlaneSeg);

  cloudForPlaneSeg.header = rawCloud.header;
  cloudForPlaneSeg.header.frame_id = "/camera_depth_frame";

  // PCL: Segmentation
  pcl::ModelCoefficients coefficientsSeg;
  pcl::PointIndices::Ptr groundPtr (new pcl::PointIndices);
  pcl::SACSegmentation<PointT> sac;
  pcl::PointCloud<PointT> tmpG, tmpS;

  sac.setInputCloud(cloudForPlaneSeg.makeShared());
  sac.setMethodType(pcl::SAC_RANSAC);
  sac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);//SACMODEL_PLANE
  sac.setDistanceThreshold(0.05);
  sac.setMaxIterations(100);
  sac.segment(*groundPtr, coefficientsSeg);
  // ROS_INFO("ground plane is (%f %f %f %f)" , coefficientsSeg.values[0],coefficientsSeg.values[1],coefficientsSeg.values[2],coefficientsSeg.values[3]);
  std::cout << "ground plane is (" << coefficientsSeg.values[0] << " "
                                  << coefficientsSeg.values[1] << " "
                                  << coefficientsSeg.values[2] << " "
                                  << coefficientsSeg.values[3] << " "
                                  << ")" << std::endl;
  
  ///////////////////get transform matrix
  std::cout << "---------start rotating---------" << std::endl;
  trueGroundHeight = fabs(coefficientsSeg.values[3]);
  Eigen::Vector3d plane_normal, ground_normal, rotateAxis;
  plane_normal << coefficientsSeg.values[0], coefficientsSeg.values[1], coefficientsSeg.values[2];
  plane_normal = plane_normal.normalized();
  if(plane_normal(1)>0){
    plane_normal = -plane_normal;
  }
  ground_normal = groundAxis;
  double rotateRad = acos(plane_normal.dot(ground_normal));
  trueRotateAngle = pcl::rad2deg(rotateRad);
  printf("camera: groundHeight: %f,  rotateAngle: %f\n", trueGroundHeight, trueRotateAngle);
  rotateAxis = plane_normal.cross(ground_normal);

  rotateAxis = rotateAxis.normalized();

  Eigen::AngleAxis<double> angleAxis(rotateRad, rotateAxis);
  Eigen::Matrix3d rotM  = angleAxis.toRotationMatrix();

  transM_<<rotM(0,0), rotM(0,1), rotM(0,2), 0,
           rotM(1,0), rotM(1,1), rotM(1,2), -trueGroundHeight,
           rotM(2,0), rotM(2,1), rotM(2,2), 0,
               0,         0,         0,     1;

  transM_ = transM_depth_optical * transM_;
  std::cout << transM_ << std::endl;

  std::cout << "---------end rotating---------" << std::endl;
  /////////////////////////


  // 利用ExtractIndices进行索引点云的提取
  extract.setInputCloud (cloudForPlaneSeg.makeShared ());
  extract.setIndices (groundPtr);
  extract.setNegative (false);
  extract.filter (tmpG);
  extract.setNegative (true);
  extract.filter (tmpS);

  pcl::PointCloud<pcl::PointXYZ>::Ptr groundBoundPoints, spaceBoundPoints, allBoundPoints;
  //get ground Boundary
  groundBoundPoints = get_cloud_boundary(tmpG);
 
  //get space boundary
  spaceBoundPoints = get_cloud_boundary(tmpS);

  //get all(ground+space) pointscloud
  pcl::PointCloud<PointT> cloudAll;
  cloudAll = tmpS + *groundBoundPoints;//space all points and ground boundary
  // cloudAll = *spaceBoundPoints + *groundBoundPoints;//both boundary
  
  allBoundPoints = cloudAll.makeShared();
  cloud_boundary_to_laser(allBoundPoints);//groundBoundPoints

  pcl::transformPointCloud( cloudForPlaneSeg, cloudForPlaneSeg, transM_);
  pcl::transformPointCloud( tmpG, tmpG, transM_);
  pcl::transformPointCloud( tmpS, tmpS, transM_);
  pcl::transformPointCloud( *groundBoundPoints, *groundBoundPoints, transM_);
  pcl::transformPointCloud( *spaceBoundPoints, *spaceBoundPoints, transM_);
  pcl::transformPointCloud( cloudAll, cloudAll, transM_);

  //pub cloudForPlaneSeg
  sensor_msgs::PointCloud2 could2ROS;
  pcl::toROSMsg(cloudForPlaneSeg , could2ROS);
  cloudForPlane_pub.publish(could2ROS);
  //pub seperated ground and space
  sensor_msgs::PointCloud2 cloud2ROS_G, cloud2ROS_S;
  pcl::toROSMsg(tmpG , cloud2ROS_G);
  ground_seg_pub.publish(cloud2ROS_G);
  pcl::toROSMsg(tmpS , cloud2ROS_S);
  space_seg_pub.publish(cloud2ROS_S);
  //pub boundaryG
  sensor_msgs::PointCloud2 cloud2ROS_boundaryG;
  pcl::toROSMsg(*groundBoundPoints, cloud2ROS_boundaryG);
  boundaryG_pub.publish(cloud2ROS_boundaryG);
  //pub boundaryS
  sensor_msgs::PointCloud2 cloud2ROS_boundaryS;
  pcl::toROSMsg(*spaceBoundPoints, cloud2ROS_boundaryS);
  boundaryS_pub.publish(cloud2ROS_boundaryS);
  //pub ground+space boundary
  sensor_msgs::PointCloud2 cloud2ROS_all;
  pcl::toROSMsg(cloudAll, cloud2ROS_all);
  allPoints_pub.publish(cloud2ROS_all);
 

  std::cout << "---end--------pointsCallback---------------------" << std::endl;

}



pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud_boundary(pcl::PointCloud<PointT> cloudInput)
{//get outline of ground
  // std::cout << "--------start outline extract------------" << std::endl;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  cloud = cloudInput.makeShared();
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<pcl::PointXYZ,pcl::Normal,pcl::Boundary> est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

  pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normEst;//其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
  normEst.setInputCloud(cloud);
  normEst.setSearchMethod(tree);
  // normEst.setRadiusSearch(2);  //法向估计的半径
  normEst.setKSearch(9);  //法向估计的点数
  normEst.compute(*normals);

  est.setInputCloud(cloud);
  est.setInputNormals(normals);
  // est.setAngleThreshold(90);
  // est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
  est.setSearchMethod (tree);
  est.setKSearch(50);  //一般这里的数值越高，最终边界识别的精度越好
  // est.setRadiusSearch(everagedistance);  //搜索半径
  est.compute (boundaries);

  //  pcl::PointCloud<pcl::PointXYZ> boundPoints;
  pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints (new pcl::PointCloud<pcl::PointXYZ>);
  boundPoints->header.frame_id = cloudInput.header.frame_id;
  int countBoundaries = 0;
  for (int i=0; i<cloud->size(); i++){
    uint8_t x = (boundaries.points[i].boundary_point);
    int a = static_cast<int>(x); //该函数的功能是强制类型转换
    if ( a == 1)
    {
      // 如果 boundaries.points[num].boundary_point == 1，则表示构造cloud的点云数组（这里假设是m_pointCloudData）m_pointCloudData[num]为模型的边界点；
      // 如果 boundaries.points[num].boundary_point == 0，则表示m_pointCloudData[num]为模型的非边界点
      if (cloud->points[i].z>1.2) //change to SAFE_DISTANCE
      {//把最近的边去掉
        (*boundPoints).push_back(cloud->points[i]);
        countBoundaries++;
      }
      // std::cout<< "boundPoints: " << boundPoints->points[i].x << " " 
      //                             << boundPoints->points[i].y << " "
      //                             << boundPoints->points[i].z << std::endl;
    }
  }
  // std::cout<<"boudary size is：" << countBoundaries <<std::endl;
  // pcl::io::savePCDFileASCII("/home/ssssubt/code/boudary.pcd", *boundPoints);
  
  // std::cout << "---------end outline extract------------" << std::endl;
  return boundPoints;
}


void cloud_boundary_to_laser(pcl::PointCloud<pcl::PointXYZ>::Ptr inputPoints)
{//boundary to laserscan
  // std::cout << "-------start boundary to laser-------" << std::endl;
  int pcindex[ULTRASOUND_NUM+1];
  for(int i=0;i<=ULTRASOUND_NUM;i++)
  {//initialize
    ray[i] = std::numeric_limits<float>::infinity();//0;
  }
  for(size_t i = 0 ; i < inputPoints->points.size() ; i++)
  {//every point of one frame
    PointT p = inputPoints->points[i];
    // std::cout << "point is: " << "(" << p.x << "," << p.y << "," << p.z << ")" << std::endl;
    
    float distance = sqrt(p.x*p.x + p.z*p.z); //x:right y:down z:forward
    // std::cout << "distance is " << distance << std::endl;
    
    double angle = atan(-p.x/p.z) * RAD_TO_DEGREE;  //x:right y:down z:forward
    // // std::cout << "angle is " << angle << std::endl;
    
    if(angle > HALF_HORIZONTAL_VIEW_DEGREE || angle < -HALF_HORIZONTAL_VIEW_DEGREE)
      continue;

    if(fabs(distance)< SAFE_DISTANCE || distance > MAX_DISTANCE)
      distance = std::numeric_limits<float>::infinity();

    int index = round( (angle + HALF_HORIZONTAL_VIEW_DEGREE) / ANGLE_INCREMENT );
    // std::cout << "index: " << index << std::endl;
   
    if(distance < ray[index])//distance > ray[index]
    {
      ray[index] = distance;
    }

  }

  //  mutex.lock();
  scan_msg.ranges.clear();
  scan_msg.header.stamp = ros::Time::now();
  for(int i=0;i<=ULTRASOUND_NUM;i++)
  {
    double data = ray[i];
    //printf("data = %f\t", data);
    scan_msg.ranges.push_back(data);
  }
  //  mutex.unlock();
  laserAll_pub.publish(scan_msg);
  // std::cout << "-------end boundary to laser-------" << std::endl;
}