/**************************************************************
 * Author: Brendon
 * Description: project pointcloud into ground plane, and build it as a laserscan in local ground frame
 * subscribe:
 *          /ground_in_imuframe             // pose of ground in imu frame (/camera_accel_optical_frame which is same as /camera_depth_optical_frame)
 *          /camera/depth/color/points      // pointcloud published by realsense d435i
 * publish:
 *          //laserscan_3d                  // data of laserscan in local ground frame
 * ************************************************************/
#include <ros/ros.h>
// PCL specific includes
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

#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>


typedef pcl::PointXYZ PointT;

const float	HORIZONTAL_VIEW_DEGREE = 60;//120.0;
const int ULTRASOUND_NUM = HORIZONTAL_VIEW_DEGREE*4;//120;
const float	HALF_HORIZONTAL_VIEW_DEGREE = HORIZONTAL_VIEW_DEGREE * 0.5;
const float ANGLE_INCREMENT = HORIZONTAL_VIEW_DEGREE / ULTRASOUND_NUM;
float HEIGNT_THRESHOLD = 0.10;
const float RAD_TO_DEGREE = 180.0 / M_PI;
const float SAFE_DISTANCE = 0.1;
const float MAX_DISTANCE = 10;

ros::Publisher colorPlane_pub;
ros::Publisher test_pub;
ros::Publisher cloudForPlane_pub;
ros::Publisher obstacle_pub;
ros::Publisher pc_Scan_pub;
ros::Publisher scan_pub;
ros::Publisher ground_pub;
ros::Publisher centroid_pub;
ros::Publisher path_pub;
ros::Publisher region_pub;
ros::Publisher boundary_pub;
ros::Publisher boundary2laser_pub;

std::ofstream outfile;
double ray[ULTRASOUND_NUM+1];
boost::mutex mutex;
sensor_msgs::LaserScan scan_msg;
Eigen::Matrix4f mTransform_inv;
// Eigen::Vector3f groundAxis(0,0,-1);
Eigen::Vector3f groundAxis(0,-1,0);


bool isInfValue(double value);
void fillFalseInf(double ray[]);
void project_to_laserscan(pcl::PointCloud<PointT>::Ptr cloud , pcl::ModelCoefficients::ConstPtr coef , ros::Time time);
void ground_in_imu_cb (const geometry_msgs::Vector3StampedConstPtr& ground);
void segmentation_with_cluster_cb (const sensor_msgs::PointCloud2ConstPtr& input);
void CB_publishCycle(const ros::TimerEvent& e);
void ground_extract(const sensor_msgs::PointCloud2ConstPtr& input);
void centroid_get(const sensor_msgs::PointCloud2ConstPtr& input);
void path_extract(const sensor_msgs::PointCloud2ConstPtr& input);
void get_region(const sensor_msgs::PointCloud2ConstPtr& input);
//void boundary_get(const sensor_msgs::PointCloud2ConstPtr& input);

int max_it = 200;
int main (int argc, char** argv)
{
  if(argc == 2)
  {
    float tmp = atof(argv[1]);
    if(tmp < 0.2 && tmp > 0.03)
      HEIGNT_THRESHOLD = tmp;
  }

  std::cout <<"Threshold of obstacle height is " << HEIGNT_THRESHOLD << std::endl;

  // Initialize ROS
  ros::init (argc, argv, "ground_extract");
  std::cout << "ros::init done.." << std::endl;
  ros::NodeHandle nh;

  double publishHz = 10;
  scan_msg.angle_max = (HORIZONTAL_VIEW_DEGREE/180.0 * M_PI)/2 - (HORIZONTAL_VIEW_DEGREE/180.0 * M_PI) /ULTRASOUND_NUM/2;
  scan_msg.angle_min = -scan_msg.angle_max;
  scan_msg.angle_increment = (HORIZONTAL_VIEW_DEGREE/180.0 * M_PI) /ULTRASOUND_NUM;
  scan_msg.scan_time = 1.0/publishHz;
  scan_msg.time_increment = scan_msg.scan_time / (double)(ULTRASOUND_NUM-1);
  scan_msg.range_min = SAFE_DISTANCE;
  scan_msg.range_max = MAX_DISTANCE;
  scan_msg.header.frame_id = "local_ground_frame";
  std::cout << "scan_msg init done.." << std::endl;

  outfile.open("ground_extract_log.dat");
  std::cout << "open file done.." << std::endl;

  // Create a ROS publisher for the output point cloud
  test_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud", 10);
  obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_obastacle", 10);
  pc_Scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_laserscan", 10);
  scan_pub = nh.advertise<sensor_msgs::LaserScan>("/laserscan_3d", 10);
  colorPlane_pub = nh.advertise<sensor_msgs::PointCloud2>("/colored_plane", 10);
  cloudForPlane_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_for_plane", 10);
  ground_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_ground", 10);//ground
  centroid_pub = nh.advertise<sensor_msgs::PointCloud2>("/centroid_point", 10);//centroid
  path_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_path", 10);//path
  region_pub = nh.advertise<sensor_msgs::PointCloud2>("/region", 10);//region
  boundary_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_boundary", 10);//boundary
  boundary2laser_pub = nh.advertise<sensor_msgs::LaserScan>("/boundary_to_laser", 10);
  std::cout << "topics advertise done.." << std::endl;

  // Create a ROS subscriber for the input point cloud
  // nh.subscribe(topic,quene_num,callback_function)
  ros::Subscriber pointCloudSub = nh.subscribe ("/camera/depth/points", 1, segmentation_with_cluster_cb);
  //ros::Subscriber groundSub = nh.subscribe ("/ground_in_imuframe", 1, ground_in_imu_cb);//get groundAxis from topic /ground_in_imufram
  ros::Subscriber planeSub = nh.subscribe("/cloud_for_plane",1, ground_extract);
  ros::Subscriber centroidSub = nh.subscribe("/pointcloud_ground", 1, centroid_get);
  ros::Subscriber pathSub = nh.subscribe("/pointcloud_ground", 1, path_extract);
  ros::Subscriber regionSub = nh.subscribe("/pointcloud_ground", 1, get_region);
  //ros::Subscriber boundarySub = nh.subscribe("/pointcloud_ground", 1, boundary_get);
  std::cout << "topics subscribe done.." << std::endl;

  ros::spin();
}

void get_region(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //std::cout << "---start get_region------------" << std::endl;
  pcl::PointCloud<PointT> cloudGround;
  pcl::fromROSMsg (*input, cloudGround);
  //get_region
  std::map<int, float> outline;
  int theta = -22;
  for(theta;theta<23;theta++){ 
    float max = 0;
    for (int i = 0; i < cloudGround.size(); i++)
    {
      double k = cloudGround[i].z/cloudGround[i].x;//slop of point
      double theta_point = atan(k) * 180.0 / M_PI;
      //std::cout << "theta: " << theta <<"   k: " << k << "  theta_point:" << theta_point << std::endl;
      //std::cout << "x:" << cloudGround[i].x << " y:" << cloudGround[i].y << " z:" << cloudGround[i].z << std::endl;
      if( abs(theta-theta_point) < 0.5 ){//x:right y:down z:forward
        //std::cout << "on line" << std::endl; 
        if (cloudGround[i].z>max){max = cloudGround[i].z;}
      }
    }
    //std::cout << "max: " << max << std::endl;
    outline.insert(std::map<int, float>::value_type (theta, max));
  }

  //std::cout << "---end get_region------------" << std::endl;
}

void path_extract(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //std::cout << "---start path_extract------------" << std::endl;
  pcl::PointCloud<PointT> cloudGround;
  pcl::fromROSMsg (*input, cloudGround);
  pcl::PointCloud<pcl::PointXYZRGB> cloudGroundRGB;
  pcl::fromROSMsg (*input, cloudGroundRGB);
  
  //filte path cruzr could pass
  //x:-0.37~0.31
  // pcl::PointCloud<PointT> cloudPath;
  // pcl::ExtractIndices<PointT> extract;
  // pcl::IndicesPtr indicesPT (new std::vector <int>);
  // pcl::PassThrough<PointT> pass;
  // pass.setInputCloud (cloudGround.makeShared());
  // pass.setFilterFieldName ("x");//x:right y:down z:forward   coordinate hight is 0.55
  // pass.setFilterLimits (-0.37, 0.31);//cruzr farthest left point from camera to farthest right point
  // pass.filter (*indicesPT);
  // extract.setInputCloud (cloudGround.makeShared ());
  // extract.setIndices (indicesPT);
  // extract.setNegative (false);
  // extract.filter (cloudPath);  // extract the inliers
  // std::cout << "after filter points: " << cloudPath.size() << std::endl;

  //extract cloudPath
  // pcl::copyPointCloud(cloudPath,cloudPathRGB);
  // pcl::PointXYZRGB p;
  // for (int i = 0; i < cloudGround.size(); ++i)
  // {
  //   p.x = cloudPath[i].x;
  //   p.y = cloudPath[i].y;
  //   p.z = cloudPath[i].z;
  //   p.r = 255;
  //   p.g = 0;
  //   p.b = 0;
  //   cloudGroundRGB.push_back(p);
  // }
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_Ptr = cloudGroundRGB.makeShared();
  // std::cout << "---points before delete: " << cloud_Ptr->size() << std::endl;
  //get centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(cloudGround, centroid);
  std::cout << "center in path " << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
  float k = centroid[2]/centroid[0]; //slop of origin to centroid line k=z/x //x:right y:down z:forward
  std::cout << "k: " << k << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::iterator index = cloud_Ptr->begin();
  for (int i=0;i<cloudGroundRGB.size();i++)
  {//pass path
    // pcl::PointXYZRGB p = cloudGroundRGB[i];
    pcl::PointXYZRGB p = cloud_Ptr->points[i];

    float xOnLine = p.z/k; //get point in line
    // std::cout << "xOnLine: " << xOnLine << std::endl;
    if (xOnLine-0.37>p.x || p.x>xOnLine+0.31 || p.z > centroid[2])
    {//pass lineX-0.37<p.x<lineX+0.31 
      //delet poinit
      // std::cout << "-----delet point----" << std::endl;
      // cloud_Ptr->erase(index+i);
      cloud_Ptr->points[i].r = 255;
      cloud_Ptr->points[i].g = 255;
      cloud_Ptr->points[i].b = 255;
    }
    else
    {//add color
      // std::cout << "-----add color----" << std::endl;
      cloud_Ptr->points[i].r = 255;
      cloud_Ptr->points[i].g = 0;
      cloud_Ptr->points[i].b = 0;
      // std::cout << "r: " << (int)cloud_Ptr->points[i].r << std::endl;
    }
  }
  // std::cout << "---points after delete: " << cloud_Ptr->size() << std::endl;

  sensor_msgs::PointCloud2 cloud2ROS;
  pcl::toROSMsg(*cloud_Ptr, cloud2ROS);
  path_pub.publish(cloud2ROS);
  //std::cout << "---end path extract------------" << std::endl;

}

/*
void boundary_get(const sensor_msgs::PointCloud2ConstPtr& input)
{//get outline of ground
  std::cout << "---start outline extract------------" << std::endl;
  pcl::PointCloud<PointT> cloudGround;
  pcl::fromROSMsg (*input, cloudGround);
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  cloud = cloudGround.makeShared();
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
  cout<<"normal size is "<< normals->size()<<endl;
  
  //normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
  est.setInputCloud(cloud);
  est.setInputNormals(normals);
  //  est.setAngleThreshold(90);
  //   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
  est.setSearchMethod (tree);
  est.setKSearch(50);  //一般这里的数值越高，最终边界识别的精度越好
  //  est.setRadiusSearch(everagedistance);  //搜索半径
  est.compute (boundaries);

  //  pcl::PointCloud<pcl::PointXYZ> boundPoints;
  pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
  int countBoundaries = 0;
  for (int i=0; i<cloud->size(); i++){
    uint8_t x = (boundaries.points[i].boundary_point);
    int a = static_cast<int>(x); //该函数的功能是强制类型转换
    if ( a == 1)
    {
      if (cloud->points[i].z>1.5)
      {//把最下面的边去掉
        (*boundPoints).push_back(cloud->points[i]);
        countBoundaries++;
      }
      // std::cout<< "boundPoints: " << boundPoints->points[i].x << " " 
      //                             << boundPoints->points[i].y << " "
      //                             << boundPoints->points[i].z << std::endl;
    }
    else
      noBoundPoints.push_back(cloud->points[i]);
  }
  std::cout<<"boudary size is：" << countBoundaries <<std::endl;
  pcl::io::savePCDFileASCII("/home/ssssubt/code/boudary.pcd", *boundPoints);
  //pcl::io::savePCDFileASCII("/home/ssssubt/code/NoBoundpoints.pcd",noBoundPoints);
  sensor_msgs::PointCloud2 cloud2ROS_boundary;
  pcl::toROSMsg(*boundPoints, cloud2ROS_boundary);
  boundary_pub.publish(cloud2ROS_boundary);
  //pcl::PointCloud<PointT> checkCloud;
  //pcl::fromROSMsg (cloud2ROS_boundary, checkCloud);
  //std::cout << "check points: " << checkCloud.size() << std::endl;
  std::cout << "---end outline extract------------" << std::endl;
}*/

void centroid_get(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //std::cout << "---start centroid------------" << std::endl;
  pcl::PointCloud<PointT> cloudGround;
  pcl::PointCloud<pcl::PointXYZRGB> cloudGroundRGB;
  pcl::fromROSMsg (*input, cloudGround);
  pcl::fromROSMsg (*input, cloudGroundRGB);
  //init RGB
  for(int i=0;i<cloudGround.size();i++)
  {
    cloudGroundRGB[i].x = std::numeric_limits<float>::quiet_NaN();
    cloudGroundRGB[i].y = std::numeric_limits<float>::quiet_NaN();
    cloudGroundRGB[i].z = std::numeric_limits<float>::quiet_NaN();
    cloudGroundRGB[i].r = std::numeric_limits<float>::quiet_NaN();
    cloudGroundRGB[i].g = std::numeric_limits<float>::quiet_NaN();
    cloudGroundRGB[i].b = std::numeric_limits<float>::quiet_NaN();
    // std::cout << "cloudGroundRGB["<< i <<"]:("<< cloudGroundRGB[i].x << "," << cloudGroundRGB[i].y << ","<< cloudGroundRGB[i].z 
    //         << "," << (int)cloudGroundRGB[i].r << "," << (int)cloudGroundRGB[i].g << "," << (int)cloudGroundRGB[i].b << ")" << std::endl;
  }

  //创建存储点云质心的对象
  Eigen::Vector4f centroid;
  // get centroid
  pcl::compute3DCentroid(cloudGround, centroid);
  // std::cout << "centroid:("<< centroid[0] << ","<< centroid[1] << ","<< centroid[2] << ")" << std::endl;
  //pcl::PointCloud<pcl::PointXYZRGB> cloudCentroid;
  pcl::PointXYZRGB pointCentroid;
  pointCentroid.x = centroid[0];
  pointCentroid.y = centroid[1];
  pointCentroid.z = centroid[2];
  pointCentroid.r = 255;
  pointCentroid.g = 0;
  pointCentroid.b = 0;
  cloudGroundRGB.push_back(pointCentroid);
  // std::cout << "cloudCentroid points: " << cloudCentroid.size() << std::endl;
  // std::cout << "centroid:("<< pointCentroid.x << "," << pointCentroid.y << ","<< pointCentroid.z 
  //           << "," << (int)pointCentroid.r << "," << (int)pointCentroid.g << "," << (int)pointCentroid.b << ")" << std::endl;
  sensor_msgs::PointCloud2 cloud2ROS;
  pcl::toROSMsg(cloudGroundRGB , cloud2ROS);
  centroid_pub.publish(cloud2ROS);
  //std::cout << "---end centroid------------" << std::endl;  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
}

void ground_extract(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //std::cout << "---start ground extract------------" << std::endl;
  pcl::PointCloud<PointT> cloudForPlaneSeg;
  pcl::fromROSMsg (*input, cloudForPlaneSeg);
  std::cout << "input points: " << cloudForPlaneSeg.size() << std::endl;
  pcl::PointCloud<PointT> cloudGround;
  pcl::ExtractIndices<PointT> extract;
  // segment out points far from camera in camera frame
  pcl::IndicesPtr indicesPT (new std::vector <int>);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloudForPlaneSeg.makeShared());
  pass.setFilterFieldName ("y");//x:right y:down z:forward   coordinate hight is 0.5~0.6
  pass.setFilterLimits (0.29, 0.35);//cruzr is 0.55-0.6; turtlebot is 0.25-0.35
  pass.filter (*indicesPT);
  extract.setInputCloud (cloudForPlaneSeg.makeShared ());
  extract.setIndices (indicesPT);
  extract.setNegative (false);
  extract.filter (cloudGround);  // extract the inliers
  std::cout << "after filter points: " << cloudGround.size() << std::endl;
  sensor_msgs::PointCloud2 cloud2ROS;
  pcl::toROSMsg(cloudGround , cloud2ROS);
  ground_pub.publish(cloud2ROS);
  //std::cout << "---end ground extract------------" << std::endl;


  //get outline of ground
  std::cout << "---start outline extract------------" << std::endl;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  cloud = cloudGround.makeShared();
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
  cout<<"normal size is "<< normals->size()<<endl;
  
  //normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
  est.setInputCloud(cloud);
  est.setInputNormals(normals);
  //  est.setAngleThreshold(90);
  //   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
  est.setSearchMethod (tree);
  est.setKSearch(50);  //一般这里的数值越高，最终边界识别的精度越好
  //  est.setRadiusSearch(everagedistance);  //搜索半径
  est.compute (boundaries);

  //  pcl::PointCloud<pcl::PointXYZ> boundPoints;
  pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
  //boundPoints->header = input->header;
  boundPoints->header.frame_id = "/camera_depth_optical_frame";
  int countBoundaries = 0;
  for (int i=0; i<cloud->size(); i++){
    uint8_t x = (boundaries.points[i].boundary_point);
    int a = static_cast<int>(x); //该函数的功能是强制类型转换
    if ( a == 1)
    {
      if (cloud->points[i].z>1)
      {//把最近的边去掉
        (*boundPoints).push_back(cloud->points[i]);
        countBoundaries++;
      }
      // std::cout<< "boundPoints: " << boundPoints->points[i].x << " " 
      //                             << boundPoints->points[i].y << " "
      //                             << boundPoints->points[i].z << std::endl;
    }
    else
      noBoundPoints.push_back(cloud->points[i]);
  }
  std::cout<<"boudary size is：" << countBoundaries <<std::endl;
  //pcl::io::savePCDFileASCII("/home/ssssubt/code/boudary.pcd", *boundPoints);
  //pcl::io::savePCDFileASCII("/home/ssssubt/code/NoBoundpoints.pcd",noBoundPoints);
  
  sensor_msgs::PointCloud2 cloud2ROS_boundary;
  pcl::toROSMsg(*boundPoints, cloud2ROS_boundary);
  boundary_pub.publish(cloud2ROS_boundary);
  //pcl::PointCloud<PointT> checkCloud;
  //pcl::fromROSMsg (cloud2ROS_boundary, checkCloud);
  //std::cout << "check points: " << checkCloud.size() << std::endl;
  std::cout << "---end outline extract------------" << std::endl;



  //boundary to laserscan
  std::cout << "---start boundary to laser-------" << std::endl;
  int pcindex[ULTRASOUND_NUM+1];
  for(int i=0;i<=ULTRASOUND_NUM;i++)
  {//initialize
    ray[i] = 0;//std::numeric_limits<float>::infinity();
    pcindex[i] = -1;
  }
  pcl::PointCloud<PointT> pcLaserScan;
  for(size_t i = 0 ; i < boundPoints->points.size() ; i++)
  {//every point of one frame
    PointT p = boundPoints->points[i];
    std::cout << "point is: " << "(" << p.x << "," << p.y << "," << p.z << ")" << std::endl;
    // std::cout << " z " << p.z << std::endl;
    float distance = sqrt(p.x*p.x + p.z*p.z);//x:right y:down z:forward
    std::cout << "distance is " << distance << std::endl;
    /*
    float heightThreshold = HEIGNT_THRESHOLD; //default is 0.1// adjust heightThreshold by the distance from origin point, because noise varys with distance
    if(distance > 3.0)
      heightThreshold = (distance * 0.2 + 0.4) * HEIGNT_THRESHOLD;
    // std::cout << "z is " << p.z << std::endl;
    // std::cout << "heightThreshold is " << heightThreshold << std::endl;
    if(p.z < heightThreshold || p.z > 2.0)
      continue;
    */
    double angle = atan(p.x/p.z) * RAD_TO_DEGREE;//x:right y:down z:forward
    std::cout << "angle is " << angle << std::endl;
    if(angle > HALF_HORIZONTAL_VIEW_DEGREE || angle < -HALF_HORIZONTAL_VIEW_DEGREE)//((angle>0 && angle<(90-HALF_HORIZONTAL_VIEW_DEGREE)) || (angle<0 && angle>(-HALF_HORIZONTAL_VIEW_DEGREE)))
      continue;

    if(fabs(distance)< SAFE_DISTANCE || distance > MAX_DISTANCE)
      distance = std::numeric_limits<float>::infinity();

    int index = round( (angle + HALF_HORIZONTAL_VIEW_DEGREE) / ANGLE_INCREMENT );
    std::cout << "index: " << index << std::endl;
    if(distance > ray[index])
    {
      ray[index] = distance;
      pcindex[index] = i;
    }
    //pcObastacle.push_back(cloud->points.at(i));

  }
  for(int i =0 ; i <= ULTRASOUND_NUM ; i++)
  {//every frame of Laser
    int index = pcindex[i];
    if(index != -1)
    {
      PointT p = boundPoints->points[index];
      pcLaserScan.push_back(PointT(p.x , p.y , 0));
    }
  }
  //pcl::transformPointCloud(pcLaserScan , pcLaserScan , mTransform);

  // sensor_msgs::PointCloud2 could2ROS;
  // pcLaserScan.header = boundPoints->header;
  // pcl::toROSMsg(pcLaserScan , could2ROS);
  // pc_Scan_pub.publish(could2ROS);

  // pcObastacle.header = boundPoints->header;
  // pcl::toROSMsg(pcObastacle , could2ROS);
  // obstacle_pub.publish (could2ROS);

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
  boundary2laser_pub.publish(scan_msg);
  std::cout << "---end boundary to laser-------" << std::endl;
}

bool isInfValue(double value)
{
  if(value>MAX_DISTANCE || value<SAFE_DISTANCE){
    return true;
  }else{
    return false;
  }
}

void fillFalseInf(double ray[])
{
  int leftIndex, rightIndex;
  int leftFence = 0;
  for(int j=1;j<=ULTRASOUND_NUM;j++){
    if(isInfValue(ray[j]))
    {  //当前值INF
      int i = j-1;
      while( i>=leftFence && isInfValue(ray[i]) )
      { //向当前值的左侧查找最近的有效值
        i--;
      }
      if(i>=leftFence)
      {
        leftIndex = i;	//找到左侧最近有效值索引为ｉ
      }
      else
      {
        leftFence = j+1;	//左侧没找到有效值，向左查询最近有效值的最左处设置为当前值的下一个位置索引
        continue;			//继续向后遍历
      }

      int k = j+1;
      while( k<=ULTRASOUND_NUM && isInfValue(ray[k]) )
      {//向当前值的右侧查找最近的有效值
        k++;
      }
      if(k<=ULTRASOUND_NUM)
      {
        rightIndex = k;		//右侧找到最近有效值的索引值为ｋ
      }
      else
      {
        break;				//右侧没有有效值，直接终止遍历
      }

      //执行到此处，说明找到左侧索引为ｉ的有效值，右侧索引为ｋ的有效值，将中间的所有INF值进行线性赋值
      for(int m=i+1;m<k;m++){
        ray[m] = ray[i] + (m-i)/(k-i) * (ray[k] - ray[i]);
      }

      //当前值直接跳到右侧最近的有效值，继续向后遍历
      j = k;
    }
  }
}

void project_to_laserscan(pcl::PointCloud<PointT>::Ptr cloud , pcl::ModelCoefficients::ConstPtr coef , ros::Time time)
{
//  static ros::Time t0 = ros::Time::now();
//  ros::Time t1 = ros::Time::now();
//  std::cout << "project periods is "<< (t1 - t0).toSec() << std::endl;
//  t0 = t1;
//  std::cout << "project cloud size is " << cloud->size() << std::endl;
  std::cout << "---start--------project_to_laserscan----------" << std::endl;
  // step1: choose two points in reference frame, and form axis x of local ground frame with the two points
  pcl::PointCloud<PointT> c , pcObastacle;
  PointT point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  c.push_back(point);
  point.z = 1;
  c.push_back(point);

  // 创建ProjectInliers对象，使用ModelCoefficients作为投影对象的模型参数
  pcl::ProjectInliers<PointT> proj;     //创建投影滤波对象
  proj.setModelType(pcl::SACMODEL_PLANE);     //设置对象对应的投影模型
  proj.setInputCloud(c.makeShared());         //设置输入点云
  proj.setModelCoefficients(coef);            //设置模型对应的系数
  proj.filter(c);                             //投影结果存储

  // step2: create local ground frame
  Eigen::Vector3f zAxis(coef->values[0] , coef->values[1] , coef->values[2]); //normal of plane
  PointT p0 = c.front();
  PointT p1 = c.back();
  Eigen::Vector3f xAxis(p1.x-p0.x , p1.y-p0.y , p1.z-p0.z);//set projection as xAxis
  Eigen::Vector3f yAxis = zAxis.cross(xAxis);//xAxis*zAxis得到垂直的yAxisd
  //ground coordinate
  xAxis.normalize();
  yAxis.normalize();
  zAxis.normalize();

  Eigen::Matrix3f rotM;
  rotM<<xAxis[0], yAxis[0], zAxis[0],
        xAxis[1], yAxis[1], zAxis[1],
        xAxis[2], yAxis[2], zAxis[2];
  Eigen::Vector3f t(p0.x , p0.y , p0.z);
  Eigen::Vector3f t_inv = -rotM.transpose() * t;
  //R.transpose = R.inverse
  Eigen::Matrix4f mTransform;
  mTransform<<xAxis[0], yAxis[0], zAxis[0], t[0],
              xAxis[1], yAxis[1], zAxis[1], t[1],
              xAxis[2], yAxis[2], zAxis[2], t[2],
                     0,        0,        0,    1;

  mTransform_inv<<xAxis[0], xAxis[1], xAxis[2], t_inv[0],
                  yAxis[0], yAxis[1], yAxis[2], t_inv[1],
                  zAxis[0], zAxis[1], zAxis[2], t_inv[2],
                         0,        0,        0,        1;
  // step3: broadcast the transform from reference frame to local ground frame
  Eigen::Quaternionf q_eigen(rotM);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(p0.x , p0.y , p0.z));
  transform.setRotation(tf::Quaternion(q_eigen.x() , q_eigen.y() , q_eigen.z() , q_eigen.w()));

  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform , time , "camera_depth_optical_frame" , "local_ground_frame"));

  // step4: transform pointcloud to local ground frame
  pcl::PointCloud<PointT>::Ptr cloudInGround(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cloud , *cloudInGround , mTransform_inv);
  cloudInGround->header = cloud->header;
  cloudInGround->header.frame_id = "local_ground_frame";
  std::cout << "transform cloud size is " << cloudInGround->size() << std::endl;

  //  sensor_msgs::PointCloud2 could2ROS;
  //  pcl::toROSMsg(*cloudInGround , could2ROS);
  //  pub.publish (could2ROS);

  // step5: transform pointcloud in local ground frame to laserscan
  int pcindex[ULTRASOUND_NUM+1];
  for(int i=0;i<=ULTRASOUND_NUM;i++)
  {//initialize
    ray[i] = std::numeric_limits<float>::infinity();
    pcindex[i] = -1;
  }
  pcl::PointCloud<PointT> pcLaserScan;
  for(size_t i = 0 ; i < cloudInGround->points.size() ; i++)
  {//every point of one frame
    PointT p = cloudInGround->points[i];
    // std::cout << "point is: " << "(" << p.x << "," << p.y << "," << p.z << ")" << std::endl;
    // std::cout << " z " << p.z << std::endl;
    float distance = sqrt(p.x*p.x + p.y*p.y);
    // std::cout << "distance is " << distance << std::endl;
    /*
    float heightThreshold = HEIGNT_THRESHOLD; //default is 0.1// adjust heightThreshold by the distance from origin point, because noise varys with distance
    if(distance > 3.0)
      heightThreshold = (distance * 0.2 + 0.4) * HEIGNT_THRESHOLD;
    // std::cout << "z is " << p.z << std::endl;
    // std::cout << "heightThreshold is " << heightThreshold << std::endl;
    if(p.z < heightThreshold || p.z > 2.0)
      continue;
    */
    double angle = atan(p.y/p.x) * RAD_TO_DEGREE;
    //std::cout << "angle is " << angle << std::endl;
    if(angle > HALF_HORIZONTAL_VIEW_DEGREE || angle < -HALF_HORIZONTAL_VIEW_DEGREE)
      continue;

    if(fabs(distance)< SAFE_DISTANCE || distance > MAX_DISTANCE)
      distance = std::numeric_limits<float>::infinity();

    int index = round( (angle + HALF_HORIZONTAL_VIEW_DEGREE) / ANGLE_INCREMENT );
    if(distance > ray[index])
    {
      ray[index] = distance;
      pcindex[index] = i;
    }
    pcObastacle.push_back(cloud->points.at(i));

  }
  for(int i =0 ; i <= ULTRASOUND_NUM ; i++)
  {//every frame of Laser
    int index = pcindex[i];
    if(index != -1)
    {
      PointT p = cloudInGround->points[index];
      pcLaserScan.push_back(PointT(p.x , p.y , 0));
    }
  }
  pcl::transformPointCloud(pcLaserScan , pcLaserScan , mTransform);

  sensor_msgs::PointCloud2 could2ROS;
  pcLaserScan.header = cloud->header;
  pcl::toROSMsg(pcLaserScan , could2ROS);
  pc_Scan_pub.publish(could2ROS);

  pcObastacle.header = cloud->header;
  pcl::toROSMsg(pcObastacle , could2ROS);
  obstacle_pub.publish (could2ROS);

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
  scan_pub.publish(scan_msg);
  std::cout << "---end--------project_to_laserscan----------" << std::endl;
}

void region_growing_segmentation(pcl::PointCloud<PointT>::Ptr cloud , std::vector <pcl::PointIndices>* clusters)
{//use region growing method to segment planes
  //求法线
  pcl::search::Search<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);
  
  //reg growing
  pcl::RegionGrowing<PointT, pcl::Normal> reg;
  reg.setMinClusterSize (cloud->points.size() * 0.1);
  reg.setMaxClusterSize (100000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (2.0);
  reg.extract (*clusters);

  // std::cout << "Number of normals is equal to " << normals->size() << std::endl;
  // std::cout << "Number of clusters is equal to " << clusters->size () << std::endl;
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

  if(clusters->empty())
    return;

  sensor_msgs::PointCloud2 cloud2ROS;
  colored_cloud->header = cloud->header;
  pcl::toROSMsg(*colored_cloud , cloud2ROS);
  colorPlane_pub.publish(cloud2ROS);
}

void ground_in_imu_cb (const geometry_msgs::Vector3StampedConstPtr& ground)
{//not use if has no ium
  //get ground normal axis, set(0,-1,0)  x,y,z => right,down,forward
  groundAxis << ground->vector.x , ground->vector.y , ground->vector.z;
  //groundAxis << 0,-1,0;
}

//initial parameters of segmentation_with_cluster_cb
pcl::ModelCoefficients coefficientsOfPlane;
bool hasFirstGroundAxis = false;
bool hasNoGround = false;
int nCountNoGround = 0;
const int MAX_COUNT_NO_GROUND = 20;
unsigned long nCount_Test = 0;
unsigned long nCountCorrect_Test = 0;
unsigned long nCountWarning_Test = 0;
unsigned long nCountError_Test = 0;
unsigned long nCountExceedTime1_Test = 0;
unsigned long nCountExceedTime2_Test = 0;
double max_time = 0;
double avg_time = 0;
double sum_time = 0;
bool isInit = false;
void segmentation_with_cluster_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //std::cout << "---start--------segmentation_with_cluster_cb---------------------" << std::endl;
  ros::Time t_base = input->header.stamp; //timestamp t_base
  // ros::Time tt = ros::Time::now();
  // std::cout << "lag time is " << (tt - input->header.stamp).toSec() <<std::endl;
  nCount_Test++;
  static ros::Time t_old = ros::Time::now();
  ros::Time t0 = ros::Time::now();//start filte z(0~5)
  ros::Time t01;//start z:0~5 seg
  ros::Time t02;//start z:-0.1~0.2 seg
  ros::Time t03;//end z:-0.1~0.2 seg

  // step2: Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<PointT> rawCloud;
  pcl::fromROSMsg (*input, rawCloud);
  std::cout<<"size of pointcloud is " << rawCloud.size() << std::endl;

  t01 = ros::Time::now(); //start z:0~5 seg
  pcl::PointCloud<PointT> cloudForPlaneSeg , cloudOthers;
  pcl::ExtractIndices<PointT> extract;
  // segment out points far from camera in camera frame (0-5)
  pcl::IndicesPtr indicesPT (new std::vector <int>);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (rawCloud.makeShared());
  pass.setFilterFieldName ("z");//x:right y:down z:forward
  pass.setFilterLimits (0, 5);//(0,5)
  pass.filter (*indicesPT);
  extract.setInputCloud (rawCloud.makeShared ());
  extract.setIndices (indicesPT);
  extract.setNegative (false);
  extract.filter (cloudForPlaneSeg);  // extract the inliers,z:0-5
  extract.setNegative (true);
  extract.filter (cloudOthers); // extract the outliers,z<0 & z>5

  // segment out points high enough from ground in last local ground frame
  if(hasFirstGroundAxis)
  {//filt cloudForPlaneSeg (-0.1~0.2), first will not do, only do when hasFirstGroundAxis
    pcl::PointCloud<PointT> tmp;//z:-0.1~0.2
    pcl::PointCloud<PointT> tmp1;//z<-0.1 & z>0.2
    pcl::transformPointCloud(cloudForPlaneSeg , tmp , mTransform_inv);
    t02 = ros::Time::now(); //start z:-0.1~0.2 seg
    //after transfer x,y,z's means are changed
    pass.setInputCloud (tmp.makeShared());
    pass.setFilterFieldName ("z");//x:backward y:right z:up
    pass.setFilterLimits (-1, 5);//-0.1~0.2
    pass.filter (*indicesPT);
    extract.setInputCloud (cloudForPlaneSeg.makeShared ());// extract points between [-0.1 0.2] from cloudForPlaneSeg
    extract.setIndices (indicesPT);
    extract.setNegative (false);
    extract.filter (tmp);// extract the inliers
    extract.setNegative (true);
    extract.filter (tmp1);// extract the inliers? (outlier)

    cloudForPlaneSeg = tmp;
    cloudOthers += tmp1;
    t03 = ros::Time::now(); //end z:-0.1~0.2 seg
  }
  std::cout<<"size of pointcloud after pass throungh is " << cloudForPlaneSeg.size() << std::endl;
  // downsample
  pcl::VoxelGrid<PointT> sor; //创建滤波对象
  sor.setDownsampleAllData(false); 
  sor.setInputCloud (cloudForPlaneSeg.makeShared());
  sor.setLeafSize (0.02f, 0.02f, 0.02f); //设置滤波时创建的体素大小为2cm立方体
  sor.filter (cloudForPlaneSeg);
  sor.setInputCloud (cloudOthers.makeShared());
  sor.filter (cloudOthers);

  cloudForPlaneSeg.header = rawCloud.header;
  cloudOthers.header = rawCloud.header;
  //All cloudpoint
  pcl::PointCloud<PointT> cloudAllDownSampled = cloudForPlaneSeg + cloudOthers;
  cloudAllDownSampled.header = rawCloud.header;

  //  std::cout<<"size of pointcloud after pass throungh is " << cloudAllDownSampled.size() << std::endl;
  //  std::cout<<"size of pointcloud for plane is " << cloudForPlaneSeg.size() << std::endl;
  //  std::cout<<"size of pointcloud others is " << cloudOthers.size() << std::endl;

  //cloudpoints between z:0~5 or z:-0.1~0.2
  sensor_msgs::PointCloud2 could2ROS;
  pcl::toROSMsg(cloudForPlaneSeg , could2ROS);
  cloudForPlane_pub.publish(could2ROS);

  // std::cout<<"size of pointcloud after pass throungh is " << rawCloud.size() << std::endl;
  std::cout<<"size of pointcloud after down sample is " << cloudForPlaneSeg.size() << std::endl;
  // outfile<<"size of pointcloud:" << rawCloud.size() << std::endl;
  // outfile<<"size of pointcloud after down sample:" << cloud.size() << std::endl;

  ros::Time t1 = ros::Time::now();  //start region_growing_seg
  std::vector <pcl::PointIndices> clusters;
  region_growing_segmentation(cloudForPlaneSeg.makeShared() , &clusters);
  ros::Time t2; //end region_growing_seg
  bool hasDetectPlane = false;
  if(!clusters.empty())
  {//get heightOfPlane & coefficientsOfPlane, set hasDetectPlane = true
    t2 = ros::Time::now();//region_growing_seg end & start cluster
    // set segmentation parameters and extraction parameters
    // 创建分割对象
    pcl::SACSegmentation<PointT> seg;
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients (true);
    // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
    
    //设置模型类型 //https://blog.csdn.net/sdau20104555/article/details/40649101
    seg.setModelType (pcl::SACMODEL_PLANE);
    // if want extract ground
    // seg.setModelType (pcl:: SACMODEL_PERPENDICULAR_PLANE);//set plane perpendicular to your wanted axis
    // seg.setAxis(groundAxis);
    // seg.setEpsAngle((5*3.14)/180);
    seg.setMethodType (pcl::SAC_RANSAC);//设置随机采样一致性方法类型
    seg.setMaxIterations (max_it);//max_it=200,表示点到估计模型的距离最大值
    seg.setDistanceThreshold (0.05);//设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
    // seg.setDistanceThreshold (0.2);


    // segment out the ground plane
    int nr_points = (int) cloudForPlaneSeg.size () + cloudOthers.size();
    float heightOfPlane = 10;
    int i = 0;
    for (std::vector<pcl::PointIndices>::const_iterator iter = clusters.begin(); iter != clusters.end(); iter++)
    {//ergodic clusters
      i++;
      pcl::PointCloud<PointT> cloudInCluster;
      pcl::PointIndices::Ptr inliersPlane(new pcl::PointIndices(*iter));//pointclouds in 1 cluster
      extract.setInputCloud (cloudForPlaneSeg.makeShared ());
      extract.setIndices (inliersPlane);
      extract.setNegative (false);
      extract.filter (cloudInCluster);  // extract the inliers of a cluster
      std::cout<<"size of cluster is " << cloudInCluster.points.size() << std::endl;
      // segment out a plane
      pcl::ModelCoefficients coefficients;
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      seg.setInputCloud (cloudInCluster.makeShared ());
      //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
      seg.segment (*inliers, coefficients);

      if (inliers->indices.size () == 0)
      {//no inliers
        ROS_INFO("error: no inliers");
        continue;
      }

      Eigen::Vector3f normal(coefficients.values[0] , coefficients.values[1] , coefficients.values[2]);//normal of plane
      float costheta = groundAxis.dot(normal);//angle of groundAxis and plan normal
      if(costheta > 0)
      {//costheta>0:0~90,costheta=0:90,costheta<0:90~180
        costheta = -costheta;
        coefficients.values[0] = -coefficients.values[0];
        coefficients.values[1] = -coefficients.values[1];
        coefficients.values[2] = -coefficients.values[2];
        coefficients.values[3] = -coefficients.values[3];
      }
      float absHeight = fabs(coefficients.values[3]);//ax+by+cz+d=0 (a,b,c）为平面法矢，abs(d)为原点至平面的距离，这四个参数可以确定一个平面
      if(costheta < -0.9848 && (inliers->indices.size() > nr_points * 0.1) && (inliers->indices.size() > inliersPlane->indices.size() * 0.5))  //check if it is the ground plane,cos(10) = 0.9848
      {//is plane && big enough && points more then half of this cluster //get heightOfPlane & coefficientsOfPlane
        ROS_INFO("plane %d is (%f %f %f %f)" , i , coefficients.values[0],coefficients.values[1],coefficients.values[2],coefficients.values[3]);
        // outfile <<"plane " << i << " is " << coefficients.values[0] <<"\t"<< coefficients.values[1] <<"\t"<< coefficients.values[2] <<"\t"<< coefficients.values[3]<< endl;
        if(!hasDetectPlane) // first candidate plane
        {//first time detected plane
          heightOfPlane = absHeight;
          coefficientsOfPlane = coefficients;
          hasDetectPlane = true;
        }
        else if(absHeight > heightOfPlane) // more accurate plane
        {//select the hightest palne
          heightOfPlane = absHeight;
          coefficientsOfPlane = coefficients;
        }
      }
    }
  }

  ros::Time t3 = ros::Time::now();//start project_to_laser

  if(!hasDetectPlane)
  {//have no plane
    std::cout << "---no plane---" << std::endl;
    nCountNoGround++;
    if(nCountNoGround < MAX_COUNT_NO_GROUND && hasFirstGroundAxis)
    {//MAX_COUNT_NO_GROUND=20, hasFirstGroundAxis init is false,
      // ROS_INFO("WARNING %d : detect no appropriate plane , use last ground plane!" , nCountNoGround);
      // ROS_INFO("ground plane is (%f %f %f %f)" , coefficientsOfPlane.values[0],coefficientsOfPlane.values[1],coefficientsOfPlane.values[2],coefficientsOfPlane.values[3]);
      // project_to_laserscan(rawCloud.makeShared() ,boost::make_shared<pcl::ModelCoefficients>(coefficientsOfPlane) , t_base);
      project_to_laserscan(cloudForPlaneSeg.makeShared() ,boost::make_shared<pcl::ModelCoefficients>(coefficientsOfPlane) , t_base);
      test_pub.publish(*input);//pub original pointclouds
      if(isInit)//no initial and no plane
        nCountWarning_Test++;//warning test num
    }
    else
    {
      hasFirstGroundAxis = false;
     // ROS_INFO("ERROR : detect no appropriate plane for a long time!");
      if(isInit)
        nCountError_Test++;//error test num
    }
  }
  else
  {//already have plane //set hasFirstGroundAxis=true
    std::cout << "---has plane---" << std::endl;
    isInit = true;
    hasFirstGroundAxis = true;
    nCountNoGround = 0;
   // ROS_INFO("ground plane is (%f %f %f %f)" , coefficientsOfPlane.values[0],coefficientsOfPlane.values[1],coefficientsOfPlane.values[2],coefficientsOfPlane.values[3]);
    // project_to_laserscan(rawCloud.makeShared() ,boost::make_shared<pcl::ModelCoefficients>(coefficientsOfPlane) , t_base);
    project_to_laserscan(cloudForPlaneSeg.makeShared() ,boost::make_shared<pcl::ModelCoefficients>(coefficientsOfPlane) , t_base);
    test_pub.publish(*input);//pub original pointclouds
    if(isInit)
      nCountCorrect_Test++;//correct test num
  }
  ros::Time t4 = ros::Time::now();//end project_to_laser & end whole system




  // for show
  // Publish the plane and pointcloud marked by the plane
  // extract.setInputCloud (cloud.makeShared ());
  // pcl::PointIndices::Ptr inliersInPlane(new pcl::PointIndices(*(clusters.begin()+index)));
  // extract.setIndices (inliersInPlane);
  // extract.setNegative (false);
  // extract.filter (cloudOfPlane);               // extract the inliers

  // ros::Time t4 = ros::Time::now();
  // int32_t rgb = (static_cast<uint32_t>(255) << 16 | static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(0));
  // for (int i = 0; i < cloudOfPlane.points.size(); ++i)                          // mark the plane with red
  // {
  //   cloudOfPlane.points.at(i).rgb = rgb;
  // }

  // ros::Time t5 = ros::Time::now();

  // sensor_msgs::PointCloud2 could2ROS;
  // cloudOfPlane.header = cloud.header;
  // pcl::toROSMsg(cloudOfPlane , could2ROS);
  // pub.publish (could2ROS);

  // std::cout<<"transfrom computation time is " << (t02-t1).toSec()<<std::endl;
  // std::cout<<"pass through computation time is " << (t03-t02).toSec()<<std::endl;

  // std::cout<<"down sample computation time is " << (t1-t0).toSec()<<std::endl;
  // std::cout<<"cluster computation time is " << (t2-t1).toSec()<<std::endl;
  // std::cout<<"segment computation time is " << (t3-t2).toSec()<<std::endl;
  // std::cout<<"project computation time is " << (t4-t3).toSec()<<std::endl;
  // std::cout<<"overall computation time is " << (t4-t0).toSec()<<std::endl;
  // std::cout<<"peroid time is " << (t0-t_old).toSec()<<std::endl;

  // outfile <<"overall computation time:" << (t4-t0).toSec() <<std::endl;
  // outfile <<"peroid time:" << (t0-t_old).toSec()<<std::endl;

  double dt = (t4-t0).toSec();//function running time
  sum_time += dt;//total time
  if ( dt > max_time )
    max_time = dt;
  if(dt > 0.1)
  {//excess time 0.1
    nCountExceedTime1_Test++;
    outfile<< "rawCloud size: " << rawCloud.size() << "," 
          << "cloudForPlaneSeg size:" << cloudForPlaneSeg.size() <<"," 
          << "(t1-t0):" << (t1-t0).toSec() << " , " 
          << "(t2-t1):" << (t2-t1).toSec() << " , " 
          << "(t3-t2):" << (t3-t2).toSec() << " , "  
          << "(t4-t3):" << (t4-t3).toSec() << " , "  
          << "(t4-t0):" << (t4-t0).toSec() << std::endl;
  }
  else if(dt > 0.08)
  {//excess time 0.08
    nCountExceedTime2_Test++;
    outfile<< "rawCloud size: " << rawCloud.size() << "," 
          << "cloudForPlaneSeg size:" << cloudForPlaneSeg.size() <<"," 
          << "(t1-t0):" << (t1-t0).toSec() << " , " 
          << "(t2-t1):" << (t2-t1).toSec() << " , " 
          << "(t3-t2):" << (t3-t2).toSec() << " , "  
          << "(t4-t3):" << (t4-t3).toSec() << " , "  
          << "(t4-t0):" << (t4-t0).toSec() << std::endl;
  }

  std::cout << "maximum of computation time is " << max_time << std::endl;
  std::cout << "average of computation time is " << sum_time / nCount_Test << std::endl;
  std::cout << "loops of test is " << nCount_Test << std::endl;
  std::cout << "loops success is " << nCountCorrect_Test << std::endl;
  std::cout << "loops warning is " << nCountWarning_Test << std::endl;
  std::cout << "loops error is " << nCountError_Test << std::endl;
  std::cout << "loops exceed time1 limit is " << nCountExceedTime1_Test << std::endl;
  std::cout << "loops exceed time2 limit is " << nCountExceedTime2_Test << std::endl;
  //std::cout << "---end------------segmentation_with_cluster_cb-------------------" << std::endl;
  t_old = t0;
}