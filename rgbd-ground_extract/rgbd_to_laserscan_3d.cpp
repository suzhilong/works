/*
camera: groundHeight: 0.862486,  rotateAngle: 21.687197  0.378195
   0.999236  -0.0383987 -0.00731542           0
  0.0383987    0.929215    0.367539           0
-0.00731542   -0.367539    0.929979           0
          0           0           0           1

*/

/*
camera2: groundHeight: 0.837087,  rotateAngle: 54.409315
   0.999763  -0.0193672 -0.00995254           0
  0.0193672    0.581991    0.812965           0
-0.00995254   -0.812965    0.582228           0
          0           0           0           1

*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Twist.h"
//#include <list>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <limits>
#include <float.h>
//#include <mutex>
using namespace std;

#define ULTRASOUND_NUM 60
#define	HORIZONTAL_VIEW_DEGREE	60
#define CX 311.9583
#define CY 237.3555
#define FX 519.2486
#define FY 519.0191

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
int m_count = 0;

//depth data subscriber
//void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

void depthCallback(const sensor_msgs::ImageConstPtr& msgD);

//Global parameter	unit: meter
double safe_distance=0.4, max_distance=6;
double ray[ULTRASOUND_NUM+1];
//mutex rayMutex;

//Eigen::Matrix4d rotMat;
//MAIN入口

ros::Publisher cloud_pub;
Eigen::Matrix4d transM_depth_optical;
Eigen::Matrix4d transM_;

double trueGroundHeight = -1;
double trueRotateAngle = -1;
bool getFirst =false;


bool isInfValue(double value){
	if(value>max_distance || value<safe_distance){
		return true;
	}else{
		return false;
	}
}

void adjustRayValue(double ray[]){
	//bool isNear[ULTRASOUND_NUM+1];
	double nearestDistance = max_distance+1;
	for(int i=0;i<=ULTRASOUND_NUM;i++){
		if(ray[i]>safe_distance && ray[i]<nearestDistance){
			nearestDistance = ray[i];
		}
		//isNear[i] = false;
	}

	for(int i=0;i<=ULTRASOUND_NUM;i++){
		if(fabs(ray[i]-nearestDistance)<0.2){
			ray[i] = nearestDistance;
		}else{
			ray[i] = max_distance+1;
		}
	}
}

void fillFalseInf(double ray[]){

	int leftIndex, rightIndex;
	int leftFence = 0;
	for(int j=1;j<=ULTRASOUND_NUM;j++){
		if(isInfValue(ray[j])){  //当前值INF
			
			int i = j-1;
			while( i>=leftFence && isInfValue(ray[i]) ){ //向当前值的左侧查找最近的有效值
				i--;
			}
			if(i>=leftFence){
				leftIndex = i;	//找到左侧最近有效值索引为ｉ
			}else{
				leftFence = j+1;	//左侧没找到有效值，向左查询最近有效值的最左处设置为当前值的下一个位置索引
				continue;			//继续向后遍历
			}

			int k = j+1;
			while( k<=ULTRASOUND_NUM && isInfValue(ray[k]) ){//向当前值的右侧查找最近的有效值
				k++;
			}
			if(k<=ULTRASOUND_NUM){
				rightIndex = k;		//右侧找到最近有效值的索引值为ｋ
			}else{
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

std::string rgbd_link_prefix_name;

int main(int argc, char **argv){
	//char buf[256];
	//char cmdbuf[512];
	ros::init(argc, argv, "rgbd_scan_3d");
	ros::NodeHandle nh;

    	ros::param::get("~rgbd_link_prefix_name", rgbd_link_prefix_name);
	
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(("/"+rgbd_link_prefix_name+"_cloud").c_str(), 100);

	ros::Publisher rgbd_pub = nh.advertise<sensor_msgs::LaserScan>(("/nav/"+rgbd_link_prefix_name+"_laserscan_3d").c_str(), 1000);

	tf::TransformListener listener;
        tf::StampedTransform transformT_depth_optical;

        try{
          
            listener.waitForTransform(("/"+rgbd_link_prefix_name+"_depth_frame").c_str(), ("/"+rgbd_link_prefix_name+"_depth_optical_frame").c_str(), ros::Time(0), ros::Duration(2.0));
            listener.lookupTransform(("/"+rgbd_link_prefix_name+"_depth_frame").c_str(), ("/"+rgbd_link_prefix_name+"_depth_optical_frame").c_str(),  ros::Time(0), transformT_depth_optical);
       
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
	sensor_msgs::LaserScan scan_msg;

	scan_msg.angle_max = (HORIZONTAL_VIEW_DEGREE/180.0 * M_PI)/2 - (HORIZONTAL_VIEW_DEGREE/180.0 * M_PI) /ULTRASOUND_NUM/2;   
	scan_msg.angle_min = -scan_msg.angle_max;
	scan_msg.angle_increment = (HORIZONTAL_VIEW_DEGREE/180.0 * M_PI) /ULTRASOUND_NUM;
	scan_msg.scan_time = 1.0/publishHz;
	scan_msg.time_increment = scan_msg.scan_time / (double)(ULTRASOUND_NUM-1);
	scan_msg.range_min = safe_distance;
	scan_msg.range_max = max_distance;

	ros::Subscriber base_rgbd_sub = nh.subscribe(("/"+rgbd_link_prefix_name+"/depth/image_raw").c_str(), 1, depthCallback);
	//ros::Subscriber base_rgbd_sub = nh.subscribe(("/"+rgbd_link_prefix_name+"/depth_registered/points").c_str(), 1, pointCloudCallback);
	ROS_INFO("begin getting camera depth image");
	ros::Rate loop_rate(publishHz);
	//此坐标系固定没有roll和pitch角度的变化，否则障碍在costmap中难以清除。
	scan_msg.header.frame_id = "camera_link";
	while (ros::ok()) {
		scan_msg.ranges.clear();
    		scan_msg.header.stamp = ros::Time::now();
    		{
    			//unique_lock<mutex> lck( rayMutex );
			fillFalseInf(ray);
			if(rgbd_link_prefix_name=="camera_astra_1"){
				adjustRayValue(ray);
			}
			
    			for(int i=0;i<=ULTRASOUND_NUM;i++){
				double data  =  ray[i];
				//printf("data = %f\t", data);
				if(fabs(data)< safe_distance || data > max_distance) 
					data = std::numeric_limits<float>::infinity();
				scan_msg.ranges.push_back(data);
			}	
    		}
		
		rgbd_pub.publish(scan_msg);
        	ros::spinOnce(); // ROS handle incoming message
		loop_rate.sleep();
		//ros::Duration(3).sleep();
	}
	
	return 0;
}


void depthCallback(const sensor_msgs::ImageConstPtr& msgD){
	cv_bridge::CvImageConstPtr cv_ptrD;
	try
	{
		cv_ptrD = cv_bridge::toCvShare(msgD);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat depth = cv_ptrD->image;

	PointCloud::Ptr tmp( new PointCloud() );

	//std::cout<<"depth: rows: "<<depth.rows<<"cols: "<<depth.cols<<std::endl;

	// point cloud is null ptr
	bool isEncoding_16UC1 = false;
	if (msgD->encoding == sensor_msgs::image_encodings::TYPE_16UC1){
		depth.convertTo(depth,CV_32FC1);
		isEncoding_16UC1 = true;
	}
	
	for ( int m=0; m<depth.rows; m+=6 )
	{
		for ( int n=0; n<depth.cols; n+=6 )
		{
		    float d = isEncoding_16UC1? depth.ptr<float>(m)[n]/1000.0:depth.ptr<float>(m)[n];	
                    if (d < 0.01 || d>max_distance || isnan(d) || isinf(d))
		    //if (d < 0.01 || d>2 || isnan(d) || isinf(d))
			continue;
		    
		    PointT p;
		    p.z = d;
		    p.y = ( m - CY ) * p.z / FY;
		    p.x = ( n - CX ) * p.z / FX;
		    tmp->points.push_back(p);
		}
	}

	//std::cout<<"points size: "<<tmp->points.size()<<std::endl;

	tmp->is_dense = false;

	
	 pcl::StatisticalOutlierRemoval<PointT> sor;
	 sor.setInputCloud (tmp);
	 sor.setMeanK (50);
	 sor.setStddevMulThresh (1.0);
	 sor.filter (*tmp);

	
	 
	if(!getFirst){
		cout<<"outlier removed"<<endl;
		PointCloud cloud_filtered(*tmp);
		pcl::SACSegmentation<PointT> seg;  
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
		//coefficients的四个参数，前三个代表平面的法向量，最后一个代表光心到面的距离。
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
		PointCloud::Ptr cloud_plane (new PointCloud ());  

		seg.setOptimizeCoefficients (true);  
		seg.setModelType (pcl::SACMODEL_PLANE);  
		//seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);  
		seg.setMaxIterations (200);  
		seg.setDistanceThreshold (0.001);   
		//seg.setAxis(Eigen::Vector3f(0,1,0));
		//seg.setEpsAngle(0.25);    

		seg.setInputCloud (cloud_filtered.makeShared());  
		seg.segment (*inliers, *coefficients);
		if(inliers->indices.size()==0){
			ROS_ERROR("camera: there is no plane to segmented");
			return;
		}

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud_filtered.makeShared());
		extract.setIndices(inliers);

	       //the fourth coefficent represents the distance between the orginal point to the plane.
		trueGroundHeight = fabs(coefficients->values[3]);

		Eigen::Vector3d plane_normal, ground_normal, rotateAxis;
		plane_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
		plane_normal = plane_normal.normalized();
		if(plane_normal(1)>0){
		    plane_normal = -plane_normal;
		}
		ground_normal << 0, -1, 0;
		double rotateRad = acos(plane_normal.dot(ground_normal));
		trueRotateAngle = pcl::rad2deg(rotateRad);
		printf("camera: groundHeight: %f,  rotateAngle: %f\n", trueGroundHeight, trueRotateAngle);
		rotateAxis = plane_normal.cross(ground_normal);

		rotateAxis = rotateAxis.normalized();

		Eigen::AngleAxis<double> angleAxis(rotateRad, rotateAxis);
		Eigen::Matrix3d rotM  = angleAxis.toRotationMatrix();
		

		transM_<<rotM(0,0), rotM(0,1), rotM(0,2), 0,
		 	 rotM(1,0), rotM(1,1), rotM(1,2), 0,
		 	 rotM(2,0), rotM(2,1), rotM(2,2), 0,
		         	 0,         0,         0, 1;
		cout<<transM_<<endl;			

		transM_ = transM_depth_optical * transM_;	
		getFirst = true;
	}
	

	PointCloud::Ptr cloud(new PointCloud);

	pcl::VoxelGrid<PointT>  voxel;
	voxel.setLeafSize( 0.05, 0.05, 0.05);
	voxel.setInputCloud( tmp );
    	voxel.filter( *cloud );

     pcl::transformPointCloud( *cloud, *cloud, transM_);

	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*cloud, cloud_msg);
	cloud_msg.header.stamp = ros::Time::now();
	cloud_msg.header.frame_id = ("/"+rgbd_link_prefix_name+"_depth_frame").c_str();
	cloud_pub.publish(cloud_msg);
	
	//unique_lock<mutex> lck( rayMutex );
    	//memset(ray, 0, sizeof(double) * ULTRASOUND_NUM);
	for(int i=0;i<=ULTRASOUND_NUM;i++){
		ray[i] = 0;
	}

    	double angleIncrement = double(HORIZONTAL_VIEW_DEGREE)/ULTRASOUND_NUM;

    	for(size_t i=0;i< cloud->points.size();i++){
		PointT p = cloud->points[i];
		if(isnan(p.x) || isnan(p.y) || isnan(p.z))
			continue;
    		double distance = hypot(p.x, p.y);
		//if(distance >2)
			//printf("distance:%f\t",distance);
    		if(distance > max_distance || p.z < -trueGroundHeight + 0.01)
    			continue;
    		double theta = atan(p.y/p.x);
    		double angle = theta/M_PI *180.0;
    		//printf("z = %f, x = %f, y = %f, distance = %f, theta=%f, angle=%d\n", p.z, p.x, p.y, distance, theta, angle);
		
    		if(fabs(angle) > HORIZONTAL_VIEW_DEGREE/2 ){
    			//printf("outlier angle = %d\n", angle);
    			continue;
    		}
		
		int index = round( (angle + HORIZONTAL_VIEW_DEGREE/2)/angleIncrement );
		
    		if(ray[index] < 1e-2){
    			ray[index] = distance;
    		}
		
    		if(distance < ray[index]){
    			ray[index] = distance;	
    		}
    	}

//    	for(int i=0; i<ULTRASOUND_NUM+1; i++){
//    		printf("aay[%d] = %f\n", i, ray[i]);
//    	}
	
}


/*void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{	

	PointCloud::Ptr points( new PointCloud() );
	PointCloud::Ptr tmp( new PointCloud() );
    	pcl::fromROSMsg(*msg, *points);

    	tmp->is_dense = false;

	if(!getFirst){
		PointCloud cloud_filtered(*points);
		pcl::SACSegmentation<PointT> seg;  
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
		PointCloud::Ptr cloud_plane (new PointCloud ());  

		seg.setOptimizeCoefficients (true);  
		seg.setModelType (pcl::SACMODEL_PLANE);  
		//seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);  
		seg.setMaxIterations (200);  
		seg.setDistanceThreshold (0.001);   
		//seg.setAxis(Eigen::Vector3f(0,1,0));
		//seg.setEpsAngle(0.25);    

		seg.setInputCloud (cloud_filtered.makeShared());  
		seg.segment (*inliers, *coefficients);
		if(inliers->indices.size()==0){
			ROS_ERROR("camera: there is no plane to segmented");
			return;
		}

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud_filtered.makeShared());
		extract.setIndices(inliers);

	       //the fourth coefficent represents the distance between the orginal point to the plane.
		trueGroundHeight = fabs(coefficients->values[3]);

		Eigen::Vector3d plane_normal, ground_normal, rotateAxis;
		plane_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
		plane_normal = plane_normal.normalized();
		if(plane_normal(1)>0){
		    plane_normal = -plane_normal;
		}
		ground_normal << 0, -1, 0;
		double rotateRad = acos(plane_normal.dot(ground_normal));
		trueRotateAngle = pcl::rad2deg(rotateRad);
		printf("camera: groundHeight: %f,  rotateAngle: %f\n", trueGroundHeight, trueRotateAngle);
		rotateAxis = plane_normal.cross(ground_normal);

		rotateAxis = rotateAxis.normalized();

		Eigen::AngleAxis<double> angleAxis(rotateRad, rotateAxis);
		Eigen::Matrix3d rotM  = angleAxis.toRotationMatrix();

		transM_<<rotM(0,0), rotM(0,1), rotM(0,2), 0,
		 	rotM(1,0), rotM(1,1), rotM(1,2), 0,
		 	rotM(2,0), rotM(2,1), rotM(2,2), 0,
		         	0,         0,         0, 1;
		cout<<transM_<<endl;
		transM_<<   1,   0,   0,   0,
			    0,   1,   0,   0,
			    0,   0,   1,   0,
			    0,   0,   0,   1;
		trueGroundHeight = 0.5745;	
		transM_ = transM_depth_optical * transM_;	
		getFirst = true;
	}


	pcl::VoxelGrid<PointT>  voxel;
	voxel.setLeafSize( 0.1, 0.1, 0.1);
	voxel.setInputCloud( points );
        	voxel.filter( *tmp );

    	 PointCloud::Ptr cloud(new PointCloud);
    	//pcl::transformPointCloud( *tmp, *cloud, rotMat);
    	 pcl::transformPointCloud( *tmp, *cloud, transM_);


	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*cloud, cloud_msg);
	cloud_msg.header.stamp = ros::Time::now();
	cloud_msg.header.frame_id = "camera_link";
	cloud_pub.publish(cloud_msg);
	

    	//memset(ray, 0, sizeof(double) * ULTRASOUND_NUM);
	for(int i=0;i<=ULTRASOUND_NUM;i++){
		ray[i] = 0;
	}
	
	double angleIncrement = double(HORIZONTAL_VIEW_DEGREE)/ULTRASOUND_NUM;

    	for(size_t i=0;i< cloud->points.size();i++){
		PointT p = cloud->points[i];
		if(isnan(p.x) || isnan(p.y) || isnan(p.z))
			continue;
    		double distance = hypot(p.x, p.y);
    		if(distance > max_distance || p.z < -trueGroundHeight + 0.15)
    			continue;
    		double theta = atan(p.y/p.x);
    		int angle = theta/M_PI *180;
    		//printf("z = %f, x = %f, y = %f, distance = %f, theta=%f, angle=%d\n", p.z, p.x, p.y, distance, theta, angle);
		
    		if(abs(angle) > HORIZONTAL_VIEW_DEGREE/2 ){
    			//printf("outlier angle = %d\n", angle);
    			continue;
    		}
		
		int index = (angle + HORIZONTAL_VIEW_DEGREE/2)/angleIncrement;
		
    		if(ray[index] < 1e-2){
    			ray[index] = distance;
    		}
		
    		if(distance < ray[index]){
    			ray[index] = distance;	
    		}
    	}

//    	for(int i=0; i<ULTRASOUND_NUM+1; i++){
//    		printf("ray[%d] = %f\n", i, ray[i]);
//    	}

}*/
