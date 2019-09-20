#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>
void AHRSupdate(const sensor_msgs::ImuConstPtr& accel , const sensor_msgs::ImuConstPtr& gyro);


Eigen::Quaterniond q(1, 0, 0, 0);           //w,x,y,z
ros::Publisher pub;
int main(int argc , char** argv)
{
  ros::init(argc , argv , "imu_estimator_node");
  ros::NodeHandle nh;

  pub = nh.advertise<geometry_msgs::Vector3Stamped>("/ground_in_imuframe" , 10); // in camera_accel_optical_frame,   xyz->right,below,front

  message_filters::Subscriber<sensor_msgs::Imu> accel_sub(nh, "/camera/accel/sample", 1);
  message_filters::Subscriber<sensor_msgs::Imu> gyro_sub(nh, "/camera/gyro/sample", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), accel_sub, gyro_sub);
  sync.registerCallback(boost::bind(&AHRSupdate, _1, _2));


  //ros::Subscriber sub = nh.subscribe("/camera/gyro/sample" , 10 , AHRSupdate);        // /camera/gyro/sample    /camera/accel/sample(sensor_msgs/Imu)



  ros::spin();
  return 0;
}

/*
 *AHRS
*/
// AHRS.c
// Quaternion implementation of the 'DCM filter' [Mayhony et al]. Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminatesthe need for a reference
// direction of flux (bx bz) to be predefined and limits the effect ofmagnetic distortions to yaw
// axis only.
// User must define 'halfT' as the (sample period / 2), and the filtergains 'Kp' and 'Ki'.
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elementsrepresenting the estimated
// orientation.  See my report foran overview of the use of quaternions in this application.
// User must call 'AHRSupdate()' every sample period and parsecalibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz')data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevantas the vector is normalised.


/* Private define------------------------------------------------------------*/
#define Kp 2.0f                       // proportional gain governs rate of convergence toaccelerometer/magnetometer
#define Ki 0.005f          // integral gain governs rate of convergenceof gyroscope biases
#define halfT 0.0025f      // half the sample period
#define ACCEL_1G 1000    //theacceleration of gravity is: 1000 mg
/* Private variables---------------------------------------------------------*/
//static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // quaternion elements representing theestimated orientation
static float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
/* Private macro-------------------------------------------------------------*/
/* Private typedef-----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/******************************************************************************
 *Function Name  : AHRSupdate
 *Description    : None
 *Input          : None
 *Output         : None
 *Return         : None
*******************************************************************************/
void AHRSupdate(const sensor_msgs::ImuConstPtr& accel , const sensor_msgs::ImuConstPtr& gyro) {
        float q1 = q.x();
        float q2 = q.y();
        float q3 = q.z();
        float q0 = q.w();
        // auxiliary variables to reduce number of repeated operations
        float q0q0 = q0*q0;
        float q0q1 = q0*q1;
        float q0q2 = q0*q2;
        float q0q3 = q0*q3;
        float q1q1 = q1*q1;
        float q1q2 = q1*q2;
        float q1q3 = q1*q3;
        float q2q2 = q2*q2;
        float q2q3 = q2*q3;
        float q3q3 = q3*q3;

        float  ax = accel->linear_acceleration.x;
        float  ay = accel->linear_acceleration.y;
        float  az = accel->linear_acceleration.z;
        float  gx = gyro->angular_velocity.x;
        float  gy = gyro->angular_velocity.y;
        float  gz = gyro->angular_velocity.z;

        // normalise the measurements
        float norm = sqrt(ax*ax + ay*ay + az*az);
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;

        // estimated direction of gravity and magnetic field (v and w)
        float vx = 2*(q1q3 - q0q2);
        float vy = 2*(q0q1 + q2q3);
        float vz = q0q0 - q1q1 - q2q2 + q3q3;

        // error is sum of cross product between reference direction of fields and direction measured by sensors
        float ex = (ay*vz - az*vy);
        float ey = (az*vx - ax*vz);
        float ez = (ax*vy - ay*vx);

        static ros::Time t_old = gyro->header.stamp;
        ros::Time t_now = gyro->header.stamp;
        double dt = (t_now - t_old).toSec();
        t_old = t_now;

        // integral error scaled integral gain
        exInt = exInt + ex*Ki* dt;
        eyInt = eyInt + ey*Ki* dt;
        ezInt = ezInt + ez*Ki* dt;
        // adjusted gyroscope measurements
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;

        // integrate quaternion rate and normalize
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*dt*0.5;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*dt*0.5;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*dt*0.5;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*dt*0.5;

        // normalise quaternion
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
        q = Eigen::Quaterniond(q0 , q1 , q2 , q3);

        Eigen::Matrix3d trans = q.matrix();
        Eigen::Vector3d ground_earth_frame(0,0,-1);
        Eigen::Vector3d ground_imu_frame = trans.inverse()*ground_earth_frame;
        geometry_msgs::Vector3Stamped groundAxis;
        groundAxis.vector.x = ground_imu_frame[0];
        groundAxis.vector.y = ground_imu_frame[1];
        groundAxis.vector.z = ground_imu_frame[2];
        groundAxis.header.stamp = gyro->header.stamp;
        groundAxis.header.frame_id = gyro->header.frame_id;

        std::cout<< "ground_imu_frame" << ground_imu_frame << std::endl;
        pub.publish(groundAxis);

//        geometry_msgs::Pose pose;
//        pose.orientation.x = q.x();
//        pose.orientation.y = q.y();
//        pose.orientation.z = q.z();
//        pose.orientation.w = q.w();
//        pub.publish(pose);
}
