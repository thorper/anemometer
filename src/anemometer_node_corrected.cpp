#include <ros/ros.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
//#include <anemometer/ImuConverged.h>
#include <anemometer/GPLocalConverged.h>
//#include <geometry_msgs/Twist.h> //Message Type out : Velocity
#include <ctime> //needed for time() in random number generator
#include <unistd.h>
//#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
//#include <fcntl.h>  /* File Control Definitions          */
//#include <termios.h>/* POSIX Terminal Control Definitions*/
//#include <unistd.h> /* UNIX Standard Definitions         */
//#include <errno.h>  /* ERROR Number Definitions          */
#include <sys/ioctl.h>
#include <cmath>

using namespace std;

//--------------Class------------------------------
class windGPLData
{
  ros::Subscriber sb_GPL_merged;
  ros::Publisher pb_merged_corrected;
  ros::NodeHandle nh_corrected;

  double R[3][3];
  double q[3];
 
  // class callback function
  // void windGPLCorrector(const sensor_msgs::Imu msg);
  void windGPLCorrector(const anemometer::GPLocalConverged msg);

  public: 
  
    windGPLData();

};
//----------------Constructor----------------------------
windGPLData::windGPLData()
{

  // sb_wind_imu = nh_corrected.subscribe("/mavros/imu/data",10, &windGPLData::windGPLCorrector,this);
  sb_GPL_merged = nh_corrected.subscribe("/mavros/wind_GPL",10, &windGPLData::windGPLCorrector,this);


  pb_merged_corrected = nh_corrected.advertise<anemometer::GPLocalConverged>("/mavros/wind_GPL_corrected", 1);
}


//------------- CALLBACK function to receive item_order data---------------

void windGPLData::windGPLCorrector(const anemometer::GPLocalConverged msg){

ROS_INFO("Hey Im in the Corrector Callback!------------------START---------->");  
  // double time1 = ros::Time::now().toSec();

// ros::Time imu_time = msg.stamp;
// double time_imu = imu_time.toSec();
// ROS_INFO("IMU time stamp is %f seconds",time_imu);
//  ROS_INFO("Difference between IMU timestamp and windGPLCorrector Callback start time is %f seconds ",time1-time_imu);
  
anemometer::GPLocalConverged corrected;

//Length of anemometer moment arm (inches)
// distance between CoM of Pixhawk to
// centroid of anemometer's measurement volume
double l_arm = 33.625 / 39.37;  

double U_d = msg.wind_U;
double V_d = msg.wind_V;
double W_d = msg.wind_W;

q[0] = msg.w_quat;
q[1] = msg.x_quat;
q[2] = msg.y_quat;
q[3] = msg.z_quat;

//  INDUCED VELOCITY CALCULATIONS
//R[row][column]
R[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
R[0][1] = 2*(q[1]*q[2] - q[0]*q[3]);
R[0][2] = 2*(q[1]*q[3] + q[0]*q[2]);
R[1][0] = 2*(q[1]*q[2] + q[0]*q[3]);
R[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
R[1][2] = 2*(q[2]*q[3] - q[0]*q[1]);
R[2][0] = 2*(q[1]*q[3] - q[0]*q[2]);
R[2][1] = 2*(q[2]*q[3] + q[0]*q[1]);
R[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

//multiplication by "Anemometer-to-Pixhawk" R Matrix


//Please, Accept the Mystery

//multiplication by Pixhawk-to-Earth R Matrix
double U_corr = U_d * R[0][0] + V_d * R[0][1] + W_d * R[0][2];
double V_corr = U_d * R[1][0] + V_d * R[1][1] + W_d * R[1][2];
double W_corr = U_d * R[2][0] + V_d * R[2][1] + W_d * R[2][2];


  //Publish out Converged Message Packet
  // With Induced Velocity Correction

corrected.seq = msg.seq;
corrected.stamp = msg.stamp;
corrected.frame_id = msg.frame_id;

corrected.x_position = msg.x_position;
corrected.y_position = msg.y_position;  
corrected.z_position = msg.z_position;

corrected.x_quat = msg.x_quat;
corrected.y_quat = msg.y_quat;
corrected.z_quat = msg.z_quat;
corrected.w_quat = msg.w_quat;

corrected.wind_U = U_corr;  //U_d
corrected.wind_V = V_corr;  //V_d
corrected.wind_W = W_corr;  //W_d
corrected.temp = msg.temp;

pb_merged_corrected.publish(corrected);

  // float CbToAne =  t_to_anemo - time1;
//  ROS_INFO("Time between CB starting Anemometer Trigger Sending is %f seconds",CbToAne);

  // double time2 = ros::Time::now().toSec();
//  ROS_INFO("Delta time for Entire windGPLCorrector callback is %f seconds",time2-time1);
//  ROS_INFO("Entire windGPLCorrector callback duration without anemometer is is %f seconds",(time2-time1)-(t_from_anemo-t_to_anemo));
//  ROS_INFO("Offset between IMU data timestamp and callback completion is %f seconds",time2-time_imu);
//  ROS_INFO("Offset between IMU data timestamp and when wind was measured is %f seconds",((t_to_anemo-time_imu)+0.01));


ROS_INFO("Call Back done----------------------------END------->\n");  

}

// main program---------------------------------------------------------------------------------
int main(int argc, char** argv)
{

  // initializing ros
  ROS_INFO(" initializing anemometer corrector node");
  ros::init(argc, argv, "anemometer_corrector"); //initiating node with unique node name
  
  windGPLData convCorr;
  ROS_INFO(" instantiated object");


  while(ros::ok)
  {
    ROS_INFO(" before ros spin()");
    ros::spin();
    ROS_INFO(" after ros spin()");

  }


}
