#include <iostream>
#include <stdexcept>

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "monarc_tf/FlyAction.h"
#include "monarc_tf/FlyGoal.h"
#include "monarc_uart_driver/FlightControl.h"

// pointCloudCallback processes a single PointCloud2 message. We currently have
// this directly linked to the creation and broadcast of a transform. As we add
// more data sources through other subscriptions, this will get more complicated.
// Alternatives to the current approach are  are to:
// - Buffer the latest value for each subscription and process at a fixed rate
// - Buffer all but one subscription, and "drive" the broadcast from one (Chris Likes this option)
// - Use a Time Synchronizer (http://wiki.ros.org/message_filters#Time_Synchronizer)
//

using namespace std;

//--------------------------- ROS Published Topics ---------------------------//
ros::Publisher simpleDist;
ros::Publisher callBackCounter;
//--------------------------- ROS Published Topics ---------------------------//


//-------------------- Point Cloud Localization Variables --------------------//
double avgDistance = 0.0;
double pastDistance = 0.0;
double cloudEdges[6];
//-------------------- Point Cloud Localization Variables --------------------//


//----------------- IMU Deadreckoning Calculation Variables ------------------//
double accelXData[1000];
double accelYData[1000];
double accelZData[1000];

double latestXDist;
double latestYDist;
double latestZDist;

int readingTimes[1000];
int accelDataCounter = 0;
//----------------- IMU Deadreckoning Calculation Variables ------------------//


int counter = 1;


//-------------------- Point Cloud Localization Functions --------------------//
void getMinMax(const sensor_msgs::PointCloud2& cloud)
//This function updates the global variable cloudEdges[], which is then used to
//to calculate the
{
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    double Xmax = 0.0;
    double Xmin = 0.0;
    double Ymax = 0.0;
    double Ymin = 0.0;
    double Zmax = 0.0;
    double Zmin = 0.0;

    int count = 0;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        if (count == 0){
            Xmax = *iter_x;
            Xmin = *iter_x;
            Ymax = *iter_y;
            Ymin = *iter_y;
            Zmax = *iter_z;
            Zmin = *iter_z;
        }
        if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
        {
           // cout << "X coord = " << (*iter_x) << "; Y coord = " <<  (*iter_y) << "; Z coord = " << (*iter_z) << ";\n";
            count ++;
            if (Xmax < *iter_x)
            {
                Xmax = *iter_x;
            }
            if (Xmin > *iter_x)
            {
                Xmin = *iter_x;
            }
            if (Ymax < *iter_y)
            {
                Ymax = *iter_y;
            }
            if (Ymin > *iter_y)
            {
                Ymin = *iter_y;
            }
            if (Zmax < *iter_z)
            {
                Zmax = *iter_z;
            }
            if (Zmin > *iter_z)
            {
                Zmin = *iter_z;
            }
        }
    }
    cloudEdges[0] = Xmax;
    cloudEdges[1] = Xmin;
    cloudEdges[2] = Ymax;
    cloudEdges[3] = Ymin;
    cloudEdges[4] = Zmax;
    cloudEdges[5] = Zmin;
}

double getMiddleAverage(const sensor_msgs::PointCloud2& cloud, double totalW, double totalH, double centerW, double centerH, double percentSize)
//using percentSize of the field of view at the center
//in order to calculate the distance change
//from last scan to now. This can be more efficient
//computationally once we figure out the best number of
//points/size of frame to run this function over.
//Assumes the vector of points is streaming in from left
//to right and then from top to bottom, so it will
//have the entire first row first in order.
{
	int right = centerW+(totalW*percentSize/2);
	int left = centerW-(totalW*percentSize/2);
	int top = centerH+(totalH*percentSize/2);
    int bottom = centerH-(totalH*percentSize/2);

	double count = 0.0;
	double totalDepth = 0.0;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Check if the point is invalid
    if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
    {
      if (*iter_y <= right && *iter_y >= left)
      {
        if (*iter_z <= top && *iter_z >= bottom)
        {
          // cout << "X coord = " << (*iter_x) << "; Y coord = " <<  (*iter_y) << "; Z coord = " << (*iter_z) << ";\n";
          count += 1.0;
          totalDepth += (*iter_x);
        }
      }
    }
  }
	if (count < 1.0)
	{
		return 0.0;
	}
	return totalDepth/count;
}


void pointCloudCallback(const sensor_msgs::PointCloud2& msg)
// pointCloudCallback processes a single PointCloud2 message. We currently have
// this directly linked to the creation and broadcast of a transform. As we add
// more data sources through other subscriptions, this will get more complicated.
// Alternatives to the current approach are  are to:
// - Buffer the latest value for each subscription and process at a fixed rate
// - Buffer all but one subscription, and "drive" the broadcast from one (Chris Likes this option)
// - Use a Time Synchronizer (http://wiki.ros.org/message_filters#Time_Synchronizer)
{
  static tf2_ros::TransformBroadcaster br;
  getMinMax(msg);
  double width = cloudEdges[2] - cloudEdges[3];
  double hieght = cloudEdges[4] - cloudEdges[5];
  double centerWidth = cloudEdges[3] + (width/2.0);
  double centerHieght = cloudEdges[5] + (hieght/2.0);
  avgDistance = getMiddleAverage(msg, width, hieght, centerWidth, centerHieght, 0.8);
  cout << "avgDistance = " << avgDistance << "\n";
  cout << "x axis change = " << avgDistance - pastDistance << "\n";

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = msg.header.stamp;
  transformStamped.header.frame_id = "map";

  transformStamped.child_frame_id = msg.header.frame_id;
  transformStamped.transform.translation.x = avgDistance + pastDistance;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  pastDistance = avgDistance;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();


  std_msgs::Float32 nDist;
  nDist.data = avgDistance;
  simpleDist.publish(nDist); // only used for plotting

  std_msgs::Int32 callCount;
  callCount.data = counter;
  callBackCounter.publish(callCount);
  br.sendTransform(transformStamped);
  counter++;
}
//-------------------- Point Cloud Localization Functions --------------------//


//----------------------- IMU Deadreckoning Functions ------------------------//
void updateIMUCallback(const std_msgs::Int32 IMU)
{
    cout << "IMU Data Structure:" << IMU.data << "\n";
    readingTimes[accelDataCounter] = clock();
}

void convertAccelToDist()
{
    latestXDist = 0.0;
    latestYDist = 0.0;
    latestZDist = 0.0;
    //function uses trapezoidal rule
    for ( int i = 0; i < (accelDataCounter-1); i++ )
    {
        latestXDist = accelXData[i]+accelXData[i+1]*(readingTimes[i+1]-readingTimes[i])*(readingTimes[i+1]-readingTimes[i]);
        latestYDist = accelYData[i]+accelXData[i+1]*(readingTimes[i+1]-readingTimes[i])*(readingTimes[i+1]-readingTimes[i]);
        latestZDist = accelZData[i]+accelXData[i+1]*(readingTimes[i+1]-readingTimes[i])*(readingTimes[i+1]-readingTimes[i]);
    }
    latestXDist /= (2*CLOCKS_PER_SEC*CLOCKS_PER_SEC);
    latestYDist /= (2*CLOCKS_PER_SEC*CLOCKS_PER_SEC);
    latestZDist /= (2*CLOCKS_PER_SEC*CLOCKS_PER_SEC);
}
//----------------------- IMU Deadreckoning Functions ------------------------//


//------------------------ GPS Localization Functions ------------------------//
void updateGPSCallback(const std_msgs::Int32 GPS)
{
    cout << "IMU Data Structure:" << GPS.data << "\n";
}
//------------------------ GPS Localization Functions ------------------------//



typedef actionlib::SimpleActionServer<monarc_tf::FlyAction> Server;

void executeAction(const monarc_tf::FlyGoalConstPtr& goal, Server* as, ros::Publisher* flight_command_pub) {
  ros::Rate hover_loop(2);
  switch (goal->command) {
    case monarc_tf::FlyGoal::TAKEOFF:
      // TODO(make decisions based on the command type)
      std::cout << "TAKEOFF\n";
      break;
    case monarc_tf::FlyGoal::LAND:
      // TODO(make decisions based on the command type)
      std::cout << "LAND\n";
      break;
    case monarc_tf::FlyGoal::HOVER:
      while (ros::ok() && !as->isNewGoalAvailable()) {
        std::cout << "HOVER\n";
        hover_loop.sleep();
      }
      ROS_INFO("HOVER CANCELLED");
      return;
    default:
      throw std::invalid_argument("unhandled command type");
  }

  int wait = 0;
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    // TODO read shit from data sources and make decisions. Publish on flight_command_pub.
    loop_rate.sleep();
    if (wait > 200) {
      break;
    }
    wait++;
  }

  as->setSucceeded();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "monarc_tf_node");
  ros::NodeHandle node;
  simpleDist = node.advertise<std_msgs::Float32>("simpleDist", 0.0);
  callBackCounter = node.advertise<std_msgs::Int32>("callbackCount", 0);
  ros::Subscriber IMUsub = node.subscribe("/IMU", 10, updateIMUCallback);
  ros::Subscriber GPSsub = node.subscribe("/GPS", 10, updateGPSCallback);
  ros::Subscriber sub = node.subscribe("/points2", 10, pointCloudCallback);

  ros::Publisher flight_command_pub = node.advertise<monarc_uart_driver::FlightControl>("flight_control", 10);
  Server server(node, "fly", boost::bind(&executeAction, _1, &server, &flight_command_pub), false);
  server.start();

  ros::spin();
  return 0;
};
