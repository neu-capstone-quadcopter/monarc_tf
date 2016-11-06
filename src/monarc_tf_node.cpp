#include <ros/ros.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <iostream>
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

using namespace std;

double avgDistance = 0.0;
double pastDistance = 0.0;
double cloudEdges[6];

ros::Publisher simpleDist;
ros::Publisher callBackCounter;
int counter = 1;

void getMinMax(const sensor_msgs::PointCloud2& cloud)
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
{
	//using percentSize of the field of view at the center
	//in order to calculate the distance change
	//from last scan to now. This can be more efficient
  //computationally once we figure out the best number of
  //points/size of frame to run this function over.
  //Assumes the vector of points is streaming in from left
  //to right and then from top to bottom, so it will
  //have the entire first row first in order.

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

int yaw = 0;

// pointCloudCallback processes a single PointCloud2 message. We currently have
// this directly linked to the creation and broadcast of a transform. As we add
// more data sources through other subscriptions, this will get more complicated.
// Alternatives to the current approach are  are to:
// - Buffer the latest value for each subscription and process at a fixed rate
// - Buffer all but one subscription, and "drive" the broadcast from one (Chris Likes this option)
// - Use a Time Synchronizer (http://wiki.ros.org/message_filters#Time_Synchronizer)
void pointCloudCallback(const sensor_msgs::PointCloud2& msg){
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
  yaw += 10;
  q.setRPY(0, 0, yaw);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  std_msgs::Float32 nDist;
  nDist.data = avgDistance;
  simpleDist.publish(nDist);
  
  std_msgs::Int32 callCount;
  callCount.data = counter;
  callBackCounter.publish(callCount);
  br.sendTransform(transformStamped);
  counter++;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "monarc_tf_node");
  ros::NodeHandle node;
  simpleDist = node.advertise<std_msgs::Float32>("simpleDist", 0.0);
  callBackCounter = node.advertise<std_msgs::Int32>("callbackCount", 0);
  ros::Subscriber sub = node.subscribe("/points2", 10, pointCloudCallback);

  ros::spin();
  return 0;
};
