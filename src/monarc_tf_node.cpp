#include <ros/ros.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <iostream>

// pointCloudCallback processes a single PointCloud2 message. We currently have
// this directly linked to the creation and broadcast of a transform. As we add
// more data sources through other subscriptions, this will get more complicated.
// Alternatives to the current approach are  are to:
// - Buffer the latest value for each subscription and process at a fixed rate
// - Buffer all but one subscription, and "drive" the broadcast from one (Chris Likes this option)
// - Use a Time Synchronizer (http://wiki.ros.org/message_filters#Time_Synchronizer)
//

using namespace std;
double avgDistance;


//double getMiddleAverage(const sensor_msgs::PointCloud2& cloud, int totalW, int totalH, double percentSize)
double getMiddleAverage(const sensor_msgs::PointCloud2& cloud)
{
	//using percentSize of the field of view at the center
	//in order to calculate the distance change
	//from last scan to now. This can be more efficient
    //computationally once we figure out the best number of
    //points/size of frame to run this function over.
    //Assumes the vector of points is streaming in from left
    //to right and then from top to bottom, so it will
    //have the entire first row first in order.

	// int right = (totalW/2)+(totalW*(percentSize/2));
	// int left = (totalW/2)-(totalW*(percentSize/2));
	// int top = (totalH/2))+(totalH*(percentSize/2);
    // int bottom = (totalH/2)-(totalH*(percentSize/2));

	double count = 0.0;
	double totalDepth = 0.0;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        cout << "X coord = " << (*iter_x) << "; Y coord = " <<  (*iter_y) << "; Z coord = " << (*iter_z) << ";\n";
        count += 1.0;
        totalDepth += (*iter_x);
        /*
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
          if (*iter_y <= right && *iter_y >= left)
          {
              if (*iter_z <= top && *iter_z >= bottom)
              {
                  cout << "X coord = " << (*iter_x) << "; Y coord = " <<  (*iter_y) << "; Z coord = " << (*iter_z) << ";\n";
                  count += 1.0;
                  totalDepth += (*iter_x);
              }
          }
      }*/
    }
	return totalDepth/count;
}

void pointCloudCallback(const sensor_msgs::PointCloud2& msg){
  static tf2_ros::TransformBroadcaster br;
  avgDistance = getMiddleAverage(msg);
  cout << "avgDistance = " << avgDistance << "\n";
  ROS_INFO( "The callback got hit\n" );
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";

  transformStamped.child_frame_id = "monarc";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "monarc_tf_node");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/points2", 10, pointCloudCallback);

  ros::spin();
  return 0;
};
