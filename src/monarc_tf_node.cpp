#include <stdexcept>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include "monarc_tf/FlyAction.h"
#include "monarc_tf/FlyGoal.h"
#include "monarc_uart_driver/FlightControl.h"

// pointCloudCallback processes a single PointCloud2 message. We currently have
// this directly linked to the creation and broadcast of a transform. As we add
// more data sources through other subscriptions, this will get more complicated. 
// Alternatives to the current approach are  are to:
// - Buffer the latest value for each subscription and process at a fixed rate
// - Buffer all but one subscription, and "drive" the broadcast from one
// - Use a Time Synchronizer (http://wiki.ros.org/message_filters#Time_Synchronizer)
//
void pointCloudCallback(const sensor_msgs::PointCloud2& msg){
  static tf2_ros::TransformBroadcaster br;

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
  ros::Subscriber sub = node.subscribe("/points2", 10, pointCloudCallback);

  ros::Publisher flight_command_pub = node.advertise<monarc_uart_driver::FlightControl>("flight_control", 10);
  Server server(node, "fly", boost::bind(&executeAction, _1, &server, &flight_command_pub), false);
  server.start();

  ros::spin();
  return 0;
};

