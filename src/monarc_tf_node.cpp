#include <iostream>
#include <math.h>
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
#define PI 3.1415926535897932
geometry_msgs::TransformStamped transformStamped;

//--------------------------- TF Tuning Parameters ---------------------------//
//this group should add to 1.0
double IMUAltFactor = 0.1
double ultrasonicFactor = 0.8
double GPSfactor = 0.05
double barometerFactor = 0.05
//--------------------------- TF Tuning Parameters ---------------------------//

//--------------------------- ROS Published Topics ---------------------------//
ros::Publisher simpleDist;
ros::Publisher callBackCounter;
ros::Publisher flightCommands;
//--------------------------- ROS Published Topics ---------------------------//


//-------------------- Point Cloud Localization Variables --------------------//
double avgDistance = 0.0;
double pastDistance = 0.0;
double cloudEdges[6];
//-------------------- Point Cloud Localization Variables --------------------//


//----------------- IMU Deadreckoning Calculation Variables ------------------//
int IMUdistAverageNumber = 10; //will calculate distance based on the last <IMUdistAverageNumber> Imu readings
double accelXData[IMUdistAverageNumber];
double accelYData[IMUdistAverageNumber];
double accelZData[IMUdistAverageNumber];

double groundTruthX[IMUdistAverageNumber];
double groundTruthY[IMUdistAverageNumber];
double groundTruthZ[IMUdistAverageNumber];

double latestXDistIMU;
double latestYDistIMU;
double latestZDistIMU;

int readingTimes[1000];
int accelDataCounter = 0;
//----------------- IMU Deadreckoning Calculation Variables ------------------//


//------------------------ GPS Localization Variables ------------------------//
double initialGPScovarianceLatLongReadings = 10.0;
double initialGPScovarianceAltReadings = 10.0;
bool GPSinitialized = false;
int GPScallbackCount = 0;
int GPSintializationCount = 60; // this is the number of GPS readings we must
                                // recieve before taking off
double maxLatLongCovariance = 3.0;
double maxAltCovariance= 4.0;

double currentGPSLong = -71.08841; //Initialized to GPS coordinates of wireless club
double currentGPSLat = 42.33924;   //Initialized to GPS coordinates of wireless club
double currentGPSAlt = 27.0;       //Initialized to GPS Altitude of wireless club
double currentCovarianceLatLong = 5.0;
double currentCovarianceAlt = 9.0;

double lastGPSLong = -71.08841; //These variables log the last GPS coordinates
double lastGPSLat = 42.33924;
double lastGPSAlt = 27.0;
double lastCovarianceLatLong = 5.0;
double lastCovarianceAlt = 9.0;

double currentGPSmetersLong = 0.0;  //these variables keep track of the exact distance
double currentGPSmetersLat = 0.0;   //from the very first GPS coordinates that were
double currentGPSmetersAlt = 0.0;   //logged.

double originGPSlong = 0.0;  //these variables keep track of the exact distance
double originGPSlat = 0.0;   //from the very first GPS coordinates that were
double originGPSalt = 0.0;   //logged.

double destinationGPSmetersLong = 0.0;  //these variables keep track of the exact distance
double destinationGPSmetersLat = 0.0;   //from the very first GPS coordinates that were
double destinationGPSmetersAlt = 0.0;   //logged.

double radiusOfEarth = (6378137.0 + 6356752.0)/2.0; // Average radius of earth in Meters
//------------------------ GPS Localization Variables ------------------------//


//------------------------ Atmospheric Localization Variables ------------------------//
double currentAtm = 0.0; //Unit is in meters from sea level
doube atmSmoothingFactor = .8;


//------------------------ Atmospheric Localization Variables ------------------------//

//------------------------ Ultrasonic Localization Variables ------------------------//
double currentUltra = 0.0;
doube ultraSmoothingFactor = .8;
//------------------------ Ultrasonic Localization Variables ------------------------//
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
    /*
    //This code should be commented out until we are ready to use the
    //camera data once again

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
    */
}
//-------------------- Point Cloud Localization Functions --------------------//


//----------------------- IMU Deadreckoning Functions ------------------------//
void updateIMUCallback(const sensor_msgs::Imu)
{
    accelXData[accelDataCounter] = Imu.linear_acceleration.x;
    accelYData[accelDataCounter] = Imu.linear_acceleration.y;
    accelZData[accelDataCounter] = Imu.linear_acceleration.z;

    groundTruthX[accelDataCounter] = transformStamped.transform.translation.x;
    groundTruthY[accelDataCounter] = transformStamped.transform.translation.y;
    groundTruthZ[accelDataCounter] = transformStamped.transform.translation.z;

    readingTimes[accelDataCounter] = clock();
    accelDataCounter++;
    if (accelDataCounter == IMUdistAverageNumber)
    {
        accelDataCounter = 0;
    }
    convertAccelToDist();
}

void convertAccelToDist()
{
    latestXDistIMU = 0.0;
    latestYDistIMU = 0.0;
    latestZDistIMU = 0.0;
    //function uses trapezoidal rule
    for ( int i = 0; i < (IMUdistAverageNumber-1); i++ )
    {
        latestXDistIMU = accelXData[i]+accelXData[i+1]*(readingTimes[i+1]-readingTimes[i])*(readingTimes[i+1]-readingTimes[i]);
        latestYDistIMU = accelYData[i]+accelXData[i+1]*(readingTimes[i+1]-readingTimes[i])*(readingTimes[i+1]-readingTimes[i]);
        latestZDistIMU = accelZData[i]+accelXData[i+1]*(readingTimes[i+1]-readingTimes[i])*(readingTimes[i+1]-readingTimes[i]);
    }
    latestXDistIMU /= (2*CLOCKS_PER_SEC*CLOCKS_PER_SEC);
    latestYDistIMU /= (2*CLOCKS_PER_SEC*CLOCKS_PER_SEC);
    latestZDistIMU /= (2*CLOCKS_PER_SEC*CLOCKS_PER_SEC);

    latestXDistIMU += groundTruthX[accelDataCounter];
    latestYDistIMU += groundTruthY[accelDataCounter];
    latestZDistIMU += groundTruthZ[accelDataCounter];
}
//----------------------- IMU Deadreckoning Functions ------------------------//


void ultraSoundCallback(std_msgs::Int32 ultraAlt)
{
    //ground is 190, units are millimeters
    currentAtm = (ultraAlt.data*(ultraSmoothingFactor))+(currentUltra*(1.0-ultraSmoothingFactor));
}

void atomspherCallback(std_msgs::Int32 atm)
{
    //units are in meters
    currentAtm = (atm.data*(atmSmoothingFactor))+(currentAtm*(1.0-atmSmoothingFactor));
}

//------------------------ GPS Localization Functions ------------------------//
void updateGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& GPS)
{
    if (GPS.status.status == 1)
    {
        if (GPSinitialized)
        {
            lastGPSLong = currentGPSLong; //These variables log the last GPS coordinates
            lastGPSLat = currentGPSLat;
            lastGPSAlt = currentGPSAlt;
            lastCovarianceLatLong = currentCovarianceLatLong;
            lastCovarianceAlt = currentCovarianceAlt;

            currentGPSLat = GPS.latitude;
            currentGPSLong = GPS.longitude;
            currentGPSAlt = GPS.altitude;
            currentCovarianceLatLong = GPS.position_covariance[0];
            currentCovarianceAlt = GPS.position_covariance[8];
        } else {
            // Everything in this else is to initialize the GPS values
            if (GPS.position_covariance[0] < initialGPScovarianceLatLongReading)
            {
                initialGPScovarianceLatLongReading = GPS.position_covariance[0];
                originGPSlat = GPS.latitude;
                originGPSlong = GPS.longitude;
            }
            if ( GPS.position_covariance[8] < initialGPScovarianceAltReading)
            {
                initialGPScovarianceAltReading = GPS.position_covariance[8];
                originGPSalt = GPS.altitude;
            }
            GPScallbackCount += 1;
            if (GPScallbackCount == GPSintializationCount)
            {
                GPSinitialized = true;
                currentGPSLat = originGPSlat;
                currentGPSLong = originGPSlong;
                currentGPSAlt = originGPSalt;
            }
        }
    }
}

double convertCoordinatesToMeters(double & coordLatest, double & coordOlder)
{
    //I based these calculations on the earth being
    //a perfect sphere. Since we are traveling small
    //distances relative to the ellipsoidal nature
    //of the earth, doing the extra calculation for
    //an ellipsoid is a waste. Also, in this orientation
    //North, East and Up are the positive directions on
    //the grid. This convetnion is transformed when
    //converting to ROS coordinate system, because
    //West or left is positive in that grid.
    return radiusOfEarth * (coordLatest-coordOlder) * PI / 180.0;
}

void setFirstGPS(const sensor_msgs::NavSatFix::ConstPtr& GPS)
{
    //write something that test for gps lock
    //Returns NANs until GPS lock
    //Wait till diaganol of covariance
    //matrix is below 2 in order to be sure of
    //position
    currentGPSLat = GPS.latitude;
    currentGPSLong = GPS.longitude;
    currentGPSAlt = GPS.altitude;
}
//------------------------ GPS Localization Functions ------------------------//

void updateTF()
{
    transformStamped.header.stamp = msg.header.stamp;
    transformStamped.header.frame_id = "map";

    transformStamped.child_frame_id = msg.header.frame_id;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = (IMUAltFactor*latestZDistIMU)+(ultrasonicFactor*currentUltra)+(GPSfactor*currentGPSmetersAlt)+(barometerFactor*currentAtm);
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
    simpleDist = node.advertise<std_msgs::Float32>("simpleDist", 0.0);
    callBackCounter = node.advertise<std_msgs::Int32>("callbackCount", 0);

    ros::Subscriber IMUsub = node.subscribe("/IMU", 10, updateIMUCallback);
    ros::Subscriber GPSsub = node.subscribe("/fix", 10, updateGPSCallback);
    ros::Subscriber ultsub = node.subscribe("/ultrasonic_altitude", 10, ultraSoundCallback);
    ros::Subscriber atosub = node.subscribe("/atmospheric_pressure", 10, atomspherCallback);
    //ros::Subscriber Camsub = node.subscribe("/points2", 10, pointCloudCallback); //disabled till after up down test flight

    ros::Publisher flight_command_pub = node.advertise<monarc_uart_driver::FlightControl>("flight_control", 10);
    Server server(node, "fly", boost::bind(&executeAction, _1, &server, &flight_command_pub), false);
    server.start();

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        /*
        * Process all callbacks, which will populate nav_cpu_state.
        */
        ros::spinOnce();
        updateTF();

        /*
        * Sleep for the remainder of the interval.
        */
        loop_rate.sleep();
    }
    return 0;
};
