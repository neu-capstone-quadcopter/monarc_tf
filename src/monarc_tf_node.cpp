#include <iostream>
#include <math.h>
#include <stdexcept>
#include <mutex>

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

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
std::mutex tfLock;

//These values need tuning
double distGain = 350.0; //124 was the origional value
double velocityGain = 500.0;
int approxHover = 920; //throttle at which hovering occurs approximatly
int centerYaw = 992;
int centerPitch = 992;
int centerRoll = 992;
int maxThrottleValue = 1800;
int minThrottleValue = 8;

//--------------------------- TF Tuning Parameters ---------------------------//
//these are simply default values
double IMUAltFactor = 0.1;
double ultrasonicFactor = 0.8;
double GPSfactor = 0.05;
double barometerFactor = 0.05;
double takeOffHeight = 1.0;
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
const int IMUdistAverageNumber = 10; //will calculate distance based on the last <IMUdistAverageNumber> Imu readings
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
double initialGPScovarianceLatLongReading = 10.0;
double initialGPScovarianceAltReading = 10.0;
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
double atmSmoothingFactor = .8;


//------------------------ Atmospheric Localization Variables ------------------------//

//------------------------ Ultrasonic Localization Variables ------------------------//
double currentUltra = 0.0;
double ultraSmoothingFactor = 1.0;
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

void updateIMUCallback(const sensor_msgs::Imu Imu)
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
//----------------------- IMU Deadreckoning Functions ------------------------//


void ultraSoundCallback(std_msgs::Int32 ultraAlt)
{
    //ground is approx 190, units are millimeters
    currentUltra = ((ultraAlt.data*ultraSmoothingFactor + currentUltra*(1.0-ultraSmoothingFactor))-190.0)/1000.0;
}

void atomspherCallback(std_msgs::Int32 atm)
{
    //units are in meters
    if (GPSinitialized)
    {
        currentAtm = (atm.data*(atmSmoothingFactor))+(currentAtm*(1.0-atmSmoothingFactor));
    } else {
        currentAtm = (atm.data*(0.1))+(currentAtm*(0.9));
    }
}

//------------------------ GPS Localization Functions ------------------------//

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

void updateGPSCallback(const sensor_msgs::NavSatFix & GPS)
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
        //TODO update these variables so that they involve a weighted average based
        //on the covariance matrix
        currentGPSmetersLat = convertCoordinatesToMeters(currentGPSLat, originGPSlat);
        currentGPSmetersLong = convertCoordinatesToMeters(currentGPSLong, originGPSlong);
        currentGPSmetersAlt = currentGPSAlt - originGPSalt; //This makes the take off altitude zero which matches it to the barometer
    }
}

//------------------------ GPS Localization Functions ------------------------//

void normalizeAltSensorFactors()
{
    double total = IMUAltFactor + ultrasonicFactor + GPSfactor + barometerFactor;
    IMUAltFactor /= total;
    ultrasonicFactor /= total;
    GPSfactor /= total;
    barometerFactor /= total;
}

void setTranslationTransform(){
    std::lock_guard<std::mutex> guard(tfLock);
    transformStamped.transform.translation.x = currentGPSmetersLat;
    transformStamped.transform.translation.y = currentGPSmetersLong;
    transformStamped.transform.translation.z = (IMUAltFactor*latestZDistIMU)+(ultrasonicFactor*currentUltra)+(GPSfactor*currentGPSmetersAlt)+(barometerFactor*currentAtm);
}

void updateTF()
{

    if (currentUltra < 5.0)
    {
        IMUAltFactor = 0.0;
        ultrasonicFactor = 1.0;
        GPSfactor = 0.0;
        barometerFactor = 0.0;
    } else {
        IMUAltFactor = 0.0;       //Set to some sort of std deviation
        ultrasonicFactor = 1.0;   //Don't use if further away than 5 meters
        GPSfactor = 0.0;          //Base
        barometerFactor = 0.0;
        normalizeAltSensorFactors();
    }

    setTranslationTransform();
    //TODO set these to the value coming orientation
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 0;
}

double getCurrentZ()
{
    std::lock_guard<std::mutex> guard(tfLock);
    return transformStamped.transform.translation.z;
}

void enterDangerZone(ros::Publisher* flight_command_pub)
{
    //controller mid values are 1000 and they range
    //from about 200 to 1800
    int throttle = 200;
    double upwardVelocity = 0.0;
    double pastD = 0;
    double currentD = getCurrentZ();

    //slowly bring up throttle to 600, about 4 seconds
    ros::Rate loop_rate(100);
    while (throttle < 600 && ros::ok())
    {
        monarc_uart_driver::FlightControl fCommands;
        fCommands.pitch = centerPitch;
        fCommands.roll = centerRoll;
        fCommands.yaw = centerYaw;

        throttle = throttle + 1;
        fCommands.throttle = throttle;
        if (fCommands.throttle > maxThrottleValue)
        {
            fCommands.throttle = maxThrottleValue;
        }
        if (fCommands.throttle < minThrottleValue)
        {
            fCommands.throttle = minThrottleValue;
        }
        flight_command_pub->publish(fCommands);
        loop_rate.sleep();
    }

    //gets to around 1000 in 200 milliseconds
    while (currentD <= 0.3 && ros::ok())
    {
        pastD = currentD;
        currentD = getCurrentZ();
        upwardVelocity = currentD - pastD; //upwardVelocity in meters per loop cycle
        monarc_uart_driver::FlightControl fCommands;
        fCommands.pitch = centerPitch;
        fCommands.roll = centerRoll;
        fCommands.yaw = centerYaw;


        throttle = throttle + 10; //turn down the last value to take off slower
        if (throttle > maxThrottleValue)
        {
            throttle = maxThrottleValue;
        }
        if (throttle < minThrottleValue)
        {
            throttle = minThrottleValue;
        }

        fCommands.throttle = throttle;
        flight_command_pub->publish(fCommands);
        loop_rate.sleep();
    }

    double deltaAlt = 0;
    double holdingAlt = 0.3;

    while(currentD < takeOffHeight && ros::ok())
    {
        pastD = currentD;
        currentD = getCurrentZ();
        upwardVelocity = currentD - pastD; //upwardVelocity in meters per loop cycle

        monarc_uart_driver::FlightControl fCommands;
        fCommands.pitch = centerPitch;
        fCommands.roll = centerRoll;
        fCommands.yaw = centerYaw;

        deltaAlt = holdingAlt - currentD;
        holdingAlt += 0.02;

        fCommands.throttle = int(approxHover+(deltaAlt*distGain)-(upwardVelocity*velocityGain));

        if (fCommands.throttle > maxThrottleValue)
        {
            fCommands.throttle = maxThrottleValue;
        }
        if (fCommands.throttle < minThrottleValue)
        {
            fCommands.throttle = minThrottleValue;
        }

        flight_command_pub->publish(fCommands);

        loop_rate.sleep();
    }
}

void touchdown(ros::Publisher* flight_command_pub)
{
    double upwardVelocity = 0.0;
    double pastD = getCurrentZ();
    double currentD = pastD;
    double holdingAlt = currentD; //This is the altitude we want to hold in meters

    ros::Rate loop_rate(100);

    monarc_uart_driver::FlightControl fCommands;
    ROS_INFO("Landing @ %f", holdingAlt);
    while ( ros::ok() && holdingAlt > 0.02 )
    {
        ROS_INFO("Landing");
        pastD = currentD;
        currentD = getCurrentZ();

        upwardVelocity = currentD - pastD; //upwardVelocity in meters per loop cycle

        if (currentD - holdingAlt <= 0.02 && currentD - holdingAlt >= -0.02)
        {
            holdingAlt *= 0.90;
        }
        fCommands.pitch = centerPitch;
        fCommands.roll = centerRoll;
        fCommands.yaw = centerYaw;

        double deltaAlt = holdingAlt - currentD;

        fCommands.throttle = int(approxHover+(deltaAlt*distGain)-(upwardVelocity*velocityGain));
        ROS_INFO("%d  %f  %f", fCommands.throttle, (deltaAlt*distGain), (upwardVelocity*velocityGain));
        if (fCommands.throttle > maxThrottleValue)
        {
            fCommands.throttle = maxThrottleValue;
        }
        if (fCommands.throttle < minThrottleValue)
        {
            fCommands.throttle = minThrottleValue;
        }
        flight_command_pub->publish(fCommands);
        loop_rate.sleep();
    }
    fCommands.pitch = centerPitch;
    fCommands.roll = centerRoll;
    fCommands.yaw = centerYaw;
    fCommands.throttle = minThrottleValue;

    flight_command_pub->publish(fCommands);
}

typedef actionlib::SimpleActionServer<monarc_tf::FlyAction> Server;


void peepingTom(ros::Publisher* flight_command_pub, Server* as)
{
    double holdingAlt = 1.0; //This is the altitude we want to hold in meters
    int throttle = 200;
    double upwardVelocity = 0.0;
    double pastD = holdingAlt;
    double currentD = holdingAlt;

    ros::Rate loop_rate(100);


    while ( ros::ok() && !as->isNewGoalAvailable() )
    {
        ROS_INFO("Hovering");
        pastD = currentD;
        currentD = getCurrentZ();

        upwardVelocity = currentD - pastD; //upwardVelocity in meters per loop cycle

        monarc_uart_driver::FlightControl fCommands;
        fCommands.pitch = centerPitch;
        fCommands.roll = centerRoll;
        fCommands.yaw = centerYaw;

        double deltaAlt = takeOffHeight - currentD;

        fCommands.throttle = int(approxHover+(deltaAlt*distGain)-(upwardVelocity*velocityGain));
        if (fCommands.throttle > maxThrottleValue)
        {
            fCommands.throttle = maxThrottleValue;
        }
        if (fCommands.throttle < minThrottleValue)
        {
            fCommands.throttle = minThrottleValue;
        }
        flight_command_pub->publish(fCommands);
        loop_rate.sleep();
    }
}

void executeAction(const monarc_tf::FlyGoalConstPtr& goal, Server* as, ros::Publisher* flight_command_pub) {
  switch (goal->command) {
    case monarc_tf::FlyGoal::TAKEOFF:
      ROS_INFO("Taking off!");
      enterDangerZone(flight_command_pub);
      as->setSucceeded();
      return;
    case monarc_tf::FlyGoal::LAND:
      ROS_INFO("Landing!");
      touchdown(flight_command_pub);
      as->setSucceeded();
      return;
    case monarc_tf::FlyGoal::HOVER:
      peepingTom(flight_command_pub, as);
      as->setPreempted();
      return;
    default:
      throw std::invalid_argument("unhandled command type");
  }
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
