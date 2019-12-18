#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include "speed_cal.h"

int main(int argc, char *argv[])
{
  ROS_INFO("plans_detection program start!");

  ros::init(argc, argv, "plans_detection");
  ros::Time::init();
  ros::Rate loop_rate(10);//hz

  planes_detection::PlanesDetection pd;

  while (ros::ok())
  {
    /* code for loop body */
    pd.runPlanesDetection();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}