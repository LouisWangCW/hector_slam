#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <stdio.h>
#include <nav_msgs/GetMap.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include "urg_test.hpp"
using namespace cv;



//}


int main(int argc, char **argv)
{
  printf("main Start\n");

  ros::init(argc, argv, "urg_test");
  ros::NodeHandle n;
  ros::Subscriber laser = n.subscribe("scan", 1, laser_callback);
  ros::Subscriber map_sub_ = n.subscribe("map", 2, mapCallback);
  ros::Rate loop_rate(10);
  
  printf("ros End\n");

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
  
}
