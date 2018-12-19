#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <stdio.h>
#include <nav_msgs/GetMap.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include "def.hpp"

using namespace cv;


//namespace
//{
void laser_callback(const sensor_msgs::LaserScan &msg)
{
	if (msg.ranges.empty())
	{
		ROS_WARN_STREAM(" URG data is empty.");
		return;
	}
	else {
		//ROS_WARN_STREAM("URG data is Ready.\n");
	}

	const auto posi = msg.ranges.size();
	//printf(" size = %d ", posi);
	int l_dir = posi / 2;
	const auto length = msg.ranges.at(l_dir);
	const auto AngleMin = msg.angle_min;
	const auto AngleMax = msg.angle_max;
	//printf(" length = %f ", length);
	//const auto intensity = msg.intensities.at(posi);
	//ROS_INFO_STREAM("ranges[" << posi << "]:" << length << ", intensities[" << posi << "]:" << intensity);

	printf("\n");
	ROS_INFO_STREAM("ranges[" << posi << "]:" << length << "_max" << AngleMin << "_min"<< AngleMax);

}

void mapCallback(const nav_msgs::OccupancyGridConstPtr &map) {
	geometry_msgs::Quaternion orientation = map->info.origin.orientation;
	double yaw, pitch, roll;
	tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
	mat.getEulerYPR(yaw, pitch, roll);
	double map_theta = yaw;

	ROS_INFO("Received a %d X %d map @ %.3f m/pix_%f_%f_%f",
		map->info.width,
		map->info.height,
		map->info.resolution,
		map->info.origin.position.x,
		map->info.origin.position.y,
		map_theta
	);

	Mat img = Mat::zeros(cv::Size(map->info.width, map->info.height), CV_8UC1);

	for (unsigned int y = 0; y < map->info.height; y++) {
		for (unsigned int x = 0; x < map->info.width; x++) {
			unsigned int i = x + (map->info.height - y - 1) * map->info.width;
			int intensity = 0;
			if (map->data[i] >= 0 && map->data[i] <= 100)
				intensity = round((float)(100.0 - map->data[i])*2.55);
			img.at<unsigned char>(y, x) = intensity;
		}
	}

	imshow("mapSHow", img);
	cvWaitKey(10);
	//ros::shutdown();
}
