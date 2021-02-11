#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <chrono>
#include "kdtree.h"

pcl::PointCloud<pcl::PointXYZI> sensor2pcl(const sensor_msgs::PointCloud2::ConstPtr& msg);

void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

int main(int argc, char **argv);

#endif