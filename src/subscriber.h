#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <chrono>
#include "kdtree.h"

int main(int argc, char **argv);

void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

pcl::PointCloud<pcl::PointXYZI>::Ptr sensor2pcl(const sensor_msgs::PointCloud2::ConstPtr& msg);

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentPlane(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold);

#endif