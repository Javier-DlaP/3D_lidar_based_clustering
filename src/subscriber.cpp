#include "subscriber.h"

int main(int argc, char **argv)
{
  // function required before using any other ROS based functions
  ros::init(argc, argv, "listener");

  // access point to ROS
  ros::NodeHandle nh;
  
  // Subscribers
  ros::Subscriber point_cloud = nh.subscribe("/carla/ego_vehicle/lidar/lidar1/point_cloud", 1, pclCallback);

  // function that calls each callback
  ros::spin();

  return 0;
}

// this function is called every time the pcl topic is updated
void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // max distance between points
  //const int max_dist = 10;
  // create the tree
  //KdTree* tree = new KdTree;
  //std::cout << (msg->height) << " - " << (msg->width) << " - " << (msg->row_step) << std::endl;

  pcl::PointCloud<pcl::PointXYZI> point_cloud = sensor2pcl(msg);
  
  //tree->insert(point, i);
  /*
  std::vector<int> ids = tree->search(point, max_dist);
  std::cout << std::to_string(ids.size()) << std::endl;
  for(std::vector<int>::iterator id = ids.begin(); id != ids.end(); ++id){
    std::cout << std::to_string(*id) << " ";
  }
  std::cout << std::endl;
  */
}

// convert sensor_msgs::PointCloud2 structure to a pcl::PointClouds structure
pcl::PointCloud<pcl::PointXYZI> sensor2pcl(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI> point_cloud;
  // iteration over every point of the pcl structure use by ROS
  auto startTime = std::chrono::steady_clock::now();
  for (int i=0; i<(int)(msg->width); i++){
    pcl::PointXYZI point;
    // iteration over x, y, z and intensity of every point
    for (int j=0; j<4; j++){
      uint32_t foo_ = (uint32_t)((msg->data[i*16+j*4+3] << 24) | (msg->data[i*16+j*4+2] << 16) | (msg->data[i*16+j*4+1] << 8) | (msg->data[i*16+j*4] << 0));
      float foo;
      std::memcpy(&foo, &foo_, sizeof(float));
      // fill the point
      switch(j){
        case 0:
          point.x = foo;
          break;
        case 1:
          point.y = foo;
          break;
        case 2:
          point.z = foo;
          break;
        case 3:
          point.intensity = foo;
          break;
      }
    }
    //std::cout << i << " - " << msg->width << std::endl;
    point_cloud.push_back(point);
  }
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << elapsedTime.count() << " milliseconds" << std::endl;
  return point_cloud;
}