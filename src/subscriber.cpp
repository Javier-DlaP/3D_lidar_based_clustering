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
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud = sensor2pcl(msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_point_cloud = filterCloud(point_cloud, 0.3, Eigen::Vector4f (-20,-20,-20,1), Eigen::Vector4f (20,20,20,1));

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = segmentPlane(filtered_point_cloud, 50, 0.2);

  std::cout << std::endl;
}

// convert sensor_msgs::PointCloud2 structure to a pcl::PointClouds structure
pcl::PointCloud<pcl::PointXYZI>::Ptr sensor2pcl(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::cout << "Creating the point cloud: ";
  auto startTime = std::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // iteration over every point of the pcl structure use by ROS
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
    point_cloud->push_back(point);
  }
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << elapsedTime.count() << " milliseconds, " << point_cloud->points.size() << " points" << std::endl;
  return point_cloud;
}

// filter the point cloud based on the region of interest and the voxel size
pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
  std::cout << "Filtering point cloud: ";
  auto startTime = std::chrono::steady_clock::now();

  // filter based on voxels
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZI>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);

  // filter based on region of interest
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRegion(new pcl::PointCloud<pcl::PointXYZI>);  
  pcl::CropBox<pcl::PointXYZI> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);
/*
  std::vector<int> indices;
  pcl::CropBox<pcl::PointXYZI> roof(true);
  roof.setMin(minPoint);
  roof.setMax(maxPoint);
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  for(int point : indices){
    inliers->indices.push_back(point);
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);
*/
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << elapsedTime.count() << " milliseconds, " << cloudFiltered->points.size() << " points" << std::endl;

  return cloudRegion;
}

// apply ransac to segment into floor and 
std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  std::cout << "Applying ransac-3D: ";
  auto startTime = std::chrono::steady_clock::now();

  std::unordered_set<int> inliersResult;
	srand(time(NULL));
    
  // iterate the number of iteration
	for(int i=0; i<maxIterations; i++){
    std::unordered_set<int> inliers;

    // get 3 random point from the cloud and create the 3D-plane
    while(inliers.size() < 3){
		  inliers.insert(rand()%(cloud->points.size()));
    }

    float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

    float a, b, c, d;
		a = ((y2 - y1)*(z3 - z1)) - ((z2 - z1)*(y3 - y1));
		b = ((z2 - z1)*(x3 - x1)) - ((x2 - x1)*(z3 - z1));
		c = ((x2 - x1)*(y3 - y1)) - ((y2 - y1)*(x3 - x1));
		d = -(a*x1 + b*y1 + c*z1);

    // get the points that lie on the plane within the distance threshold
		for(int index = 0; index < (int) cloud->points.size(); index++){
			if(inliers.count(index) > 0) continue;
            
			pcl::PointXYZI point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float distance = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a + b*b + c*c);

			if(distance <= distanceThreshold){
				inliers.insert(index);
			}
		}

    // store the results of the plane on which lies the most points
		if(inliers.size()>inliersResult.size()){
			inliersResult = inliers;
		}
	}

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

  // divide the floor and the rest in 2 different point clouds
  for (int index = 0; index < (int) cloud->points.size(); index++){
    pcl::PointXYZI point = cloud->points[index];
    if(inliersResult.count(index)){
      cloudInliers->points.push_back(point);
    }else{
      cloudOutliers->points.push_back(point);
    }
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << elapsedTime.count() << " milliseconds, " << cloudInliers->points.size() << " points" << std::endl;

  return std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> (cloudOutliers,cloudInliers);
}