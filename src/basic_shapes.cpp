#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

  while (ros::ok())
  {
    
    visualization_msgs::MarkerArray ms;
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "86";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    ms.markers.push_back(marker);





    visualization_msgs::Marker marker2;
    
    marker2.header.frame_id = "86";
    marker2.header.stamp = ros::Time::now();

    marker2.ns = "basic_shapes";
    marker2.id = 1;

    marker2.type = visualization_msgs::Marker::SPHERE;

    marker2.action = visualization_msgs::Marker::ADD;

    marker2.pose.position.x = 0;
    marker2.pose.position.y = 0;
    marker2.pose.position.z = 3;
    marker2.pose.orientation.x = 0.0;
    marker2.pose.orientation.y = 0.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 1.0;

    marker2.scale.x = 1;
    marker2.scale.y = 2;
    marker2.scale.z = 1;

    marker2.color.r = 1.0;
    marker2.color.g = 1.0;
    marker2.color.b = 1.0;
    marker2.color.a = 1.0;

    marker2.lifetime = ros::Duration();

    ms.markers.push_back(marker2);



    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    std::cout << "." << std::endl;

    marker_pub.publish(ms);

    r.sleep();
    
  }
  
}