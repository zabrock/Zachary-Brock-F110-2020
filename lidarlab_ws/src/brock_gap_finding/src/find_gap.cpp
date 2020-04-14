#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "brock_gap_finding/gaps.h"

class gapFinder
{
  private:
    double min_distance;
    double max_distance;
    ros::NodeHandle node_handle;
    ros::Subscriber lidar_sub;
    ros::Publisher gaps_pub;
    ros::Publisher largest_gap_pub;
  public:
    gapFinder();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

gapFinder::gapFinder() :
    node_handle(ros::NodeHandle()),
    lidar_sub(node_handle.subscribe("scan", 100, &gapFinder::scanCallback, this)),
    gaps_pub(node_handle.advertise<brock_gap_finding::gaps>("lidar_gaps", 100)),
    largest_gap_pub(node_handle.advertise<geometry_msgs::Vector3>("gap_center",100))
{

}

void gapFinder::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  brock_gap_finding::gaps gap_msg;
  geometry_msgs::Vector3 vec_template;
  vec_template.x = 1;
  vec_template.y = 2;
  vec_template.z = 3;
  std_msgs::Float64 temp_width;
  temp_width.data = 54.3;
  for(int i{0}; i < 3; i++)
  {
    gap_msg.gap_centers.push_back(vec_template);
    gap_msg.gap_widths.push_back(temp_width);
  }
  this->gaps_pub.publish(gap_msg);
  this->largest_gap_pub.publish(vec_template);
}

int main(int argc, char **argv)
{
  // Start a gap finding node
  ros::init(argc, argv, "brock_average");
  gapFinder gap_finder;

  // Spin to let the algorithm run until ROS shut down
  ros::spin();

  return 0;
}
