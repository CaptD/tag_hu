#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>


class StereoCloud
{
public:
	StereoCloud();
	void readDistance(const sensor_msgs::PointCloud2::ConstPtr& mcurrScan);	
private:
	ros::NodeHandle nh;
	ros::Subscriber pcloud_sub;
    ros::Publisher dist_pub;  
};

StereoCloud::StereoCloud() {
	pcloud_sub = nh.subscribe("/elp/points2", 10, &StereoCloud::readDistance, this);
	dist_pub = nh.advertise<std_msgs::Float32>("stereo_distance",10);
};

void StereoCloud::readDistance(const sensor_msgs::PointCloud2::ConstPtr& mcurrScan) {
    sensor_msgs::PointCloud currScan;
    sensor_msgs::convertPointCloud2ToPointCloud(*mcurrScan, currScan);
    //std::cout << currScan.points.size() << std::endl;
    std_msgs::Float32 distance_msg;
    distance_msg.data = 100000000.0;
    for (int i = 0; i < currScan.points.size(); i++) {
        if (currScan.points[i].z < distance_msg.data) {
            distance_msg.data = currScan.points[i].z;
        }
    }
    
	dist_pub.publish(distance_msg);

};

int main(int argc, char** argv){
  ros::init(argc, argv, "stereo_cloud");
  ros::start(); // start the node resource managers (communication, time, etc)
  
  StereoCloud sc;
  
  ros::spin();

  return 0;
};
