#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>
#include <time.h>
#include <unordered_map>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <deque>
#include <unordered_map>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Boost
#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "readparam.h"
#include "tracking/tracker.h"
#include "detecting/detector.h"
#include "gridmapping/globalgridmap.hpp"
#include "gridmapping/localgridmap.hpp"
#include "utility.h"

// Ros
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/OccupancyGrid.h" //TODO gridmap另外封装

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
typedef pcl::PointXYZI PointType;

ros::Publisher publidar;
ros::Publisher pubmarker;
ros::Publisher pubtextmarker;
ros::Publisher pubbackground;
ros::Publisher pubobject;
ros::Publisher pubcluster;
ros::Publisher pubtracked;
ros::Publisher pubstaticobjlocal;
ros::Publisher pubdynamicobjlocal;
ros::Publisher pubstaticobjglobal;
ros::Publisher pubdynamicobjglobal;
ros::Publisher pubglobalmap;
ros::Publisher publocalmap;

ros::Subscriber subcloud;
ros::Subscriber subodom;
ros::Subscriber subdets;


std::deque<sensor_msgs::PointCloud2> cloudQueue;
std::deque<visualization_msgs::MarkerArray> detsQueue;

ros::Time timeLaserInfoStamp;
ros::Time timeDetsStamp;

Param param;



float time_pre = 0;
double init_time;
bool init = false;

unordered_map<int, vector<int>> idcolor;
cv::RNG rng(12345);




void readParams(ros::NodeHandle nh){
	// read params
	nh.param<int>("min_pt", param.min_pt, 50);
	nh.param<int>("max_pt", param.max_pt, 1000);
	nh.param<float>("min_size_length", param.min_size[0], 0.15);
	nh.param<float>("min_size_width",  param.min_size[1], 0.1);
	nh.param<float>("min_size_height", param.min_size[2], 0.4);
	nh.param<float>("max_size_length", param.max_size[0], 1.8);
	nh.param<float>("max_size_width",  param.max_size[1], 1.5);
	nh.param<float>("max_size_height", param.max_size[2], 1.8);
	nh.param<int>("Horizon_SCAN", param.Horizon_SCAN, 1024);
	nh.param<int>("N_SCAN", param.N_SCAN, 64);
	nh.param<int>("downsampleRate", param.downsampleRate, 2);
	nh.param<float>("scan_range", 	param.scan_range, 15.0);

	ROS_INFO("scan_range: %f", param.scan_range);
	ROS_INFO("min_pt: %d", param.min_pt);
	ROS_INFO("max_pt: %d", param.max_pt);
	ROS_INFO("min_size_x: %f", param.min_size[0]);
	ROS_INFO("min_size_y: %f", param.min_size[1]);
	ROS_INFO("min_size_z: %f", param.min_size[2]);
	ROS_INFO("max_size_x: %f", param.max_size[0]);
	ROS_INFO("max_size_y: %f", param.max_size[1]);
	ROS_INFO("max_size_z: %f", param.max_size[2]);
	ROS_INFO("Horizon_SCAN: %d", param.Horizon_SCAN);
	ROS_INFO("N_SCAN: %d", param.N_SCAN);
	ROS_INFO("downsampleRate: %d", param.downsampleRate);
	

}

std::vector<Eigen::VectorXd> trackingFC(std::vector<Detect> dets, float relative_time)
{
	static Tracker tracker(param);
	std::vector<Eigen::VectorXd> result;
	std::vector<Eigen::VectorXd> tracked;
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> result_cloud;
	tracker.track(dets, relative_time, result, result_cloud);

	for (int i = 0; i < result.size(); ++i)
	{
		Eigen::VectorXd r = result[i];
		for (int j = 0; j < dets.size(); ++j)
		{
			if (abs(r(1) - dets[j].position[0]) < 0.5 && abs(r(2) - dets[j].position[1]) < 0.5)
			{
				tracked.push_back(r);
				break;
			}
		}
	}
	return tracked;
}


void pubMarker(std::vector<Eigen::VectorXd> tracked){
	// publish marker
	visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray());
	marker_array->markers.reserve(tracked.size() + 1);
	int marker_id = 0;
	for (auto &r : tracked)
	{
		if (!idcolor.count(int(r(0)))) // r(0)=id
		{
			// 随机生成一个颜色
			int red = rng.uniform(0, 255);
			int green = rng.uniform(0, 255);
			int blue = rng.uniform(0, 255);
			idcolor[int(r(0))] = {red, green, blue};
		}
		visualization_msgs::Marker marker;
		marker.header.stamp = timeLaserInfoStamp;
		marker.header.frame_id = "os_sensor";
		marker.ns = "";
		marker.id = marker_id;
		marker.lifetime = ros::Duration(3.2);
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = r(1);
		marker.pose.position.y = r(2);
		marker.pose.position.z = 0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = r(6);
		marker.scale.y = r(7);
		marker.scale.z = r(8);
		marker.color.r = float(idcolor[int(r(0))][0]) / 255;
		marker.color.g = float(idcolor[int(r(0))][1]) / 255;
		marker.color.b = float(idcolor[int(r(0))][2]) / 255;
		marker.color.a = 0.6f;
		marker_array->markers.push_back(marker); 

		++marker_id;
	}
	pubmarker.publish(marker_array);
}


void detsHandler(const visualization_msgs::MarkerArrayConstPtr &detsMsg)
{
	detsQueue.push_back(*detsMsg);
	visualization_msgs::MarkerArray centerpoint_dets = detsQueue.front();
	detsQueue.pop_front();

	std::vector<Detect> dets;

	for(auto marker : centerpoint_dets.markers){
		Detect det;
		det.position = Eigen::VectorXd(2);
		det.position << marker.pose.position.x, marker.pose.position.y;
		det.box = {float(marker.scale.x), float(marker.scale.y), float(marker.scale.z)};
		det.z = marker.pose.position.z;

		dets.push_back(det);

	}

	timeDetsStamp = centerpoint_dets.markers[0].header.stamp;

	float relative_time;
	if (!init)
	{
		init_time = centerpoint_dets.markers[0].header.stamp.toSec();
		relative_time = 0;
		init = true;
	}
	else
	{
		relative_time = static_cast<float>(centerpoint_dets.markers[0].header.stamp.toSec() - init_time);
	}

	// tracking
	std::vector<Eigen::VectorXd> tracked = trackingFC(dets, relative_time);

	// 发布marker
	pubMarker(tracked);
	ROS_INFO("Tracked %d objects", tracked.size());
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "tracking_mapping_node");
	// Set the logger level
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {ros::console::notifyLoggerLevelsChanged();}

	ros::NodeHandle nh;
	readParams(nh);

	pubmarker = nh.advertise<visualization_msgs::MarkerArray>("/mot_tracking/box", 1);
	subdets   = nh.subscribe<visualization_msgs::MarkerArray>("/centerpoint/dets", 1, detsHandler);

	ros::spin();

	return 0;
}
