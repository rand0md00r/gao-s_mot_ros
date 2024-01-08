/*
 * @Author: Gaoeee gjy1518@163.com
 * @Date: 2023-05-16 15:22:40
 * @LastEditors: Gaoeee gjy1518@163.com
 * @LastEditTime: 2023-05-16 16:52:10
 * @FilePath: /Multi-Object-Tracking/mot_tracking/src/utility.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef UTILITY_
#define UTILITY_


#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


struct Detect
{
	int classname = 0;
	float z;
	float yaw;
	Eigen::VectorXd position; // x,y
	std::vector<float> box;	  // 3D box in lidar x,y,z
	std::vector<float> box2D; // 2D box in camera left top point and right down point
	cv::RotatedRect rotbox;
	pcl::PointCloud<pcl::PointXYZI>::Ptr objcloud;
};

struct grid_obj
{
    int x;
    int y;
};

#endif
