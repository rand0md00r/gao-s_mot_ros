/*
 * @Author: Gaoeee gjy1518@163.com
 * @Date: 2023-05-11 17:05:19
 * @LastEditors: Gaoeee gjy1518@163.com
 * @LastEditTime: 2023-05-20 17:56:16
 * @FilePath: /Multi-Object-Tracking/mot_tracking/src/detecting/detector.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef DETECTOR_
#define DETECTOR_

#include <iostream>
#include <vector>
#include <queue>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "dp_means.hpp"
#include "pointinfo.h"
#include "utility.h"

// struct Detect
// {
// 	int classname = 0;
// 	float z;
// 	float yaw;
// 	Eigen::VectorXd position; // x,y
// 	std::vector<float> box;	  // 3D box in lidar x,y,z
// 	std::vector<float> box2D; // 2D box in camera left top point and right down point
// 	cv::RotatedRect rotbox;
// 	pcl::PointCloud<pcl::PointXYZI>::Ptr objcloud;
// };

class Detector
{
public:
	Detector(int min_pt_, int max_pt_, const std::vector<float> &min_size_, const std::vector<float> &max_size_, int Horizon_SCAN, int N_SCAN_, int downsampleRate_)
		: min_pt(min_pt_), max_pt(max_pt_), min_size(min_size_), max_size(max_size_), image_cols(Horizon_SCAN),
		  background_cloud(new pcl::PointCloud<pcl::PointXYZI>()), object_cloud(new pcl::PointCloud<pcl::PointXYZI>())
	{
		image_rows = N_SCAN_ / downsampleRate_;
		horizontal_resolution = 360.0 / Horizon_SCAN;

		depth_image.assign(image_rows, std::vector<PointInfo>(image_cols, PointInfo(-1, -1)));
		label_image.assign(image_rows, std::vector<int>(image_cols, -1));
		clusters_indices.clear();
		background_indices.clear();
		object_indices.clear();
	}

	std::vector<Detect> detect(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

	void generateDepthImage(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

	bool calculateCoordinate(const pcl::PointXYZI &point, int &row, int &col);

	void labelComponents(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

	bool judgmentCondition(const std::pair<int, int> &target_point, const std::pair<int, int> &neigh_point);

	bool warpPoint(std::pair<int, int> &pt);

	void setTransAndRot(const Eigen::Vector2f &trans_, const Eigen::Matrix2f &rot_)
	{
		trans = trans_;
		rot = rot_;
	}

	std::vector<int> getMergedClustersIndex()
	{
		std::vector<int> res_resturn;
		for (auto i : clusters_indices)
		{
			res_resturn.insert(res_resturn.end(), i.begin(), i.end());
		}
		return res_resturn;
	}

	std::vector<std::vector<int>> getClustersIndex()
	{
		return clusters_indices;
	}

	pcl::PointCloud<pcl::PointXYZI> getobject()
	{
		return *object_cloud;
	}
	pcl::PointCloud<pcl::PointXYZI> getbackground()
	{
		return *background_cloud;
	}

	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> getobjectVec()
	{
		return object_cloud_vec;
	}

	std::vector<int> getBackgroundIndex()
	{
		return background_indices;
	}

	std::vector<std::vector<int>> getObjectIndex()
	{
		return object_indices;
	}

private:
	int min_pt;
	int max_pt;
	std::vector<float> min_size;
	std::vector<float> max_size;

	int image_cols;
	int image_rows;
	int N_SCAN;
	int downsampleRate;
	float horizontal_resolution;

	pcl::PointCloud<pcl::PointXYZI>::Ptr background_cloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr object_cloud;

	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> object_cloud_vec;

	std::vector<std::vector<PointInfo>> depth_image;
	std::vector<std::vector<int>> label_image;

	std::vector<std::vector<int>> clusters_indices;
	std::vector<int> background_indices;
	std::vector<std::vector<int>> object_indices;

	Eigen::Vector2f trans;
	Eigen::Matrix2f rot;
};

#endif