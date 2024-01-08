/*
 * @Author: Gaoeee gjy1518@163.com
 * @Date: 2023-05-20 19:43:49
 * @LastEditors: Gaoeee gjy1518@163.com
 * @LastEditTime: 2023-05-21 00:42:10
 * @FilePath: /src/Multi-Object-Tracking/mot_tracking/src/gridmapping/gridmapping.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE#o
 */
#ifndef GLOBAL_GRID_MAP_
#define GLOBAL_GRID_MAP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utility.h"

class GlobalMap
{
public:
  GlobalMap(int mapsize_, float resolution_, pcl::PointCloud<pcl::PointXYZI>::Ptr background_, Eigen::Vector2f position_) : mapsize(mapsize_), resolution(resolution_), position(position_), background((new pcl::PointCloud<pcl::PointXYZI>(*background_)))
  {
    originx = mapsize / (2 * resolution);
    originy = mapsize / (2 * resolution);
    cols = mapsize / resolution;
    rows = mapsize / resolution;
  }

  void process()
  {
    // 小车所在栅格
    static Eigen::MatrixXf map_probability=Eigen::MatrixXf::Ones(rows, cols) * 0.5;
    grid_obj grid_robot;
    grid_robot.x = floor((position[0]) / resolution) + originx;
    grid_robot.y = floor((position[1]) / resolution) + originy;
    map_probability(grid_robot.x, grid_robot.y) = 0.1;

    pcl::PointCloud<pcl::PointXYZI> currentpoints;
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_points(new pcl::PointCloud<pcl::PointXYZI>);
    float radius = 0.04;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (SlidingQue.size() == windowsize)
    {
      *final_points = SlidingQue.front();
      SlidingQue.pop();
      kdtree.setInputCloud(final_points);
    }

    for (int i = 0; i < background->points.size(); ++i) // 处理当前帧点云,遍历
    {
      if (background->points[i].z > z_max || background->points[i].z < z_min) // 高度超过阈值的点不要
        continue;
      float dx = background->points[i].x - position[0];
      float dy = background->points[i].y - position[1];
      float range = sqrt(dx * dx + dy * dy);
      // if (range > scan_range) // x米外的点不参与建图 室内10，室外20-30
      //   continue;
      
      if (range < 15) // n米范围点剔除动态点
      {
        bool flag = false;
        currentpoints.push_back(background->points[i]);
        if (final_points->size()) // 滑窗满
        {
          pcl::PointXYZI searchPoint;
          searchPoint.x = background->points[i].x;
          searchPoint.y = background->points[i].y;
          searchPoint.z = background->points[i].z;

          for (unsigned int j = 0; j < final_points->size(); ++j)
          {
            if (abs(searchPoint.x - final_points->points[j].x) < 2 && abs(searchPoint.y - final_points->points[j].y) < 2 && abs(searchPoint.z - final_points->points[j].z) < 2) // 限制视野
            {
              if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0) // 没在前几帧内搜到点
              {
                flag = true;
                break;
              }
              // break; //确定searchPoint.在视野内后停止循环
            }
          }
        }
        if (flag == true)
          continue;
      }

      // 扫描点所在栅格
      grid_obj grid_point;
      grid_point.x = floor((background->points[i].x) / resolution) + originx;
      grid_point.y = floor((background->points[i].y) / resolution) + originy;
      if (grid_point.x >= (rows) || grid_point.y >= (cols)) // 超过地图范围的点不要  TODO 考虑地图扩张
        continue;

      // hit更新
      float p_old = map_probability(grid_point.x, grid_point.y); // 栅格原概率
      float odd_log = log(p_old / (1 - p_old));                  // odd=p/(1-p)
      odd_log = odd_log + hit_log;                               // log(odd_new)=log(odd_old)+hit_log
      float p_new = 1 - 1 / (1 + exp(odd_log));                  // 反解p
      p_new = check(p_new);                                      // 检查概率阈值
      map_probability(grid_point.x, grid_point.y) = p_new;       // 更新概率栅格矩阵   3Dhit后固定


      // 小车和扫描点之间连线对应的栅格 miss更新
      std::vector<grid_obj> visitor; // 连线经过的栅格
      CastRay(grid_robot, grid_point, visitor);

      // miss更新
      for (int i = 0; i < visitor.size(); ++i)
      {
        float p_old = map_probability(visitor[i].x, visitor[i].y); // 栅格原概率

        if (p_old > 0.7) // 写死 不能减
          continue;

        float odd_log = log(p_old / (1 - p_old));            // odd=p/(1-p)
        odd_log = odd_log + miss_log;                        // log(odd_new)=log(odd_old)+miss_log
        float p_new = 1 - 1 / (1 + exp(odd_log));            // 反解p
        p_new = check(p_new);                                // 检查概率阈值
        map_probability(visitor[i].x, visitor[i].y) = p_new; // 更新概率栅格矩阵
      }
    }
    SlidingQue.push(currentpoints);
     map_probability_= map_probability;
  }

  // 检查概率阈值
  float check(float &p)
  {
    float p_checked;
    if (p < probability_min)
    {
      p_checked = probability_min;
      return p_checked;
    }
    else if (p > probability_max)
    {
      p_checked = probability_max;
      return p_checked;
    }
    else
    {
      p_checked = p;
      return p_checked;
    }
  }

  void CastRay(const grid_obj &begin, const grid_obj &end, std::vector<grid_obj> &_visitor)
  {
    if (begin.x == end.x) // 如果连线为一条竖线
    {
      int top = (begin.y > end.y ? begin.y : end.y);
      int bottom = (begin.y < end.y ? begin.y : end.y);
      for (int i = bottom + 1; i < top; ++i)
      {
        grid_obj sub_visitor;
        sub_visitor.x = begin.x;
        sub_visitor.y = i;
        _visitor.push_back(sub_visitor);
      }
      return;
    }
    else
    {
      float dy = end.y - begin.y;
      float dx = end.x - begin.x;
      float k = dy / dx; // 连线斜率  被除数和除数为整型 结果也为整型
      int topx, bottomx, bottomy;
      if (begin.x > end.x)
      {
        topx = begin.x;
        bottomx = end.x;
        bottomy = end.y;
      }
      else
      {
        topx = end.x;
        bottomx = begin.x;
        bottomy = begin.y;
      }
      for (int i = bottomx + 1; i < topx; ++i)
      {
        int sub_visitor_y = floor(bottomy + k * (i - bottomx));
        grid_obj sub_visitor;
        sub_visitor.x = i;
        sub_visitor.y = sub_visitor_y;
        _visitor.push_back(sub_visitor);
      }
      return;
    }
  }

  Eigen::MatrixXf getMapbability()
  {
    return map_probability_;
  }

private:
  float hit_probability = 0.75;
  float miss_probability = 0.34;
  float probability_max = 0.9;
  float probability_min = 0.1;

  float hit_log = log(hit_probability / (1 - hit_probability));
  float miss_log = log(miss_probability / (1 - miss_probability));

  float resolution = 0.1;
  int mapsize = 200;
  float z_max = 0.5;
  float z_min = -0.4;
  float scan_range = 25; // 设定过了
  int windowsize = 10;

  int originx, originy;
  int cols, rows;

  pcl::PointCloud<pcl::PointXYZI>::Ptr background;
  Eigen::MatrixXf map_probability_;
  Eigen::Vector2f position;

  std::queue<pcl::PointCloud<pcl::PointXYZI>> SlidingQue;
};

#endif