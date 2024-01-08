/*
 * @Author: Gaoeee gjy1518@163.com
 * @Date: 2023-05-20 19:43:49
 * @LastEditors: Gaoeee gjy1518@163.com
 * @LastEditTime: 2023-05-21 00:40:35
 * @FilePath: /src/Multi-Object-Tracking/mot_tracking/src/gridmapping/gridmapping.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE#o
 */
#ifndef LOCAL_GRID_MAP_
#define LOCAL_GRID_MAP_

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

class LocalMap
{
public:
    LocalMap(int mapsize_, float resolution_, pcl::PointCloud<pcl::PointXYZI>::Ptr background_, pcl::PointCloud<pcl::PointXYZI>::Ptr static_, pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_) : mapsize(mapsize_), resolution(resolution_),
                                                                                                                                                                                               background((new pcl::PointCloud<pcl::PointXYZI>(*background_))),
                                                                                                                                                                                               staticc((new pcl::PointCloud<pcl::PointXYZI>(*static_))), dynamic((new pcl::PointCloud<pcl::PointXYZI>(*dynamic_)))
    {
        originx = mapsize / (2 * resolution);
        originy = mapsize / (2 * resolution);
        cols = mapsize / resolution;
        rows = mapsize / resolution;
        map_probability = Eigen::MatrixXf::Ones(rows, cols) * 0.5;
        map_state = Eigen::MatrixXi::Zero(rows, cols);
    }

    void process()
    {
        map_probability(originx, originy) = 0.1;
        processcloud(background,1);
        processcloud(staticc,2);
        processcloud(dynamic,3);
    }

    void processcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int state)
    {
        for (int i = 0; i < cloud->points.size(); ++i) // 处理当前帧点云,遍历
        {
            if (cloud->points[i].z > z_max || cloud->points[i].z < z_min) // 高度超过阈值的点不要
                continue;
            float dx = cloud->points[i].x;
            float dy = cloud->points[i].y;
            //   float range = sqrt(dx * dx + dy * dy);
            // if (range > scan_range) // x米外的点不参与建图 室内10，室外20-30
            //   continue;

            // 扫描点所在栅格
            grid_obj grid_point;
            grid_point.x = floor((cloud->points[i].x) / resolution) + originx;
            grid_point.y = floor((cloud->points[i].y) / resolution) + originy;
            if (grid_point.x >= (rows) || grid_point.y >= (cols)) // 超过地图范围的点不要  TODO 考虑地图扩张
                continue;

            // hit更新
            float p_old = map_probability(grid_point.x, grid_point.y); // 栅格原概率
            float odd_log = log(p_old / (1 - p_old));                  // odd=p/(1-p)
            odd_log = odd_log + hit_log;                               // log(odd_new)=log(odd_old)+hit_log
            float p_new = 1 - 1 / (1 + exp(odd_log));                  // 反解p
            p_new = check(p_new);                                      // 检查概率阈值
            map_probability(grid_point.x, grid_point.y) = p_new;       // 更新概率栅格矩阵   3Dhit后固定
            map_state(grid_point.x, grid_point.y)=state;

            // 小车和扫描点之间连线对应的栅格 miss更新
            std::vector<grid_obj> visitor; // 连线经过的栅格
            grid_obj grid_robot;
            grid_robot.x = originx;
            grid_robot.y = originy;
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
        return map_probability;
    }

    Eigen::MatrixXi getMapstate()
    {
        return map_state;
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
    float z_max = 0.7;
    float z_min = -0.4;
    float scan_range = 25; // 设定过了
    int windowsize = 10;

    int originx, originy;
    int cols, rows;

    pcl::PointCloud<pcl::PointXYZI>::Ptr background;
    pcl::PointCloud<pcl::PointXYZI>::Ptr staticc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic;
    Eigen::MatrixXf map_probability;
    Eigen::MatrixXi map_state;
    Eigen::Vector2f position;
};

#endif