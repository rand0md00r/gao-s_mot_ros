/*
 * @Author: Gaoeee gjy1518@163.com
 * @Date: 2023-05-11 17:04:56
 * @LastEditors: Gaoeee gjy1518@163.com
 * @LastEditTime: 2023-05-20 18:55:53
 * @FilePath: /Multi-Object-Tracking/mot_tracking/src/detecting/detector.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "detector.h"

std::vector<Detect> Detector::detect(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
  // euclidean clustering:
  std::vector<Detect> dets;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> object_clouds;

  generateDepthImage(cloud);

  labelComponents(cloud); // 聚类打标签

  for (auto cluster_vec : clusters_indices) // 遍历每个聚类
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*cloud, cluster_vec, *cluster_points); // 提取聚类点

    // 坐标转换
    int intensity = rand() % 255;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_points_trans(new pcl::PointCloud<pcl::PointXYZI>());
    for (auto &p : *cluster_points)
    {
      Eigen::Vector2f po = Eigen::Vector2f(p.x, p.y);
      Eigen::Vector2f p_trans = rot * po + trans;
      pcl::PointXYZI point;
      point.x = p_trans[0];
      point.y = p_trans[1];
      point.z = p.z;
      point.intensity = intensity;
      cluster_points_trans->push_back(point);
    }

    if (cluster_vec.size() < min_pt)  
    {
      continue;
    }
    else if (cluster_vec.size() > max_pt)
    {
      *background_cloud += *cluster_points_trans;
      background_indices.insert(background_indices.end(), cluster_vec.begin(), cluster_vec.end());
      continue;
    }

    float minx = cluster_points_trans->at(0).x;
    float miny = cluster_points_trans->at(0).y;
    float minz = cluster_points_trans->at(0).z;
    float maxx = cluster_points_trans->at(0).x;
    float maxy = cluster_points_trans->at(0).y;
    float maxz = cluster_points_trans->at(0).z;

    for (auto &p : *cluster_points_trans)
    {
      minx = p.x < minx ? p.x : minx;
      miny = p.y < miny ? p.y : miny;
      minz = p.z < minz ? p.z : minz;

      maxx = p.x > maxx ? p.x : maxx;
      maxy = p.y > maxy ? p.y : maxy;
      maxz = p.z > maxz ? p.z : maxz;
    }

    float sizel = (maxx - minx) > (maxy - miny) ? (maxx - minx) : (maxy - miny);
    float sizew = (maxx - minx) < (maxy - miny) ? (maxx - minx) : (maxy - miny);
    float sizeh = maxz - minz;

    // if (sizel > min_size[0] && sizel < max_size[0]  && sizeh > min_size[2] && sizeh < max_size[2] && minz < 0)
    if (sizel > min_size[0] && sizel < max_size[0] && sizew > min_size[1] && sizew < max_size[1] && sizeh > min_size[2] && sizeh < max_size[2] && minz < 0 && maxz < 1.2)
    // if (sizeh > min_size[2] && sizeh < max_size[2] && minz < 0)
    {
      Detect det;
      det.position = Eigen::VectorXd(2);
      det.position << (maxx + minx) / 2.0, (maxy + miny) / 2.0;
      det.objcloud = cluster_points_trans;
      det.box = {maxx - minx, maxy - miny, maxz - minz};
      det.z = (maxz + minz) / 2.0f;

      dets.push_back(det);

      *object_cloud += *cluster_points_trans;
      object_indices.push_back(cluster_vec);
      object_cloud_vec.push_back(cluster_points_trans);
    }
    else
    {
      *background_cloud += *cluster_points_trans;
      background_indices.insert(background_indices.end(), cluster_vec.begin(), cluster_vec.end());
    }
  }
  return dets;
}

void Detector::generateDepthImage(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
  auto cloud_size = cloud->points.size();

  for (int i = 0; i < cloud_size; i++)
  {
    const pcl::PointXYZI &point = cloud->points[i];

    int row_index, col_index;
    if (!calculateCoordinate(point, row_index, col_index)) // 投影
    {
      continue;
    }
    float depth = sqrt(point.x * point.x + point.y * point.y + point.z * point.z); //[2,104]
    // if (depth < 1 || depth > 150)   //TODO
    // { // delete too long or too short points
    //   continue;
    // }

    depth_image[row_index][col_index].depth_ = depth;
    depth_image[row_index][col_index].index_ = i;
  }
}

bool Detector::calculateCoordinate(const pcl::PointXYZI &point, int &row, int &col)
{
  float horizon_angle = atan2(point.x, point.y) * 180 / M_PI; //[-180,180]
  int row_index = point.intensity;
  int col_index = -round((horizon_angle) / horizontal_resolution) + image_cols / 2.0;

  if (col_index >= image_cols)
    col_index -= image_cols;
  if (col_index < 0 || col_index >= image_cols)
  {
    return false;
  }
  row = row_index;
  col = col_index;
  return true;
}

void Detector::labelComponents(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
  // std::vector<std::pair<int8_t, int8_t>> neighbor = {{1, 0},
  //                                                    {1, 1},
  //                                                    {1, -1},
  //                                                    {-1, 0},
  //                                                    {-1, -1},
  //                                                    {-1, 1},
  //                                                    {0, 1},
  //                                                    {0, -1}};
  std::vector<std::pair<int8_t, int8_t>> neighbor = {{1, 0},
                                                   {-1, 0},
                                                   {0, 1},
                                                   {0, -1}};

  int label_val = 1;
  int rows = image_rows;
  int cols = image_cols;
  const double eps = 1.0e-6; //

  std::vector<std::vector<int>> clusters_indices_vec;
  // clock_t time_start = clock();
  for (int row = 0; row < rows; row++)
  {
    for (int col = 0; col < cols; col++)
    {
      if (label_image[row][col] == -1 && depth_image[row][col].depth_ > 0)
      { // has been labeled or ground points
        std::queue<std::pair<int, int>> q;
        q.push(std::make_pair(row, col));
        label_image[row][col] = label_val;

        std::vector<int> cluster_indices_vec;
        cluster_indices_vec.push_back(depth_image[row][col].index_);

        while (!q.empty())
        { // 广度优先搜索
          auto target_point = q.front();
          q.pop();
          for (const auto &neigh : neighbor)
          {
            int x = target_point.first + neigh.first;
            int y = target_point.second + neigh.second;
            std::pair<int, int> neigh_point = std::make_pair(x, y);

            if (warpPoint(neigh_point) && judgmentCondition(target_point, neigh_point) &&
                label_image[neigh_point.first][neigh_point.second] == -1 && depth_image[neigh_point.first][neigh_point.second].depth_ > 0)
            { // valid point
              q.push(neigh_point);
              label_image[neigh_point.first][neigh_point.second] = label_val;
              cluster_indices_vec.push_back(depth_image[neigh_point.first][neigh_point.second].index_);
            }
          }
        }
        label_val++;
        if (cluster_indices_vec.size() > min_pt)
        {
          clusters_indices_vec.push_back(cluster_indices_vec);
        }
      }
    }
  }

  // clock_t time_end = clock();
  // auto time_used = 1000.0*double(time_end - time_start)/(double) CLOCKS_PER_SEC;
  // cout << "Cluster Algorithm used time: " << time_used << " ms" << endl;

  auto total_clusters_size = clusters_indices_vec.size();
  clusters_indices = clusters_indices_vec;
}

// bool Detector::judgmentCondition(const std::pair<int, int> &target_point, const std::pair<int, int> &neigh_point)
// {
//   float distance_sum = fabs(depth_image[target_point.first][target_point.second].depth_ - depth_image[neigh_point.first][neigh_point.second].depth_);
//   return distance_sum < 0.5;
// }

bool Detector::judgmentCondition(const std::pair<int, int> &target_point, const std::pair<int, int> &neigh_point)
{
  float res;
  if(target_point.first==neigh_point.first) //同一条线上
   res=horizontal_resolution;
   else
   res=36.6/(image_rows-1);

   float d1= std::max(depth_image[neigh_point.first][neigh_point.second].depth_,  depth_image[target_point.first][target_point.second].depth_);
   float d2= std::min(depth_image[neigh_point.first][neigh_point.second].depth_,  depth_image[target_point.first][target_point.second].depth_);
   float angle=atan2(d2*sin(res*M_PI/180),(d1 -d2*cos(res*M_PI/180)));
  return angle>10*M_PI/180;
}

bool Detector::warpPoint(std::pair<int, int> &pt)
{
  if (pt.first < 0 || pt.first >= image_rows)
    return false;
  if (pt.second < 0)
    pt.second += image_cols;
  if (pt.second >= image_cols)
    pt.second -= image_cols;
  return true;
}
