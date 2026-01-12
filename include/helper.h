#pragma once

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <vector>

struct PathPoint {
    pcl::PointXYZ position;
    Eigen::Vector3f normal;
};

struct PickingState {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    std::vector<pcl::PointXYZ> picked_points;
    float row_spacing = 0.05f;
    std::vector<std::string> trajectory_ids;
};

Eigen::Vector3f computeCentroid(const std::vector<pcl::PointXYZ>& points);

void orderRectanglePoints(const std::vector<pcl::PointXYZ>& picked, 
                          pcl::PointXYZ& a, pcl::PointXYZ& b, 
                          pcl::PointXYZ& c, pcl::PointXYZ& d);


inline pcl::PointXYZ toPCL(const Eigen::Vector3f& v) { return pcl::PointXYZ(v.x(),v.y(),v.z()); }

pcl::PointXYZ projectToSurface(const pcl::PointXYZ& p,
                               pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);
void saveTrajectoryFile(const std::vector<PathPoint>& trajectory);