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

inline pcl::PointXYZ toPCL(const Eigen::Vector3f& v) { return pcl::PointXYZ(v.x(), v.y(), v.z()); }

// Project a query point onto the surface by searching only in the XY plane,
// then returning the full 3D point (including Z) from the cloud.
// This prevents the wavy surface from biasing the nearest-neighbour search
// toward off-axis points and keeps zigzag rows geometrically straight.
pcl::PointXYZ projectToSurfaceXY(const pcl::PointXYZ& p,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_2d);

// Build a flattened (Z=0) copy of the cloud and a KD-tree over it.
// The returned cloud and tree are used exclusively for XY-nearest-neighbour
// queries; index correspondence with the original cloud is preserved.
void buildFlatCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& flat_cloud,
                    pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree_2d);

void saveTrajectoryFile(const std::vector<PathPoint>& trajectory);