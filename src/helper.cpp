#include "helper.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

// ---------------------------------------------------------------------------
// Centroid / ordering helpers
// ---------------------------------------------------------------------------

Eigen::Vector3f computeCentroid(const std::vector<pcl::PointXYZ>& points) {
    Eigen::Vector3f centroid(0, 0, 0);
    for (const auto& p : points)
        centroid += Eigen::Vector3f(p.x, p.y, p.z);
    centroid /= static_cast<float>(points.size());
    return centroid;
}

void orderRectanglePoints(const std::vector<pcl::PointXYZ>& picked,
                          pcl::PointXYZ& a, pcl::PointXYZ& b,
                          pcl::PointXYZ& c, pcl::PointXYZ& d) {

    Eigen::Vector3f centroid = computeCentroid(picked);

    // Build a local 2-D coordinate frame from the first three points
    Eigen::Vector3f p0(picked[0].x, picked[0].y, picked[0].z);
    Eigen::Vector3f p1(picked[1].x, picked[1].y, picked[1].z);
    Eigen::Vector3f p2(picked[2].x, picked[2].y, picked[2].z);

    Eigen::Vector3f normal = (p1 - p0).cross(p2 - p0).normalized();
    Eigen::Vector3f u = normal.unitOrthogonal();
    Eigen::Vector3f v = normal.cross(u);

    // Sort counter-clockwise by angle in the local plane
    // Reference: https://stackoverflow.com/questions/6880899
    struct Angled { pcl::PointXYZ p; float angle; };
    std::vector<Angled> pts;
    pts.reserve(picked.size());

    for (const auto& p : picked) {
        Eigen::Vector3f d(p.x - centroid.x(), p.y - centroid.y(), p.z - centroid.z());
        pts.push_back({p, std::atan2(d.dot(v), d.dot(u))});
    }

    std::sort(pts.begin(), pts.end(), [](const Angled& x, const Angled& y) {
        return x.angle < y.angle;
    });

    a = pts[0].p;
    b = pts[1].p;
    c = pts[2].p;
    d = pts[3].p;
}

// ---------------------------------------------------------------------------
// Flat-cloud construction
// ---------------------------------------------------------------------------

void buildFlatCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& flat_cloud,
                    pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree_2d) {

    flat_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    flat_cloud->resize(cloud->size());

    // Copy every point but zero out Z so the KD-tree measures only XY distance
    for (size_t i = 0; i < cloud->size(); ++i) {
        flat_cloud->points[i].x = cloud->points[i].x;
        flat_cloud->points[i].y = cloud->points[i].y;
        flat_cloud->points[i].z = 0.0f;
    }

    tree_2d.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    tree_2d->setInputCloud(flat_cloud);
}

// ---------------------------------------------------------------------------
// Surface projection (XY-only nearest neighbour)
// ---------------------------------------------------------------------------

// The key insight: the point cloud surface is wavy in Z, so a naive 3-D
// nearest-neighbour query can pick a neighbour that is close in XY but at a
// different Z-height, or – worse – prefer a point that is far in XY but
// happens to have a similar Z.  Either way the projected path drifts and the
// zigzag rows become wavy.
//
// Fix: flatten both the query point and the cloud to Z = 0, find the nearest
// neighbour in that 2-D sense (using a pre-built flat KD-tree whose indices
// match the original cloud), then return the original 3-D point.  The search
// is now purely planar, so "closest along the row direction" always wins over
// "closest in height".
pcl::PointXYZ projectToSurfaceXY(const pcl::PointXYZ& p,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_2d) {
    // Query with Z = 0 to match the flat cloud
    pcl::PointXYZ query(p.x, p.y, 0.0f);

    std::vector<int>   idx(1);
    std::vector<float> dist(1);
    if (tree_2d->nearestKSearch(query, 1, idx, dist) > 0)
        return cloud->points[idx[0]];  // return the real 3-D point
    return p;
}

// ---------------------------------------------------------------------------
// Trajectory I/O
// ---------------------------------------------------------------------------

void saveTrajectoryFile(const std::vector<PathPoint>& trajectory) {
    std::ofstream out("trajectory.txt", std::ios::out | std::ios::trunc);

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& pt = trajectory[i];
        out << i             << ' '
            << pt.position.x << ' '
            << pt.position.y << ' '
            << pt.position.z << ' '
            << pt.normal.x() << ' '
            << pt.normal.y() << ' '
            << pt.normal.z() << '\n';
    }

    std::cout << "Trajectory saved to trajectory.txt\n";
}