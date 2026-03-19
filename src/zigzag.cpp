#include "zigzag.h"
#include "visualization.h"
#include <pcl/features/normal_3d.h>

std::vector<PathPoint> generateZigZagPath(void* cookie, float step_size) {
    auto* state = static_cast<PickingState*>(cookie);
    std::vector<PathPoint> trajectory;

    if (state->picked_points.size() != 4 || state->row_spacing <= 0.f)
        return trajectory;

    // ------------------------------------------------------------------
    // Build a 3-D KD-tree for normal estimation and a *flat* (Z=0) KD-tree
    // for surface projection.  The flat tree searches only in XY, which
    // keeps each zigzag row geometrically straight even when the surface
    // is wavy in Z (see projectToSurfaceXY in helper.cpp for details).
    // ------------------------------------------------------------------
    static pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_3d;
    static pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_2d;
    static pcl::PointCloud<pcl::PointXYZ>::Ptr      flat_cloud;
    static pcl::PointCloud<pcl::Normal>::Ptr         cloud_normals;

    if (!tree_3d) {
        tree_3d.reset(new pcl::search::KdTree<pcl::PointXYZ>);
        tree_3d->setInputCloud(state->cloud);
    }

    if (!tree_2d) {
        buildFlatCloud(state->cloud, flat_cloud, tree_2d);
    }

    if (!cloud_normals) {
        cloud_normals.reset(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(state->cloud);
        ne.setSearchMethod(tree_3d);
        ne.setKSearch(10);
        ne.compute(*cloud_normals);
    }

    // Order quadrilateral corners counter-clockwise: A, B, C, D
    pcl::PointXYZ A_p, B_p, C_p, D_p;
    orderRectanglePoints(state->picked_points, A_p, B_p, C_p, D_p);

    Eigen::Vector3f A(A_p.x, A_p.y, A_p.z);
    Eigen::Vector3f B(B_p.x, B_p.y, B_p.z);
    Eigen::Vector3f C(C_p.x, C_p.y, C_p.z);
    Eigen::Vector3f D(D_p.x, D_p.y, D_p.z);

    float total_width = (D - A).norm();
    int   num_rows    = static_cast<int>(std::ceil(total_width / state->row_spacing));

    bool forward = true;

    for (int i = 0; i <= num_rows; ++i) {
        float t = static_cast<float>(i) / num_rows;  // 0 -> 1 along A->D

        // Interpolate row endpoints along the left (A->D) and right (B->C) edges
        Eigen::Vector3f row_start = A + t * (D - A);
        Eigen::Vector3f row_end   = B + t * (C - B);

        if (!forward) std::swap(row_start, row_end);

        Eigen::Vector3f row_dir    = (row_end - row_start).normalized();
        float           row_length = (row_end - row_start).norm();

        // Sample along the row and snap each sample onto the surface
        for (float s = 0.f; s <= row_length; s += step_size) {
            Eigen::Vector3f pos = row_start + row_dir * s;

            // XY-only projection keeps rows straight on wavy geometry
            pcl::PointXYZ p = projectToSurfaceXY(toPCL(pos), state->cloud, tree_2d);

            // Look up the surface normal at the projected point
            std::vector<int>   idx(1);
            std::vector<float> dist(1);
            Eigen::Vector3f    normal(0, 0, 1);  // fallback: world-up
            if (tree_3d->nearestKSearch(p, 1, idx, dist) > 0) {
                normal = Eigen::Vector3f(
                    cloud_normals->points[idx[0]].normal_x,
                    cloud_normals->points[idx[0]].normal_y,
                    cloud_normals->points[idx[0]].normal_z
                ).normalized();
            }

            trajectory.push_back({p, normal});
        }

        forward = !forward;
    }

    visualizeTrajectory(*state, trajectory);
    return trajectory;
}
