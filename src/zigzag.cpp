#include "zigzag.h"
#include <pcl/features/normal_3d.h>

std::vector<PathPoint> generateZigZagPath(void* cookie, float step_size) {
    auto* state = static_cast<PickingState*>(cookie);
    std::vector<PathPoint> trajectory;

    // Need exactly 4 points and positive spacing
    if (state->picked_points.size() != 4 || state->row_spacing <= 0.f)
        return trajectory;

    // Build KD-tree for nearest neighbor search
    static pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    if (!tree) {
        tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(state->cloud);
    }

    // Estimate normals if not done yet
    static pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    if (!cloud_normals) {
        cloud_normals.reset(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
        ne.setInputCloud(state->cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(10);
        ne.compute(*cloud_normals);
    }

    // Order quadrilateral points counter-clockwise: A,B,C,D
    pcl::PointXYZ A_p,B_p,C_p,D_p;
    orderRectanglePoints(state->picked_points,A_p,B_p,C_p,D_p); // reuse function

    Eigen::Vector3f A(A_p.x,A_p.y,A_p.z);
    Eigen::Vector3f B(B_p.x,B_p.y,B_p.z);
    Eigen::Vector3f C(C_p.x,C_p.y,C_p.z);
    Eigen::Vector3f D(D_p.x,D_p.y,D_p.z);

    // Compute the total width along the left edge (A->D)
    float total_width = (D-A).norm();
    int num_rows = static_cast<int>(std::ceil(total_width / state->row_spacing));

    bool forward = true;

    // Loop over rows (interpolating along left/right edges)
    for (int i=0;i<=num_rows;++i)
    {
        float t = float(i)/num_rows; // 0->1 along A->D

        // Interpolate row start and end along left (A->D) and right (B->C) edges
        Eigen::Vector3f row_start = A + t*(D-A);
        Eigen::Vector3f row_end   = B + t*(C-B);

        if (!forward) std::swap(row_start,row_end); // zig-zag direction

        Eigen::Vector3f row_dir = (row_end-row_start).normalized();
        float row_length = (row_end-row_start).norm();

        // Sample points along the row
        for (float s=0.f;s<=row_length;s+=step_size)
        {
            Eigen::Vector3f pos = row_start + row_dir*s;
            pcl::PointXYZ p = projectToSurface(toPCL(pos), tree);

            // Find nearest normal
            std::vector<int> idx(1);
            std::vector<float> dist(1);
            Eigen::Vector3f normal(0,0,1); // fallback
            if (tree->nearestKSearch(p,1,idx,dist) > 0)
            {
                normal = Eigen::Vector3f(
                    cloud_normals->points[idx[0]].normal_x,
                    cloud_normals->points[idx[0]].normal_y,
                    cloud_normals->points[idx[0]].normal_z
                ).normalized();
            }

            trajectory.push_back({p, normal});

            if (trajectory.size()>1)
            {
                const auto& prev = trajectory[trajectory.size()-2];
                std::string id = "traj_" + std::to_string(trajectory.size());
                state->viewer->addLine(prev.position,p,0.8f,0.8f,0.8f,id);
                state->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,2.0,id);
                state->trajectory_ids.push_back(id);
            }
        }

        forward = !forward; // zig-zag direction flips
    }

    return trajectory;
}