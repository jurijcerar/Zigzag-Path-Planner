#include "callbacks.h"
#include "helper.h"
#include <pcl/common/common.h>

void pointPickingCallback(const pcl::visualization::PointPickingEvent &event, void *cookie) {
    auto *state = static_cast<PickingState *>(cookie);

    // Only allow 4 picked points
    if (state->picked_points.size() >= 4) return;

    int idx = event.getPointIndex();
    if (idx == -1) return; // No point was picked

    // Use the exact cloud point at this index
    const auto &picked = state->cloud->points[idx];
    state->picked_points.push_back(picked);

    // Automatically scale the sphere radius based on cloud dimensions
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*state->cloud, min_pt, max_pt);
    float cloud_diag = std::sqrt(
        (max_pt.x - min_pt.x)*(max_pt.x - min_pt.x) +
        (max_pt.y - min_pt.y)*(max_pt.y - min_pt.y) +
        (max_pt.z - min_pt.z)*(max_pt.z - min_pt.z)
    );

    float sphere_radius = cloud_diag * 0.007f; // 0.7% of diagonal length

    // Visualize the picked point as a red sphere
    std::string id = "pick_" + std::to_string(state->picked_points.size());
    state->viewer->addSphere(picked, sphere_radius, 1.0, 0.0, 0.0, id);
}


void keyboardCallback(const pcl::visualization::KeyboardEvent& e, void* cookie) {
    if (!e.keyDown()) return;

    if (e.getKeySym() == "z") {
        //drawRectangle(cookie);
        auto trajectory = generateZigZagPath(cookie, 0.005f);
        saveTrajectoryFile(trajectory);
    }

    if (e.getKeySym() == "c")
        clearData(cookie);
}
