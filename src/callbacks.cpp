#include "callbacks.h"
#include "helper.h"
#include <pcl/common/common.h>

void pointPickingCallback(const pcl::visualization::PointPickingEvent &event, void *cookie) {
    auto *state = static_cast<PickingState *>(cookie);

    // Only allow 4 picked points
    if (state->picked_points.size() >= 4) return;

    int idx = event.getPointIndex();
    if (idx == -1) return; // No point was picked

    const auto &picked = state->cloud->points[idx];
    state->picked_points.push_back(picked);


    std::string id = "pick_" + std::to_string(state->picked_points.size());
    state->viewer->addSphere(picked, 0.02, 1.0, 0.0, 0.0, id);
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
