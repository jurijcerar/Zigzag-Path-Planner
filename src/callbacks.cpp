#include "callbacks.h"
#include "helper.h"

void pointPickingCallback(const pcl::visualization::PointPickingEvent &event, void *cookie) {
    

    auto *state = static_cast<PickingState *>(cookie);
    if (event.getPointIndex() == -1 || state->picked_points.size() >= 4) return; //if no point was picked or already have 4 points
    // Get the picked point and save it
    pcl::PointXYZ p;
    event.getPoint(p.x, p.y, p.z);
    state->picked_points.push_back(p);

    // Visualize the picked point as a red sphere
    std::string id = "pick_" + std::to_string(state->picked_points.size());
    state->viewer->addSphere(p, 0.01, 1, 0, 0, id);
}


void keyboardCallback(const pcl::visualization::KeyboardEvent& e, void* cookie) {
    if (!e.keyDown()) return;

    if (e.getKeySym() == "z") {
        drawRectangle(cookie);
        auto trajectory = generateZigZagPath(cookie, 0.005f);
        saveTrajectoryFile(trajectory);
    }

    if (e.getKeySym() == "c")
        clearData(cookie);
}
