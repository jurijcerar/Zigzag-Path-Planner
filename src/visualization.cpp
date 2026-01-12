#include "visualization.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include <iostream>

void drawRectangle(void* cookie) {
    auto* state = static_cast<PickingState*>(cookie);
    if (state->picked_points.size() != 4)
        return;

    pcl::PointXYZ a, b, c, d;
    orderRectanglePoints(state->picked_points, a, b, c, d);

    state->viewer->addLine(a, b, 1, 0, 0, "_ab");
    state->viewer->addLine(b, c, 1, 0, 0, "_bc");
    state->viewer->addLine(c, d, 1, 0, 0, "_cd");
    state->viewer->addLine(d, a, 1, 0, 0, "_da");

    for (auto& s : {"_ab", "_bc", "_cd", "_da"})
        state->viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, s);
}

void clearData(void* cookie) {
    auto* state = static_cast<PickingState*>(cookie);

    for (int i = 0; i < 4; ++i)
        state->viewer->removeShape("pick_" + std::to_string(i + 1));

    for (auto& s : {"_ab", "_bc", "_cd", "_da"})
        state->viewer->removeShape(s);

    for (auto& id : state->trajectory_ids)
        state->viewer->removeShape(id);

    state->picked_points.clear();
    state->trajectory_ids.clear();
}
