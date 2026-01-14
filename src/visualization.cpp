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

    state->viewer->removeShape("trajectory");

    state->picked_points.clear();
    state->trajectory_ids.clear();
}

void visualizeTrajectory(PickingState& state, const std::vector<PathPoint>& trajectory)
{
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto lines = vtkSmartPointer<vtkCellArray>::New();

    auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
    polyLine->GetPointIds()->SetNumberOfIds(trajectory.size());

    for (size_t i = 0; i < trajectory.size(); ++i)
    {
        const auto& p = trajectory[i].position;
        points->InsertNextPoint(p.x, p.y, p.z);
        polyLine->GetPointIds()->SetId(i, i);
    }

    lines->InsertNextCell(polyLine);

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    state.viewer->addModelFromPolyData(polyData, "trajectory");
}

