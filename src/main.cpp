#include <iostream>
#include <thread>
#include <chrono>
#include <string>

#include "helper.h"
#include "visualization.h"
#include "zigzag.h"
#include "callbacks.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    if (argc < 2) { std::cerr << "Usage: snake_line input.pcd [row_spacing]\n"; return 1; }

    PickingState state;
    state.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>); //setup cloud pointer

    // Load point cloud
    if (pcl::io::loadPCDFile(argv[1], *state.cloud) != 0) {
        std::cerr << "Failed to load " << argv[1] << "\n"; return 1;
    }

    //if we have row spacing argument
    if (argc >= 3) state.row_spacing = std::stof(argv[2]);

    // Setup viewer
    state.viewer.reset(new pcl::visualization::PCLVisualizer("Snake Line"));
    state.viewer->addCoordinateSystem(0.1);
    state.viewer->setBackgroundColor(0,0,0);

    // Color by Z value
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> z_color(state.cloud, "z");
    state.viewer->addPointCloud(state.cloud, z_color, "cloud");
    state.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    // Register callbacks
    state.viewer->registerPointPickingCallback(pointPickingCallback, &state);
    state.viewer->registerKeyboardCallback(keyboardCallback, &state);

    //main loop refresh rate 50ms
    while (!state.viewer->wasStopped()) {
        state.viewer->spinOnce(50);
        std::this_thread::sleep_for(10ms);
    }

    return 0;
}
