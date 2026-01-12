#pragma once

#include "helper.h"
#include "visualization.h"
#include "zigzag.h"

#include <pcl/visualization/pcl_visualizer.h>

void pointPickingCallback(const pcl::visualization::PointPickingEvent &event, void *cookie);
void keyboardCallback(const pcl::visualization::KeyboardEvent &e, void *cookie);
