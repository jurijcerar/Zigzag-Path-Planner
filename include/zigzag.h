#pragma once

#include "helper.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <string>

std::vector<PathPoint> generateZigZagPath(void* cookie, float step_size = 0.05f);
