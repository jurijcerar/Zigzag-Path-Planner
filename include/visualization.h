#pragma once

#include <vector>
#include "helper.h"
#include <vtkSmartPointer.h>
#include <vtkPolyLine.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>

void visualizeTrajectory(PickingState& state, const std::vector<PathPoint>& trajectory);
void drawRectangle(void* cookie);
void clearData(void* cookie);
