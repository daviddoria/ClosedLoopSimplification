/*=========================================================================
 *
 *  Copyright David Doria 2011 daviddoria@gmail.com
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

#ifndef CLOSED_LOOP_SIMPLIFICATION_H
#define CLOSED_LOOP_SIMPLIFICATION_H

#include <vtkPolyData.h>

#include "Types.h"

void Visualize(vtkPolyData* graph, vtkPolyData* path);

void OutlineApproximation(vtkPolyData* inputContour, float straightnessErrorTolerance, vtkPolyData* simplifiedContour);

void WriteGraph(Graph& g, vtkPolyData* polydata, std::string filename);

float StraightnessError(Graph g, vtkPolyData* points, unsigned int start, unsigned int end);

std::vector<unsigned int> GetShortestClosedLoop(Graph& g);

#endif
