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

#ifndef HELPERS_H
#define HELPERS_H

#include "Types.h"

#include "itkImage.h"

#include <vector>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace Helpers
{

void WritePoints(vtkPolyData*, std::string filename);
void WritePathAsPolyLine(std::vector<unsigned int> order, vtkPolyData* graphPolyData, std::string filename);
void WritePathAsLines(std::vector<unsigned int> order, vtkPolyData* graphPolyData, std::string filename);

std::vector<unsigned int> GetShortestPath(Graph& g, Graph::vertex_descriptor start, Graph::vertex_descriptor end);
float GetShortestPathDistance(Graph& g, Graph::vertex_descriptor start, Graph::vertex_descriptor end);

float GetDistanceBetweenPoints(vtkPolyData*, vtkIdType a, vtkIdType b);

void OutputVector(std::vector<unsigned int> &v);

} // end namespace Helpers
#endif
