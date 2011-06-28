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

// This algorithm is based on "Using Aerial Lidar Data to Segment And Model Buildings" by Oliver Wang

// VTK
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

// ITK
#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkImageRegionConstIterator.h>

// Custom
#include "Helpers.h"
#include "Types.h"
#include "ClosedLoopSimplification.h"

// Boost
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>


int main(int argc, char *argv[])
{
  // Verify arguments
  if(argc < 2)
    {
    std::cerr << "Required arguments: filename [straightnessErrorTolerance]" << std::endl;
    return EXIT_FAILURE;
    }
    
  // Parse arguments
  std::string fileName = argv[1];
  
  float straightnessErrorTolerance = 1.0;
  if(argc == 3)
  {
    std::stringstream ss;
    ss << argv[2];
    ss >> straightnessErrorTolerance;
  }
  
  // Output arguments
  std::cout << "Input: " << fileName << std::endl;
  std::cout << "Straightness error tolerance: " << straightnessErrorTolerance << std::endl;
  
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(fileName.c_str());
  reader->Update();
  
  std::vector<unsigned int> shortestPath = OutlineApproximation(reader->GetOutput(), straightnessErrorTolerance);
  std::cout << "shortestPath has " << shortestPath.size() << " points." << std::endl;
  
  std::cout << "shortestPath:" << std::endl;
  Helpers::OutputVector(shortestPath);

  //Helpers::WritePathAsLines(shortestPath, polydata, "OutlineApproximation.vtp");
  //WriteGraph(g, polydata, "OutlineApproximation.vtp");
  
  //Visualize(polydata, path);
 
  return EXIT_SUCCESS;
}
