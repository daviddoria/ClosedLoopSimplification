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
  if(argc < 4)
    {
    std::cerr << "Required arguments: inputFileName outputFileName straightnessErrorTolerance" << std::endl;
    return EXIT_FAILURE;
    }
    
  // Parse arguments
  std::string inputFileName = argv[1];
  std::string outputFileName = argv[2];
  
  float straightnessErrorTolerance = .0;
  std::stringstream ss;
  ss << argv[3];
  ss >> straightnessErrorTolerance;
  
  // Output arguments
  std::cout << "Input: " << inputFileName << std::endl;
  std::cout << "Output: " << outputFileName << std::endl;
  std::cout << "Straightness error tolerance: " << straightnessErrorTolerance << std::endl;
  
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(inputFileName.c_str());
  reader->Update();
  
  vtkSmartPointer<vtkPolyData> simplifiedContour = vtkSmartPointer<vtkPolyData>::New();
  OutlineApproximation(reader->GetOutput(), straightnessErrorTolerance, simplifiedContour);
 
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFileName.c_str());
  writer->SetInputConnection(simplifiedContour->GetProducerPort());
  writer->Write();
  
  return EXIT_SUCCESS;
}
