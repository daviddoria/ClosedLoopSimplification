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

// VTK
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0, 0, 0);
  points->InsertNextPoint(1, 0, 0);
  points->InsertNextPoint(1.3, 0.5, 0);
  points->InsertNextPoint(1, 1, 0);
  points->InsertNextPoint(0, 1, 0);
 
  // Create a cell array to store the lines in and add the lines to it
  vtkSmartPointer<vtkCellArray> lines =
    vtkSmartPointer<vtkCellArray>::New();
 
  for(unsigned int i = 0; i < points->GetNumberOfPoints() - 1; i++)
    {
    //Create the first line (between Origin and P0)
    vtkSmartPointer<vtkLine> line =
      vtkSmartPointer<vtkLine>::New();
    line->GetPointIds()->SetId(0,i);
    line->GetPointIds()->SetId(1,i+1);
    std::cout << "Line between " << i << " and " << i + 1 << std::endl;
    lines->InsertNextCell(line);
    }
    
  // Close the loop
  vtkSmartPointer<vtkLine> line =
    vtkSmartPointer<vtkLine>::New();
  line->GetPointIds()->SetId(0,points->GetNumberOfPoints()-1);
  line->GetPointIds()->SetId(1,0);
  lines->InsertNextCell(line);
 
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  polydata->SetLines(lines); 
  
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("StartingPointDemoGraph.vtp");
  writer->SetInputData(polydata);
  writer->Write();
  
  return EXIT_SUCCESS;
}
