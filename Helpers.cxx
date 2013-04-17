#include "Helpers.h"

// ITK
#include "itkImageRegionConstIterator.h"

// VTK
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkMath.h>
#include <vtkPolyLine.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLPolyDataWriter.h>

// Boost
#include <boost/graph/dijkstra_shortest_paths.hpp>

namespace Helpers
{

std::vector<unsigned int> GetShortestPath(Graph& g, Graph::vertex_descriptor start, Graph::vertex_descriptor end)
{
  //std::cout << "There are " << boost::num_vertices(g) << " vertices in the graph." << std::endl;
  
  // Create things for Dijkstra
  std::vector<Graph::vertex_descriptor> parents(boost::num_vertices(g)); // To store parents
  std::vector<WeightType> distances(boost::num_vertices(g)); // To store distances

  // Compute shortest paths from 'start' to all vertices, and store the output in parents and distances
  boost::dijkstra_shortest_paths(g, start, boost::predecessor_map(&parents[0]).distance_map(&distances[0]));

  // Output distances and parents for inspection
//   std::cout << "distances and parents:" << std::endl;
//   boost::graph_traits < Graph >::vertex_iterator vertexIterator, vend;
//   for (boost::tie(vertexIterator, vend) = boost::vertices(g); vertexIterator != vend; ++vertexIterator) 
//   {
//     std::cout << "distance(" << *vertexIterator << ") = " << distances[*vertexIterator] << ", ";
//     std::cout << "parent(" << *vertexIterator << ") = " << parents[*vertexIterator] << std::endl;
//   }
//   std::cout << std::endl;
  
  // Create a vector in which to store the path
  std::vector<unsigned int> shortestPath;
  
  // Start at the end and work back to the beginning (aka Backtracking algorithm)
  Graph::vertex_descriptor currentVertex = end;
  
  //std::cout << "Starting at " << currentVertex << " and looking for " << start << std::endl;

  /* Work in vertexId space:
  while(parents[currentVertex] != start)
  {
    std::cout << "currentVertex: " << currentVertex << std::endl;
    std::cout << "current parent: " << parents[currentVertex] << std::endl;
    shortestPath.push_back(currentVertex);
    currentVertex = parents[currentVertex];
  }
  
  // The next to last vertex will not be added (one after 'start'), so add it manually
  shortestPath.push_back(currentVertex);
  
  // Add the 'start' vertex to the path
  shortestPath.push_back(start);
  */
  
  // Work in pointId space:
  while(parents[currentVertex] != start)
  {
    shortestPath.push_back(g[currentVertex].PointId);
    currentVertex = parents[currentVertex];
  }
  
  // The next to last vertex will not be added (one after 'start'), so add it manually
  shortestPath.push_back(g[currentVertex].PointId);
  
  // Add the 'start' vertex to the path
  shortestPath.push_back(g[start].PointId);
  
  std::reverse (shortestPath.begin( ), shortestPath.end( ) );
  
  return shortestPath;
}


float GetShortestPathDistance(Graph& g, Graph::vertex_descriptor start, Graph::vertex_descriptor end)
{
  //std::cout << "There are " << boost::num_vertices(g) << " vertices in the graph." << std::endl;
  
  // Create things for Dijkstra
  std::vector<Graph::vertex_descriptor> parents(boost::num_vertices(g)); // To store parents
  std::vector<int> distances(boost::num_vertices(g)); // To store distances

  // Compute shortest paths from 'start' to all vertices, and store the output in parents and distances
  boost::dijkstra_shortest_paths(g, start, boost::predecessor_map(&parents[0]).distance_map(&distances[0]));

  // Output distances and parents for inspection
//   std::cout << "distances and parents:" << std::endl;
//   boost::graph_traits < Graph >::vertex_iterator vertexIterator, vend;
//   for (boost::tie(vertexIterator, vend) = boost::vertices(g); vertexIterator != vend; ++vertexIterator) 
//   {
//     std::cout << "distance(" << *vertexIterator << ") = " << distances[*vertexIterator] << ", ";
//     std::cout << "parent(" << *vertexIterator << ") = " << parents[*vertexIterator] << std::endl;
//   }
//   std::cout << std::endl;

  return distances[end];
}

float GetDistanceBetweenPoints(vtkPolyData* polydata, vtkIdType a, vtkIdType b)
{
  double currentPoint[3];
  double nextPoint[3];
  
  polydata->GetPoint(a, currentPoint);
  polydata->GetPoint(b, nextPoint);
  return sqrt(vtkMath::Distance2BetweenPoints(currentPoint, nextPoint));  
}

void OutputVector(std::vector<unsigned int> &v)
{
  for(unsigned int i = 0; i < v.size(); ++i)
    {
    std::cout << v[i] << " ";
    }
  std::cout << std::endl;
}

void WritePoints(vtkPolyData* polydata, std::string filename)
{
  // Write the points ina polydata to a vtp file
  
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
    vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputData(polydata);
  glyphFilter->Update();
  
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(filename.c_str());
  writer->SetInputData(glyphFilter->GetOutput());
  writer->Write();
}

void WritePathAsPolyLine(std::vector<unsigned int> order, vtkPolyData* graphPolyData, std::string filename)
{
  // Save the order as a PolyLine in a PolyData
  vtkSmartPointer<vtkPolyLine> polyLine = 
    vtkSmartPointer<vtkPolyLine>::New();
  polyLine->GetPointIds()->SetNumberOfIds(order.size());
  for(unsigned int i = 0; i < order.size(); ++i)
    {
    polyLine->GetPointIds()->SetId(i,order[i]);
    //std::cout << "Line point " << i << " is id " << order[i] << std::endl;
    }
    
  // Create a cell array to store the lines in and add the lines to it
  vtkSmartPointer<vtkCellArray> cells = 
      vtkSmartPointer<vtkCellArray>::New();
  cells->InsertNextCell(polyLine);
  
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = 
    vtkSmartPointer<vtkPolyData>::New();
  
  // Add the points to the dataset
  polyData->SetPoints(graphPolyData->GetPoints());
  
  // Add the lines to the dataset
  polyData->SetLines(cells);
  
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(filename.c_str());
  writer->SetInputData(polyData);
  writer->Write();
   
}

void WritePathAsLines(std::vector<unsigned int> order, vtkPolyData* polyData, std::string filename)
{
  // Save the order as a PolyLine in a PolyData
  vtkSmartPointer<vtkCellArray> lines =
    vtkSmartPointer<vtkCellArray>::New();

  for(unsigned int i = 0; i < order.size(); ++i)
    {
    
    vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();

    line->GetPointIds()->SetId(0,order[i]);
    if(i != order.size()-1)
    {
      line->GetPointIds()->SetId(1,order[i+1]);
    }
    else // connect the last point to the first point
    {
      line->GetPointIds()->SetId(1,order[0]);
    }

    lines->InsertNextCell(line);
    }
    
  vtkSmartPointer<vtkPolyData> outputPolyData = 
    vtkSmartPointer<vtkPolyData>::New();
  
  // Add the points to the dataset
  outputPolyData->SetPoints(polyData->GetPoints());
  
  // Add the lines to the dataset
  outputPolyData->SetLines(lines);
    
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(filename.c_str());
  writer->SetInputData(outputPolyData);
  writer->Write();
   
}

} // end namespace Helpers

