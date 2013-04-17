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

// Custom
#include "ClosedLoopSimplification.h"
#include "Helpers.h"

// VTK
#include <vtkCellArray.h>
#include <vtkGraphToPolyData.h>
#include <vtkKdTreePointLocator.h>
#include <vtkLine.h>
#include <vtkMath.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkProperty.h>
#include <vtkXMLPolyDataWriter.h>

void OutlineApproximation(vtkPolyData* inputContour, float straightnessErrorTolerance, vtkPolyData* simplifiedContour)
{
  // Create a graph from the input contour
  Graph g;
  
  // We must add vertices from the original rough outline twice (the reason for this is explained later)!
  for(unsigned int loopCounter = 0; loopCounter < 2; ++loopCounter)
    {
    inputContour->GetLines()->InitTraversal();
    vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();
    while(inputContour->GetLines()->GetNextCell(idList))
      {
      Graph::vertex_descriptor v = boost::add_vertex(g);
      g[v].PointId = idList->GetId(0);
      }
    }
    
  // Add weighted edges between adjacent vertices
  for(unsigned int i = 0; i < boost::num_vertices(g) - 1; ++i)
    {
    unsigned int currentVertexId = i;
    unsigned int nextVertexId = i+1;
  
    float distance = Helpers::GetDistanceBetweenPoints(inputContour, g[currentVertexId].PointId, g[nextVertexId].PointId);

    EdgeWeightProperty weight(distance);
    boost::add_edge(currentVertexId, nextVertexId, weight, g);
    //std::cout << "Added edge between vertices " << currentVertexId << " and " << nextVertexId
	//      << " which corresponds to points " << g[currentVertexId].PointId << " and " << g[nextVertexId].PointId << std::endl;
    }
  /*
  // Close the second loop
  boost::add_edge(boost::num_vertices(g) - 1, 0, 
		  Helpers::GetDistanceBetweenPoints(inputContour, g[boost::num_vertices(g) - 1].PointId, g[0].PointId), g);
  */
  // This is just to ensure we built the graph properly - it should be identical to the inputContour
  WriteGraph(g, inputContour, "OutlineGraph.vtp");
  
  // Add all other edges which pass the straightness test
  // We loop over all possible 'start' and 'end' vertices and determine if their
  // edge meets the requirements to be added to the graph
  for(unsigned int start = 0; start < boost::num_vertices(g); ++start)
    {
    for(unsigned int end = start+1; end < boost::num_vertices(g); ++end)
      {
      float error = StraightnessError(g, inputContour, start, end);
      if(error < straightnessErrorTolerance)
	{
	// Add an edge between start and end
	double startPoint[3];
	double endPoint[3];
	
	inputContour->GetPoint(g[start].PointId, startPoint);
	inputContour->GetPoint(g[end].PointId, endPoint);
	float distance = sqrt(vtkMath::Distance2BetweenPoints(startPoint, endPoint));

	EdgeWeightProperty weight(distance);
	boost::add_edge(start, end, weight, g);
	//std::cout << "Added edge between " << g[start].PointId << " and " << g[end].PointId << std::endl;
	}
      }
    }

  // This graph shows all of the edges in the graph that the shortest path operations are performed on
  WriteGraph(g, inputContour, "StraigtnessGraph.vtp");

  std::vector<unsigned int> approximateOutline = GetShortestClosedLoop(g);
  
  // Create a polydata contour of the approximate outline
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
  for(unsigned int i = 0; i < approximateOutline.size() - 1; ++i)
    {
    vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
 
    line->GetPointIds()->SetId(0,approximateOutline[i]);
    line->GetPointIds()->SetId(1,approximateOutline[i+1]);
    lines->InsertNextCell(line);
    
    //std::cout << "Adding line between " << approximateOutline[i] << " and " << approximateOutline[i+1] << std::endl;
    }
   
  // Add the points and lines to the output
  simplifiedContour->SetPoints(inputContour->GetPoints());
  simplifiedContour->SetLines(lines);
}

float StraightnessError(Graph g, vtkPolyData* points, unsigned int startId, unsigned int endId)
{
  // This function finds the sum of the distances from each point between vertices[startId] and vertices[endId]
  // to the line formed between order[start] and order[end].
  
  double startPoint[3];
  points->GetPoint(g[startId].PointId, startPoint);
  
  double endPoint[3];
  points->GetPoint(g[endId].PointId, endPoint);
  
  float totalDistance = 0.;
  
  unsigned int numberOfPoints = 0;
  for(unsigned int i = startId+1; i < endId; ++i)
    {
    double currentPoint[3];
    points->GetPoint(g[i].PointId, currentPoint);
  
    totalDistance += vtkLine::DistanceToLine(currentPoint, startPoint, endPoint);
    numberOfPoints++;
    }
  
  //return distance; // sum, as in original paper
  return totalDistance/static_cast<float>(numberOfPoints); // average, makes more sense
}


std::vector<unsigned int> GetShortestClosedLoop(Graph& g)
{
  unsigned int numberOfPoints = boost::num_vertices(g)/2;
  //std::cout << "boost::num_vertices(g)/2 = " << numberOfPoints << std::endl;
  
  float shortestPathDistance = std::numeric_limits<float>::max();
  std::vector<unsigned int> shortestPath;
  
  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    float distance = Helpers::GetShortestPathDistance(g, i, i + numberOfPoints);
    //std::cout << "Distance between " << i << " and " << i + numberOfPoints << " is " << distance << std::endl;
  
    if(distance < shortestPathDistance)
      {
      shortestPath = Helpers::GetShortestPath(g, i, i + numberOfPoints);
      }
    }
    
  return shortestPath;
}

void WriteGraph(Graph& g, vtkPolyData* points, std::string filename)
{
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    
  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
  IndexMap index = get(boost::vertex_index, g);
  
  typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
  std::pair<edge_iter, edge_iter> edgePair;
  for(edgePair = boost::edges(g); edgePair.first != edgePair.second; ++edgePair.first)
    {
    vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
    //unsigned int source = index[boost::source(*edgePair.first, g)];
    //unsigned int target = index[boost::target(*edgePair.first, g)];
    
    unsigned int sourcePointId = g[boost::source(*edgePair.first, g)].PointId;
    unsigned int targetPointId = g[boost::target(*edgePair.first, g)].PointId;

    line->GetPointIds()->SetId(0,sourcePointId);
    line->GetPointIds()->SetId(1,targetPointId);
    lines->InsertNextCell(line);
    
    //std::cout << "Adding line between " << sourcePointId << " and " << targetPointId << std::endl;
    }
  
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = 
    vtkSmartPointer<vtkPolyData>::New();
  
  // Add the points to the dataset
  polyData->SetPoints(points->GetPoints());
  
  // Add the lines to the dataset
  polyData->SetLines(lines);
  
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(filename.c_str());
  writer->SetInputData(polyData);
  writer->Write();
   
}
