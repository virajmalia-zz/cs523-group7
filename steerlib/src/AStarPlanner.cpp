//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

// Define which heuristic to use: EUC or MAN
#define MODE EUC
#define HWEIGHT 1.0f

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	// Extra Credit Min Heap
	MinHeap::MinHeap()
	{
	}

	void MinHeap::Heapify()
	{
		int length = _vector.size();
		for (int i = length - 1; i >= 0; --i)
		{
			BubbleDown(i);
		}
	}

	void MinHeap::BubbleDown(int index)
	{
		int length = _vector.size();
		int leftChildIndex = 2 * index + 1;
		int rightChildIndex = 2 * index + 2;

		if (leftChildIndex >= length)
			return; //index is a leaf

		int minIndex = index;

		if (_vector[index] > _vector[leftChildIndex])
		{
			minIndex = leftChildIndex;
		}

		if ((rightChildIndex < length) && (_vector[minIndex] > _vector[rightChildIndex]))
		{
			minIndex = rightChildIndex;
		}

		if (minIndex != index)
		{
			//need to swap
			AStarPlannerNode temp = _vector[index];
			_vector[index] = _vector[minIndex];
			_vector[minIndex] = temp;
			BubbleDown(minIndex);
		}
	}

	void MinHeap::BubbleUp(int index)
	{
		if (index == 0)
			return;

		int parentIndex = (index - 1) / 2;

		if (_vector[parentIndex] > _vector[index])
		{
			AStarPlannerNode temp = _vector[parentIndex];
			_vector[parentIndex] = _vector[index];
			_vector[index] = temp;
			BubbleUp(parentIndex);
		}
	}

	void MinHeap::insertKey(AStarPlannerNode newValue)
	{
		int length = _vector.size();
		_vector.push_back(newValue);

		BubbleUp(length);
	}

	AStarPlannerNode MinHeap::extractMin()
	{
		AStarPlannerNode front = _vector[0];
		DeleteMin();
		return front;
	}

	void MinHeap::DeleteMin()
	{
		int length = _vector.size();

		if (length == 0)
		{
			return;
		}

		_vector[0] = _vector[length - 1];
		_vector.pop_back();

		BubbleDown(0);
	}

	bool MinHeap::isEmpty()
	{
		return _vector.empty();
	}


	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
	
		closedList.clear();
		parentList.clear();
		fValues.clear();
		
		int start_index = gSpatialDatabase->getCellIndexFromLocation(start);
		parentList.insert({ start_index ,start_index });

		// Get start and goal point from grid
		start = getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(start));
		goal = getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(goal));

		// Add startPoint to the fringe
		AStarPlannerNode startPoint = AStarPlannerNode(start, 0.0f, computeHValue(start, goal), nullptr);
		fringe.insertKey(startPoint);
		
		// Counter for personal use
		int counter = 0;

		while (!fringe.isEmpty()) 
		{
			AStarPlannerNode curr = fringe.extractMin();

			// Reach Goal
			if (curr.point == goal)
			{
				int index = parentList[gSpatialDatabase->getCellIndexFromLocation(goal)];

				while (parentList[index] != index)
				{
					agent_path.push_back(getPointFromGridIndex(index));
					index = parentList[index];
				}

				agent_path.push_back(start);
				std::reverse(agent_path.begin(), agent_path.end());

				//std::cout << "Path found with " << counter << " expands" << std::endl;
				return true;
			}

			// Expand min from fringe 
			generateNodes(curr, goal);	
			counter++;
		}
		
		//std::cout << "Path not found" << std::endl;
		return false;
	}

	void AStarPlanner::generateNodes(AStarPlannerNode curr, Util::Point goal)
	{
		double diag_cost = std::sqrt(2);

		// Add curr_index to closedList
		int curr_index = gSpatialDatabase->getCellIndexFromLocation(curr.point);
		if (closedList.find(curr_index) != closedList.end())
			return;

		//printf("\nExpanded: (%f, %f): %f\n", curr.point.x, curr.point.z, curr.f);
		closedList.insert(curr_index);

		unsigned int curr_x, curr_z;
		gSpatialDatabase->getGridCoordinatesFromIndex(curr_index, curr_x, curr_z);

		// Set searching range
		int start_x = MAX((curr_x - 1), 0);
		int end_x = MIN((curr_x + 1), gSpatialDatabase->getNumCellsX());

		int start_z = MAX((curr_z - 1), 0);
		int end_z = MIN((curr_z + 1), gSpatialDatabase->getNumCellsZ());

		for (int i = start_x; i <= end_x; i++)
		{
			for (int j = start_z; j <= end_z; j++)
			{
				if (i == curr_x && j == curr_z)
					continue;

				// Check neighbors 
				int nb_index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				if (closedList.find(nb_index) == closedList.end() && canBeTraversed(nb_index))
				{
					// First time reaching this cell
					if (fValues.find(nb_index) == fValues.end())
					{
						fValues.insert({nb_index, 99999.9f});
						parentList.insert({ nb_index, nb_index });
					}

					// Calculate values
					Util::Point nb_pos = getPointFromGridIndex(nb_index);
					double nb_h = computeHValue(nb_pos, goal);

					double nb_g;
					if (((int)(nb_pos.x + nb_pos.z) % 2) == ((int)(curr.point.x + curr.point.z) % 2))
					{
						nb_g = curr.g + diag_cost;
					}
					else
					{
						nb_g = curr.g + 1.0f;
					}

					double nb_f = nb_g + nb_h;

					// Add to fringe
					if (nb_f < fValues[nb_index])
					{
						fValues[nb_index] = nb_f;
						AStarPlannerNode nb_Point = AStarPlannerNode(nb_pos, nb_g, nb_f, &curr);
						fringe.insertKey(nb_Point);
						parentList[nb_index] = curr_index;

						//printf("Fringe Add: (%f, %f) f: %f g: %f\n", nb_pos.x, nb_pos.z, nb_f, nb_g);
					}
				}
			}
		}
	}

	double AStarPlanner::computeHValue(Util::Point curr, Util::Point goal)
	{
		switch (MODE)
		{
		// Euclidean Distance
		case EUC: return (std::sqrt(std::pow(curr.x - goal.x, 2.0) + std::pow(curr.z - goal.z, 2.0))) * HWEIGHT; break;

		// Manhattan Distance
		case MAN: return (std::abs(curr.x - goal.x) + std::abs(curr.z - goal.z)) * HWEIGHT; break;
		}	
	}
}