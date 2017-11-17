
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
		//std::cout << "\n #############################ARA*################################";
		//return true;
		return ARAStar(agent_path, start, goal, append_to_path);
	}
	bool AStarPlanner::ARAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, bool append_to_path)
	{
		double epsilon = 1000.0;
		double timeLimit = 30.0;
		double decrementFactor = 200.0;

		time_t startTimestamp;
		time_t endTimeStamp;


		startTimestamp=time(NULL);  


		std::vector<SteerLib::AStarPlannerNode*> openList, inconsList, closedList, pathList;

		//Goal state retrieved
		SteerLib::AStarPlannerNode* goalState = new SteerLib::AStarPlannerNode(goal, DBL_MAX, DBL_MAX, NULL);
		

		//Adding start state to openlist 
		openList.push_back(new SteerLib::AStarPlannerNode(start, 0, 0, NULL));

		//Calling ImprovePath subfunction 
		ImprovePath(openList, closedList, inconsList, pathList, goal, goalState, epsilon);
		//std::cout << "\nAfter IP\n";
		if (pathList.empty())
		{
			std::cout << "\nNo path exists\n";
			return false;
		}
		//std::cout << "\nBefore epsilon new\n";
		
		double min1 = (double) (epsilon>goalState->g)? goalState->g: epsilon;
		double min2 = (double) getMin(openList, inconsList, goal);
		//std::cout << "\nMIN1 " << typeid(min2).name() << std::endl;
		//std::cout << "\nMIN2" << min2<<std::endl;
		double epsilonNew = 0.00;
		epsilonNew= min1/min2;
		//std::cout << "\nNew epsilon is " << epsilonNew << std::endl;
		endTimeStamp = time(NULL);
		//std::cout << (timeLimit > difftime(endTimeStamp, startTimestamp)) << std::endl;
		while (epsilonNew > 1 && timeLimit > difftime(endTimeStamp, startTimestamp))
		{
			//std::cout << "\nWhile ep > 1 \n";
			epsilon -= decrementFactor;

			//Checking if epsilon is invalid
			if (epsilon < 1)
				epsilon = 1;

			//Inconsistent to open
			while (!inconsList.empty())
			{
				SteerLib::AStarPlannerNode* node = inconsList.at(0);

				inconsList.pop_back();
				openList.push_back(node);
			}
			//std::cout << "\nFor openlist\n" << std::endl;
			//update priorities for all states (a) in open
			for (int a = 0; a < openList.size(); a++)
			{
				openList.at(a)->f = openList.at(a)->g + epsilon*distanceBetween(openList.at(a)->point, goal);
			}
			
			pathList.clear();
			closedList.clear();
			resetList(openList, start);
			ImprovePath(openList, closedList, inconsList, pathList, goal, goalState, epsilon);
			
			if (epsilon == 1)
			{
				std::cout << "\Optimal solution\n" << std::endl;
				break;
			}
			epsilonNew = min(epsilon, goalState->g) / getMin(openList, inconsList, goal);
			//std::cout << "\nNew epsilon is "<< epsilonNew << std::endl;
		}
		storePath(pathList, agent_path, goal);
		
		return true;

	}
	double AStarPlanner::getMin(std::vector<SteerLib::AStarPlannerNode*> openList,
		std::vector<SteerLib::AStarPlannerNode*> inconsList, Util::Point goal)
	{
		double min = DBL_MAX;

		for (int a = 0; a < openList.size(); a++)
		{
			if ((openList.at(a)->g + Util::distanceBetween(openList.at(a)->point, goal)) < min)
				min = openList.at(a)->g + Util::distanceBetween(openList.at(a)->point, goal);
		}
		for (int a = 0; a < inconsList.size(); a++)
			{
				if ((inconsList.at(a)->g + Util::distanceBetween(inconsList.at(a)->point, goal)) < min)
					min = inconsList.at(a)->g + Util::distanceBetween(inconsList.at(a)->point, goal);
			}
		//std::cout << "\nRETURN"<<min;
		return min;

	}
	double AStarPlanner::getMinFromOpen(std::vector<SteerLib::AStarPlannerNode*> openList, Util::Point goal)
	{
		double min = DBL_MAX;

		for (int a = 0; a < openList.size(); a++)
		{
			if ((openList.at(a)->g + Util::distanceBetween(openList.at(a)->point, goal)) < min)
				min = openList.at(a)->g + Util::distanceBetween(openList.at(a)->point, goal);
		}
		return min;

	}

	bool ifExists(AStarPlannerNode* currentNode, std::vector<AStarPlannerNode*> list) {
		for (int i = 0; i < list.size(); i++) {
			if (currentNode->point == list[i]->point) {
				return true;
			}
		}
		return false;
	}
	void AStarPlanner::storePath(std::vector<SteerLib::AStarPlannerNode*> &pathList, std::vector<Util::Point>& agent_path, Util::Point goal)
	{
		for (int i = pathList.size() - 1; i > 0; i--) {
			agent_path.push_back(pathList[i]->point);
		}
		agent_path.push_back(goal);
	}
	int getLowestFIndex(std::vector<AStarPlannerNode*> list) {
		int returnIndex;
		double bestFValue = DBL_MAX;

		for (int i = 0; i < list.size(); i++) {
			if (list[i]->f <= bestFValue) {
				bestFValue = list[i]->f;
				returnIndex = i;
			}
		}
		return returnIndex;
	}

	bool AStarPlanner::checkSuccessor(AStarPlannerNode* parentNode, Util::Point location, std::vector<AStarPlannerNode*> &successors, Util::Point goal) {
		if (!canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(location))) {
			return false;
		}

		AStarPlannerNode* successorNode = new AStarPlannerNode(location, parentNode->g + distanceBetween(parentNode->point, location), double(0), parentNode);
		successors.push_back(successorNode);
		return true;
	}

	std::vector<AStarPlannerNode*> AStarPlanner::getSuccessors(AStarPlannerNode* currentNode, Util::Point goal) {
		std::vector<AStarPlannerNode*> successors;

		int size = 3;
		int possibilities[3] = { -1,0,1 };
		for (int a = 0; a < size; a++)
		{
			for (int b = 0; b < size; b++)
			{
				if (possibilities[a] == 0 && possibilities[b] == 0)
				{
					continue;
				}
				checkSuccessor(currentNode,
					Util::Point(currentNode->point.x + possibilities[a], currentNode->point.y, currentNode->point.z + possibilities[b]),
					successors, goal);
			}
		}

		return successors;
	}
	void AStarPlanner::ImprovePath(std::vector<SteerLib::AStarPlannerNode*> &openList,
		std::vector<SteerLib::AStarPlannerNode*> &closedList,
		std::vector<SteerLib::AStarPlannerNode*> &inconsList,
		std::vector<SteerLib::AStarPlannerNode*> &pathList, Util::Point goal,
		SteerLib::AStarPlannerNode* &goalState, double epsilon)
	{

		//std::cout << "\nIn improved path!\n";
		while(openList.size() != 0){
			//std::cout <<"\n"<< openList.size();
			//std::cout << "\nWhile in improve";
			//State with lowest F value
			int index = getLowestFIndex(openList);

			AStarPlannerNode* current = openList[index];

			openList.erase(openList.begin() + index);
			closedList.push_back(current);


			if (current->point == goal) {
				std::cout << "Reached goal!\n";
				AStarPlannerNode* traceBack = current;
				while (traceBack != NULL) {
					pathList.push_back(traceBack);
					traceBack = traceBack->parent;
				}

				goalState->g = current->g;
				return;
			}


			std::vector<AStarPlannerNode*> successors = getSuccessors(current, goal);

			for (int i = 0; i < successors.size(); i++) {

				//if successor not visited
				if (!ifExists(successors[i], closedList)) {
					successors[i]->g = DBL_MAX;
				}
				//calc the temporary g 
				double temp = current->g + distanceBetween(current->point, successors[i]->point);
				//adding successors to openList with F values
				if (temp <= successors[i]->g) {
					successors[i]->g = temp;
					successors[i]->f = successors[i]->g + epsilon*distanceBetween(successors[i]->point, goal);
					if (!ifExists(successors[i], openList)) {
						openList.push_back(successors[i]);
					}
					else {
						inconsList.push_back(successors[i]);
					}
				}
			}
		}
	}
	void AStarPlanner::resetList(std::vector<SteerLib::AStarPlannerNode*> &list, Util::Point p)
	{
		list.clear();
		list.push_back(new SteerLib::AStarPlannerNode(p, 0, 0, NULL));
	}
}