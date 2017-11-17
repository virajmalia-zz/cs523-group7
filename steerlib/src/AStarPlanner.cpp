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
#define INFLATION_FACTOR 2

namespace SteerLib
{
	double eps = INFLATION_FACTOR;
	AStarPlannerNode* startNode;
	AStarPlannerNode* goalNode;
	double h = 0;
	//std::priority_queue< AStarPlannerNode, std::vector<AStarPlannerNode>, std::less<AStarPlannerNode> >
	std::vector<AStarPlannerNode*> open, closed, incons;

	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

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
		std::cout << "Starting AD*\n";

		//set values for starting node
		startNode = new AStarPlannerNode(start, DBL_MAX, double(0), NULL);
		startNode->rhs = DBL_MAX;
		eps = 4.0;

		//Push goal node into openlist (default value of rhs = 0)
		AStarPlannerNode* goalNode = new AStarPlannerNode(goal, DBL_MAX, double(0), NULL);
		goalNode->key = key(startNode, goalNode);
		open.push_back(goalNode);
		computeOrImprovePath(open, closed, incons, startNode, goalNode);
		
		//PUBLISH TO AGENT_PATH

		while (true /*TODO time remains*/) {
			ADStarUpdate(agent_path, start, goal, append_to_path, goalNode);
		}

		return true;
	}

	std::tuple<double, double> AStarPlanner::key(AStarPlannerNode* startNode, AStarPlannerNode* node) {
		if (node->g > node->rhs) {
			return{ (node->rhs + 1/* EPS*/ * distanceBetween(startNode->point, node->point)), node->rhs };
		}
		else {
			return{ (node->g + distanceBetween(startNode->point, node->point)), node->g };
		}
	}

	int getLowestFIndex(std::vector<AStarPlannerNode*> set) {
		int returnIndex;
		double bestFValue = DBL_MAX;

		for (int i = 0; i < set.size(); i++) {
			if (set[i]->f <= bestFValue) {
				bestFValue = set[i]->f;
				returnIndex = i;
			}
		}
		return returnIndex;
	}

	bool keyLessthan(AStarPlannerNode* s1, AStarPlannerNode* s2) {

		if (std::get<0>(s1->key) < std::get<0>(s2->key) || (std::get<0>(s1->key) == std::get<0>(s2->key) && (std::get<1>(s1->key) < std::get<1>(s2->key)))) {
			return true;
		}
		return false;
	}

	bool AStarPlanner::canBeTraversed1(Util::Point point) {
		return canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(point));
	}

	bool AStarPlanner::checkAddSuccessor(AStarPlannerNode* parentNode, Util::Point location, std::vector<AStarPlannerNode*> &successors, Util::Point goal) {
		if (!canBeTraversed1(location)) {
			//std::cout << "The location: " << location << " is blocked\n";
			return false;
		}

		//std::cout << "The location: " << location << " can be traversed and is added to the list of successors\n";
		AStarPlannerNode* successorNode = new AStarPlannerNode(location, parentNode->g + distanceBetween(parentNode->point, location), double(0), parentNode);
		successors.push_back(successorNode);
		return true;
	}

	std::vector<AStarPlannerNode*> AStarPlanner::getSuccessors(AStarPlannerNode* currentNode, Util::Point goal) {
		std::vector<AStarPlannerNode*> successors;

		int size = 3;
		int foo[3] = { -1,0,1 };
		for (int a = 0; a < size; a++)
		{
			for (int b = 0; b < size; b++)
			{
				if (foo[a] == 0 && foo[b] == 0)
				{
					continue;
				}
				checkAddSuccessor(currentNode,
					Util::Point(currentNode->point.x + foo[a], currentNode->point.y, currentNode->point.z + foo[b]),
					successors, goal);
			}
		}

		return successors;
	}

	bool AStarPlanner::UpdateState(AStarPlannerNode* startNode, AStarPlannerNode* s, AStarPlannerNode* goal, std::vector<AStarPlannerNode*> openList, std::vector<AStarPlannerNode*> closedList, std::vector<AStarPlannerNode*> inconsList){
		if (std::find(closedList.begin(), closedList.end(), s) == closedList.end())
		{
			s->g = DBL_MAX;
		}
		if (s->point != goal->point)
		{
			std::vector<AStarPlannerNode*> successors = getSuccessors(s, goal->point);

			float minS = DBL_MAX;
			for (int i = 0; i < successors.size(); i++) {
				float f = successors[i]->g + distanceBetween(s->point, successors[i]->point);
				if (f < minS) {
					minS = f;
				}
			}
			s->rhs = minS;
		}
		if (std::find(openList.begin(), openList.end(), s) != openList.end())
		{
			openList.erase(std::find(openList.begin(), openList.end(), s)); // remove curr from openNodes
		}
		if (s->g != s->rhs)
		{
			if (std::find(closedList.begin(), closedList.end(), s) == closedList.end())
			{
				s->key = key(startNode, s);
				openList.push_back(s);
			}
			else
			{
				inconsList.push_back(s);
			}
		}
	}

	void AStarPlanner::computeOrImprovePath(std::vector<AStarPlannerNode*> openList, std::vector<AStarPlannerNode*> closedList, std::vector<AStarPlannerNode*> inconsList, AStarPlannerNode* startNode, AStarPlannerNode* goal) {
		//std::cout << "Inside computeImprovePath" << std::endl;
		int index = getLowestFIndex(openList);
		AStarPlannerNode* currentNode = openList[index];
		while (keyLessthan(currentNode, startNode) || startNode->rhs != startNode->g) {
			openList.erase(openList.begin() + index);
			if (currentNode->g > currentNode->rhs) {
				currentNode->g = currentNode->rhs;
				closedList.push_back(currentNode);

				AStarPlannerNode* traceBack = currentNode->parent;
				while (traceBack != NULL) {
					UpdateState(startNode, traceBack, goal, openList, closedList, inconsList);
					traceBack = traceBack->parent;
				}

			}
			else
			{
				currentNode->g = DBL_MAX;
				AStarPlannerNode* traceBack = currentNode;
				while (traceBack != NULL) {
					UpdateState(startNode, traceBack, goal, openList, closedList, inconsList);
					traceBack = traceBack->parent;
				}

				//for all s' contained in predecessors including s updateState(s')
			}
		}
	}

	bool AStarPlanner::ADStarUpdate(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, bool append_to_path, AStarPlannerNode* goalNode) {
		if (append_to_path) {
			for (AStarPlannerNode * node : open, incons, closed) {
				//UpdateState(node);
			}
			eps *= 2;
		}
		else {
			eps = max(1.0, eps / 2); //halve epsilon, must have minimum value of 1.0
		}
		for (int i = 0; i < incons.size(); i++) {
			AStarPlannerNode* node = incons[i];
			incons.erase(incons.begin() + i);
			open.push_back(node);
		}
		for (AStarPlannerNode* node : open) {
			node->key = key(startNode, node);
		}
		for (int i = 0; i < closed.size(); i++) {
			closed.erase(closed.begin() + i);
		}
		computeOrImprovePath(open, closed, incons, startNode, goalNode);
		//PUBLISH TO AGENT_PATH
		return true;
	}

}