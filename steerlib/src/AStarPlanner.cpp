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
	AStarPlannerNode startNode(start, INT_MAX, 0, INT_MAX, NULL);
	AStarPlannerNode goalNode(goal, INT_MAX, 0, 0, NULL);
	double h = 0;
	std::priority_queue< AStarPlannerNode, std::vector<AStarPlannerNode>, std::less<AStarPlannerNode> >
		open, closed, incons;

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
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout<<"\nIn A*";

		/*
		01. g(sstart) = rhs(sstart) = ∞; g(sgoal) = ∞;
		02. rhs(sgoal) = 0;  = 0;
		03. OPEN = CLOSED = INCONS = ∅;
		04. insert sgoal into OPEN with key(sgoal);
		05. ComputeorImprovePath();
		06. publish current  - suboptimal solution;
		07. forever
			08. if changes in edge costs are detected
			09. for all directed edges(u, v) with changed edge costs
			10. Update the edge cost c(u, v);
		11. UpdateState(u);
		12. if significant edge cost changes were observed
			13. increase  or replan from scratch;
		14. else if  > 1
			15. decrease ;
		16. Move states from INCONS into OPEN;
		17. Update the priorities for all s ∈ OPEN according to key(s);
		18. CLOSED = ∅;
		19. ComputeorImprovePath();
		20. publish current  - suboptimal solution;
		21. if  = 1
			22. wait for changes in edge costs;
		*/

		
		key(goalNode);
		open.push(goalNode);

		AStarPlannerNode curr = open.top();
		
		// Improve path
		improvePath(curr);

		// Publish current eps-suboptimal solution

		//Forever
		while (1) {
			// Updates
			if (/*change in edge costs are detected*/) {
				//for all edges with changed costs
				// update edge cost
				//updateState();
			}
			if (/*significant edge cost changes*/) {
				// increase eps
			}
			else if (eps > 1) {
				// decrease eps
			}

			// move states from incon to open

			// min-heapify open

			while (!closed.empty()) {
				closed.pop();
			}

			improvePath(curr);

			// Publish curent eps-suboptimal solution

			if (eps == 1) {
				// wait for changes in edge costs
			}

			////////////////////////////////////////
		}
		return false;
	}

	void improvePath(AStarPlannerNode curr) {
		while (key(curr) < key(startNode) || curr.rhs != curr.g) {
			open.pop();
			if (curr.g > curr.rhs) {
				curr.g = curr.rhs;
				// CLOSED = CLOSED U {s}.........add state to closed queue?

				//for
			}
			else {
				curr.g = INT_MAX;
				//for
			}

		}
	}

	void updateState(AStarPlannerNode state, AStarPlannerNode goal) {
		if (/*State was not visited b4*/) {
			state.g = INT_MAX;
		}

		if (state != goal) {
			state.rhs = min();
		}

		if (/*state in open*/) {
			// remove state from open
		}

		if (state.g != state.rhs) {
			if (/*state is not in closed*/) {
				key(state);
				open.push(state);
			}
			else
				incons.push(state);
		}
	}

	double key(AStarPlannerNode &s) {
		h = sqrt( (s.point - startNode.point)*(s.point - startNode.point) );
		if (s.g > s.rhs) {
			s.f = min(s.rhs + eps*h, s.rhs);
		}
		else {
			s.f = min(s.g + h, s.g);
		}
		return s.f;
	}

}