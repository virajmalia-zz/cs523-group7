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
	AStarPlannerNode startNode(start, INT_MAX, 0, INT_MAX, NULL);
	AStarPlannerNode goalNode(goal, INT_MAX, 0, 0, NULL);
	double eps = INFLATION_FACTOR;
	double h = 0;

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

		
		std::priority_queue< AStarPlannerNode, std::queue<AStarPlannerNode>, std::less<AStarPlannerNode> > open, incons;
		
		key(goalNode);
		open.push(goalNode);

		improvePath();

		// Publish current eps-suboptimal solution

		while (1) {
			// Updates




		}
		return false;
	}



	void key(AStarPlannerNode &s) {
		h = sqrt( (s.point - startNode.point)*(s.point - startNode.point) );
		if (s.g > s.rhs) {
			s.f = min(s.rhs + eps*h, s.rhs);
		}
		else {
			s.f = min(s.g + h, s.g);
		}
	}

}