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
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout<<"\nIn A*";

		if (edgeCostChanges == 0) {

			//3. OPEN=CLOSED=INCONS=empty
			open.clear();
			closed.clear();
			incons.clear();
			
			//4. insert s_goal into OPEN with key(s_goal)
			goalNode.key = key(&goalNode);
			//std::cout << "goalNode->g: " << goalNode.g << std::endl;
			//std::cout << "goalNode->rhs: " << goalNode.rhs << std::endl;
			//std::cout << "goalNode->h(s_start,goal): " << euclidean_distance(startNode.point, goalNode.point) << std::endl;
			//std::cout << "goalNode->key[0]: " << goalNode.key[0] << ". goalNode->key[1]: " << goalNode.key[1] << std::endl;
			open.push_back(&goalNode);
			
			//5.computeorImprovePath()
			computeShortestPathAD();	// LG

			//6.publish current w-suboptimal solution
			agent_path = trace(&goalNode);	// LG
			return true;
		}

		//7. forever
		while (1) {

			//8. if changes in edge costs are detected
			if (edgeCostChanges == 1) {

			}


			//12. if significant edge cost changes were observed


			//14. else if w>1
			else if (eps > 1) {
				eps = eps - 0.5;
			}

			//16. move states from INCONS into OPEN
			for (int i = 0; i < incons.size(); i++) {
				open.push_back(incons[i]);
			}
			incons.clear();

			//17. update the priorities for all s in open according to key(s)
			for (int i = 0; i < open.size(); i++) {
				open[i]->key = key(open[i]);
			}

			//18.CLOSED=empty
			closed.clear();

			//19.computeorImprovePath()
			computeShortestPathAD();

			//20. publish current solution
			agent_path = trace(&goalNode);

			//21. if w=1
			if (eps == 1) {

				//22. wait for changes in edge costs
				return true;
			}
		}


		return true;
	}

	std::vector<Util::Point> AStarPlanner::trace(AStarPlannerNode* node) {
		std::vector<Util::Point> trace;
		AStarPlannerNode* temp = node;
		trace.push_back(temp->point);
		//std::cout << temp->point << std::endl;
		while (temp->parent != NULL) {
			temp = temp->parent;
			trace.push_back(temp->point);
			//std::cout << temp->point << std::endl;
		}
		std::vector<Util::Point> traceTemp;
		for (int i = 0; i < trace.size(); i++) {

			//traceTemp[trace.size() - 1 - i] = trace[i];
			traceTemp.push_back(trace[trace.size() - 1 - i]);
		}
		//std::cout << "trace:---------------------" << traceTemp << std::endl;
		return traceTemp;
	}

	double AStarPlanner::euclidean_distance(Util::Point a, Util::Point b) {
		return std::sqrt(
			(a.x - b.x) * (a.x - b.x) +
			(a.y - b.y) * (a.y - b.y) +
			(a.z - b.z) * (a.z - b.z)
		);
	}

	bool AStarPlanner::KeyAlessthanB(AStarPlannerNode *s, AStarPlannerNode *s2) {
		//std::cout << " 14.----------open_list-------" << std::endl;
		return ((s->key)[0] < (s2->key)[0]) || (((s->key)[0] == (s2->key)[0]) && (s->key)[1] < (s2->key)[1]);
	}

	void AStarPlanner::computeShortestPathAD() {
		//7. while (min of s in OPEN) (key(s))<key(s_star) OR rhs(s_start) not equal to g(s_start)
		AStarPlannerNode* minkey;
		int minkeyPosition;
		for (int j = 0; j < open.size(); j++) {
			if (j == 0) {
				minkey = open[0];
				minkeyPosition = 0;
			}
			if (KeyAlessthanB(open[j], minkey)) {
				minkey = open[j];
				minkeyPosition = j;
			}
		}
		startNode.key = key(&startNode);

		while (KeyAlessthanB(minkey, &startNode) || (startNode.rhs != startNode.g)) {

			//15. remove state s with the minimum key from open
			open.erase(open.begin() + minkeyPosition);
			
			//16. if(g(s)>rhs(s))
			if (minkey->g > minkey->rhs) {
				
				//17. g(s)=rhs(s)
				minkey->g = minkey->rhs;
				
				//18. CLOSED = CLOSEDu{s}
				closed.push_back(minkey);
				
				//19. for all s' which is a predecessor of s, updateState(s')
				std::vector<AStarPlannerNode*> predecessors = getNeighbors(minkey);
				for (int i = 0; i < predecessors.size(); i++) {
					// if successor is the goal, stop the search
					updateStateAD(predecessors[i]);
				}
			}

			//20. else
			else {
				
				//21. g(s)=inf
				minkey->g = INT_MAX;
				
				//22. for all s' which is a predecessor of s and also s, updateState(s')
				updateStateAD(minkey);
				std::vector<AStarPlannerNode*> predecessors = getNeighbors(minkey);
				for (int i = 0; i < predecessors.size(); i++) {
					// if successor is the goal, stop the search
					updateStateAD(predecessors[i]);
				}
			}

			//14. while loop
			//minkey = new AStarPlannerNode;
			for (int j = 0; j < open.size(); j++) {
				if (j == 0) {
					minkey = open[0];
					minkeyPosition = 0;
				}
				if (KeyAlessthanB(open[j], minkey)) {
					minkey = open[j];
					minkeyPosition = j;
				}
			}
			minkey->key = key(minkey);
			startNode.key = key(&startNode);
		}
	}

	bool AStarPlanner::addNeighborIfGood(AStarPlannerNode* parent, std::vector<AStarPlannerNode*> &neighbors, Util::Point point) {
		int dbIndex = gSpatialDatabase->getCellIndexFromLocation(point);
		if (canBeTraversed(dbIndex)) {
			AStarPlannerNode* node = new AStarPlannerNode(point, double(0), double(0), double(0), parent);
			neighbors.push_back(node);
			return true;
		}
		return false;
	}

	std::vector<AStarPlannerNode*> AStarPlanner::getNeighbors(AStarPlannerNode* a) {
		std::vector<AStarPlannerNode*> neighbors;
		// Top 
		addNeighborIfGood(a, neighbors, Util::Point(a->point.x, 0, a->point.z + 1));

		// Top Right
		addNeighborIfGood(a, neighbors, Util::Point(a->point.x + 1, 0, a->point.z + 1));

		// Right
		addNeighborIfGood(a, neighbors, Util::Point(a->point.x + 1, 0, a->point.z));

		// Right Bottom
		addNeighborIfGood(a, neighbors, Util::Point(a->point.x + 1, 0, a->point.z - 1));

		// Bottom
		addNeighborIfGood(a, neighbors, Util::Point(a->point.x, 0, a->point.z - 1));
		
		// Bottom Left
		addNeighborIfGood(a, neighbors, Util::Point(a->point.x - 1, 0, a->point.z - 1));
		
		// Left
		addNeighborIfGood(a, neighbors, Util::Point(a->point.x - 1, 0, a->point.z));
		
		// Left Top
		addNeighborIfGood(a, neighbors, Util::Point(a->point.x - 1, 0, a->point.z + 1));
		
		return neighbors;
	}

	void updateState(AStarPlannerNode state) {
		//5.  if(s was not visitited before)
		bool skipOPEN = false;
		bool skipINCONS = false;
		bool skipCLOSED = false;
		int indexOfS = -1;
		for (int j = 0; j < open.size(); j++) {
			if (open[j]->point == state.point) {
				indexOfS = j;
				skipOPEN = true;
			}
		}
		for (int j = 0; j < incons.size(); j++) {
			if (incons[j]->point == state.point) {
				skipINCONS = true;
			}
		}
		for (int j = 0; j < closed.size(); j++) {
			if (closed[j]->point == state.point) {
				skipCLOSED = true;
			}
		}
		if (!skipOPEN && !skipINCONS && !skipCLOSED) {
			//6. g(s)=inf
			state.g = 10000000;
		}

		//7. if(s is not equal to s_goal)
		if (s->point != goalNode.point) {
			//rhs(s)=min for s' in successors of s (c(s,s')+g(s'))
			std::vector<AStarPlannerNode*> successors = getNeighbors(s);
			// generate q's 8 successors and set their parents to q
			float minimumS_s = 100000000;
			for (int i = 0; i < successors.size(); i++) {
				AStarPlannerNode* s_s = successors[i];
				float costOfMoving = s_s->g + euclidean_distance(s->point, s_s->point);
				if (costOfMoving < minimumS_s) {
					minimumS_s = costOfMoving;
				}
			}
			s->rhs = minimumS_s;
		}
		//8. if(s is in open, remove s from Open)
		if (skipOPEN) {
			open.erase(open.begin() + indexOfS);
		}


		//9. if(g(s)!=rhs(s))
		if (s->g != s->rhs) {
			//10. if(s!=CLOSED)
			if (!skipCLOSED) {
				//11. Insert s into OPEN with key(s)
				s->key = key(s);
				open.push_back(s);
			}
			//12. else
			else {
				//13. insert s into incons
				incons.push_back(s);
			}

		}
	}

	std::vector<double> AStarPlanner::key(AStarPlannerNode *s) {
		std::vector<double> values;
		//1.if(g(s)>rhs(s))
		if (s->g > s->rhs) {
			//2. return [rhs(s)+w*h(s_start,s);rhs(s)]
			values.push_back(s->rhs + eps*euclidean_distance(startNode.point, s->point));
			values.push_back(s->rhs);
		}
		//3. else
		else {
			//4. return [g(s)+h(s_start,s);g(s)]
			values.push_back(s->g + euclidean_distance(startNode.point, s->point));
			values.push_back(s->g);
		}

		return values;
	}

}