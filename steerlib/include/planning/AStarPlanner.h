//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"

#include <unordered_set>
#include <unordered_map>

// Temp
#include <queue>
#include <functional>

enum HMODE
{
	EUC, MAN
};

namespace SteerLib
{

	/*
		@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
		@attributes 
		f : the f value of the node
		g : the cost from the start, for the node
		point : the point in (x,0,z) space that corresponds to the current node
		parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
		@operators 
		The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			Util::Point point;
			AStarPlannerNode* parent;	

			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
			}

			// Modified operator for rule 1
			bool operator<(AStarPlannerNode other) const
		    {
				if (this->f < other.f)
					return true;
				else if (this->f == other.f && this->g < other.g)
					return true;
				else
					return false;
		    }
		    bool operator>(AStarPlannerNode other) const
		    {
				if (this->f > other.f)
					return true;
				else if (this->f == other.f && this->g > other.g)
					return true;
				else
					return false;
		    }
		    bool operator==(AStarPlannerNode other) const
		    {
		        return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		    }

	};

	// Extra Credit Min Heap
	class STEERLIB_API MinHeap {
	private:
		std::vector<AStarPlannerNode> _vector; 
		void BubbleDown(int index);
		void BubbleUp(int index);
		void Heapify();

	public:
		MinHeap();
		void insertKey(AStarPlannerNode newValue);
		AStarPlannerNode extractMin();
		void DeleteMin();
		bool isEmpty();
	};

	

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			~AStarPlanner();
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/
			bool canBeTraversed ( int id );
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);

			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/

			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);

			// Customized Variables and Functions
			MinHeap fringe = MinHeap();

			std::unordered_set<int> closedList;
			std::unordered_map<int, double> fValues;
			std::unordered_map<int, int> parentList;

			void ImprovePath(std::vector<AStarPlannerNode*> &openSet, std::vector<AStarPlannerNode*> &closedSet, std::vector<AStarPlannerNode*> &inconsistentList, std::vector<AStarPlannerNode*> &path, Util::Point goal, AStarPlannerNode* &goalState, double inflationFactor);
			void storePath(std::vector<AStarPlannerNode*> &path, std::vector<Util::Point>& agent_path, Util::Point goal);
			bool ARAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, bool append_to_path);
			void resetList(std::vector<SteerLib::AStarPlannerNode*> &o, Util::Point p);
			double getMin(std::vector<AStarPlannerNode*> openList, std::vector<AStarPlannerNode*> inconsistentList, Util::Point goal);
			double getMinFromOpen(std::vector<AStarPlannerNode*> openList, Util::Point goal);
			std::vector<AStarPlannerNode*> getSuccessors(AStarPlannerNode* currentNode, Util::Point goal);
			bool checkSuccessor(AStarPlannerNode* parentNode, Util::Point location, std::vector<AStarPlannerNode*> &successors, Util::Point goal);


		private:
			SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	};
}


#endif
