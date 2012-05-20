#ifndef __PLANNING_H__
#define __PLANNING_H__

#include "physics.h"

struct Node {
	
	// default constructor
	Node() {}
	
	Node(const Node& node) {
		
		point = node.point;
		ancestor = node.ancestor;
		fCost = node.fCost;
		hCost = node.hCost;
		gCost = node.gCost;
		
	}
	
	// constructor
	Node(Point iP, Node* iAncestor, double iGCost, double iHCost, bool iRoot) 
				: point(iP), gCost(iGCost), 
					hCost(iHCost), fCost(iGCost+iHCost), root(iRoot) { 
				
		if (iAncestor != 0)
			ancestor = iAncestor;
		else
			ancestor = 0;
						
	}
						
						
	// to string function
  friend std::ostream& operator<<(std::ostream& os, const Node& n) {
		return os << "Node[point(" << n.point.x << ", " 
							<< n.point.y << "), " << n.hCost << ", " 
							<< n.gCost << "];" << std::endl;
	}
	
	// data members
	Point point;			// the x,y point
	Node *ancestor;	// the node we came from
	double fCost;			// estimated total cost
	double hCost;			// the hueristic cost
	double gCost;			// the cost so far
	bool root;
	
};

//
// plan to multiple goals
// calls 'planToGoal' for each of the goal specified here
//
Path PlanPath( Point src, Path dests );

//
// plan to a single goal
//
Path planToGoal(Point src, Point dest);

//
// Find the the point in our custom map that is closest 
// to the given point.
// returns the indeces to the closest point in the 2D array.
//
int* findClosestPoint(Point p);

//
// Check the 'moveTo' neighbor of the 'current' node and adds it to
// 'openList' if it's a valid move.
//
// moveTo can take a value from 1-9 (5 being the current node)
// based on the matrix below:
// 	1		2		3
// 	4		5		6
// 	7		8		9
//
// This function changes the global state i.e. modifies the 'openList'
// data structure.
//
int checkNeighbor(Node* current, int* indeces, int moveTo);

//
// check whether the specified point is in the openList
// returns true if it is, false otherwise
//
bool inOpenList(Point p);

//
// check whether the specified point is in the closedList
// returns true if it is, false otherwise
//
bool inClosedList(Point p);

//
// backtracks the goal node to the root node and constructs a
// vector of nodes start fropm root to goal.
//
std::vector<Node*> constructPath(Node* cameFrom, Node* currentNode);

#endif

