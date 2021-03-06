/**
 * file: planning.cpp
 * 
 * Description: This class uses the preconstructed map array below
 * 		to perform an A* search for the shortest path from point A
 * 		to point B. Other classes are expected to call planPath
 * 		which plans a path through any number of goals. The algorithm
 * 		expects the starting point and the goal(s) point(s) to be passed
 * 		in in global coordinate system.
 * 		
 * Developer's notes:
 * 		The algorithm can be improved to plan the most effecient path
 * 		when planning through multiple goal by modifiying the order
 * 		of the goals so that the robot goes through all the goal by
 * 		travelling the shortest distance possible.
 * 
 * Author(s): Hashem Assayari (hya4542), Dana Burkart (dsb3573)
 */

#include "planning.h"

#define HEIGHT 	4
#define WIDTH 	13

using namespace std;

int map[HEIGHT][WIDTH] = 	{	{0,1,1,1,1,1,1,1,1,1,1,1,0},
										{1,1,0,1,0,1,0,1,0,1,0,1,0},
										{0,0,0,1,0,1,0,0,0,1,0,1,0},
										{0,1,1,1,1,1,1,1,1,1,0,1,1}
									};

Point mapPoints[HEIGHT][WIDTH] ={	{(Point){0.0,0.0}			,	(Point){-56.24, 13.1}	, 	(Point){-51.10, 13.10}	, (Point){-47.68, 13.1}	, (Point){-19.59, 13.1}	, (Point){8.50, 13.10}, (Point){15.0, 13.1}		, (Point){20.91, 13.1},	(Point){23.50, 13.1}	,	(Point){28.60, 13.1}	,	(Point){34.80, 13.1}, (Point){40.80, 13.1}	,	(Point){0.0,0.0}			},
									{(Point){-58.75, 7.80}		, 	(Point){-56.24, 7.80}	,	(Point){0.0,0.0}		, (Point){-47.68, 5.46}	,	(Point){0.0,0.0}			, (Point){8.50, 5.46}	,	(Point){0.0,0.0}			, (Point){20.91, 10.0},	(Point){0.0,0.0}			, (Point){28.60, 5.46}	,	(Point){0.0,0.0}		,	(Point){40.80, 5.46}	,	(Point){0.0,0.0}			},
									{(Point){0.0,0.0}			,	(Point){0.0,0.0}		,	(Point){0.0,0.0}		,	(Point){-47.68, -2.16},	(Point){0.0,0.0}			,	(Point){8.50, -2.16},	(Point){0.0,0.0}			,	(Point){0.0,0.0}		,	(Point){0.0,0.0}			,	(Point){28.60, -2.16}	,	(Point){0.0,0.0}		,	(Point){40.80, -2.16}	,	(Point){0.0,0.0}			},
									{(Point){0.0,0.0}			,	(Point){-55.15, -9.8}	,	(Point){-51.10, -9.8}	,	(Point){-47.68, -9.8}	,	(Point){-19.59, -9.8}	,	(Point){8.50, -9.8}	,	(Point){13.525, -9.8}	,	(Point){18.55, -9.8},	(Point){23.575, -9.8}	, (Point){28.60, -9.8}	,	(Point){0.0,0.0}		,	(Point){40.80, -9.8}	,	(Point){45.50, -9.8}	}
								};


// Globals and data structures
Point start;
Point goal;
Path path;
std::vector<Node*> openList;
std::vector<Node*> closedList;

//
// plan to multiple goals
// calls 'planToGoal' for each of the goal specified here.
// src point and dests points are expected to be in GLOBAL
// 	coordinate system.
// returns a Path object containing the path from the 'src'
// 	to each of the points in 'dests' in LOCAL coordinate system.
//
Path PlanPath( Point src, Path dests, Robot *robot ) {

/*	
	011111111111110
	110100010101010
	000100010001010
	011111111111011
*/

/*	
	00000000000000000
	00111111111111100
	01101000101010100
	00001000100010100
	00111111111110110
	00000000000000000
*/
	
	path.push_back(src);		// this line is ciritcal

	for (int i = 0; i < dests.size(); i++) {
		
		// the following call modifies state (adds waypoints to
		// the global variable 'path')
		cleanup();
		openList.clear();
		closedList.clear();
		planToGoal(path.back(), dests[i]);
		
	}
	
	// garbage collection
	cleanup();
	openList.clear();
	closedList.clear();
	
	path.erase(path.begin());
	
	cout << "============= Planning =============" << endl;
	cout << "Path (global): " << endl;
	// converting all the points to local coordinate system
	// 	before returning the 'path' object back to caller.
	for (int i = 0; i < path.size(); i++) {
		std::cout << " -> (" << path[i].x << ", " << path[i].y << ")";	
	}
	cout << endl << endl << "Path (local): " << endl;
	// converting all the points to local coordinate system
	// 	before returning the 'path' object back to caller.
	for (int i = 0; i < path.size(); i++) {
		path[i] = robot->ToLocal( path[i] );
		std::cout << " -> (" << path[i].x << ", " << path[i].y << ")";	
	}
	cout << endl;
	
	return path;
}

//
// plan to a single goal
// expects 'src' & 'dests' to be in GLOBAL coordinate system
//
Path planToGoal(Point src, Point dest) {
	
	int startIndex_h;
	int startIndex_w;
	int goalIndex_h;
	int goalIndex_w;
	
	// 1- find out the indeces of the start and goal in
	//	our custom map structure.
	int *indeces = findClosestPoint(src);
	startIndex_h = indeces[0];
	startIndex_w = indeces[1];
	indeces = findClosestPoint(dest);
	goalIndex_h = indeces[0];
	goalIndex_w = indeces[1];
	
	// 1a- start and goal points based on our custom drawn map
	start	= mapPoints[startIndex_h][startIndex_w];
	goal	= mapPoints[goalIndex_h][goalIndex_w];
	
	// 2- create the first node and calculate hCost for that node
	double hCost = start.distanceTo(goal);
	Node* root = new Node(start, 0, 0, hCost, true);
	
	// 2a- add root to openList
	openList.push_back(root);
	
	int level = 1;
	
	while (openList.size() != 0) {
		
		Node* current = openList[0];
		for (int i = 0; i < openList.size(); i++) {
			
			if ((*openList[i]).fCost < (*current).fCost)
				current = openList[i];
			
		}
		
		
		// after the previous for-loop, 'current' holds the node in OpenList
		// with the lowest fCost (hCost + gCost).
		
		if ((*current).point == goal) {

			std::vector<Node*> pathOfNodes = constructPath((*current).ancestor, current);
			if (!((*pathOfNodes.front()).point == src) && !(path.back() == src))
				path.push_back(src);
			for (int i = 0; i < pathOfNodes.size(); i++) {
				
				path.push_back( (*pathOfNodes[i]).point );
				
			}
			if (!((*pathOfNodes.back()).point == dest))
				path.push_back(dest);
			return path;
		}
		
		// remove current from openList
		for (int i = 0; i < openList.size(); i++) {
			if ((*current).point == (*openList[i]).point) {
				openList.erase(openList.begin()+i);
				break;
			}
		}
		
		// add current to closedList
		closedList.push_back(current);
		
		// add all valid neighbors to openList
		// could be left, right, up or down
		indeces = findClosestPoint((*current).point);
		Node* tmpNeighbor;
		Point neighborPoint;
		
		
		// try left
		checkNeighbor(current, indeces, 4); // left
		checkNeighbor(current, indeces, 6); // right
		checkNeighbor(current, indeces, 2); // up
		checkNeighbor(current, indeces, 8); // down
		

		level++;
		
	}
	
	return Path();
}

//
// Find the the point in our custom map that is closest 
// to the given point.
// returns the indeces to the closest point in the 2D array.
//
int* findClosestPoint(Point p) {
	
	int index_h = 0;
	int index_w = 1;
	
	double smallestDist = mapPoints[0][1].distanceTo(p);
	for (int h = 0; h < HEIGHT; h++ ) {
		
		for (int w = 0; w < WIDTH; w++) {
			
			if (mapPoints[h][w] == (Point){0.0,0.0})
				continue;
				
			double tempDist = mapPoints[h][w].distanceTo(p);
			

			if (tempDist < smallestDist) {
			
				index_h = h;
				index_w = w;
				smallestDist = tempDist;
				
			}
			
		}
		
	}
	
	int *result = new int[2];
	result[0] = index_h;
	result[1] = index_w;
	
	return result;
	
}

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
int checkNeighbor(Node* current, int* indeces, int moveTo) {
	
	Point neighborPoint;
	
	int xMove = 0;
	int yMove = 0;
	char* label = (char*)"";
		
	// below are the values or moveTO
	//	1		2		3
	//	4		5		6
	//	7		8		9
		
	switch (moveTo) {
		
		case 1: {xMove = -1; yMove = -1; label = (char*)"northwest neighbor"; break;}	// northwest
		case 2: {xMove =  0; yMove = -1; label = (char*)"noth neighbor"; break;}	// north
		case 3: {xMove =  1; yMove = -1; label = (char*)"northeast neighbior"; break;}	// northeast
		case 4: {xMove = -1; yMove =  0; label = (char*)"west neighbor"; break;}	// west
		case 5: {xMove =  0; yMove =  0; label = (char*)"me"; break;}	// center
		case 6: {xMove =  1; yMove =  0; label = (char*)"east neighbor"; break;}	// east
		case 7: {xMove = -1; yMove =  1; label = (char*)"southeast neighbor"; break;}	// southwest
		case 8: {xMove =  0; yMove =  1; label = (char*)"south neighbor"; break;}	// south
		case 9: {xMove =  1; yMove =  1; label = (char*)"southeasst neighbor"; break;}	// southeast
		default: return 0; break;				//invalid option, failure
		
	}
	
	if ( 	(indeces[0] == 0 					&& (moveTo == 1 || moveTo == 2 || moveTo == 3)) ||
				(indeces[0] == (HEIGHT-1)	&& (moveTo == 7 || moveTo == 8 || moveTo == 9)) ||
				(indeces[1] == 0 					&& (moveTo == 1 || moveTo == 4 || moveTo == 7)) ||
				(indeces[1] == (WIDTH-1)	&& (moveTo == 3 || moveTo == 6 || moveTo == 9)) ) {
			
			// no neighbor.
			return 0;
			
	} else {
		
		neighborPoint = mapPoints[indeces[0]+yMove][indeces[1]+xMove];
		
		// if the neighbor point isn't a valid move i.e. wall
		if (!(neighborPoint == (Point){0.0,0.0}) && !(inClosedList(neighborPoint))) {
			
			double gCost = (*current).gCost + (*current).point.distanceTo(neighborPoint);
				
			// add neighbor in openList if it's not there already
			if (!inOpenList(neighborPoint)) {
				
				double hCost = neighborPoint.distanceTo(goal);
				Node* temp = new Node(neighborPoint, current, gCost, hCost, false);
				openList.push_back(temp);
				
			}
		}
		else {
			
			return 0;
			
		}
		
	}
	
	return 1;	// success
	
}

//
// check whether the specified point is in the openList
// returns true if it is, false otherwise
//
bool inOpenList(Point p) {
	
	for (int i = 0; i < openList.size(); i++) {
		
		if ((*openList[i]).point == p)
			return true;
			
	}
	
	return false;
	
}

//
// check whether the specified point is in the closedList
// returns true if it is, false otherwise
//
bool inClosedList(Point p) {
	
	for (int i = 0; i < closedList.size(); i++) {
		
		if ((*closedList[i]).point == p)
			return true;
			
	}
	
	return false;
	
}

//
// backtracks the goal node to the root node and constructs a
// vector of nodes start fropm root to goal.
//
std::vector<Node*> constructPath(Node* cameFrom, Node* currentNode) {
	
	
	if (!(*currentNode).root) {
		
			std::vector<Node*> tmpPath = constructPath(cameFrom, (*currentNode).ancestor);
			tmpPath.push_back(currentNode);
			return tmpPath;
		
	}
	else {
		
		std::vector<Node*> tmpPath;
		tmpPath.push_back(currentNode);
		
		return tmpPath;
		
	}
	
	
}

int cleanup() {
	
	// 1- clear openList
	for (int i = 0; i < openList.size(); i++) {
		
		delete openList[i];
		
	}
	
	// 2- clear closedList
	for (int i = 0; i < closedList.size(); i++) {
		
		delete closedList[i];
		
	}
	
	return 1;
	
}
