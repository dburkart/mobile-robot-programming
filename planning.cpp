#include "planning.h"

#define HEIGHT 	4
#define WIDTH 	13

int map[HEIGHT][WIDTH] = 	{	{0,1,1,1,1,1,1,1,1,1,1,1,0},
										{1,1,0,1,0,1,0,1,0,1,0,1,0},
										{0,0,0,1,0,1,0,0,0,1,0,1,0},
										{0,1,1,1,1,1,1,1,1,1,0,1,1}
									};

Point mapPoints[HEIGHT][WIDTH] = 	{ {(Point){0.0,0.0}			,	(Point){-56.24, 13.1}	, (Point){-51.10, 13.10}, (Point){-47.68, 13.1}	, (Point){-19.59, 13.1}	, (Point){8.50, 13.10}, (Point){15.0, 13.1}		, (Point){20.91, 13.1},	(Point){23.50, 13.1}	,	(Point){28.60, 13.1}	,	(Point){34.80, 13.1}, (Point){40.80, 13.1}	,	(Point){0.0,0.0}			},
														{(Point){-58.75, 7.80}, (Point){-56.24, 7.80}	,	(Point){0.0,0.0}			, (Point){-47.68, 5.46}	,	(Point){0.0,0.0}			, (Point){8.50, 5.46}	,	(Point){0.0,0.0}			, (Point){20.91, 8.25},	(Point){0.0,0.0}			, (Point){28.60, 5.46}	,	(Point){0.0,0.0}		,	(Point){40.80, 5.46}	,	(Point){0.0,0.0}			},
														{(Point){0.0,0.0}			,	(Point){0.0,0.0}			,	(Point){0.0,0.0}			,	(Point){-47.68, -2.16},	(Point){0.0,0.0}			,	(Point){8.50, -2.16},	(Point){0.0,0.0}			,	(Point){0.0,0.0}		,	(Point){0.0,0.0}			,	(Point){28.60, -2.16}	,	(Point){0.0,0.0}		,	(Point){40.80, -2.16}	,	(Point){0.0,0.0}			},
														{(Point){0.0,0.0}			,	(Point){-55.15, -9.8}	,	(Point){-51.10, -9.8}	,	(Point){-47.68, -9.8}	,	(Point){-19.59, -9.8}	,	(Point){8.50, -9.8}	,	(Point){13.525, -9.8}	,	(Point){18.55, -9.8},	(Point){23.575, -9.8}	, (Point){28.60, -9.8}	,	(Point){0.0,0.0}		,	(Point){40.80, -9.8}	,	(Point){45.50, -9.8}	}
													};


// Globals and data structures
Point start;
Point goal;
Path path;
std::vector<Node*> openList;
std::vector<Node*> closedList;


Path PlanPath( Point src, Path dests, Point offset ) {

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

	path.push_back((Point){src.x - offset.x, src.y - offset.y});

	for (int i = 0; i < dests.size(); i++) {
		
		// the following call modifies state (adds waypoints to
		// the global variable 'path')
		openList.clear();
		closedList.clear();
		planToGoal(path.back(), dests[i], offset);
		
	}
	
	std::cout << "Path";

	for (int i = 0; i < path.size(); i++) {
		
		std::cout << " => (" << path[i].x << ", " << path[i].y << ")";
		
	}

	return path;
}

Path planToGoal(Point src, Point dest, Point offset) {
	
	int startIndex_h;
	int startIndex_w;
	int goalIndex_h;
	int goalIndex_w;
	
	// TODO: find out the indeces of the start and goal in
	//				our custom map structure.
	int *indeces = findClosestPoint(src);
	startIndex_h = indeces[0];
	startIndex_w = indeces[1];
	indeces = findClosestPoint(dest);
	goalIndex_h = indeces[0];
	goalIndex_w = indeces[1];
	
	std::cout << "0" << std::endl;
	
	// start and goal points based on our custom drawn map
	start	= mapPoints[startIndex_h][startIndex_w];
	goal	= mapPoints[goalIndex_h][goalIndex_w];
	
	std::cout << "(" << start.x << ", " << start.y << ") => ("
						<< goal.x << ", " << goal.y << ")" << std::endl;

	// create the first node and calculate hCost for that node
	double hCost = start.distanceTo(goal);
	Node* root = new Node(start, 0, 0, hCost, true);
	
	// add root to openList
	openList.push_back(root);
	
	//std::cout << "1" << std::endl;
	
	int level = 1;
	
	while (openList.size() != 0) {
		
		std::cout << "level: " << level << std::endl;
		
		Node* current = openList[0];
		for (int i = 0; i < openList.size(); i++) {
			
			if ((*openList[i]).fCost < (*current).fCost)
				current = openList[i];
			
		}
		
		std::cout << "Current(" << (*current).point.x << ", " 
							<< (*current).point.y << ")" << std::endl;
		
		
		// after the previous for-loop, 'current' holds the node in OpenList
		// with the lowest fCost (hCost + gCost).
		
		if ((*current).point == goal) {
			// TODO: return path
			std::cout << "openList:{";
			for (int i = 0; i < openList.size(); i++) {
				
				std::cout << "(" << (*openList[i]).point.x << ", " 
									<< (*openList[i]).point.y << "),";
				
			}
			std::cout << "}" << std::endl;
			std::cout << "done! openList.size = " << openList.size() << " constructing path..." << std::endl;
			std::vector<Node*> pathOfNodes = constructPath((*current).ancestor, current);
			if (!((*pathOfNodes.front()).point == src) && !(path.back() == src))
				path.push_back((Point){src.x - offset.x, src.y - offset.y});
			for (int i = 0; i < pathOfNodes.size(); i++) {
				
				path.push_back((Point){(*pathOfNodes[i]).point.x - offset.x, (*pathOfNodes[i]).point.y - offset.y} );
				
			}
			if (!((*pathOfNodes.back()).point == dest))
				path.push_back((Point){dest.x - offset.x, dest.y - offset.y});
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
		
		if (level == 16)
			std::cout << indeces[0] << "," << indeces[1] << std::endl;
		
		// try left
		checkNeighbor(current, indeces, 4); // left
		checkNeighbor(current, indeces, 6); // right
		checkNeighbor(current, indeces, 2); // up
		checkNeighbor(current, indeces, 8); // down
		/*if (indeces[1] > 0 )
			neighborPoint = mapPoints[indeces[0]][indeces[1]-1];
		else
			neighborPoint = (Point){0.0,0.0};
		
		if (!(neighborPoint == (Point){0.0,0.0}) && !(inClosedList(neighborPoint))) {
			std::cout << "leftNeighbor(" << neighborPoint.x << ", " 
							<< neighborPoint.y << ")" << std::endl;
			double gCost = (*current).gCost + (*current).point.distanceTo(neighborPoint);
			
			// add neighbor in openList if it's not there already
			if (!inOpenList(neighborPoint)) {
				
				double hCost = neighborPoint.distanceTo(goal);
				Node* temp = new Node(neighborPoint, current, gCost, hCost, false);
				openList.push_back(temp);
				
			}
			
		}
		
		// try right
		if (indeces[1] < WIDTH-1)
			neighborPoint = mapPoints[indeces[0]][indeces[1]+1];
		else
			neighborPoint = (Point){0.0,0.0};
		
		if (!(neighborPoint == (Point){0.0,0.0}) && !(inClosedList(neighborPoint))) {
			std::cout << "rightNeighbor(" << neighborPoint.x << ", " 
								<< neighborPoint.y << ")" << std::endl;
			double gCost = (*current).gCost + (*current).point.distanceTo(neighborPoint);
			
			// add neighbor in openList if it's not there already
			if (!inOpenList(neighborPoint)) {
				
				double hCost = neighborPoint.distanceTo(goal);
				Node* temp = new Node(neighborPoint, current, gCost, hCost, false);
				openList.push_back(temp);
				
			}
			
		}
		
		// try up
		if (indeces[0] > 0)
			neighborPoint = mapPoints[indeces[0]-1][indeces[1]];
		else
			neighborPoint = (Point){0.0,0.0};
		
		if (!(neighborPoint == (Point){0.0,0.0}) && !(inClosedList(neighborPoint))) {
			std::cout << "upNeighbor(" << neighborPoint.x << ", " 
								<< neighborPoint.y << ")" << std::endl;
			double gCost = (*current).gCost + (*current).point.distanceTo(neighborPoint);
			
			// add neighbor in openList if it's not there already
			if (!inOpenList(neighborPoint)) {
				
				double hCost = neighborPoint.distanceTo(goal);
				Node* temp = new Node(neighborPoint, current, gCost, hCost, false);
				openList.push_back(temp);
				
			}
			
		}
		
		// try down
		if (indeces[0] < HEIGHT-1)
			neighborPoint = mapPoints[indeces[0]+1][indeces[1]];
		else
			neighborPoint = (Point){0.0,0.0};
		
		if (!(neighborPoint == (Point){0.0,0.0}) && !(inClosedList(neighborPoint))) {
			std::cout << "downNeighbor(" << neighborPoint.x << ", " 
								<< neighborPoint.y << ")" << std::endl;
			double gCost = (*current).gCost + (*current).point.distanceTo(neighborPoint);
			
			// add neighbor in openList if it's not there already
			if (!inOpenList(neighborPoint)) {
				
				double hCost = neighborPoint.distanceTo(goal);
				Node* temp = new Node(neighborPoint, current, gCost, hCost, false);
				openList.push_back(temp);
				
			}
			
		}
		*/
		
		std::cout << "##########################################" << std::endl;
		level++;
		
	}
	
	return Path();
}

int* findClosestPoint(Point p) {
	
	int index_h = 0;
	int index_w = 1;
	
	double smallestDist = mapPoints[0][1].distanceTo(p);
	for (int h = 0; h < HEIGHT; h++ ) {
		
		for (int w = 0; w < WIDTH; w++) {
			
			if (mapPoints[h][w] == (Point){0.0,0.0})
				continue;
				
			double tempDist = mapPoints[h][w].distanceTo(p);
			
			//std::cout << "distCmp(" << tempDist << " vs. " 
			//					<< smallestDist << ")" << std::endl;
			
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
			
			std::cout << label << "(" << neighborPoint.x << ", " 
								<< neighborPoint.y << ")" << std::endl;
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
	
	//std::cout << "here" << std::endl;
	
	if (!(*currentNode).root) {
		
			//std::cout << "ancestor(" << (*(*currentNode).ancestor).point.x << ", " << (*(*currentNode).ancestor).point.x << ")" << std::endl;
			std::vector<Node*> tmpPath = constructPath(cameFrom, (*currentNode).ancestor);
			tmpPath.push_back(currentNode);
			//std::cout << " => (" << (*currentNode).point.x << ", " << (*currentNode).point.y << ")";
			return tmpPath;
		
	}
	else {
		
		std::vector<Node*> tmpPath;
		tmpPath.push_back(currentNode);
		
		//std::cout << "(" << (*currentNode).point.x << ", " << (*currentNode).point.y << ")";
		
		return tmpPath;
		
	}
	
	
}
