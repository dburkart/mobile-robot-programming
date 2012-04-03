/**
 * file: safegoto.cpp
 * 
 * Description: Read in a set of points, and try go to each one while 
 * 		avoiding obstacles. Because of the restrictions on submitting
 * 		multiple source files, this file is choc-full of data structures.
 * 
 * Author: Dana Burkart (dsb3573)
 */

#include <iostream>
#include <fstream>
#include <string>
#include <libplayerc++/playerc++.h>
#include <iomanip>
#include <math.h>

#include "safegoto.h"

using namespace PlayerCc;

/**
 * The point structure is used to hold an (x, y) tuple, and provides a
 * few useful operations.
 */
struct Point {
	double x, y;
	
	//
	// We can define the minus operator as the euclidean distance 
	// between two points.
	//
	double operator-( const Point& p ) const {
		return sqrt((x - p.x)*(x - p.x) + (y - p.y)*(y - p.y));
	}
	
	Point operator+( const Point& p ) const {
		return (Point){ x + p.x, y + p.y };
	}
	
	//
	// Return a 0-based indication of what quadrant this point is in.
	//
	int quadrant() {
		if ( x >= 0.0 && y >= 0.0 ) return 0;
		if ( x < 0.0 && y >= 0.0 ) return 1;
		if ( x < 0.0 && y < 0.0 ) return 2;
		return 3;
	}
	
	//
	// Using this point as the 'origin', calculate the angle to another
	// point.
	// 
	// We'll use acos( adjacent / hypotenuse ) and some quadrant checking
	// to figure this out.
	//
	double theta( Point p ) {
		double t = 0.0;
		double a, b;
		int quad;
		
		// Calculate the hypotenuse
		a = (*this) - p;
		
		// Translate p with respect to ourself
		p = (Point){ p.x - x, p.y - y };
		
		// Calculate adjacent
		b = fabs(p.x);
		
		// Get theta.
		t = acos( (b + .0001) / (a + .0001) );
		
		switch (p.quadrant()) {
			case 0:
				// Do nothing -- theta was already calculated for quadrant 0
				break;
			case 1:
				// We're in the first quadrant, so we first need to 'flip' our
				// triangle
				t = dtor( 90 ) - t;
				
				// Add in another 90 degrees to compensate for quadrant 1
				t += dtor( 90 );
				break;
			case 2:
				// Flip our triangle like we do for quadrant 1
				t = dtor( 90 ) - t;
				
				// Add in another 90 degrees
				t += dtor( 90 );
			case 3: 
				// We're in quadrant 2 or 3, so negate theta
				t *= -1.0;
				break;
		}
		
		return t;
	}
};

/**
 * The Vector struct is a tuple (direction, magnitude) that provides some
 * useful functions on vectors.
 */
struct Vector {
	double direction;
	double magnitude;
	
	//
	// Return the components of this vector in the form of a point.
	//
	Point components() const {
		return (Point){ magnitude * cos(direction), magnitude * sin(direction) };
	}
	
	//
	// Vector addition
	//
	Vector operator+( const Vector& v ) const {
		Point cx = components() + v.components();
		Vector vn;
		
		vn.direction = atan(cx.y / cx.x);
		vn.magnitude = pow(cx.y*sin(vn.direction), -1);
		
		return vn;
	}
	
	void print() {
		std::cout << "(m: " << magnitude << ", d: " << rtod( direction )
			<< ")" << std::endl;
	}
};

/**
 * The robot class abstracts over a robot, providing an interface for
 * adding processing hooks.
 */
class Robot {
public:

	Robot( PlayerClient *c, Path p ) :
		client( c ),
		pp( c, 0 ),
		rp( c, 0 ),
		path( p )
	{
		position = (Point){ 0.0, 0.0 };
		velocity = (Vector){ 0.0, 0.0 };
		currentGoal = path.begin();
		rdata.resize(102);
	}
	
	~Robot() {}
	
	//
	// Add a processing hook to this robot. The order hooks are called
	// is fifo, so add lowest processing layers first.R
	//
	void AddHook( RobotHook h ) {
		hooks.push_back( h );
	}
	
	//
	// Enter the run loop
	//
	int Run() {
		while (!rp.IsValid()) client->Read();
		
		for(;;) {
			RobotHookList::iterator it;
			
			UpdateRangeData();
			
			Point p = position 	= (Point){ pp.GetXPos(), pp.GetYPos() };
			Vector v = velocity = (Vector){ pp.GetYaw(), pp.GetXSpeed() };
			
			for (it = hooks.begin(); it < hooks.end(); it++ ) {
				RobotHook h = *it;
				h( this, &p, &v );
			}
			
			pp.SetSpeed( v.magnitude, v.direction );
			
			if ( currentGoal == path.end() ) break;
			
			client->Read();
		}
	}
	
	Point GetGoal() {
		return *currentGoal;
	}
	
	void GoalAchieved() {
		currentGoal++;
	}
	
	void UpdateRangeData() {
		for ( int i = 0; i < 102; i++ ) {
			Vector vAvg = (Vector){0.0, 0.0};
			
			for ( int j = 0; j < 5; j++ ) {
				Vector vTemp = (Vector){ (3.14159 * (i * j)) / (510), 
										 rp[ 30 + (i * j) ] };
										 
				if ( j == 0 ) vAvg = vTemp;
				else vAvg = vAvg + vTemp;
			}
			
			vAvg.magnitude  /= 5.0;
			rdata[i] = vAvg;
		}
	}
	
	RangeData *GetRangeData() {
		return &rdata;
	}
	
	Vector GetVelocity() {
		return velocity;
	}

protected:
	
	Point 		position;
	Vector		velocity;
	Path		path;
	RangeData	rdata;
	
private:
	
	PlayerClient	*client;
	Position2dProxy	pp;
	RangerProxy		rp;
	
	RobotHookList 	hooks;
	Path::iterator	currentGoal;
	
};

bool turning = true;

int goToPoint(Robot *robot, Point *at, Vector *velocity) {
	Point dest = robot->GetGoal();
	Point dtrans;
	double accel = 0.0;
	double theta = 0.0, dtheta = 0.0;
	
	// Calculate our relative coordinate
	dtrans  = (Point){ dest.x - at->x, dest.y - at->y };
	
	// Calculate the theta we should be
	theta = at->theta( dest );
	
	// Calculate our delta theta
	dtheta = theta - velocity->direction;
	
	// Calculate our acceleration
	accel = 1.0*((*at) - dest) + 1.0*(0.0 - velocity->magnitude);
	
	if ( turning && fabs(dtheta) > .02 ) {
		velocity->direction = dtheta;
		velocity->magnitude = 0.0;
	}
	
	// If we haven't started moving yet, OR we have and are still far enough
	// away, and we haven't overshot.
	else if ( !velocity->magnitude || velocity->magnitude == 1 || ((*at) - dest > .02 && accel <= 0.0) ) {
		velocity->direction = 0.0;
		velocity->magnitude = (1 < velocity->magnitude + accel) ? 
			1 : velocity->magnitude + accel;
		turning = false;
	} 
	
	// We've gotten to the point, so stop the robot, get the next point, and
	// reset 'turning'
	else {
		velocity->direction = 0.0;
		velocity->magnitude = 0.0;
		robot->GoalAchieved();
		turning = true;
	}
}

int obstacleAvoidance(Robot *robot, Point *at, Vector *velocity) {
	RangeData *rdata = robot->GetRangeData();
	RangeData::iterator it;
	Vector rForce = (Vector){ 0.0, 0.0 };
	Vector currentVelocity = robot->GetVelocity();
	
	// Calculate a repelling force (rForce) to push our robot away
	// from obstacles.
	for ( it = rdata->begin(); it < rdata->end(); it++ ) {
		// Throw out data we don't care about
		if ( it->magnitude == it->magnitude && it->magnitude > 0 ) {
			it->magnitude = 1.0 / (it->magnitude * 2);
			it->direction *= -1.0;
			
			if (rForce.magnitude == 0.0) rForce = *it;
			else rForce = rForce + (*it);
		}
	}
	
	if ( velocity->magnitude > 0 ) {
		velocity->direction = (currentVelocity + rForce).direction - currentVelocity.direction;
	}
}

int main( int argc, char *argv[] ) {
	Path path;
	std::ifstream input;
	char host[] = "localhost:6665";
	
	input.open( argv[2], std::ifstream::in );
	while (!input.eof()) {
		char buff[12];
		Point p;
		
		input.getline( buff, 12 );
		
		if (input.eof()) break;
		
		p.x = atof( strtok( buff, " " ) );
		p.y = atof( strtok( NULL, " " ) );
		
		path.push_back( p );
	}
	
	PlayerClient *client;
	
	// TODO: Fix this!
	if ( argc == 4 ) {
		client = new PlayerClient( argv[3] );
	} else {
		client = new PlayerClient( "localhost", 6665 );
	}
	
	Robot robot( client, path );
	
	robot.AddHook( &goToPoint );
	robot.AddHook( &obstacleAvoidance );
	
	robot.Run();
	
	return 0;
}
