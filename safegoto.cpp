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

#define MAX_TURNRATE 	1.5
#define MIN_TURNRATE	0.1
#define MAX_XSPEED		0.4

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
		double d = direction;
	
		return (Point){ magnitude * cos(d), magnitude * sin(d) };
	}
	
	//
	// Vector addition
	//
	Vector operator+( const Vector& v ) const {
		Point cx = components() + v.components();
		Vector vn;
		
		vn.direction = atan2(cx.y, cx.x);
		vn.magnitude = pow(cx.x*cos(vn.direction), -1);

		return vn;
	}
	
	Vector operator-() const {
		Point c = components();
		c = (Point){-c.x, -c.y};
		return (Vector){atan2(c.y, c.x), magnitude};
	}
	
	int quadrant() const {
		double dir = rtod( direction );
		if ( dir >= 0 && dir <= 90 ) return 0;
		if ( dir > 90 && dir <= 180 ) return 1;
		if ( dir < 0 && dir >= -90 ) return 2;
		return 3;
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

	Robot( PlayerClient *c, Path p, bool laser ) :
		client( c ),
		pp( c, 0 ),
		path( p )
	{
		position = (Point){ 0.0, 0.0 };
		velocity = (Vector){ 0.0, 0.0 };
		currentGoal = path.begin();
		
		if ( laser ) {
			rp = new RangerProxy( c, 0 );
			rdata.resize(102);
			bunchSize = 5;
		} else {
			sp = new SonarProxy( c, 0 );
			rdata.resize(8);
			bunchSize = 1;
		}
		
		ranger = laser;
		
		pp.SetMotorEnable(true);
		pp.ResetOdometry();
	}
	
	~Robot() {}
	
	//
	// Add a processing hook to this robot. The order hooks are called
	// is fifo, so add lowest processing layers first.
	//
	void AddHook( RobotHook h ) {
		hooks.push_back( h );
	}
	
	//
	// Enter the run loop
	//
	int Run() {
		while (ranger && !rp->IsValid()) client->Read();
		
		for(;;) {
			RobotHookList::iterator it;
			UpdateRangeData();
			
			std::cout << "Position: (" << pp.GetXPos() << ", " << pp.GetYPos() << ")\n" << std::endl;
			Point p = position 	= (Point){ pp.GetXPos(), pp.GetYPos() };
			Vector v = velocity = (Vector){ pp.GetYaw(), velocity.magnitude };
			
			for (it = hooks.begin(); it < hooks.end(); it++ ) {
				RobotHook h = *it;
				h( this, &p, &v );
			}
			
			pp.SetSpeed( v.magnitude, v.direction );
			
			// Save our magnitude (no GPS!)
			velocity.magnitude = v.magnitude;
			
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
	
		for (int i = 0; i < rdata.size(); i++ ) {
			Vector vAvg = (Vector){0.0, 0.0};
			double c = 0.0;
			
			for ( int j = 0; j < bunchSize; j++ ) {
				double sample = GetRangeSample((i * bunchSize) + j);
				
				Vector vTemp = (Vector){ ((3.14159 * ((i * bunchSize) + j)) / GetSampleSize()), 
										 sample };
				 
				if ( sample > 0.0 ) {
					if ( j == 0 ) vAvg = vTemp;
					else vAvg.direction += vTemp.direction, vAvg.magnitude += vTemp.magnitude;
					c = c + 1.0;
				}
			}
			
			if ( c > 1.0 ) {
				vAvg.magnitude /= c;
				vAvg.direction /= c;
			}
			
			rdata[i] = vAvg;
		}
	}
	
	RangeData *GetRangeData() {
		return &rdata;
	}
	
	const Vector *GetVelocity() {
		return &velocity;
	}
	
	double GetRangeSample(int n) {
		if ( ranger ) {
			return (*rp)[30 + n];
		} else {
			return (*sp)[GetSampleSize() - n];
		}
	}
	
	int GetSampleSize() {
		if ( ranger ) {
			return 510;
		} else {
			return 8;
		}
	}
	
	bool Ranger() {
		return ranger;
	}

protected:
	
	Point 		position;
	Vector		velocity;
	Path		path;
	RangeData	rdata;
	int			bunchSize;
	
private:
	
	PlayerClient	*client;
	Position2dProxy	pp;
	RangerProxy		*rp;
	SonarProxy		*sp;
	bool 			ranger;
	
	RobotHookList 	hooks;
	Path::iterator	currentGoal;
	
};

bool turning = false;
bool adjusting = true;

double k_1 = 1.0, k_2 = 1.0;

int goToPoint(Robot *robot, Point *at, Vector *velocity) {
	Point dest = robot->GetGoal();
	Point dtrans;
	double accel = 0.0;
	double theta = 0.0, dtheta = 0.0;
	
	// Calculate our relative coordinate
	//dtrans  = (Point){ dest.x - at->x, dest.y - at->y };
	
	// Calculate the theta we should be
	theta = at->theta( dest );
	
	// Calculate our delta theta
	dtheta = theta - velocity->direction;
	
	// Calculate our acceleration
	accel = k_1*((*at) - dest) + k_2*(0.0 - velocity->magnitude);
	
	if (fabs(dtheta) <= MIN_TURNRATE) turning = false; 
	
	std::cout << "distance: " << (*at) - dest << std::endl;
	std::cout << "magnitude: " << velocity->magnitude << std::endl;
	std::cout << "acceleration: " << accel << std::endl;
	
	if ( adjusting && fabs(dtheta) > MIN_TURNRATE ) {
		velocity->direction = theta;
		velocity->magnitude = 0.0;
	}
	
	// If we haven't started moving yet, OR we have and are still far enough
	// away, and we haven't overshot.
	else if ( !velocity->magnitude || velocity->magnitude == MAX_XSPEED || ((*at) - dest > .05 && accel <= 0.0) ) {
		velocity->direction = (turning) ? theta : robot->GetVelocity()->direction;
		velocity->magnitude = ( MAX_XSPEED < velocity->magnitude + accel) ? 
			MAX_XSPEED : velocity->magnitude + accel;
		adjusting = false;
	} 
	
	// We've gotten to the point, so stop the robot, get the next point, and
	// reset 'turning'
	else {
		velocity->direction = robot->GetVelocity()->direction;
		robot->GoalAchieved();
		adjusting = true;
	}
}

int obstacleAvoidance(Robot *robot, Point *at, Vector *velocity) {
	RangeData *rdata = robot->GetRangeData();
	RangeData::iterator it;
	Vector rForce = (Vector){ 0.0, 0.0 }, force;
	const Vector currentVelocity = *robot->GetVelocity();
	
	std::cout << "====== Obstacles ======" << std::endl;
	// Calculate a repelling force (rForce) to push our robot away
	// from obstacles.
	for ( it = rdata->begin(); it < rdata->end(); it++ ) {
		// Throw out data we don't care about (35 degrees for laser)
		if ( it->magnitude == it->magnitude && it->magnitude > 0.0 && 
			 it->magnitude < 1.0 && it->direction > dtor(45) &&
			 it->direction < dtor(135) ) {
			it->print();
			if (rForce.magnitude == 0.0) rForce = *it;
			else rForce = rForce + (*it);
		}
	}
	
	std::cout << "rForce: ";
	rForce.print();
	
	rForce.direction -= (.5 * 3.14159);
	rForce.direction += robot->GetVelocity()->direction;
	
	force = (-rForce) + *robot->GetVelocity();
	
	std::cout << "force: ";
	force.print();
	
	if ( fabs(force.direction - robot->GetVelocity()->direction) > MIN_TURNRATE && 
		 velocity->magnitude > 0.0 && rForce.magnitude > 0.0) {
		
		velocity->direction = force.direction;
		
		turning = true;
	}
	
	std::cout << "resulting vector: ";
	velocity->print();
}

int convertToTurnrate(Robot *robot, Point *at, Vector *velocity) {
	double dtheta = 0.0;
	
	// Calculate our delta theta
	dtheta = velocity->direction - robot->GetVelocity()->direction;
	
	// Make sure dtheta is actually efficient
	dtheta = ( fabs( dtheta ) > dtor( 180 ) ) ? 
		((dtheta > 0.0) ? (dtor( 360 ) - dtheta)*-1.0 : dtor( 360 ) + dtheta) : 
		dtheta;
	
	// Set our velocity's direction to a turnrate
	velocity->direction = (fabs(dtheta) > MAX_TURNRATE) ? ((dtheta > 0.0) ? MAX_TURNRATE : -MAX_TURNRATE) : dtheta;
	
	velocity->print();
}

int main( int argc, char *argv[] ) {
	Path path;
	std::ifstream input;
	
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
	
	if ( argc == 4 ) {
		char *host;
		int port;
		host = strtok( argv[3], ":" );
		port = atoi( strtok( NULL, ":" ) );
		client = new PlayerClient( host, port );
		
	} else {
		//client = new PlayerClient( "129.21.133.159", 6665 );
		client = new PlayerClient( "localhost", 6665 );
	}
	
	Robot robot( client, path, (strcmp( argv[1], "laser" ) == 0) );
	
	robot.AddHook( &goToPoint );
	robot.AddHook( &obstacleAvoidance );
	robot.AddHook( &convertToTurnrate );
	
	robot.Run();
	
	return 0;
}
