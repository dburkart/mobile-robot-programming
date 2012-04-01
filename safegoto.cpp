/*
 * file: safegoto.cpp
 * 
 * Description: Read in a set of points, and try go to each one while avoiding
 *		obstacles.
 * 
 * Author: Dana Burkart (dsb3573)
 */

#include <iostream>
#include <fstream>
#include <libplayerc++/playerc++.h>
#include <iomanip>
#include <math.h>

using namespace PlayerCc;

struct Point {
	double x, y;
	
	//
	// We can define the minus operator as the euclidean distance 
	// between two points.
	//
	double operator-( const Point& p ) const {
		return sqrt((x - p.x)*(x - p.x) + (y - p.y)*(y - p.y));
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

typedef struct Point Point;
typedef std::vector< Point > Path;

int main( int argc, char *argv[] ) {
	Path path;
	double dtheta = 0.0;
	bool turning = true;
	
	char host[] = "localhost:6665";
	
	int current = 0;
	
	std::ifstream input;
	input.open( argv[2], std::ifstream::in );
	
	while (!input.eof()) {
		char buff[12];
		Point p;
		
		input.getline(buff, 12);
		
		if ( input.eof() ) break;
		
		p.x = atof( strtok( buff, " " ) );
		p.y = atof( strtok( NULL, " " ) );
		
		std::cout << "Adding point: (" << p.x << ", " << p.y << ")" << std::endl;
		
		path.push_back( p );
	}
	
	PlayerClient *robot;
	
	// TODO: Fix this!
	if ( argc == 4 ) {
		robot = new PlayerClient( argv[3] );
	} else {
		robot = new PlayerClient( "localhost", 6665 );
	}
	
	RangerProxy		rp( robot, 0 );
	Position2dProxy	pp( robot, 0 );
	
	while (!rp.IsValid())
		robot->Read();
	
	for(;;)
	{
		Point dest = path[current];
		Point dtrans;
		Point pos;
		double accel = 0.0;
		double yaw;
		double theta = 0.0;
		
		// read from the proxies
		robot->Read();
		
		pos 	= (Point){ pp.GetXPos(), pp.GetYPos() };
		yaw 	= pp.GetYaw();
		dtrans  = (Point){ dest.x - pos.x, dest.y - pos.y };
		
		theta = pos.theta( dest );

		std::cout << "Got data " << rp.GetRangeCount() << std::endl;
		
		std::cout << "Pos: (" << pos.x << "," << pos.y << "), Yaw: " << pp.GetYaw() << std::endl; 
		std::cout << "Distance from (" << dest.x << ", " << dest.y << "): " << pos - dest << ", theta: " << theta << std::endl;
		
		dtheta = theta - yaw;
		
		if (fabs(dtheta) < .001) turning = false;
		if (!turning) dtheta = 0.0;
		
		accel = 1.0*(pos - dest) + 1.0*(0.0 - pp.GetXSpeed());
		
		
		// dtheta == dtheta looks wrong, but it's not! We need to guard against
		// dtheta being NaN.
		if ( dtheta == dtheta && turning ) {
			pp.SetSpeed( 0.0, dtheta );
		} 
		
		// If we haven't started moving yet, OR we have and are still far enough
		// away, and we haven't overshot.
		else if ( !pp.GetXSpeed() || pp.GetXSpeed() == 1 || (pos - dest > .02 && accel <= 0.0) ) {
			pp.SetSpeed( (1 < pp.GetXSpeed() + accel) ? 1 : pp.GetXSpeed() + accel, dtheta );
		} 
		
		// We've gotten to the point, so stop the robot, get the next point, and
		// reset 'turning'
		else {
			pp.SetSpeed( 0.0, 0.0 );
			current += 1;
			turning = true;
		}
		
		std::cout << "dtheta: " << dtheta << std::endl;
		
		if ( current == path.size() ) break;
	}
	
    return 0;
}
