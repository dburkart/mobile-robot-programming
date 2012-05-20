#ifndef __PHYSICS_H__
#define __PHYSICS_H__

#include <math.h>
#include <libplayerc++/playerc++.h>

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
	
	bool operator==( const Point& p) const {
		
		if (p.x == x && p.y == y)
			return true;
		else
			return false;
		
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

	//
	// Calculates the distance from this point to point p;
	//
	double distanceTo(Point p) {

		return (double) sqrt(	(p.x - x) * (p.x - x) +
					(p.y - y) * (p.y - y) );

	}
};

typedef struct Point Point;
typedef std::vector< Point > Path;

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

typedef struct Vector Vector;
typedef std::vector<Vector> RangeData;

#endif

