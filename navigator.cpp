/**
 * file: safegoto.cpp
 * 
 * Description: Read in a set of points, and try go to each one while 
 * 		avoiding obstacles. Because of the restrictions on submitting
 * 		multiple source files, this file is choc-full of data structures.
 * 
 * Author: Hashem Assayari (hya4542), Dana Burkart (dsb3573)
 */

#include <iostream>
#include <fstream>
#include <string>
#include <libplayerc++/playerc++.h>
#include <iomanip>
#include <math.h>

#include "physics.h"
#include "robot.h"

#define MAX_TURNRATE 	1.5
#define MIN_TURNRATE	0.1
#define MAX_XSPEED		0.4

#define PI				3.14159

using namespace PlayerCc;

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
    if ( dtheta > 3.14159 ) dtheta -= 2*3.14159;                                          
    if ( dtheta < -3.14159 ) dtheta += 2*3.14159;
    	
	// Calculate our acceleration
	accel = k_1*((*at) - dest) + k_2*(0.0 - velocity->magnitude);
	
	if (fabs(dtheta) <= MIN_TURNRATE) turning = false; 
	
	//std::cout << "distance: " << (*at) - dest << std::endl;
	//std::cout << "magnitude: " << velocity->magnitude << std::endl;
	//std::cout << "acceleration: " << accel << std::endl;
	
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
	double ignoreAngle = dtor(45);
	
	if ( robot->Ranger() ) {
		ignoreAngle = dtor(35);
	}
	
	// Calculate a repelling force (rForce) to push our robot away
	// from obstacles.
	for ( it = rdata->begin(); it < rdata->end(); it++ ) {
		if ( it->magnitude == it->magnitude && it->magnitude > 0.0 && 
			 it->magnitude < 1.0 && it->direction > ignoreAngle &&
			 it->direction < (dtor(180) - ignoreAngle) ) {
			//std::cout << "Obstacle: ";
			//it->print();
			if (rForce.magnitude == 0.0) rForce = *it;
			else rForce = rForce + (*it);
		}
	}
	
	//std::cout << "rForce: ";
	//rForce.print();
	
	rForce.direction -= (.5 * 3.14159);
	rForce.direction += robot->GetVelocity()->direction;
	
	force = (-rForce) + *robot->GetVelocity();
	
	//std::cout << "force: ";
	//force.print();
	
	if ( fabs(force.direction - robot->GetVelocity()->direction) > MIN_TURNRATE && 
		 velocity->magnitude > 0.0 && rForce.magnitude > 0.0) {
		
		//std::cout << "*Obstacle Avoidance*: ";
		//velocity->print();
		velocity->direction = force.direction;
		
		turning = true;
	}
}

// Localization bookkeeping
bool localizing = true;
int spinning = 0;
double probs[8] = { 1/8, 1/8, 1/8, 1/8, 1/8, 1/8, 1/8, 1/8 };
double data[360];

int localize( Robot *robot, Point *at, Vector *velocity ) {
	Point goal = robot->GetGoal();
	
	if ( goal.x == 0 && goal.y == 0 ) {
		RangeData *d = robot->GetRangeData();
		
		// Turn around 360 degrees
		if ( spinning < 360 ) {
			double avg = 0.0;
			for ( int i = 0; i < d->size(); i++ ) {
				avg += (*d)[i].magnitude;
			}
			avg /= d->size();
			
			data[ (int)rtod(velocity->direction) ] = avg;
			
			velocity->direction += dtor(7);
			
			spinning += 1;
			
		} else {
			double a = 0.0, b = 0.0, sd = 0.0;
			
			for (int i = 0; i < 360; i++) {
				a += data[i] * data[i];
				b += data[i];
			}
			
			sd = sqrt( (a / 360) - ((b / 360) * (b / 360)) );
			
			std::cout << "standard deviation: " << sd << std::endl;
			
		// compare all readings with expected readings
		
		// update probablities based on result of previous step
		
		// if not in ambiguous place:
		// choose the highest probability
		
		//     get path (call PlanPath)
		
		//     update robot's path
		
		// else
		// drive in one direction, use nearest intersection to localize
		}
		
		// zero out velocity->magnitude
		velocity->magnitude = 0;
	}
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
	
	//velocity->print();
}

int main( int argc, char *argv[] ) {
	Path path;
	std::ifstream input;
	PlayerClient *client;
	
	path.push_back( (Point){ 0, 0 } );
	
	if ( argc == 3 ) {
		char *host;
		int port;
		host = strtok( argv[2], ":" );
		port = atoi( strtok( NULL, ":" ) );
		client = new PlayerClient( host, port );
		
	} else {
		//client = new PlayerClient( "129.21.133.159", 6665 );
		client = new PlayerClient( "localhost", 6665 );
	}
	
	Robot robot( client, path, (strcmp( argv[1], "laser" ) == 0) );
	
	robot.AddBehavior( &goToPoint );
	robot.AddBehavior( &obstacleAvoidance );
	robot.AddBehavior( &convertToTurnrate );
	robot.AddBehavior( &localize );
	
	robot.Run();
	
	return 0;
}
