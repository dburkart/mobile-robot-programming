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
#include "planning.h"

#define MAX_TURNRATE 	1.5
#define MIN_TURNRATE	0.1
#define MAX_XSPEED		0.4

using namespace PlayerCc;

bool turning = false;
bool adjusting = true;
bool localizing = true;

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
    if ( dtheta > PI ) dtheta -= 2*PI;                                          
    if ( dtheta < -PI ) dtheta += 2*PI;
    	
	// Calculate our acceleration
	accel = k_1*((*at) - dest) + k_2*(0.0 - velocity->magnitude);
	
	if (fabs(dtheta) <= MIN_TURNRATE) turning = false; 
	
	if ( !localizing ) {
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
	
	rForce.direction -= (.5 * PI);
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
bool travelling = false;
int spinning = 0, ind = -1;
double probs[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double data[360];
Path goals;

Point bounds[6] = {
		(Point){ .109, .116 },			// 6665, 6672
		(Point){ .117, .125 },			// 6666, 6667
		(Point){ .135, .153 },			// 6668
		(Point){ .126, .134 },			// 6669
		(Point){ 0.0 , .108 },			// 6670
		(Point){ .154, .2 }				// 6671
	};
	
Point initials[8] = {
		(Point){ 8.5, -7.0 },			// 6665
		(Point){ -3.0, -9.8 },			// 6666
		(Point){ -43.0, -9.8 },			// 6667
		(Point){ -27.0, 13.1 },			// 6668
		(Point){ -11.7, 13.1 },			// 6669
		(Point){ 21.0, 9.5 },			// 6670
		(Point){ 27.0, 13.3 },			// 6671
		(Point){ 8.5, 3.5 }				// 6672
		
	};

Point intersections[8] = {
		(Point){ 8.6, 11.65},
		(Point){ 8.5, -8.5 },
		(Point){ 7.5, -10.0 },
		(Point){ -46.5, -9.9 },
		(Point){ -47.5, -8.8 },
		(Point){ -47.5, 11.8 },
		(Point){ -46.5, 13.1 },    
		(Point){ 7.5, 13.1 },
	};

int localize( Robot *robot, Point *at, Vector *velocity ) {
	Point goal = robot->GetGoal();
	
	if ( goal.x == 0 && goal.y == 0 ) {
		RangeData *d = robot->GetRangeData();
		
		if ( ind != -1 ) {
			double left, right;
			
			left = robot->GetRangeSample( 1 );
			right = robot->GetRangeSample( 8 );
			
			std::cout << left << ", " << right << std::endl;
			
			if ( left <= right + .02 && left >= right - .02 &&
				 left < 1.1  && right < 1.1 ) {
				 travelling = true;
				 velocity->magnitude = .5;
			} else {
				if ( !travelling ) {
					velocity->direction -= dtor(7);
				} else {
					if ( left < 4.9 && right < 4.9) {
						if ( left > right ) {
							velocity->direction += dtor( 7 );
						} else {
							velocity->direction -= dtor( 7 );
						}
					} else {
						double distance = (Point){0.0, 0.0} - (*at);
						double best = -1.0;
						double yaw;
						int bestIndex = -1;
						
						velocity->magnitude = 0;
						
						for ( int i = 0; i < 8; i++ ) {
							if ( probs[i] == .5 ) {
								double a, b, closest;
								
								switch (i) {
									case 0:
										a = intersections[0] - initials[0];
										b = intersections[1] - initials[0];
										yaw = PI/2;
										
										break;
									case 7:
										a = intersections[0] - initials[7];
										b = intersections[1] - initials[7];
										yaw = PI/2;
										
										break;
									case 1:
										a = intersections[2] - initials[1];
										b = intersections[3] - initials[1];
										yaw = PI;
										
										break;
									case 2:
										a = intersections[2] - initials[2];
										b = intersections[3] - initials[2];
										yaw = 0.0;
										
										break;
								}
								
								closest = fabs(a - distance);
								closest = (closest < fabs(b - distance)) ? closest : fabs(b - distance);
								
								if ( bestIndex == -1 || closest < best ) {
									bestIndex = i;
									best = closest;
								}
							}
						}
						
						robot->SetInternals( initials[bestIndex], yaw );
						robot->UpdatePath( PlanPath( robot->ToLocal( *at ), goals, robot ) );
						localizing = false;
					}
				}
			}
		}
		// Turn around 360 degrees
		else if ( spinning < 360 ) {
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
			
			//std::cout << "standard deviation: " << sd << std::endl;
			
			for (int i = 0; i < 6; i++) {
				if ( sd >= bounds[i].x && sd <= bounds[i].y ) {
					ind = i;
					break;
				}
			}
			
			switch (ind) {
				case 0:
					probs[ 0 ] = probs[ 7 ] = .5;
					break;
				case 1:
					probs[ 1 ] = probs[ 2 ] = .5;
					break;
				default:
				
					double yaw;
					
					switch (ind) {
						case 2:
							yaw = 0.0;
							break;
						case 3:
						case 4:
							yaw = -.5 * PI;
							break;
						case 5:
							yaw = PI / 2;
							break;
					}
					
					robot->SetInternals( initials[ind + 1], yaw );
					robot->UpdatePath( PlanPath( robot->ToLocal( *at ), goals, robot ) );
					localizing = false;
					
					break;
			}
		}
		
		// zero out velocity->magnitude
		if ( localizing && !travelling ) {
			velocity->magnitude = 0;
		}
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
	
	input.open( argv[2], std::ifstream::in );
	while (!input.eof()) {
		char buff[12];
		Point p;
		
		input.getline( buff, 12 );
		
		if (input.eof()) break;
		
		p.x = atof( strtok( buff, " " ) );
		p.y = atof( strtok( NULL, " " ) );
		
		goals.push_back( p );
		
	}
	
	path.push_back( (Point){ 0.0, 0.0 } );
	
	if ( argc == 4 ) {
		char *host;
		int port;
		host = strtok( argv[3], ":" );
		port = atoi( strtok( NULL, ":" ) );
		client = new PlayerClient( host, port );
		
	} else {
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
