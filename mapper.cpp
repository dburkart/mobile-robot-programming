// Example of pixel-plotting together with Player/Stage robot control
// zjb 4/10, based on code by John Hawley and original uncredited
//   Player/Stage example code.
// Not guaranteed to be decent code style but seems to work OK.

#include <GL/glut.h>
#include <libplayerc++/playerc++.h>
#include <pthread.h>
#include <math.h>

#include "physics.h"
#include "robot.h"
#include "headlessRobot.h"

using namespace PlayerCc;

#define WIN_X 	600
#define WIN_Y 	600
#define PPM		20				// pixels per meter

#define E       2.718

static PlayerClient *pRobot;
static HeadlessRobot *robot;

static int good;

// values here should be between 0 and 1 to plot correctly.
// 0 will plot as white and 1 will plot as black.
static double localMap[WIN_X][WIN_Y];

static double oddsMap[WIN_X][WIN_Y];

static void display() {
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(	0, WIN_X, 0, WIN_Y );

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  glBegin( GL_POINTS );
  for( int x = 0; x < WIN_X; x++ ) {
    for( int y = 0; y < WIN_Y; y++ ) {
      glColor3f( 1-localMap[x][y], 1-localMap[x][y], 1-localMap[x][y] );
      
      glVertex2i(x, y);
    }
  }
  glEnd();
  
  // Flush the pipeline
  glFlush();

  // Swap the buffers now that we're done drawing.
  glutSwapBuffers();
  good = 1;
}

void redisplay() {
  for (int i = 0; i < WIN_X * WIN_Y; i++ ) {
  	double odds = oddsMap[ i % WIN_X ][ i / WIN_Y ];
  	localMap[ i % WIN_X ][ i / WIN_Y ] = odds / (odds + 1);
  }
  

  if(good)
    glutPostRedisplay();
}

void mapVector( Point a, Vector v ) {
	double dist = 0.0, mag = v.magnitude;
	Point b = a + v.components();
	
	if ( v.magnitude < 5.0 ) {
		
		while (dist < mag ) {
			if ( dist > .5 ) {
				oddsMap [ (WIN_X / 2) + (int)floor(PPM * b.x) ]
						[ (WIN_Y / 2) + (int)floor(PPM * b.y) ] *= .3;
			} else {
			
			double odds = oddsMap [ (WIN_X / 2) + (int)floor(PPM * b.x) ]
								  [ (WIN_Y / 2) + (int)floor(PPM * b.y) ];
			odds = (pow( E, -1.0 * dist) / (1 - pow( E, -1.0 * dist) + .1)) *
					odds;
					
			oddsMap [ (WIN_X / 2) + (int)floor(PPM * b.x) ]
					[ (WIN_Y / 2) + (int)floor(PPM * b.y) ] = odds;
			
			}
			
			dist += .1;
			v.magnitude -= .1;
			b = a + v.components();
		}
	}
}

int mapper(Robot *robot, Point *at, Vector *velocity) {
	RangeData *sensorData = robot->GetRangeData();
	
	redisplay();
	
	for ( int i = 0; i < sensorData->size(); i++ ) {
		Vector v = (*sensorData)[i];
		v.direction += velocity->direction;
		
    	//if ( v.direction > 3.14159 ) v.direction -= 2*3.14159;                                          
    	//if ( v.direction < -3.14159 ) v.direction += 2*3.14159;
    	
    	for ( double j = dtor(15); j >= 0.0; j -= .01 ) {
    		mapVector( *at, (Vector){v.direction + j, v.magnitude} );
    		mapVector( *at, (Vector){v.direction - j, v.magnitude} );
		}
	}
}

void* robotLoop(void* args) {
	robot->Run();
}

int main(int argc, char *argv[]) {
  int port = 0;
  char* host = "localhost";

  if (argc > 1) {
    port = atoi( argv[2] );
    host = argv[1];
  }

  pRobot = new PlayerClient( host, port );
  robot = new HeadlessRobot( pRobot, false );
  robot->AddBehavior( &mapper );
  
  for ( int i = 0; i < WIN_X * WIN_Y; i++ ) {
  	oddsMap[ i / WIN_X ][ i % WIN_Y ] = 1;
  	localMap[ i / WIN_X ][ i % WIN_Y ] = 1;
  }
  
  pthread_t thread_id;
  pthread_create(&thread_id, NULL, robotLoop, NULL);
  
  glutInit( &argc, argv );
  glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE );
  glutInitWindowPosition( 50, 50 );
  glutInitWindowSize( WIN_X, WIN_Y );
  glutCreateWindow( "Map" );
  
  // Callbacks
  glutDisplayFunc( display );

  glutMainLoop();
   
  return 0;
}
