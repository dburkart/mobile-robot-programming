// Example of pixel-plotting together with Player/Stage robot control
// zjb 4/10, based on code by John Hawley and original uncredited
//   Player/Stage example code.
// Not guaranteed to be decent code style but seems to work OK.

#include <GL/glut.h>
#include <libplayerc++/playerc++.h>
#include <pthread.h>

using namespace PlayerCc;

#define WIN_X 600
#define WIN_Y 600

static PlayerClient *pRobot;
static Position2dProxy *pPosition;
static SonarProxy *pSonar;

static int good;

// values here should be between 0 and 1 to plot correctly.
// 0 will plot as white and 1 will plot as black.
static double localMap[WIN_X][WIN_Y];

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
  if(good)
    glutPostRedisplay();
}

void* robotLoop(void* args) {

  while(true) {
    
    redisplay();
    
    double turnrate, speed;

    // read from the proxies
    pRobot->Read();

    // Here is where you do your robot stuff
    // including presumably updating your map somehow

  }

}


int main(int argc, char *argv[]) {
  int port = 0;
  char* host = "localhost";

  if (argc > 1) {
    port = atoi( argv[2] );
    host = argv[1];
  }

  pRobot = new PlayerClient( host, port );
  pPosition = new Position2dProxy( pRobot, 0 );
  pSonar = new SonarProxy( pRobot, 0 );

  printf("player connected on port %d, proxies allocated\n", port);
  pPosition->SetMotorEnable(1);
  
  //  pthread_mutex_init(&mut, NULL);
  
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
