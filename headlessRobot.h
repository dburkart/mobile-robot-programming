#ifndef __HEADLESS_ROBOT_H__
#define __HEADLESS_ROBOT_H__

#include "robot.h"

/**
 * The HeadlessRobot class implements a read-only robot for situations where 
 * the robot doesn't necessarily need to be controlled, but data may need to be
 * read (like mapping).
 */
class HeadlessRobot : public Robot {
	public:
		
		HeadlessRobot( PlayerClient* client, bool laser ) 
			: Robot( client, path, laser ) {}
		~HeadlessRobot() {}
		
		int Run();
	
};

#endif

