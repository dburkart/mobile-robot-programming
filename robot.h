#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "physics.h"

class Robot;

typedef int (*Behavior)(Robot *, Point *, Vector *);
typedef std::vector<Behavior> BehaviorList;

/**
 * The robot class abstracts over a robot, providing an interface for
 * adding processing hooks.
 */
class Robot {
public:

	Robot( PlayerClient *c, Path p, bool laser );
	
	~Robot() {}
	
	//
	// Add a processing hook to this robot. The order hooks are called
	// is fifo, so add lowest processing layers first.
	//
	void AddHook( Behavior h );
	
	//
	// Enter the run loop
	//
	int Run();
	
	Point GetGoal();
	
	void GoalAchieved();
	
	void UpdateRangeData();
	
	RangeData *GetRangeData();
	
	const Vector *GetVelocity();
	
	double GetRangeSample(int n);
	
	int GetSampleSize();
	
	bool Ranger();

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
	
	BehaviorList 	hooks;
	Path::iterator	currentGoal;
};

#endif

