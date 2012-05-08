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
	// Add a processing behavior to this robot. The order hooks are called
	// is fifo, so add lowest processing layers first.
	//
	virtual void AddBehavior( Behavior h );
	
	//
	// Enter the run loop
	//
	int Run();
	
	Point GetGoal();
	
	void GoalAchieved();
	
	void UpdateRangeData();
	
	void UpdatePath( Path );
	
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
	
	PlayerClient	*client;
	Position2dProxy	pp;
	RangerProxy		*rp;
	SonarProxy		*sp;
	bool 			ranger;
	
	BehaviorList 	hooks;
	Path::iterator	currentGoal;
};

#endif

