#ifndef __SAFE_GOTO_H__
#define __SAFE_GOTO_H__

//
// Point definitions
//
struct Point;
typedef struct Point Point;
typedef std::vector< Point > Path;

//
// Vector definitions
//
struct Vector;
typedef struct Vector Vector;
typedef std::vector<Vector> RangeData;

//
// Robot definitons
//
class Robot;
typedef int (*RobotHook)(Robot *, Point *, Vector *);
typedef std::vector<RobotHook> RobotHookList;

#endif
