#include "robot.h"

Robot::Robot( PlayerClient *c, Path p, bool laser ) :
    client( c ),
    pp( c, 0 ),
    path( p )
{
    position = (Point){ 0.0, 0.0 };
    velocity = (Vector){ 0.0, 0.0 };
    currentGoal = path.begin();

    if ( laser ) {
        rp = new RangerProxy( c, 0 );
        rdata.resize(102);
        bunchSize = 5;
    } else {
        sp = new SonarProxy( c, 0 );
        rdata.resize(8);
        bunchSize = 1;
    }

    ranger = laser;

    pp.SetMotorEnable(true);
    pp.ResetOdometry();
}

//
// Add a processing hook to this robot. The order hooks are called
// is fifo, so add lowest processing layers first.
//
void Robot::AddBehavior( Behavior h ) {
    hooks.push_back( h );
}

//
// Enter the run loop
//
int Robot::Run() {
    while (ranger && !rp->IsValid()) client->Read();

    for(;;) {
        BehaviorList::iterator it;
        UpdateRangeData();

        //std::cout << "Position: (" << pp.GetXPos() << ", " << pp.GetYPos() << ")\n" << std::endl;
        Point p = position 	= (Point){ pp.GetXPos(), pp.GetYPos() };
        Vector v = velocity = (Vector){ pp.GetYaw(), velocity.magnitude };

        for (it = hooks.begin(); it < hooks.end(); it++ ) {
            Behavior h = *it;
            h( this, &p, &v );
        }

        pp.SetSpeed( v.magnitude, v.direction );

        // Save our magnitude (no GPS!)
        velocity.magnitude = v.magnitude;

        if ( currentGoal == path.end() ) break;

        client->Read();
    }
}

Point Robot::GetGoal() {
    return *currentGoal;
}

void Robot::GoalAchieved() {
    currentGoal++;
}

void Robot::UpdateRangeData() {

    for (int i = 0; i < rdata.size(); i++ ) {
        Vector vAvg = (Vector){0.0, 0.0};
        double c = 0.0;

        for ( int j = 0; j < bunchSize; j++ ) {
            double sample = GetRangeSample((i * bunchSize) + j);

            Vector vTemp = (Vector){ ((3.14159 * ((i * bunchSize) + j)) / GetSampleSize()), 
                sample };

            if ( sample > 0.0 ) {
                if ( j == 0 ) vAvg = vTemp;
                else vAvg.direction += vTemp.direction, vAvg.magnitude += vTemp.magnitude;
                c = c + 1.0;
            }
        }

        if ( c > 1.0 ) {
            vAvg.magnitude /= c;
            vAvg.direction /= c;
        }

        rdata[i] = vAvg;
    }
    
}

void Robot::UpdatePath( Path p ) {
	path = p;
}

RangeData *Robot::GetRangeData() {
    return &rdata;
}

const Vector *Robot::GetVelocity() {
    return &velocity;
}

double Robot::GetRangeSample(int n) {
    if ( ranger ) {
        return (*rp)[30 + n];
    } else {
        return (*sp)[GetSampleSize() - n];
    }
}

int Robot::GetSampleSize() {
    if ( ranger ) {
        return 510;
    } else {
        return 8;
    }
}

bool Robot::Ranger() {
    return ranger;
}
