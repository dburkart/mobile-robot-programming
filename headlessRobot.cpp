#include "headlessRobot.h"

int HeadlessRobot::Run() {
    while (ranger && !rp->IsValid()) client->Read();

    for(;;) {
        BehaviorList::iterator it;
        UpdateRangeData();

      	Point p = position 	= (Point){ pp.GetXPos(), pp.GetYPos() };
        Vector v = velocity = (Vector){ pp.GetYaw(), velocity.magnitude };

        for (it = hooks.begin(); it < hooks.end(); it++ ) {
            Behavior h = *it;
            h( this, &p, &v );
        }

        client->Read();
    }
}
