#include"track.h"


void trackArmor::chooseTrackMode(int mode)
{
    switch(mode)
    {
            case BOOSTING :
                tracker = TrackerBoosting::create();
                break;
            case MIL:
                tracker = TrackerMIL::create();
                break;
            case KCF :
                tracker = TrackerKCF::create();
                break;
            case TLD :
                tracker = TrackerTLD::create();
                break;
            case MEDIANFLOW :
                tracker = TrackerMedianFlow::create();
                break;
            case GOTURN:
                tracker = TrackerGOTURN::create();
                break;
            case MOSSE :
                tracker = TrackerMOSSE::create();
                break;
            case CSRT :
                tracker = TrackerCSRT::create();
                break;
    }
}
void trackArmor::trackInit(Mat src ,Rect rect)
{
    tracker->init(src, rect);
}
bool trackArmor::trackUpdate(Mat src,Rect2d &rect)
{
    return tracker->update(src, rect);
}