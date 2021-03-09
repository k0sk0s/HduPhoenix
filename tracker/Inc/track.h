#include<iostream>
#include<string>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

using namespace std;
using namespace cv;
enum
{
    BOOSTING = 100, 
    MIL,
    KCF,
    TLD,
    MEDIANFLOW,
    GOTURN, 
    MOSSE, 
    CSRT
};
class trackArmor
{
public:
    trackArmor(){}
    void chooseTrackMode(int mode);
    void trackInit(Mat src  ,Rect rect);
    bool trackUpdate(Mat src,Rect2d &rect);
    ~trackArmor(){}
private:
    Ptr<Tracker> tracker;
};