#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
class MykalmanKF
{
public:
    MykalmanKF();
    void kalmanInit();
    Point2f kalmanPredict(float x,float y);
    ~MykalmanKF(){}
private:
    int stateNum;    
    int measureNum; 
    Mat measurement;
    KalmanFilter KF;
};