
#include"kalmanKF.h"
MykalmanKF::MykalmanKF()
{
    stateNum = 4;
    measureNum = 2;
    measurement = Mat::zeros(measureNum, 1, CV_32F);
}
void MykalmanKF::kalmanInit()
{
    KF.init(stateNum,measureNum); 
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
    KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,//A 状态转移矩阵
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);
    //这里没有设置控制矩阵B，默认为零
    setIdentity(KF.measurementMatrix);//H=[1,0,0,0;0,1,0,0] 测量矩阵
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪声，单位阵
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪声，单位阵
    setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值

}
Point2f MykalmanKF::kalmanPredict(float x ,float y)
{
     //2.kalman prediction
    Mat prediction = KF.predict();
    Point predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));

    measurement.at<float>(0) = x;
    measurement.at<float>(1) = y;
    KF.correct(measurement);
    return predict_pt;
}
