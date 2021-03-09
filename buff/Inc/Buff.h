//
// Created by yons on 2020/5/21.
//
#ifndef HDU_BUFF_BUFF_H
#define HDU_BUFF_BUFF_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <string.h>
#include <random>
#include <firFilter.h>

#include"Timestamp.h"
using namespace std;
using namespace cv;

#define NO_TARGET -1
#define MAX_NUM 921600

const string lenet_model_file = "/home/qianchen/CLionProjects/HDU_buff/lenet/lenet_iter_80000加了负样本.caffemodel";
const string lenet_txt_file = "/home/qianchen/CLionProjects/HDU_buff/lenet/deploy.prototxt";

//#define GET_ROI

//用于选择我方的颜色
enum detectMode {
    RED_ANCLOCK = 3,
    BLUE_ANCLOCK = 4,
    RED_CLOCK = 5,
    BLUE_CLOCK = 6,
    RED_STATIC = 7,
    BLUE_STATIC = 8
};

//主类
class Detect {
    //二值化模式
    enum binaryMode {
        BGR = 1,
        HSV = 2,
        BGR_useG = 3,
        OTSU = 4,
        GRAY = 5,
        YCrCb = 6,
        LUV = 7,
    };
    enum predictMode {
        FIT_CIRCLE = 1,
        PUSH_CIRCLE = 2,
        TANGENT = 3
    };

    struct armorData {
        Point2f armorCenter;
        Point2f R_center;
        Point2f preArmorCenter;
        Point2f predictCenter;
        vector<Point2f> buffPoints;
        bool wLarger;
        Point2f barycenter;
        double angularVelocity;
        float runTime;
        float prerunTime;
        float radius;
        double preAngularVelocity;
        float angle;
        int quadrant;
        bool isFind;
        armorData() {
            armorCenter = cv::Point2f(0, 0);
            R_center = cv::Point2f(0, 0);
            preArmorCenter = cv::Point2f(0,0);
            predictCenter = cv::Point2f(0,0);
            angle = 0;
            radius =0;
            angularVelocity=0;
            preAngularVelocity = 0;
            quadrant = 0;
            isFind = 0;// 0: 未识别，1: 全部识别到
        }
    };

    struct switchParam {
        bool debug;
        bool use_yolo;
        bool use_lenet;
        switchParam() {
            debug = 1;
            use_yolo = 0;
            use_lenet = 0;
        }
    };

    struct DectParam {
        int bMode;
        int pMode;
        cv::Mat element;
        int  radius;
        float noise_point_area;
        int flabellum_area_min;
        float flabellum_whrio_min;
        float flabellum_whrio_max;
        float armor_whrio_min;
        float armor_whrio_max;
        float armor_rev_thres;
        int armor_area_min;
        int cutLimitedTime;
        float preAngle;
        DectParam() {
            // radius
            radius = 148;
            // mode
            //bMode = BGR_useG;
            bMode=OTSU;
            //bMode = LUV;
            pMode = TANGENT;
            // getArmorCenter
            element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            noise_point_area = 200;//200
            flabellum_area_min = 2000;// standard:7000
            flabellum_whrio_min = 1.5;
            flabellum_whrio_max = 2.7;//standard:2
            armor_whrio_min = 1.5;
            armor_whrio_max = 2.7;// standard:2
            armor_rev_thres = 0.3;// standard: 0.0x
            armor_area_min = 300;
            // cutLimit
            cutLimitedTime = 40;// 400ms
            // predict
            preAngle = CV_PI / 7.8;
        }
    };

private:
    Point2f aimPoint;
  //Time
    CELLTimestamp _tTime;
    // param
    DectParam param;
    switchParam sParam;
    vector<Point2f> fan_armorCenters; // 用来拟合椭圆的装甲板点集
    // init
    int mode= RED_CLOCK;
    Mat debug_src;
    armorData lostData;
    uint frame_cnt = 0;
    bool dirFlag;

    //用于计算一阶差分和二阶差分

    vector<Point2f> threeSamples;//取三个样
    int sampleId=0;
    double accumulationA=0;
    int signA[5];
    int signA_Id=0;
    FirFilter buffFir;

private:
    bool isClockwise(armorData &data);
    float distance(const Point2f pt1, const Point2f pt2) {
        return sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
    }
    int sum(int array[],int cnt)
    {
        int sum=0;
        for(int i=0;i<cnt;i++)sum+=array[i];
        return sum;
    }
    double calAngle(double time ,double t);
    double getAcceleration(double t);
    double factTime(double angularVelocity);
    double countRotationAngle(Point2f nowCenter , Point2f preCenter , double radius , Point2f  roundCenter);
    bool makeRectSafe(const cv::Rect rect, const cv::Size size);
    bool circleLeastFit(std::vector<cv::Point2f> &points, cv::Point2f &R_center,float &radius);
    
public:
    double spd, firSPd;
    Detect(){}
    void init()
    {
        double hn[1000]={-0.001343, -0.000931, 0.000039, 0.002805, 0.008628, 0.018365, 
                0.032094, 0.048911, 0.066982, 0.083851, 0.096930, 0.104077, 0.104077,
                0.096930, 0.083851, 0.066982, 0.048911, 0.032094, 0.018365, 0.008628,
                0.002805, 0.000039, -0.000931, -0.001343};
        buffFir.init(128, hn, 24);
    }
    int armorCornerSort(vector<Point2f> &corners);
    bool setBinary(cv::Mat src, cv::Mat &binary, int bMode);
    bool getArmorCenter_new( cv::Mat &src, const int bMode,armorData &data ,cv::Point2f offset = cv::Point2f(0, 0),const int classiMode=1);
    bool detect_new( Mat  &frame);
    bool preArmorCentor(armorData &data , double time,int pMode);
    Point2f nextCoordinate(Point2f nowCenter ,Point2f R_Center,double increaseAngle ,bool isClockwise,int pMode);
    bool RansacCircle(vector<Point2f>& cloud, cv::Point2f &R_center,float &radius);
    void pushSample()
    {
        if(threeSamples.size()!=3&&data.armorCenter!=Point2f(0,0))
        {
             threeSamples.push_back(data.armorCenter);
             sampleId++;
        }
           
        else if(threeSamples.size()==3&&data.armorCenter!=Point2f(0,0))
        {
            threeSamples[sampleId++]=data.armorCenter;
        }
        if(sampleId == 3 || threeSamples.size() == 0) sampleId=0;
    }
    double A()
    {
        if(threeSamples.size()<3)
        {
            return 0;
        }
        else
        {
            double A=((distance(threeSamples[2],threeSamples[1])/data.runTime)-(distance(threeSamples[1],threeSamples[0])/data.prerunTime))/data.runTime;

            if(A>0) signA[signA_Id++]=1;
            else if(A<0) signA[signA_Id++]=-1;
            else signA[signA_Id++]=0;
            if(signA_Id==5) signA_Id=0;
            cout << "A:  "<<A<<endl;
            return A;
        }
    }
    void calAccumulationA()
    {
        accumulationA+=A();
        if(sum(signA,5)>0 && accumulationA<0) accumulationA=0;
        else if (sum(signA,5)<0 && accumulationA>0) accumulationA=0;
    }
    double linearFuncA(double k)
    {
        calAccumulationA();
        return k*accumulationA;
    }

Point2f getPredictCenter()
    {
        return data.predictCenter;
    }
vector<Point2f> getBuffPoints()
{
    return data.buffPoints;
}
// 返回RotateRect的angle ，配合armorSort使用可获得锤子的实际角度乡县
double getBuffAngle()
{
    return data.angle;
}
    string name;

    armorData data;
    double increaseAngle ;
};


#endif //HDU_BUFF_BUFF_H
