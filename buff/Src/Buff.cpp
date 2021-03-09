#include "Buff.h"
#include <math.h>
#define PI 3.14159265
#define  DATAPOINTS 230


enum classiMode {
    mySonNum = 1,
    useLenet
};
enum buffMode
{
    SMALL_BUFF ,
    BIG_BUFF
};



/// \brief 计算旋转角度
/// \param nowCenter 当前识别中心
/// \param preCenter 上一次识别中心
/// \param radius 拟合圆的半径
/// \param roundCenter 拟合圆的中心
double Detect::countRotationAngle(Point2f nowCenter , Point2f preCenter , double radius , Point2f  roundCenter)
{
    double dis = distance(nowCenter ,preCenter );
    double cos = (2*radius*radius - dis * dis ) / (2 * radius*radius);
    return acos(cos);
}
/// \brief 根据点集使用最小二乘法拟合圆
/// \param points 点集
/// \param R_center 圆心
bool Detect::circleLeastFit( std::vector<cv::Point2f> &points, cv::Point2f &R_center, float &radius) {
    float center_x = 0.0f;
    float center_y = 0.0f;
    //float radius = 0.0f;
    if (points.size() < 3) {
        return false;
    }

    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

    int N = points.size();
    for (int i = 0; i < N; i++) {
        double x = points[i].x;
        double y = points[i].y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }

    double C, D, E, G, H;
    double a, b, c;

    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

    center_x = a / (-2);
    center_y = b / (-2);
    radius = sqrt(a * a + b * b - 4 * c) / 2;
    R_center = cv::Point2f(center_x, center_y);
    return true;
}


/// \brief 二值化图像
/// \param src 原图
/// \param binary 得到的二值图
/// \param bMode 二值方法
bool Detect::setBinary(cv::Mat src, cv::Mat &binary, int bMode) {
    if (src.empty() || src.channels() != 3) return false;
    cv::Mat gray, gray_binary, tempBinary;

    if (bMode == BGR) {
        // 灰度阈值二值
        cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        threshold(gray, gray_binary, 50, 255, cv::THRESH_BINARY);
        //imshow("grayBinary", gray_binary);

        // 红蓝通道相减
        std::vector<cv::Mat> splited;
        split(src, splited);
        if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
            subtract(splited[2], splited[0], tempBinary);//红-蓝
            threshold(tempBinary, tempBinary, 70, 255, THRESH_BINARY);
        } else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
            subtract(splited[0], splited[2], tempBinary);//蓝-红
            threshold(tempBinary, tempBinary, 70, 255, cv::THRESH_BINARY);
        } else {
            return false;
        }
        //dilate(tempBinary, tempBinary, getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1)),Point(-1,-1),3);
        //imshow("tempBinary", tempBinary);
        // mask 操作
        binary = tempBinary & gray_binary;
    } else if (bMode == HSV) {// 如果明的话是v通道，暗的话可以直接用灰度图

        // 亮度图
        cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        threshold(gray, gray_binary, 80, 255, cv::THRESH_BINARY);

        // 颜色阈值分割
        cv::Mat imgHSV;
        cvtColor(src, imgHSV, cv::COLOR_BGR2HSV);
        if (mode == RED_ANCLOCK || mode == RED_CLOCK || mode == RED_STATIC) {
            cv::Mat temp;
            inRange(imgHSV, Scalar(0, 60, 80), Scalar(25, 255, 255), temp);
            inRange(imgHSV, Scalar(156, 60, 80), Scalar(181, 255, 255), tempBinary);
            tempBinary = temp | tempBinary;
        } else if (mode == BLUE_ANCLOCK || mode == BLUE_CLOCK || mode == BLUE_STATIC) {
            inRange(imgHSV, cv::Scalar(35, 46, 80), cv::Scalar(99, 255, 255), tempBinary);
        } else {
            return false;
        }
        //imshow("tempBinary", tempBinary);
        dilate(tempBinary, tempBinary, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        // mask 操作
        binary = tempBinary & gray_binary;
    } else if (bMode == BGR_useG) {
        cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        threshold(gray, gray_binary, 80, 255, THRESH_BINARY);
        //imshow("gray_binary", gray_binary);

        // 与绿通道相减
        std::vector<cv::Mat> splited;
        split(src, splited);
        //Mat tmp_tmp_binary;
        if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
            subtract(splited[2], splited[1], tempBinary);
            //threshold(tempBinary, tmp_tmp_binary, 150, 255, THRESH_BINARY);
            threshold(tempBinary, tempBinary, 80, 255, THRESH_BINARY);
        } else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
            subtract(splited[0], splited[1], tempBinary);
            //threshold(tempBinary, tmp_tmp_binary, 120, 255, THRESH_BINARY);
            threshold(tempBinary, tempBinary, 80, 255, THRESH_BINARY);
        } else {
            return false;
        }
        //imshow("tmp_tmp_binary", tmp_tmp_binary);
        dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(5, 5)));
        //imshow("tempBinary", tempBinary);
        // mask 操作
        binary = tempBinary & gray_binary;
        //erode(binary,binary,getStructuringElement(MORPH_RECT, Size(3, 3)));
        //morphologyEx(binary, binary, MORPH_OPEN,getStructuringElement(MORPH_RECT, Size(3, 3)));
    } else if (bMode == OTSU) {
        // 大津算法
        cvtColor(src, gray, COLOR_BGR2GRAY);
        double test = threshold(gray, tempBinary, 0, 255, THRESH_OTSU);// 可以得出一个阈值
        cout << "test:" << test << endl;
        binary = tempBinary;
    } else if (bMode == GRAY) {
        // 灰度阈值
        cvtColor(src, gray, COLOR_BGR2GRAY);
        threshold(gray, gray_binary, 40, 255, THRESH_BINARY);
        binary = gray_binary;
    } else if (bMode == YCrCb) {

        Mat Ycrcb;
        cvtColor(src, Ycrcb, COLOR_BGR2YCrCb);
        vector<Mat> splited;
        split(Ycrcb, splited);

        // 亮度图
        threshold(splited[0], gray_binary, 60, 255, THRESH_BINARY);

        // cr和cb通道
        if (mode == RED_CLOCK || mode == RED_ANCLOCK || mode == RED_STATIC) {
            subtract(splited[1], splited[2], tempBinary);
            threshold(tempBinary, tempBinary, 20, 255, THRESH_BINARY);
        } else if (mode == BLUE_CLOCK || mode == BLUE_ANCLOCK || mode == BLUE_STATIC) {
            subtract(splited[2], splited[1], tempBinary);
            threshold(tempBinary, tempBinary, 40, 255, THRESH_BINARY);
        } else {
            return false;
        }

        dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(3, 3)));
        // mask 操作
        binary = tempBinary & gray_binary;
    } else if (bMode == LUV) {
        Mat luv;
        cvtColor(src, luv, COLOR_BGR2Luv);
        vector<Mat> splited;
        split(luv, splited);

        // 亮度图
        threshold(splited[0], gray_binary, 60, 255, THRESH_BINARY);

        // 颜色阈值
        if (mode == RED_ANCLOCK || mode == RED_CLOCK || mode == RED_STATIC) {
            threshold(splited[2], tempBinary, 160, 255, THRESH_BINARY);
        } else if (mode == BLUE_ANCLOCK || mode == BLUE_CLOCK || mode == BLUE_STATIC) {
            threshold(splited[1], tempBinary, 70, 255, THRESH_BINARY_INV);

        } else {
            return false;
        }
       // imshow("tempBinary", tempBinary);
        //dilate(tempBinary, tempBinary, getStructuringElement(MORPH_RECT, Size(3, 3)));

        // mask操作
        binary = gray_binary & tempBinary;
    } else {
        return false;
    }

    return true;
}

//返回锤子的实际角度与装甲顶点排序
// 所有角度都为正
int Detect::armorCornerSort(vector<Point2f> &corners)
{
    if (corners.size() == 0)    return -1;
    int angle =-data.angle;
    Point2f center1, center2;
    double dist1, dist2;
    if (data.wLarger)
    {
        center1 = (corners[0]+corners[3])/2;
        center2 = (corners[1]+corners[2])/2;
        dist1 = distance(center1, data.barycenter);
        dist2 = distance(center2, data.barycenter);
        if (dist1 > dist2)
        {
            swap(corners[0], corners[2]);
            swap(corners[1], corners[3]);
            angle +=270;
        }
        else
        {
            angle +=90;
        }
        
    }
    else 
    {
        center1 = (corners[0]+corners[1])/2;
        center2 = (corners[2]+corners[3])/2;
        dist1 = distance(center1, data.barycenter);
        dist2 = distance(center2, data.barycenter);
        if (dist1 > dist2)
        {
            swap(corners[0], corners[3]);
            swap(corners[1], corners[2]);
            swap(corners[1], corners[3]);
            angle +=180;
        }
        else
        {
            swap(corners[0], corners[1]);
            swap(corners[1], corners[2]);
            swap(corners[2], corners[3]);
            angle += 0;
        }
    }
    return angle%360;
}


/// 新版本，思想是希望把已经击打过的装甲板中心也用在椭圆拟合上
/// \brief 检测装甲板_mode2
/// \param src 原图
/// \param bMode 二值方法
/// \param data 装甲板信息
/// \param offset ROI的偏移

static float radius_sum=0;
static int radius_num=0;
static int center_index = 0; // 用于表示所找到的装甲板中心的下标（个数）

bool Detect::getArmorCenter_new(Mat &src, const int bMode, armorData &data, Point2f offset, const int classiMode) 
{
   data.buffPoints.clear();
    /********************************* 二值化 ************************************/
    Mat binary;
    if (setBinary(src, binary, bMode) == false)
        return false;
    dilate(binary, binary, param.element,Point(-1,-1));// 膨胀程度
    Mat elementex = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    morphologyEx(binary,binary,MORPH_CLOSE,elementex,Point(-1,-1),1);
   if (sParam.debug)
        imshow(name, binary);

    /******************************* 反转二值图 *************************************/

    vector<vector<Point> > armorContours;
    vector<Vec4i> armorHierarchy;

    findContours(binary, armorContours, armorHierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    int armorContours_size = armorContours.size();
    //int findCount[armorContours_size]={0};
    int *findCount = new int[armorContours_size + 10]();

    /*********************************筛选条件1：只有存在子轮廓的轮廓才可能是扇叶***************************************/

    for (size_t i = 0; i < armorContours_size; ++i) {
        // 选择有父轮廓的
        if (armorHierarchy[i][3] != -1)//armorHierarchy[i]是有父轮廓的轮廓
        {// 可以尝试加入0个的
            // 去掉噪点
            if ((contourArea(armorContours[i]) > param.noise_point_area) && (armorHierarchy[i][3] >= 0)) {
                // armorHierarchy[i][3]是父轮廓，也就是其父轮廓的值++
                // 最后findCount就代表了该父轮廓所拥有的子轮廓数目
                findCount[armorHierarchy[i][3]]++;
            }
        }
    }
    /*********************************分类模式1：根据子轮廓的数目进行分类*******************************/
    /*适用于符文扇叶完整，并且光照条件良好情况
     * 优点：速度快
     * 缺点：没有考虑语义信息，鲁棒性不够
     * */

    Point2f center;
    Point2f predictAimPoint;
    int hammerToFall;//用于统计本帧中是否存在锤子
    if (classiMode==mySonNum) {
        for (size_t i = 0; i < armorContours_size; ++i)// 遍历每个轮廓
        {
            // drawContours(src,armorContours,i,Scalar(0,255,0));

            double contour_area = contourArea(armorContours[i]);
            if (contour_area < 2000) continue; // 如果面积太小了，那就直接不管了

            //如果子轮廓只有一个，大概率是个锤子
            if (findCount[i] == 1) {
                hammerToFall++;

                int son = armorHierarchy[i][2];//子轮廓的ID
                //cout<<"son "<<i<<": "<<son<<endl;

                //调试
                if ((son != -1) && (armorContours[son].size() > 2))//子轮廓ID不为-1，并且该轮廓点集数目必须大于2(才可以获得外接矩形)
                {
                    RotatedRect son_rect = minAreaRect(armorContours[son]);//锤子中子轮廓（装甲板）的外接矩形
                    RotatedRect dad_rect = minAreaRect(armorContours[i]);//锤子中子轮廓（装甲板）的外接矩形
                    if (son_rect.center == Point2f(0, 0)) return false;
                    Point2f p[4];//装甲板四个点
                    data.wLarger = son_rect.size.width > son_rect.size.height; 
                    data.angle = son_rect.angle;
                    data.barycenter = dad_rect.center;
                    son_rect.points(p);
                    for (int j = 0; j < 4; j++)//画出装甲板矩形
                    {
                        data.buffPoints.push_back(p[j]);
                        line(src, p[j], p[(j + 1) % 4], Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条边
                    }
                    if(data.armorCenter.x != 0 && data.armorCenter.y != 0)
                    {
                        data.preArmorCenter  =  data.armorCenter;
                    }
                    data.armorCenter = son_rect.center + offset;//在原图中的坐标
                     if(abs(data.armorCenter .x - data.preArmorCenter.x ) > 10 && abs(data.armorCenter.y - data.preArmorCenter.y)> 10)
                    {
                        data.preArmorCenter = data.armorCenter;
                    }
                    spd = distance(data.preArmorCenter, data.armorCenter) / data.runTime;
                    firSPd = buffFir.filter(spd);
                    data.prerunTime=data.runTime;
                    data.runTime = _tTime.getElapsedTimeInMilliSec();
                    if(preArmorCentor(data ,0.7,SMALL_BUFF) == false)
                    {
                        return false;
                    }
                    _tTime.update();
                    //std::cout << data. preArmorCenter<< endl;
                    data.predictCenter = aimPoint;
                    std::cout << "运行时间   " << data. runTime <<  endl << endl;

                    std::cout << "预测   " << data.predictCenter<<endl;
                }
                pushSample();
                //将找到的点放入点集中为后面拟合椭圆做准备
                if (data.armorCenter != Point2f(0, 0)) {
                    if (fan_armorCenters.size() < DATAPOINTS) {
                        fan_armorCenters.push_back(data.armorCenter);
                    } else if (fan_armorCenters.size() >= DATAPOINTS) {
                        if (center_index == DATAPOINTS)
                        {
                            center_index = 0;
                            radius_sum=0;
                            radius_num=0;
                        }
                        fan_armorCenters[center_index] = data.armorCenter;
                        center_index++;
                        //cout<<"center_index:"<<center_index<<endl;

                    }
                }
                else
                {
                    //圆拟合数据重置
                    fan_armorCenters.clear();
                    radius_sum=0;
                    radius_num=0;
                }
                //cout<<"size:"<<fan_armorCenters.size()<<endl;

            } else if (findCount[i] > 2)//子轮廓>2，有可能是六边形
            {
                /*//用来画六边形的轮廓*/
                drawContours(src,armorContours,i,Scalar(0,255,0));
                RotatedRect FatherRect = minAreaRect(armorContours[i]);//有可能是六边形的轮廓
                if (FatherRect.size.width * FatherRect.size.height < 5000) continue;//六边形外接矩形面积不能太小


                vector<int> sons;
                int final_son = armorHierarchy[i][2];
                //把所有子轮廓找到，然后把其下标放入sons
                while (final_son != -1) {
                    //画出所有子轮廓
                    //cout<<"final_son:"<<final_son<<endl;
                    drawContours(src,armorContours,final_son,Scalar(0,255,255));

                    //子轮廓都放入sons中
                    sons.push_back(final_son);
                    final_son = armorHierarchy[sons.back()][0];
                }
                //for(int it =0;it<sons.size();it ++) cout<<sons[it]<<" ";  //子轮廓序号

                vector<RotatedRect> SonsRect;
                for (int s = 0; s < sons.size(); s++) {
                    RotatedRect SonRect = minAreaRect(armorContours[sons[s]]);
                    SonsRect.push_back(SonRect);
                }

                //调试：画出六边形外接矩形
                Point2f p[4];
                FatherRect.points(p);
                for (int j = 0; j < 4; j++) {
                    line(src, p[j], p[(j + 1) % 4], Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条边
                }
                

                //调试
                // cout << "父亲的角度：" << "(" << FatherRect.angle << ")" << endl;
                // cout << "差值：" << endl;//认为父轮廓的角度应该是相近的
                int qualifiedSubAngle = 0;//如果有子轮廓和父轮廓角度的差值小于5就++
                for (int s = 0; s < sons.size(); s++) {
                    //画出子轮廓
                    Point2f p2[4];
                    SonsRect[s].points(p2);
                    for (int j = 0; j < 4; j++) {
                        line(src, p2[j], p2[(j + 1) % 4], Scalar(0, 255, 255), 2, 8);  //绘制最小外接矩形每条边
                    }

                    // 角度差统计
                    // if (abs(SonsRect[s].angle - FatherRect.angle) < 5.0) qualifiedSubAngle++;
                    // cout << "(" << SonsRect[s].angle << "-" << FatherRect.angle << ")"
                    //      << abs(SonsRect[s].angle - FatherRect.angle) << endl;
                }
                //
                if (qualifiedSubAngle < 2) continue;

                vector<float> Sons_whratio;//子轮廓的长宽比
                for (int j = 0; j < SonsRect.size(); j++) {
                    float a = SonsRect[j].size.height;
                    float b = SonsRect[j].size.width;
                    if (a < b) swap(a, b);
                    Sons_whratio.push_back(a / b);
                }

                /*//调试：写出每个子轮廓的长宽比
                cout<<"ratios:"<<endl;
                for(auto ratio : Sons_whratio)
                    cout<<ratio<<" ";*/


                //利用子轮廓的长宽比（过于简单）来区分装甲板和其他两个扁长的矩形
                for (int j = 0; j < Sons_whratio.size(); j++) {
                    if ((Sons_whratio[j] > 1) && (Sons_whratio[j] < 2.5)&&((SonsRect[j].center+offset)!=Point2f(0, 0))) {
                        if (fan_armorCenters.size() < DATAPOINTS) {
                            fan_armorCenters.push_back(SonsRect[j].center + offset);
                        } else if (fan_armorCenters.size() >= DATAPOINTS) {
                            if (center_index == DATAPOINTS)
                            {
                                center_index = 0;
                                radius_sum=0;
                                radius_num=0;
                            }
                            fan_armorCenters[center_index] = SonsRect[j].center + offset;
                            center_index++;
                        }
                    }
                }


            } else {
                continue;
            }

        }
    }
    /*********************************分类模式2：利用Lenet进行分类*******************************/

    else if (classiMode==useLenet){

    }
    if (hammerToFall == 0) {
        cout << endl;
        cout << "本帧中没有找到锤子!" << endl;
        return false;
    }//不再进行圆的拟合


    /*********************拟合圆************************/
    //static float radius_sum;
    //static int radius_num;
    float radius_temp=0.0;
    circleLeastFit(fan_armorCenters, data.R_center, radius_temp);

    if(radius_temp>0.0)
    {
        radius_sum+=radius_temp;
        radius_num++;
    }

    float radius=radius_sum/radius_num;
    data .radius =radius;
    if (sParam.debug && name == "base"&& !src.empty()) {
        if(data.armorCenter.x >0 && data.armorCenter.y > 0)
            circle(src, data.armorCenter, 5, Scalar(255, 255, 255), 2);
        if(data.preArmorCenter.x > 0 && data.preArmorCenter.y >0)
        {
            circle(src , data.preArmorCenter , 5, Scalar(255,0,0),2);
        }
       // circle(src, data.preArmorCenter, 5, Scalar(255, 0, 0), 2);
       if(data.predictCenter .x > 0 && data.predictCenter.y > 0)
       {
           circle(src, data.predictCenter, 5, Scalar(255, 255, 0), 2);
       }
        if(radius>0 && data.R_center.x>0 && data.R_center.y >0)
        {
            circle(src, data.R_center, 5, Scalar(255, 255, 255), 2);
            circle(src, data.R_center, radius, Scalar(20, 100, 100), 2);
        }
   }
    // imshow("father_son", src);
    delete[]findCount;
    return true;
}

bool Detect::detect_new( Mat & frame) {
    // setImage
    Point2f offset = Point2f(0, 0);
    if (getArmorCenter_new(frame, 3, data, offset,mySonNum) == true)
        return true;
    return false;
}


# define INCREASEGAIN 1.1
bool  Detect::preArmorCentor(armorData &data , double time ,int pMode)
{
    if (abs(firSPd) < 0.01) firSPd=0;
    if(isClockwise(data))
    {
        std::cout << "isClockwise   " << endl;
        if(pMode == SMALL_BUFF)
        {
            // increaseAngle = time * (2 * PI / 6)* INCREASEGAIN+linearFuncA(1);//-----------------------增加的角度-------------------//
            // aimPoint = nextCoordinate(data.armorCenter , data.R_center , increaseAngle , true ,SMALL_BUFF);
                        float gianAccumulation=0;
            gianAccumulation = firSPd *48;
            increaseAngle = gianAccumulation;
            aimPoint = nextCoordinate(data.armorCenter , data.R_center , increaseAngle , true ,SMALL_BUFF);
             data.predictCenter = aimPoint;
        }
        else if(pMode == BIG_BUFF)
        {
            //increaseAngle = time * (2 * PI / 6)* INCREASEGAIN+linearFuncA(10);//-----------------------增加的角度-------------------//
            //aimPoint = nextCoordinate(data.armorCenter , data.R_center , increaseAngle , true ,SMALL_BUFF);
             float gianAccumulation=0;
            gianAccumulation = firSPd *48;
            increaseAngle = gianAccumulation;
            aimPoint = nextCoordinate(data.armorCenter , data.R_center , increaseAngle , true ,SMALL_BUFF);
            data.predictCenter = aimPoint;
        }
    }
    else
    {
         if(pMode == SMALL_BUFF)
        {
             //increaseAngle = time * (2 * PI / 6)* INCREASEGAIN;


            float gianAccumulation=0;
            gianAccumulation = firSPd *48;
            increaseAngle = gianAccumulation;
            
            aimPoint = nextCoordinate(data.armorCenter , data.R_center , increaseAngle , false ,SMALL_BUFF);
            data.predictCenter = aimPoint;
        }
        else if(pMode == BIG_BUFF)
        {
            float gianAccumulation=0;
            gianAccumulation = firSPd *48;
            increaseAngle = gianAccumulation;
            aimPoint = nextCoordinate(data.armorCenter , data.R_center , increaseAngle , false ,SMALL_BUFF);
            data.predictCenter = aimPoint;
        }
    }
    cout << "firSPd" << firSPd << endl;
    return true;
}

bool Detect::isClockwise(armorData &data)
{
    Point2f preCenter = data.preArmorCenter;
    Point2f nowCenter = data.armorCenter;
    Point2f R_Center = data.R_center;
    if(nowCenter.x < R_Center.x  && nowCenter.y <= R_Center.y )  //第一向限
    {
            if(preCenter.x <= nowCenter.x && preCenter.y > nowCenter.y)
            {
                return true;
            }
            return false;
    }
    else if(nowCenter.x > R_Center.x  && nowCenter.y <= R_Center.y )  //第二向限
    {
        if(preCenter.x <= nowCenter.x && preCenter.y < nowCenter.y)
        {
            return true;
        }
        return false;
    }
    else if(nowCenter.x > R_Center.x  && nowCenter.y >= R_Center.y )  //第三向限
    {
        if(preCenter.x >= nowCenter.x  &&  preCenter.y < nowCenter.y)
        {
            return true;
        }
        return false;
    }
    else if(nowCenter.x < R_Center.x  && nowCenter.y >= R_Center.y )  //第四向限
    {
        if(preCenter.x >= nowCenter.x  &&  preCenter.y > nowCenter.y)
        {
            return true;
        }
        return false;
    }
}

Point2f Detect::nextCoordinate(Point2f nowCenter ,Point2f R_Center,double increaseAngle ,bool isClockwise ,int pMode)
{

    Point aim = Point2f(0, 0);
    cout << "isClockwise:  " << isClockwise << endl; 
    if( !isClockwise )   //逆时针
    {
        aim.x = (nowCenter.x - R_Center.x) * cos(-increaseAngle) - (nowCenter.y - R_Center.y) * sin(-increaseAngle) + R_Center.x;
        aim.y = (nowCenter.x - R_Center.x) * sin(-increaseAngle) + (nowCenter.y - R_Center.y) * cos(-increaseAngle) + R_Center.y;
    }
    else    //顺时针
    {
        aim.x = (nowCenter.x - R_Center.x) * cos(increaseAngle) - (nowCenter.y - R_Center.y) * sin(increaseAngle) + R_Center.x;
        aim.y = (nowCenter.x - R_Center.x) * sin(increaseAngle) + (nowCenter.y - R_Center.y) * cos(increaseAngle) + R_Center.y;
    }
    return aim;
}

double Detect::factTime(double angularVelocity)
{
    return asin((angularVelocity - 1.305)  / 0.785 ) / 1.884 ;
}

double Detect::getAcceleration(double t)
{
    return 1.47894 * cos(1.884 * t);
}

double Detect::calAngle(double time ,double t)
{
    return (-1 * 0.785 / 1.884) *(cos(1.884 * (time + t )) - cos(1.884 * t) ) + 1.305 * time ;  
}


bool Detect::RansacCircle(vector<Point2f>& cloud, cv::Point2f &R_center,float &radius)
{
    int cloudSize = cloud.size();
    int maxIterCnt = cloudSize/2;  //最大迭代次数
    double maxErrorThreshold = 20; //最大误差阈值（其实就是点到圆周的距离阈值）
    int consensusCntThreshold = maxIterCnt;

    if (!cloudSize || cloudSize <= 3)
    {
        return false;
    }

    //在点云链表的下标取值范围内产生随机数，目的是可以随机选择一个点云
    std::default_random_engine rng;
    std::uniform_int_distribution <int> uniform(0, cloudSize - 1);
    rng.seed(777);  //seed函数可以接收任意整数作为实参

    vector<int> selectIndexs;            //随机选择的点云的索引
    vector<Point2f> selectPoints;   //随机选择的点云对象
    vector<int> consensusIndexs;         //满足一致性条件的点云的索引

    double centerX = 0, centerY = 0, R = 0; //平面圆参数
    double modelMeanError = 0;              //平均误差
    int bestConsensusCnt = 0;               //满足一致性的点的个数
    int iter = 0;                           //迭代次数

    //开始迭代计算
    while (iter < maxIterCnt)
    {
        selectIndexs.clear();
        selectPoints.clear();
        //在原始点云中随机取出3个点，因为至少需要3个点才能确定一个平面圆
        for (int c = 0; c < 3; ++c)
        {
            selectPoints.push_back(cloud.at(uniform(rng)));
        }
        Point2f center;
        float R;
        circleLeastFit(selectPoints, center,R);
        centerX = center.x;
        centerY = center.y;

        double meanError = 0;
        vector<int> tmpConsensusIndexs; //满足一致性条件的点索引集合，这只是临时索引集合，需要随时更新。
        for (int j = 0; j < cloudSize; ++j)
        {
            const Point2f & point = cloud.at(j);
            double distance = abs(R - sqrt((point.x - centerX)*(point.x - centerX) + (point.y - centerY)*(point.y - centerY)));
            if (distance < maxErrorThreshold)
            {
                tmpConsensusIndexs.push_back(j);
            }
            meanError += distance;
        }

        if (tmpConsensusIndexs.size() >= bestConsensusCnt && tmpConsensusIndexs.size() >= consensusCntThreshold)
        {
            bestConsensusCnt = consensusIndexs.size();  // 更新一致性索引集合元素个数
            modelMeanError = meanError / cloudSize;
            consensusIndexs.clear();
            consensusIndexs = tmpConsensusIndexs;        // 更新一致性索引集合
            R_center.x = centerX;  //圆心X
            R_center.y = centerY;  //圆心Y
            radius = R;        //半径
        }
        iter++;
    }
    return true;
}