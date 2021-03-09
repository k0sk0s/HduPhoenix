#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <typeinfo>
#include<string>
#include "thread"

#include "ArmorDector.h"
#include "general.h"
#include "ArmorDector.h"
#include "opencv_extend.h"
#include "log.h"
#include "ImgProdCons.h"
#include "img_buffer.h"
#include"general.h"
#include"Timestamp.h"
#include "sample.h"
using namespace std;
using namespace cv;
Point2f getSymmetry(Point2f center ,Point2f point ,double radius)
{
    Point2f aimPoint = Point2f(0,0);
    double dis = sqrt((center.x - point.x) * (center.x - point.x) + (center.y - point.y) * (center.y - point.y));
    double angle  = abs(atan((point.y - center.y) / (point.x - center.x)));
    if((point.x - center.x) < 0 && (point.y - center.y) < 0)
    {
        aimPoint.x =center.x + dis * cos(angle);
        aimPoint.y = center.y + dis * sin(angle);
    }
    else  if((point.x - center.x) > 0 && (point.y - center.y) < 0)
    {
        aimPoint.x =center.x - dis * cos(angle);
        aimPoint.y = center.y + dis * sin(angle);
    }
    else if((point.x - center.x) > 0 && (point.y - center.y) > 0)
    {
        aimPoint.x =center.x - dis * cos(angle);
        aimPoint.y = center.y - dis * sin(angle);
    }
    else if((point.x - center.x) < 0 && (point.y - center.y) > 0)
    {
        aimPoint.x =center.x + dis * cos(angle);
        aimPoint.y = center.y - dis * sin(angle);
    }
    else
    {
        aimPoint = center;
    }
    
    return aimPoint;
}
ImgProdCons::ImgProdCons()
{
    sem_init(&sem_pro , 0 , 0 );
    sem_init(&sem_com , 0 , 1 );
    _trackFlag = 0;
    prePoint=Point2f(0,0);
    center = Point2f(0,0);
    _angleSpeed = 0;
    _task = Serial::AUTO_SHOOT;
    _shootTask = Serial::ARMOR_SHOOT;
    bool err = serial.InitPort();
    if(err == Serial::USB_CANNOT_FIND)
    {
        LOG_ERROR << "USB_CANNOT_FIND";
    }
    KF.kalmanInit();
   //初始化相机参数
    p4psolver.SetCameraMatrix(1432.8,1436.1,352.4,295.1);
     //设置畸变参数
    p4psolver.SetDistortionCoefficients(-0.0653 , 0.4867 , 0,0,0);
    //small armor
    p4psolver.Points3D.push_back(cv::Point3f(-30, -60, 0));		//P1三维坐标的单位是毫米
    p4psolver.Points3D.push_back(cv::Point3f(60, -30, 0));	//P2
    p4psolver.Points3D.push_back(cv::Point3f(30, 60, 0));	//P3
    p4psolver.Points3D.push_back(cv::Point3f(-60, 30, 0));	//P4
    //cout << "装甲版世界坐标 = " << endl << p4psolver.Points3D << endl;
}
ImgProdCons::~ImgProdCons()
{
    sem_destroy(&sem_pro);
    sem_destroy(&sem_com);
}
 ImgProdCons* ImgProdCons::getInstance()
{
    instance = new ImgProdCons();
    return instance;
}
void ImgProdCons::Produce()
{
    cv::Mat src;
	//打开
	while (!mycamera.open(0))
		;
	//设置相机参数

	while (!mycamera.setVideoparam(MAT_WIDTH,MAT_HEIGHT))
		;
	//不断读取图片
	while (!mycamera.startStream());
	while (1)
	{
		if (!mycamera.getVideoimage())
		{
			continue;
		}
		if (!mycamera.rgbtocv())
		{
			continue;
		}
        src = mycamera.getiamge();
        if(src.empty())
        {
            LOG_ERROR << "src empty";
            continue;
        }
        else
        {
            sem_wait(&sem_com);
            try
            {
                buffer_.ImgEnterBuffer(src);
            }
            catch (...)
            {
                std::cout << "照片读如出错" << std::endl;
                throw;
            }
            sem_post(&sem_pro);
        }
	}
	while (!mycamera.closeStream());
}
unsigned short int  decode(unsigned char *buff)
{
    if(buff[0] == 'a'  && buff[3] == 'b')
    {
        return (unsigned short int )(buff[2] << 8 | buff[1]);  
    }
}
void ImgProdCons::Sense()
{
    while(1)
    {
        unsigned char buff[7];
        fd_set fdRead;
        FD_ZERO(&fdRead);
        FD_SET(serial.getFd(),&fdRead);
        int fdReturn = select(serial.getFd()+1,&fdRead,0,0,nullptr);
        if(fdReturn <0)
        {
            cout << "select 失败"<<endl;
            continue;
        }
        if(FD_ISSET(serial.getFd(),&fdRead))
        {
            bool is_read=serial.ReadData(buff,7);
           // std::cout << decode(buff) << std::endl;
            // if(buff[0]!= 'a' || buff[2] != 'b')
            // {
            //     std::cout << "串口接收数据错误"  << std::endl;
            //     continue;
            // }
            //puts((const char *)buff);
            if(is_read==false)
            {
                cout  << "读取串口失败" << endl;
                continue;
            }
            // for(int i =0 ;i < 7;i++)
            // {
            //     cout << buff[i] << "   ";
            // }
            // cout << endl;
            if(buff[0] != 's' || buff[6] != 'e')
            {
                //std::cout << "接受数据错误" <<std::endl;
                continue;
            }
            int mode = (int)buff[1];
            switch (mode)
            {
                case 1:
                {
                    _task = Serial::AUTO_SHOOT;
                    _shootTask = Serial::BUFF_SHOOT;
                    break;
                }
                case 2:
                {
                    _task = Serial::AUTO_SHOOT;
                    _shootTask = Serial::ARMOR_SHOOT;
                    break;
                }
                case 3:
                {
                     _task = Serial::NO_TASK;
                    _shootTask = Serial::NO_SHOOT;
                    break;
                }
            }
            union f_data {
                float temp;
                unsigned char fdata[4];
            } float_v;
            for(int i = 2 ; i< 6;i++)
            {
                float_v.fdata[i - 2] = buff[i];
            }
            _mutex.lock();
            _angleSpeed = float_v.temp;
            //std::cout << "_angleSpeed   " << _angleSpeed <<std::endl; 
            _mutex.unlock();
        }
    }
}
void ImgProdCons::Consume()
{
    Mat src;
    int buffindex;
    double px, py;
    SampleWriter sw({{"x", &detect.spd}, {"y", &detect.firSPd}});
    Arm.setEnemyColor(BLUE);
    //Arm.tesseractInit();
    detect.init();
    detect2.init();
    // 1700 21m/s
    int offsetX[8]={23,//右
                                    18,//右上
                                    18,//上
                                    13,//左上
                                    23,//左
                                    23,//左下
                                    15,//下
                                    18};//右下
    int offsetY[8]={105,//右
                                    115,//右上
                                    120,//上
                                    110,//左上
                                    100,//左
                                    100,//左下
                                    100,//下
                                    105};//右下
    while (1)
    {
        double time = _tTime2.getElapsedTimeInMilliSec();
        cout << "time2 :  " << time<<endl;
        _tTime2.update();
        sem_wait(&sem_pro);
        try{
                buffer_.GetImage(src);
        }catch(...){ 
                std::cout << "读取相机图片出错" << std::endl;
                exit(-1);
        }
        sem_post(&sem_com);
        switch (_task)
        {
            case  Serial::AUTO_SHOOT:
                //putText( src, "AUTO_SHOOT", Point(10,30),FONT_HERSHEY_SIMPLEX,0.5, Scalar (0,255,0),2);
                    switch(_shootTask)
                    {
                        case Serial::ARMOR_SHOOT:
                            //putText( src, "ARMOR_SHOOT", Point(10,50),FONT_HERSHEY_SIMPLEX,0.5, Scalar (80,150,80),2);
                            break;
                        case Serial::BUFF_SHOOT:
                            //putText( src, "BUFF_SHOOT", Point(10,50),FONT_HERSHEY_SIMPLEX,0.5, Scalar (80,150,80),2);
                            break;
                    }
                    break;
            case Serial::NO_SHOOT :
                //putText( src, "NO_TASK", Point(10,30),FONT_HERSHEY_SIMPLEX,0.5, Scalar (255,0,0),2);
                    break;
        }
        if (!src.empty())
        {
            cv::imshow("a",src);
            cv::waitKey(1);
        }
        else
        {
            continue;
        }
        

        if (src.size().width !=MAT_WIDTH || src.size().height != MAT_HEIGHT)
        {
            //LOG_ERROR << "size error";
            cv::waitKey(1000);
           continue;
        }
        if(!src.empty() && buffer_.get_headIdx()!=buffindex)
        {
            buffindex = buffer_.get_headIdx();
            if(_task == Serial::AUTO_SHOOT)
            {
                if(_shootTask == Serial::ARMOR_SHOOT)
                {

                    int findEnemy;
                    Arm.loadImg(src);
                    int type = Arm.getArmorType();
                    findEnemy=Arm.detect();
                    if(findEnemy==ArmorDetector::ARMOR_NO)
                    {
                            uint8_t buff[17];
                            buff[0] = 's';
                            for(int i=1; i<16;i++)
                            {
                                buff[i] = '0';
                            }
                            buff[16] = 'e';
                            serial.WriteData(buff, sizeof(buff));
                    }
                    else
                    {
                            Point offset = cv::Point(0,0);
                            std::vector<cv::Point2f>  t =Arm.getArmorVertex();
                            cv::Rect r(t[0].x,t[0].y,t[1].x-t[0].x,t[2].y-t[1].y);
                            changeArmorMode(Arm ,type);
                            cv::Point2f t_rt = cv::Point2f(t[0].x+(t[3].y-t[0].y)*2 ,t[0].y); 
                            cv::Point2f t_rb = cv::Point2f(t[0].x+(t[3].y-t[0].y)*2 ,t[3].y); 
                            p4psolver.Points2D.push_back(t[0]);	//P1
                            p4psolver.Points2D.push_back(t_rt);	//P2
                            p4psolver.Points2D.push_back(t_rb);	//P3
                            p4psolver.Points2D.push_back(t[3]);	//P4
                            //cout << "test1:图中特征点坐标 = " << endl << p4psolver.Points2D << endl;
                            if (p4psolver.Solve(PNPSolver::METHOD::CV_P3P) != 0)
                            {
                                cout << "距离解算错误" <<endl;
                            }
                            double distance  = -p4psolver.Position_OcInW.z / 1000;
                            if(Arm.getArmorType() == BIG_ARMOR)
                            {
                                distance = 2*distance -0.1 + 0.8;
                            }
                            else
                            {
                                distance = distance;
                            }
                            if (p4psolver.Solve(PNPSolver::METHOD::CV_P3P) == 0)
		                            cout <<  "目标距离  =   " << distance << "米" <<endl;
                            prePoint = center;
                            center = Arm.getCenterPoint(Arm);
                            //cout << "center             :  " << abs(prePoint.x - center.x) << endl;
                            //判断小陀螺
                            if(abs(prePoint.x - center.x) >=100)
                            {
                                i ++;
                                if(i >= 10)
                                {
                                    k = 0;
                                    i = 0;
                                    state = 1;
                                }
                            }
                            else
                            {
                                k++;
                                if(k >= 200)
                                {
                                    i = 0;
                                    k = 0;
                                    state = 0;
                                }  
                            }
                            if(state == 1)
                            {
                                cout << "小陀螺" << endl;
                            }
                            cout << "   i    =    " << i << "   k   =   " << k <<endl;
                            offsetSolve solver(center,distance);
                            offset  = solver.getoffset();
                            Point2f predictPoint = KF.kalmanPredict(center.x,center.y);
                            float r1 = abs(predictPoint.x - center.x);
                            float r2 = abs(predictPoint.y - center.y);
                            if(sqrt(r1 * r1 + r2 * r2)> Arm.getArmorLength(Arm))
                            {
                               predictPoint = prePoint;
                            } 
                            if(sqrt(r1 * r1 + r2 * r2) < Arm.getArmorLength(Arm)/1.5)
                            {
                                predictPoint = center;
                            }
                            if(prePoint.x >0 && prePoint.y > 0)
                                circle(src,prePoint,4,Scalar(0, 255, 255),2);
                            if(center.x >0 && center.y > 0)
                                circle(src,center,Arm.getArmorLength(Arm),Scalar(0, 255, 0),2);
                            if(predictPoint.x > 0 && predictPoint.y > 0)
                                circle(src,getSymmetry(center,predictPoint,Arm.getArmorLength(Arm)), 4, Scalar(0, 0, 255), 4);
                            serial.sendBoxPosition(center,serial,1,distance,_angleSpeed,offset);
                            //debug
                            string dis = "distance : ";
                            switch(Arm.getArmorType())
                            {
                                case BIG_ARMOR:
                                    //putText( src, "BIG_ARMOR", Point(100,460),FONT_HERSHEY_SIMPLEX,0.7, Scalar (0,255,255),2);
                                    break;
                                case SMALL_ARMOR:
                                    //putText( src, "SMALL_ARMOR", Point(100,460),FONT_HERSHEY_SIMPLEX,0.7, Scalar (0,255,255),2);
                                    break;
                            }
                            //circle(src,center, 2, Scalar(255, 0, 255), 2);
                            dis += to_string(distance);
                            //putText( src, dis.c_str(), Point(300,460),FONT_HERSHEY_SIMPLEX,0.7, Scalar (0,0,255),3);
                            p4psolver.Points2D.clear();
                            cv::rectangle(src, r, Scalar(0, 255, 255), 3);
                    }
                    cout << endl << endl;
                }
                else if(_shootTask == Serial::BUFF_SHOOT)
                {
                    vector<Point2f> cameraBase;
                    vector<Point2f> cameraUp;
                    vector<Point2f> srcPoints;
                    vector<Point2f> dstPoints;
                    Point2f predictPosition;

                    Mat homographyMat;
                    bool isDetect;
                    if(!_src.empty() && !src.empty())
                    {
                        Mat src2 = _src;
                        cout << "--------------------- base detect ----------------------" << endl;
                        detect2.name = "base";
                        isDetect  = detect2.detect_new(src2);
                        srcPoints.push_back(detect2.getPredictCenter());
                        if(isDetect)
                        {
                            cameraBase = detect2.getBuffPoints();
                            cout << "cameraBase" << cameraBase <<endl;
                           detect2.armorCornerSort(cameraBase);
                        }
                        if (!src2.empty())
                        {
                           imshow("底盘视角",src2);
                        }
                        else
                        {
                            continue;
                        }
                    }
                    cout << "--------------------- up detect ----------------------"<<endl;
                   if( detect.detect_new(src))
                   {
                       detect.name = "upside";
                       cameraUp = detect.getBuffPoints();
                       cout << "cameraUp" << cameraUp << endl;
                       int dstAngle = detect.armorCornerSort(cameraUp) +detect.increaseAngle*360/2.0/3.1415;
                    // int dstAngle = detect.armorCornerSort(cameraUp) ;
                       dstAngle %=360;
                       if(cameraUp.size()==4 && cameraBase.size()==4)
                       {
                            homographyMat = findHomography(cameraBase,cameraUp,0);
                            cout << "Homography Matrix is : \n"<< homographyMat << endl;
                            if(!homographyMat.empty())
                                perspectiveTransform(srcPoints, dstPoints, homographyMat);
                                predictPosition=dstPoints[0];
                            
                            if(dstPoints[0].x !=0 && dstPoints[0].y !=0 && predictPosition.x!=0 && predictPosition.y!=0)
                            {
                                cout << "dstPiont" << dstPoints[0] << endl;
                                if (dstPoints[0].x >0 && dstPoints[0].y>0)
                                    circle(src,dstPoints[0],5,Scalar(0, 255, 0),3);
                                //putText( src,"angle: "+to_string(dstAngle), Point(10,80),FONT_HERSHEY_SIMPLEX,0.5, Scalar (0,255,0),2);
                                int dx = offsetX[dstAngle/45]+(offsetX[(dstAngle/45+1)%8]-offsetX[dstAngle/45])*(dstAngle%45)/45.0;
                                int dy = offsetY[dstAngle/45]+(offsetY[(dstAngle/45+1)%8]-offsetY[dstAngle/45])*(dstAngle%45)/45.0;
                                // cout << "angle: "<< detect.increaseAngle*360/2.0/3.1415 << endl;
                                //putText( src,"dx: "+to_string(dx), Point(10,100),FONT_HERSHEY_SIMPLEX,0.5, Scalar (0,255,0),2);
                                //putText( src,"dy: "+to_string(dy), Point(10,120),FONT_HERSHEY_SIMPLEX,0.5, Scalar (0,255,0),2);
                                serial.sendBoxPosition(dstPoints[0],serial,1,0,0,Point(dx,dy));                                    
                                //上
                                 //serial.sendBoxPosition(dstPoints[0],serial,1,0,0,Point(20,90));
                                // //下
                                 //serial.sendBoxPosition(dstPoints[0],serial,1,0,0,Point(20,85));
                                // //左
                                 //serial.sendBoxPosition(dstPoints[0],serial,1,0,0,Point(25,90));
                                // //右
                                 //serial.sendBoxPosition(dstPoints[0],serial,1,0,0,Point(30,90));
                                // //左上
                                // serial.sendBoxPosition(dstPoints[0],serial,1,0,0,Point(15,110));
                                // //右上
                                //   serial.sendBoxPosition(dstPoints[0],serial,1,0,0,Point(20,115));
                                // //左下
                                 //serial.sendBoxPosition(dstPoints[0],serial,1,0,0,Point(25,90));
                                // //右下
                                // serial.sendBoxPosition(dstPoints[0],serial,1,0,0,Point(20,100));


                                // 数据写到Log里面
                            // if (sw.update() > 3000)
                            // {
                            //     sw.write();
                            // }
                            // if (predictPosition.x >0 && predictPosition.y>0)
                            //     circle(src,predictPosition,5,Scalar(0, 255, 255),3);
                            }
                            
                            
                       }
                       else
                       {
                           uint8_t buff[17];
                            buff[0] = 's';
                            for(int i=1; i<16;i++)
                            {
                                buff[i] = '0';
                            }
                            buff[16] = 'e';
                            serial.WriteData(buff, sizeof(buff));
                       }
                       

                        
                   }
                   else
                   {
                       uint8_t buff[17];
                            buff[0] = 's';
                            for(int i=1; i<16;i++)
                            {
                                buff[i] = '0';
                            }
                            buff[16] = 'e';
                            serial.WriteData(buff, sizeof(buff));
                       continue;
                   }
                   
                }
            }
            else if(_task == Serial::NO_TASK)
            {

                 //std::cout << "change mode to NO_TASK"  << std:: endl;

            } 
        }
    }
}
void ImgProdCons::changeArmorMode(ArmorDetector  &Arm , int type)
{
     if(Arm.getArmorType() != type)
     {
        switch(Arm.getArmorType())
        {
            case SMALL_ARMOR:
                std::cout << "change" << endl;
                    p4psolver.Points3D.clear();
                    p4psolver.Points3D.push_back(cv::Point3f(0, 0, 0));		//P1三维坐标的单位是毫米
                    p4psolver.Points3D.push_back(cv::Point3f(135, 0, 0));	//P2
                    p4psolver.Points3D.push_back(cv::Point3f(60, 135, 0));	//P3
                    p4psolver.Points3D.push_back(cv::Point3f(0, 60, 0));	//P4
                    break;
            case BIG_ARMOR:
                std::cout << "change" << endl;
                p4psolver.Points3D.clear();
                p4psolver.Points3D.push_back(cv::Point3f(0, 0, 0));		//P1三维坐标的单位是毫米
                p4psolver.Points3D.push_back(cv::Point3f(230, 0, 0));	//P2
                p4psolver.Points3D.push_back(cv::Point3f(60, 230, 0));	//P3
                p4psolver.Points3D.push_back(cv::Point3f(0, 60, 0));	//P4
                break;
        }
    }
}
thread ImgProdCons::ConsumeThread()
{
    return thread(&ImgProdCons::Consume ,this);
}
thread ImgProdCons::ProduceThread()
{
    return thread(&ImgProdCons::Produce ,this);
}
thread ImgProdCons::SenseThread()
{
    return thread(&ImgProdCons::Sense ,this);
}

thread ImgProdCons::Produce2Thread()
{
    return thread(&ImgProdCons::Produce2 ,this);
}

bool distanceSolve(double dis)
{
    if(dis < 0.4 || dis > 4) return false;
}

void ImgProdCons::Produce2()
{
    
	cv::Mat src;
	//打开
	while (!mycamera2.open(1))
		;
	//设置相机参数

	while (!mycamera2.setVideoparam(MAT_WIDTH,MAT_HEIGHT))
		;
	//不断读取图片
	while (!mycamera2.startStream());
	while (1)
	{
		if (!mycamera2.getVideoimage())
		{
			continue;
		}
		if (!mycamera2.rgbtocv())
		{
			continue;
		}
        src = mycamera2.getiamge();
        if(src.empty())
        {
            LOG_ERROR << "src empty";
            continue;
        }
        _mutex.lock();
        _src = src;
        _mutex.unlock();
	}
	while (!mycamera2.closeStream());
}
