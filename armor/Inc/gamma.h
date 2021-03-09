#ifndef __GAMMA_H
#define __GAMMA_H
#include<iostream>
#include<opencv2/opencv.hpp>


void gammaCorrect(const cv::Mat src, cv::Mat &dst, float gamma);

#endif
