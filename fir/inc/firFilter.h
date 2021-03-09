#ifndef FIRFILTER_H
#define FIRFILTER_H

#include<iostream>
#include<deque>
#include<fftw3.h>

class FirFilter
{
    public:
        FirFilter(){};
        
        void init(int size, double hn[], int len);
        double filter(double x);

    private:
        double *_hn;
        double *_yn;
        double *_xn;
        fftw_complex *XK, *HK, *YK;
        std::deque<double> xn_dq;
        fftw_plan forward, backword;
        int _size=0;
        double filter_out=0;
        int _Ksize=0;
};


#endif
