#include "firFilter.h"

void FirFilter::init(int size, double hn[], int len)
{
    _size = size;
    _Ksize = _size/2 +1;
    _hn = new double[size]();
    _yn = new double[size]();
    _xn = new double[size]();

    for (int i = 0; i < len; i++)
    {
        _hn[i] = hn[i];
    }
    XK = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * _size);
    HK = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * _size);
    YK = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * _size);
    forward = fftw_plan_dft_r2c_1d(_size, _hn, HK, FFTW_ESTIMATE);
    backword = fftw_plan_dft_c2r_1d(_size, YK, _yn, FFTW_ESTIMATE);
    fftw_execute_dft_r2c(forward, _hn, HK);
}

double FirFilter::filter(double x)
{
    xn_dq.push_back(x);
    if (xn_dq.size() >= _size)
    {
        xn_dq.pop_front();
        for (int i = 0; i < _size; i++)
        {
            _xn[i] = xn_dq[i];
        }
        fftw_execute_dft_r2c(forward, _xn, XK);
        for (int i = 0; i< _Ksize;i++)
        {
            YK[i][0] = (XK[i][0] * HK[i][0] - XK[i][1] * HK[i][1])/1000.0;
            YK[i][1] = (XK[i][0] * HK[i][1] + XK[i][1] * HK[i][0])/1000.0;
        }
        fftw_execute_dft_c2r(backword, YK, _yn);
        return _yn[_size-1];
    }
    return 0;

}