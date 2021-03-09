#ifndef _SAMPLE_H
#define _SAMPLE_H

#include<iostream>
#include<vector>
#include<fstream>
#include<ctime>

using namespace std;


class SampleWriter
{
    public:
        SampleWriter(std::vector<std::pair<std::string, double*>> vars):
            _vars(vars){}
        bool  flag;
        int update();
        void write();
    private:
        std::vector<std::pair<std::string, double*>> _vars;
        std::vector<vector<double>> histroy;
};


#endif
