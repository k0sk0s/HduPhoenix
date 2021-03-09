#include "sample.h"

using namespace std;

int SampleWriter::update()
{
    vector<double> v;
    for (int i = 0; i < _vars.size(); i++)
    {
        v.push_back(*(_vars[i].second));
    }
    histroy.push_back(v);
    return histroy.size();
}

void SampleWriter::write()
{
    time_t now_time;
    now_time = time(NULL);    
    ofstream csv(to_string(now_time)+".csv");
    csv << "i";
    for (auto p : _vars)
    {
        csv << ","<< p.first;
    }
    csv << endl;
    for (int i = 0; i < histroy.size(); i++)
    {
        csv << to_string(i+1);
        for (auto p : histroy[i])
        {
            csv << "," << to_string(p);
        }
        csv << endl;
    }
}