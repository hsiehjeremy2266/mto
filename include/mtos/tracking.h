#pragma once

namespace mto{
    
class location{
    public:
    float x;
    float y;
    float theta;
    
};
location get_current(float x=-1,float y=-1,float theta = -1);

void position();

void init();


};