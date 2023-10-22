#pragma once


namespace mto{
    

    class Obstacle{
    public:
    float error;
    float error_angle;
    float angle;
    float length;
    
    Obstacle(float error,float error_angle,float angle,float length) : error(error),error_angle(error_angle),angle(angle),length(length){}
};

    

Obstacle square(float start_x,float start_y,float end_x,float end_y,float width,float height);

Obstacle circle (float center_x,float center_y,float diameter=15.0);

Obstacle sum(const Obstacle& accum,const Obstacle& s);

};