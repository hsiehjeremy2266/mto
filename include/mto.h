
#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"


#ifndef MTO_H_
#define MTO_H_

namespace  mto {


//define funtion: 
    class Motor{

    public:

    pros::Motor *left_front;
    pros::Motor *right_front;
    pros::Motor *left_back;
    pros::Motor *right_back;

    float motor2center;
    float wheeldiameter;
    float max_width;
    float ratio;
    
    void leftgroup(double LVel);
    void rightgroup(double RVel);
    void brake();
};
class rotation{
    public:
    pros::Rotation *front;
    pros::Rotation *back;
    pros::Rotation *right;
    pros::Rotation *left;
    float wheeldiameter=3.25;//inch
    pros::Imu *inertial_sensor;
    float front_length;
    float back_length;
    float right_length;
    float left_length;
    
    float hor();
    float ver();
    float heading();
};

class encoder{
    public:
    pros::ADIEncoder *front;
    pros::ADIEncoder *back;
    pros::ADIEncoder *right;
    pros::ADIEncoder *left;
    float wheeldiameter;//inch
    pros::Imu *inertial_sensor;
    float front_length;
    float back_length;
    float right_length;
    float left_length;

    float hor ();
    float ver (); 
    float heading();
};

class location{
    public:
    float x;
    float y;
    float theta;
    
};
class Obstacle{
    public:
    float error;
    float error_angle;
    float angle;
    float length;
    
    Obstacle(float error,float error_angle,float angle,float length) : error(error),error_angle(error_angle),angle(angle),length(length){}
};



//function:
void setPID(float latKP=-1,float ngKP=-1,float OlatKP=-1,float OngKP=-1,float latKD=-1,float ngKD=-1,float latKI=-1,float ngKI=-1); 

location get_current(float x=-1,float y=-1,float theta = -1);

Obstacle square(float start_x,float start_y,float end_x,float end_y,float width,float height);

Obstacle circle (float center_x,float center_y,float diameter=15.0);

Obstacle obstacle ();

Obstacle sum(const Obstacle& accum,const Obstacle& s);

//main function:

void init();//背景odometry

void move_abs(float target_x,float target_y,float final_theta,float speed,bool reverse=false ,float latKP=-1,float ngKP=-1,float OlatKP=-1,float OngKP=-1,float latKD=-1,float ngKD=-1,float latKI=-1,float ngKI=-1);

void move(float target_x,float target_y,float final_theta,float speed,bool reverse=false ,float latKP=-1,float ngKP=-1,float OlatKP=-1,float OngKP=-1,float latKD=-1,float ngKD=-1,float latKI=-1,float ngKI=-1);

void turn(float speed,float final_theta,bool reverse=false,float ngKP=-1,float ngKD=-1,float ngKI=-1);


//========================================================================
//
//                       initial settup
//
//========================================================================
static pros::Rotation right_rot(7, false); // port 1, not reversed
static pros::Rotation front_rot(8, true);
static pros::Imu imu(6);
static pros::Motor left_front_motor(1, true); // port 1, reversed			是否接返
static pros::Motor left_back_motor(2, true); // port 2, reversed
static pros::Motor right_front_motor(9, false); // port 3, not reversed
static pros::Motor right_back_motor(10, false); // port 4,not  reversed

static Motor motor{&left_front_motor,&right_front_motor,&left_back_motor,&right_back_motor,12.7,3.25,20.7,1.7};
static rotation tracking{&front_rot,nullptr,&right_rot,nullptr,2.75,&imu,16,0,6,0};
static location current{0,0,0};


};



#endif