#include "main.h"
#include "obstacle.h"
#include "tracking.h"
#include "movement.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

#pragma once
using namespace mto;

namespace mto{

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

Motor motor();//馬達設定
rotation tracking();//編碼器設定
extern location current;//起始位置設定
Obstacle obstacle ();//障礙物設定

float heading2(bool reverse =false);//車向角度(-180~180)


}

