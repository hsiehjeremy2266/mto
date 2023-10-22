#include "setting.h"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

using namespace mto;

//設定:

static pros::Rotation right_rot(7, false); // port 1, not reversed
static pros::Rotation front_rot(8, true);
static pros::Imu imu(6);
static pros::Motor left_front_motor(1, true); // port 1, reversed			
static pros::Motor left_back_motor(2, true); // port 2, reversed
static pros::Motor right_front_motor(9, false); // port 3, not reversed
static pros::Motor right_back_motor(10, false); // port 4,not  reversed

//設定成mto版本
Motor mto::motor(){
    return {
    &left_front_motor,
    &right_front_motor,
    &left_back_motor,
    &right_back_motor,
    12.7,
    3.25,
    20.7,
    1.7};
}

rotation mto::tracking(){
    return {
    &front_rot,
    nullptr,
    &right_rot,
    nullptr,
    2.75,
    &imu,
    16,
    0,
    6,
    0};
}

location mto::current{0,0,0};

Obstacle mto::obstacle (){

    std::vector<Obstacle>obstacle_i={
        //write here

        square(-7.5, 60, 7.5, 75, 15, 15)

    };

    return std::accumulate(obstacle_i.begin(),obstacle_i.end(),Obstacle(0.0,0.0,0.0,0.0),sum);

}


//----------------------------------------------------
//
//                  sensors output
//
//----------------------------------------------------
void mto::Motor::leftgroup(double LVel){

        this->left_front->move_velocity(LVel);

        this->left_back->move_velocity(LVel);

    };

void mto::Motor::rightgroup(double RVel){

        this->right_front->move_velocity(RVel);

        this->right_back->move_velocity(RVel);

}

void mto::Motor::brake(){

        this->left_front->brake();

        this->left_back->brake();

        this->right_front->brake();

        this->right_back->brake();

    };

float mto::rotation::hor(){

    float value=0;

    int count=0;

    if(this->front!=nullptr){

        value+=this->front->get_position();

        count+=1;

    }

    if(this->back!=nullptr){

        value+=this->back->get_position();

        count+=1;

    }



    return value/count*this->wheeldiameter*M_PI/36000.0*2.54;

}

float mto::rotation::ver(){

    float value=0;

    int count=0;

    if(this->right!=nullptr){

        value+=this->right->get_position();

        count+=1;

    }

    if(this->left!=nullptr){

        value+=this->left->get_position();

        count+=1;

    }

    return value/count*this->wheeldiameter*M_PI/36000.0*2.54;



}

float mto::rotation::heading(){

    return this->inertial_sensor->get_heading();

}

float mto::encoder::hor() {

    float value=0;

    int count=0;

    if(this->front!=nullptr){

        value+=this->front->get_value();

        count+=1;

    }

    if(back!=nullptr){

        value+=this->back->get_value();

        count+=1;

    }

    return value/count*this->wheeldiameter*M_PI/360.0*2.54;

    }

float mto::encoder::ver(){

    float value=0;

    int count=0;

    if(right!=nullptr){

        value+=this->right->get_value();

        count+=1;

    }

    if(left!=nullptr){

        value+=this->left->get_value();

        count+=1;

    }

    return value/count*this->wheeldiameter*M_PI/360.0*2.54;

}

float mto::encoder::heading(){

    return this->inertial_sensor->get_heading();

}

float mto::heading2(bool reverse){//轉成範圍180~-180

    return (tracking().heading()+(reverse*180)-(360*(tracking().heading()+(reverse*180)>180)));//*(1-(reverse*2));

}

