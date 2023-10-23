#include "setting.h"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

using namespace mto;

//----------------------------------------------------
//
//                  里程計odometry
//
//----------------------------------------------------

float prehorizontal=0;

float prevertical=0;

float preheading=0;

void position(){

    //輸出編碼器的值

    float raw_hor=tracking().hor();
    
    float raw_ver=tracking().ver();

    float raw_imu=heading2();

    //編碼器和陀螺儀的變化量

    float delta_hor=raw_hor-prehorizontal;

    float delta_ver=raw_ver-prevertical;

    float delta_heading=raw_imu-preheading;

    delta_heading=delta_heading-((fabs(delta_heading)>180)*(delta_heading/fabs(delta_heading+(delta_heading==0))*360));//算出的值大於180度或小於-180度進行修正

    //轉成實際車子橫移的座標量
    
    float delta_x=delta_hor+(delta_heading*M_PI/180.0*((tracking().front_length-tracking().back_length)/((tracking().front!=nullptr)+(tracking().back!=nullptr))));

    float delta_y=delta_ver+(delta_heading*M_PI/180.0*((tracking().right_length-tracking().left_length)/((tracking().right!=nullptr)+(tracking().left!=nullptr))));

    //實際座標

    current={

        current.x+(delta_y*std::sin(float(raw_imu*M_PI/180.0)))-(delta_x*std::cos(float(raw_imu*M_PI/180.0))),

        current.y+(delta_y*std::cos(float(raw_imu*M_PI/180.0)))+(delta_x*std::sin(float(raw_imu*M_PI/180.0))),

        heading2(),

        };

    //記錄過去的編碼器值
    
        prehorizontal=raw_hor;

        prevertical=raw_ver;

        preheading=raw_imu;    

}

pros::Task*position_task=nullptr;

void mto::init() {

        position_task = new pros::Task {[=] {

            while(true){
            
            position();

            pros::c::delay(10);
            
            }
        }

    };

}
