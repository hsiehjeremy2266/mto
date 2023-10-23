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
//                  座標移動pure pursuit
//
//---------------------------------------------------

//記錄過去的pid值

float old_latKP=1.3;//1.2
float old_ngKP=0.0015;//0.003
float old_OlatKP=0.2;//5
float old_OngKP=0.003;//0.05
float old_latKD=1;//1
float old_ngKD=0.001;//0.001
float old_latKI=0.0001;//0.0001
float old_ngKI=0.000002;//0.000002

void mto::setPID(float latKP,float ngKP,float OlatKP,float OngKP,float latKD,float ngKD,float latKI,float ngKI){
    if (latKP!=-1)old_OlatKP=latKP;
    if (ngKP!=-1)old_ngKP=ngKP;
    if (OlatKP!=-1)old_OlatKP=OlatKP;
    if (OngKP!=-1)old_OngKP=OngKP;
    if (latKI!=-1)old_latKI=latKI;
    if (ngKI!=-1)old_ngKI=ngKI;
    if (latKD!=-1)old_latKD=latKD;
    if (ngKD!=-1)old_ngKD=ngKD;
     
}


float  delta_length,error_delta_length,total_delta_length;

float ACC(float max_speed,float Distance,float input_distance){

  Distance=fabs(Distance);

  input_distance=fabs(input_distance);

  //max_speed[理想最高速度]  

float m=1.5;//斜率[(最佳最高速度-30.0[初始速度])/移動距離)]

float high_speed=20.0+((Distance/2.0)*m);//high_speed[實際最高速度]

    float output_speed=0;

  delta_length=Distance-input_distance;  

if (high_speed>max_speed){

       high_speed=max_speed;

  }

  if (20.0+(input_distance*m)<high_speed){//加速度

    output_speed=20.0+(input_distance*m);

  }

  else if(input_distance>(Distance*2/3)){//減速度

    output_speed=fabs(high_speed-(high_speed*pow(0.9,((delta_length*old_latKP)+(total_delta_length*old_latKI)+((delta_length-error_delta_length)*old_latKD)))));

  }

  else{//等速

    output_speed=high_speed;

  }

  total_delta_length+=delta_length;

  error_delta_length=delta_length;

  return output_speed;

}

void turn_coordinate(float final_theta,bool reverse,float ngKP,float ngKD,float ngKI,float start_theta,float final_speed){

    int condition2=0;

    float total_delta_theta=0,error_delta_theta=0;

    old_ngKP=ngKP;

    old_ngKD=ngKD;

    old_ngKI=ngKI;

    while (!(condition2>5)) {    //當condition1 成立時跳出迴圈時 只校正車子方向

        float delta_theta=final_theta-heading2(reverse) + start_theta;//方向角度差

        delta_theta=delta_theta-((fabs(delta_theta)>180)*(delta_theta/fabs(delta_theta+(delta_theta==0))*360));//算出的值大於180度或小於-180度進行修正

        float turn=((delta_theta*ngKP)+(total_delta_theta*ngKI)+((delta_theta-error_delta_theta)*ngKD))*motor().motor2center*final_speed;//轉換成實際速度

        motor().leftgroup(turn*3.25/motor().wheeldiameter*motor().ratio/1.7);

        motor().rightgroup(-turn*3.25/motor().wheeldiameter*motor().ratio/1.7);    

        condition2+=(fabs(delta_theta)<2);

        //紀錄當前的誤差值

        total_delta_theta+=delta_theta;

        error_delta_theta=delta_theta;

        pros::c::delay(5);   

    }

}

void move_coordinatev3(float target_x,float target_y,float final_theta,float speed,bool reverse ,float latKP,float ngKP,float OlatKP,float OngKP,float latKD,float ngKD,float latKI,float ngKI,bool center){

    float start_x=current.x,start_y=current.y,start_theta=heading2(reverse);//如果執行move()會啟用次變數

    int condition1 = 0 ;//跳出回回圈條件事

    //pid計算所需的變數

    float error_delta_theta=0, total_delta_theta=0.0;

    total_delta_length=0;

    target_x_2=target_x,target_y_2=target_y,reverse2=reverse;//this is for obstacle function

    float final_speed=0;//output motor final velocity

    old_latKP=latKP;//renew the pid value

    old_OlatKP=OlatKP;

    old_OngKP=OngKP;

    old_latKD=latKD;

    old_latKI=latKI;

    while(!condition1){

    Obstacle delta_wall= obstacle();//計算障礙物的距離

    float delta_x = (target_x*(1 - (reverse * 2)))+(start_x*center)-(current.x),delta_y=(target_y*(1 - (reverse * 2)))+(start_y*center)-(current.y); //與終點的座標誤差   

    float target_theta = atan2(delta_x,delta_y)*180.0/M_PI-heading2(reverse);//車子目前的車向到目標點的角度

    target_theta=target_theta-((fabs(target_theta)>180)*(target_theta/fabs(target_theta+(target_theta==0))*360));//算出的值大於180度或小於-180度進行修正

    float final_delta_theta=final_theta - heading2(reverse) + (start_theta*center);//目前車子朝向與目標朝向的角度差

    final_delta_theta= final_delta_theta-((fabs(final_delta_theta)>180)*(final_delta_theta/fabs(final_delta_theta+(final_delta_theta==0))*360));//算出的值大於180度或小於-180度進行修正

    float delta_theta = ( final_delta_theta - (2 * target_theta) ) ;//車子目前的車向到結束後車向的角度

    std::vector<float>start2target={target_x-(start_x*(!center)),target_y-(start_y*(!center))};//起始點到終點的向量(絕對座標)

    std::vector<float>start2current={(current.x)-start_x,(current.y)-start_y};//起始點到車子目前位置的向

    float input_distance = (start2target[0]*start2current[0]+(start2target[1]*start2current[1]))/hypot(start2target[0],start2target[1]);//進度條(車子走的距離)

    float ACCspeed = ACC(speed, hypot(start2target[0],start2target[1]), input_distance);//加減速

    final_speed = (ACCspeed-((delta_wall.error!=0)*ACCspeed*std::pow(0.1,fabs(delta_wall.length)*OlatKP)))*(1 - (reverse * 2));//躲避障礙物減速度修正   (listed)

    float deltaVel = final_speed * (2 * motor().motor2center * std::sin(target_theta * M_PI / 180.0)/hypotf(delta_x, delta_y))*1;//橫移的速度差(走圓)   (listed)

    float turn = ( delta_theta * ngKP + (total_delta_theta*ngKI) + ((delta_theta-error_delta_theta) * ngKD))* motor().motor2center * final_speed;//車子轉向速度差   (listed)

    float Oturn = ( delta_wall.error_angle * OngKP ) * motor().motor2center * final_speed;//障礙物轉向  (listed)  

    motor().leftgroup((final_speed + deltaVel - turn - Oturn)*3.25/motor().wheeldiameter*motor().ratio/1.7);

    motor().rightgroup((final_speed - deltaVel + turn + Oturn)*3.25/motor().wheeldiameter*motor().ratio/1.7);

    total_delta_theta +=delta_theta;//KI

    error_delta_theta=delta_theta;//KD

    condition1=(fabs(delta_x)<5&&fabs(delta_y)<5);    

    pros::c::delay(10);

    }   

    turn_coordinate(final_theta, reverse, ngKP, ngKD, ngKI,start_theta*center,30);

    motor().brake();

    pros::c::delay(10);

}

//----------------------------------------------------
//
//                  人性化
//
//----------------------------------------------------

void mto::move(float target_x,float target_y,float final_theta,float speed,bool reverse ,float latKP,float ngKP,float OlatKP,float OngKP,float latKD,float ngKD,float latKI,float ngKI){

    if(latKP==-1)latKP=old_latKP;

    if(ngKP==-1)ngKP=old_ngKP;

    if(OlatKP==-1)OlatKP=old_OlatKP;

    if(OngKP==-1)OngKP=old_OngKP;

    if(latKD==-1)latKD=old_latKD;

    if(ngKD==-1)ngKD=old_ngKD;

    if(latKI==-1)latKI=old_latKI;

    if(ngKI==-1)ngKI=old_ngKI;

    move_coordinatev3(target_x,target_y,final_theta,speed,reverse,latKP,ngKP,OlatKP,OngKP,latKD,ngKD,latKI,ngKI,true);

}

void mto::move_abs(float target_x,float target_y,float final_theta,float speed,bool reverse ,float latKP,float ngKP,float OlatKP,float OngKP,float latKD,float ngKD,float latKI,float ngKI){

    if(latKP==-1)latKP=old_latKP;

    if(ngKP==-1)ngKP=old_ngKP;

    if(OlatKP==-1)OlatKP=old_OlatKP;

    if(OngKP==-1)OngKP=old_OngKP;

    if(latKD==-1)latKD=old_latKD;

    if(ngKD==-1)ngKD=old_ngKD;

    if(latKI==-1)latKI=old_latKI;

    if(ngKI==-1)ngKI=old_ngKI;

    move_coordinatev3(target_x,target_y,final_theta,speed,reverse,latKP,ngKP,OlatKP,OngKP,latKD,ngKD,latKI,ngKI,false);

}

void mto::turn(float speed,float final_theta,bool reverse,float ngKP,float ngKD,float ngKI){

    if(ngKP==-1)ngKP=old_ngKP;

    if(ngKD==-1)ngKD=old_ngKD;

    if(ngKI==-1)ngKI=old_ngKI;

    turn_coordinate(final_theta, reverse,  ngKP,  ngKD,  ngKI,0,speed);

}