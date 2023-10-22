

#pragma once



namespace mto {

void setPID(float latKP=-1,float ngKP=-1,float OlatKP=-1,float OngKP=-1,float latKD=-1,float ngKD=-1,float latKI=-1,float ngKI=-1); 

void move_abs(float target_x,float target_y,float final_theta,float speed,bool reverse=false ,float latKP=-1,float ngKP=-1,float OlatKP=-1,float OngKP=-1,float latKD=-1,float ngKD=-1,float latKI=-1,float ngKI=-1);

void move(float target_x,float target_y,float final_theta,float speed,bool reverse=false ,float latKP=-1,float ngKP=-1,float OlatKP=-1,float OngKP=-1,float latKD=-1,float ngKD=-1,float latKI=-1,float ngKI=-1);

void turn(float speed,float final_theta,bool reverse=false,float ngKP=-1,float ngKD=-1,float ngKI=-1);

extern float target_x_2,target_y_2;

extern bool reverse2;

// void move_coordinatev3(float target_x,float target_y,float final_theta,float speed,bool reverse ,float latKP,float ngKP,float OlatKP,float OngKP,float latKD,float ngKD,float latKI,float ngKI,bool center);

// void turn_coordinate(float final_theta,bool reverse,float ngKP,float ngKD,float ngKI,float start_theta,float final_speed);

};
