#include "main.h"
#include "mtos/movement.h"
#include "mtos/setting.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "pros/vision.h"
#include "pros/vision.hpp"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <vector>



//設定感應器

pros::Imu inertial_sensor(6);
pros::Rotation right_rot(7, false); 
pros::Rotation front_rot(8, true);
void initialize(){

    //感測器初始化

    inertial_sensor.reset();
    inertial_sensor.is_calibrating();
    right_rot.reset_position();
    front_rot.reset_position();
    pros::c::delay(3000);
    
    //啟動odometry 持續記錄車子的座標

    mto::init();
}

int main() {
    
    //以車子中心點為原點向Y正向方向移動180公分 以車頭為0度移動時車頭方向不變 速度為50
    
    mto::move(0, 180, 0, 50);

    //以場地中心為中心點為原點移動到(30,60) 以車頭為0度 到終點時 車頭最終朝向90度 速度為50

    mto::move_abs(30, 60, 90, 50);

    //以車頭為0度 向右轉向135度 速度為50

    mto::turn(50, 135);
        
        }



