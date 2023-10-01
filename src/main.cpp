#include "main.h"
#include "mto.h"
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

// /**
//  * Runs initialization code. This occurs as soon as the program is started.
//  *
//  * All other competition modes are blocked by initialize; it is recommended
//  * to keep execution time for this mode under a few seconds.
//  */
 
pros::Imu inertial_sensor(6);
pros::Rotation right_rot(7, false); // port 1, not reversed
pros::Rotation front_rot(8, true);
void initialize(){
    inertial_sensor.reset();
    inertial_sensor.is_calibrating();
    right_rot.reset_position();
    front_rot.reset_position();
    pros::c::delay(3000);
    mto::init();
}

void screen() {
    // loop forever

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    //讀取SD卡檔案
    //file name: path.txt
    //timeout: 2000 ms
    //lookahead distance: 10 inches
    //lookhead值越大，路線越粗糙但越快
    //chassis.follow("path.txt", 20000, 10);


}

void opcontrol() {
    
    // while(1){
        
    //     pros::screen::print(TEXT_MEDIUM,4,"h:%f",mto::tracking.hor());

    //     pros::screen::print(TEXT_MEDIUM,5,"v:%f",mto::tracking.ver());
        
    //     pros::c::delay(10);
        
    //     pros::screen::print(TEXT_MEDIUM,1,"%f",mto::get_current().x);

    //     pros::screen::print(TEXT_MEDIUM,2,"%f",mto::get_current().y);
        
    //     pros::screen::print(TEXT_MEDIUM,3,"%f",mto::get_current().theta);
    
    // }
    mto::move(0, 180, 0, 50);
        // mto::move(30, 30, 0, 50);
        // mto::move(-30, -30, 180, 50);
        // mto::move(30, -30, 30, 50);

        
        }



