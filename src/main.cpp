#include "main.h"
#include "okapi/api.hpp"
#include <vector>
#include "PID.h"
#include <iostream>

using namespace okapi;
//update pros.c
//pros v5 run 8
pros::Controller master(pros::E_CONTROLLER_MASTER);
//Drivetrain
pros::Motor right0(2,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor right1( 3,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor right2(4,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor left0(17,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor left1(13,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor left2(14,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
// Intake
pros::Motor left_intake(16,true);
pros::Motor right_intake(15,false);
//Rollers
pros::Motor bottom_rollers0(7,true);
pros::Motor bottom_rollers1(8,true);

pros::Motor top_roller(5,false);

pros::Imu imu (20);

std::vector<pros::Motor> leftDrive;
std::vector<pros::Motor> rightDrive;

using Motorgroup = std::vector<pros::Motor>;

double last_target = 0;
double last_current = 0;

void run_pid(
        std::function<double()> current_function,
        const std::function<void(double)>& apply_function,
        double target,
        double p,
        double i,
        double d,
        double max_i,
        double threshold,
        uint32_t count_needed,
        uint32_t timeout_ms
){
    auto end = pros::millis() + timeout_ms;
    last_target = target;
    last_current = current_function();
    SpencerPID::PID pid{target, std::move(current_function), p, i, d, max_i};
    uint32_t count = 0;
    while (count < count_needed && (timeout_ms == 0 || pros::millis() < end)){
        auto out = pid.run();
        if (std::abs(out.error) < threshold){
            count++;
        }
        else{
            count = 0;
        }
        apply_function(out.power);
        pros::delay(5);
    }
    apply_function(0);
}
//tick to inches
double ticktoinch=49.2;
//PID constants
#define ROBOT_TARGET_15
#ifdef ROBOT_TARGET_15
//turning
constexpr double TURN_P = 10;
constexpr double TURN_I = 1;
constexpr double TURN_D = 0.8;
constexpr double TURN_I_MAX = 50;
//driving
constexpr double DRIVE_P = 0.5;
constexpr double DRIVE_I = 0.0001;
constexpr double DRIVE_D = 0.001;
constexpr double DRIVE_I_MAX = 30;
//secondary driving making sure angle doesn't change
constexpr double DRIVE_MATCH_P = 2;
constexpr double DRIVE_MATCH_I = 0;
constexpr double DRIVE_MATCH_D = 0;
constexpr double DRIVE_MATCH_MAX_I = 50;
#endif
#ifdef ROBOT_TARGET_24
constexpr double TURN_P = 5;
constexpr double TURN_I = 0.5;
constexpr double TURN_D = 3;
constexpr double TURN_I_MAX = 50;

constexpr double DRIVE_P = 5;
constexpr double DRIVE_I = 0.5;
constexpr double DRIVE_D = 3;
constexpr double DRIVE_I_MAX = 50;

constexpr double DRIVE_MATCH_P = 5;
constexpr double DRIVE_MATCH_I = 0.5;
constexpr double DRIVE_MATCH_D = 3;
constexpr double DRIVE_MATCH_MAX_I = 50;
#endif

/// angle: angle to turn to in degrees
/// multiplier: 1.0 for normal speed, 0.5 for half speed
/// timeout_ms: timeout in ms, 0 for no timeout
void turn(double degrees, Motorgroup& left_drive, Motorgroup& right_drive, double multiplier = 1, uint32_t timeout_ms = 1000){
    const auto start = imu.get_rotation();
    const auto target = start + degrees;
    run_pid(
            [](){return imu.get_rotation();},
            [left_drive, right_drive, multiplier](double in_val){
                double result = std::max(std::min(in_val, 127.0), -127.0) * multiplier;
                for(auto& motor : left_drive){
                    motor = -(int32_t)result;
                }
                for(auto& motor: right_drive){
                    motor = (int32_t)result;
                }
            },
            target,
            TURN_P, TURN_I, TURN_D, TURN_I_MAX, 3, 20, timeout_ms);
    for(auto& motor : left_drive){
        motor = 0;
    }
    for(auto& motor: right_drive){
        motor = 0;
    }
}

/// angle: angle to turn to in degrees
/// multiplier: 1.0 for normal speed, 0.5 for half speed
/// timeout_ms: timeout in ms, 0 for no timeout
void turn_to_angle(double angle, Motorgroup& left_drive, Motorgroup& right_drive, double multiplier = 1, uint32_t timeout_ms = 0){
    run_pid(
            [](){return imu.get_heading();},
            [left_drive, right_drive, multiplier](double in_val){
                double result = std::max(std::min(in_val, 127.0), -127.0) * multiplier;
                for(auto& motor : left_drive){
                    motor = -(int32_t)result;
                }
                for(auto& motor: right_drive){
                    motor = (int32_t)result;
                }
            },
            angle,
            TURN_P, TURN_I, TURN_D, TURN_I_MAX, 3, 20, timeout_ms);
    for(auto& motor : left_drive){
        motor = 0;
    }
    for(auto& motor: right_drive){
        motor = 0;
    }
}

/// angle: angle to turn to in degrees
/// multiplier: 1.0 for normal speed, 0.5 for half speed
/// timeout_ms: timeout in ms, 0 for no timeout
void drive_for_distance(int32_t distance_in, Motorgroup& left_drive, Motorgroup& right_drive, double multiplier = 1, uint32_t timeout_ms = 0){
    int32_t distance=distance_in*ticktoinch;
    auto end = pros::millis() + timeout_ms;
    const auto start_angle = imu.get_rotation();
    const auto left_start = left_drive[0].get_raw_position(nullptr);
    const auto right_start = right_drive[0].get_raw_position(nullptr);
    double real_drive_i_max = DRIVE_I_MAX/multiplier;
    SpencerPID::PID left_distance_pid{
            (double)left_start + distance,
            [&left_drive](){
                return left_drive[0].get_raw_position(nullptr);
            },
            DRIVE_P, real_drive_i_max, DRIVE_D, DRIVE_I_MAX
    };
    SpencerPID::PID right_distance_pid{
            (double)right_start + distance,
            [&right_drive](){
                return right_drive[0].get_raw_position(nullptr);
            },
            DRIVE_P, real_drive_i_max, DRIVE_D, DRIVE_I_MAX
    };
    SpencerPID::PID match_pid{
            start_angle,
            [](){
                return imu.get_rotation();
            },
            DRIVE_MATCH_P, DRIVE_MATCH_I, DRIVE_MATCH_D, DRIVE_MATCH_MAX_I
    };

    uint32_t count = 0;
    int left_stop = 0;

    constexpr double forward_backward_tolerance = 9;

    //while(count < 100 && (timeout_ms == 0 || pros::millis() < end)){
    while(pros::millis() < end){


        auto left_result = left_distance_pid.run();
        auto right_result = right_distance_pid.run();
        auto match_result = match_pid.run();
        pros::lcd::set_text(2, std::to_string(left_result.error));
        pros::lcd::set_text(3, std::to_string(left_stop));

        if( abs(left_result.error)<=10){
           left_stop+=1;
       }
       else{
           left_stop = 0;
       }
       if(left_stop>=100){
           break;
       }
        if(
                left_result.error < forward_backward_tolerance
                && right_result.error < forward_backward_tolerance
                && match_result.error < 3){
            count++;
        }
        else{
            count = 0;
        }

        auto left = std::max(std::min(left_result.power, 127.0), -127.0) + std::max(std::min(match_result.power, 127.0), -127.0);
        auto right = std::max(std::min(right_result.power, 127.0), -127.0) - std::max(std::min(match_result.power, 127.0), -127.0);
        left = left * multiplier;
        right = right * multiplier;

        for(auto& motor : left_drive){
            motor = -(int32_t)left;
        }
        for(auto& motor : right_drive){
            motor = -(int32_t)right;
        }

        pros::delay(5);
    }
    for(auto& motor : left_drive){
        motor = 0;
    }
    for(auto& motor: right_drive){
        motor = 0;
    }
}

void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    //default
    pros::lcd::initialize();
    pros::lcd::register_btn1_cb(on_center_button);
    imu.reset();
    do {
        pros::delay(10);
    }while (imu.is_calibrating());
    //Motors
    left0.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right0.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    top_roller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::lcd::set_text(1, "Winner Winner Chicken Dinner!");

    leftDrive.push_back(left0);
    leftDrive.push_back(left1);
    leftDrive.push_back(left2);

    rightDrive.push_back(right0);
    rightDrive.push_back(right1);
    rightDrive.push_back(right2);
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


///Aarons Stupid code
void setDrive(int left,int right){
    left0=left;
    left1=left;
    left2=left;

    right0=right;
    right1=right;
    right2=right;
}

void intake4ever(int value){
    left_intake=value;
    right_intake=value;
}
void bottom4ever(int value){
    bottom_rollers0=value;
    bottom_rollers1=value;
}
void stoprollers(){
    left_intake =0;
    right_intake=0;
    bottom_rollers0=0;
    bottom_rollers1=0;
}
void intake(int value, float time){
    left_intake=value;
    right_intake=value;
    pros::delay(time);
    left_intake=0;
    right_intake=0;
}
void bottom_rollers(int value, float time){
    bottom_rollers0=value;
    bottom_rollers1=value;
    pros::delay(time);
    bottom_rollers0=0;
    bottom_rollers1=0;
}
void top_rollers(int value, float time){
    top_roller=value;
    pros::delay(time);
    top_roller=0;
}
void intake_bottom_rollers(int value, float time){
    bottom_rollers0=-value;
    bottom_rollers1=-value;
    left_intake=value;
    right_intake=value;
    pros::delay(time);
    bottom_rollers0=0;
    bottom_rollers1=0;
    left_intake=0;
    right_intake=0;
}

void bottom_top_rollers(int value, float time){

    bottom_rollers0=value;
    bottom_rollers1=value;
    top_roller=-value;
    pros::delay(time);
    bottom_rollers0=0;
    bottom_rollers1=0;
    top_roller=0;

}
void pickup(int value, float time){
    left_intake=value;
    right_intake=value;
    bottom_rollers0=value;
    bottom_rollers1=value;
    top_roller=-value;
    pros::delay(time);
    bottom_rollers0=0;
    bottom_rollers1=0;
    top_roller=0;
    left_intake=0;
    right_intake=0;
}
void drivetime(int value,float time){
    right0=value;
    right1=value;
    right2=value;
    left1=value;
    left2=value;
    left0=value;
    pros::delay(time);
    right0=0;
    right1=0;
    right2=0;
    left1=0;
    left2=0;
    left0=0;
}

void destow(){
    left_intake=-127;
    right_intake=-127;
    bottom_rollers0=127;
    bottom_rollers1=127;
    pros::delay(200);
    bottom_rollers0=-127;
    bottom_rollers1=-127;
    left_intake=-127;
    right_intake=-127;
    top_roller=127;
    pros::delay(400);
    bottom_rollers0=0;
    bottom_rollers1=0;
    left_intake=0;
    right_intake=0;
    top_roller=0;
}
void wallslide(){
    top_rollers(-127,250);
    top_rollers(127,250);
    top_rollers(-127,250);
    top_rollers(127,250);
    drivetime(52,500);
    pros::delay(2000);
    intake4ever(-127);
    drivetime(69,5000);
    intake4ever(-127);
    bottom4ever(80);

}


void autonomous() {
    pros::delay(36000);
    wallslide();

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void opcontrol() {
//Typhoon

    int lastPower_left = 0;
    int lastPower_right = 0;


    while (true) {


#define INTAKE_MASK 1
#define ROLLER_MASK 2
#define TOP_MASK 4

        int mask = 0;

        int rollers = 0;
        int topRollers = 0;
        int intake = 0;

        int arcade_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int arcade_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //arcade to tank
        int tank_left = arcade_y + arcade_x;
        int tank_right = arcade_y + -arcade_x;

        //update drive
        left0 = tank_left;
        left1 = tank_left;
        left2 = tank_left;

        right0 = tank_right;
        right1 = tank_right;
        right2 = tank_right;

        // updateMotorGroup(topRollers, -0);
        // updateMotorGroup(rollers, -0);
        // updateMotorGroup(intake, -0);

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            if(!(mask & ROLLER_MASK)){
                rollers = 127;
            }
            if(!(mask & TOP_MASK)){
                topRollers = 127;
            }

            mask |= ROLLER_MASK | TOP_MASK;
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            if(!(mask & TOP_MASK)){
                topRollers = -127*.5;
            }
            if(!(mask & ROLLER_MASK)){
                rollers = 127;
            }

            if(!(mask & INTAKE_MASK)){
                intake = 127;
            }
            mask |= ROLLER_MASK | TOP_MASK | INTAKE_MASK;
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            if(!(mask & TOP_MASK)){
                topRollers = -127;
            }
            if(!(mask & ROLLER_MASK)){
                rollers = -127;
            }

            if(!(mask & INTAKE_MASK)){
                intake = -127;
            }
            mask |= ROLLER_MASK | TOP_MASK | INTAKE_MASK;
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            if(!(mask & ROLLER_MASK)){
                rollers = -127;
            }

            if(!(mask & TOP_MASK)){
                topRollers = -127;
            }
            mask |= ROLLER_MASK | TOP_MASK;
        }

        if(!(mask & 1)){
            intake = 0;
        }
        if(!(mask & 2)){
            rollers = 0;
        }
        if(!(mask & 4)){
            topRollers = 0;
        }

        intake = -intake;

        left_intake = intake;
        right_intake = intake;

        bottom_rollers0 = rollers;
        bottom_rollers1 = rollers;

        top_roller = topRollers;

        continue;

        //OLD TRENT CODE

        //drivetrain Code
        // split arcade

        int power =master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //arcade_drive(power/127.0, turn/127.0);

        int left = power+turn;
        int right=power-turn;

        left0=left;
        left1=left;
        left2=left;

        right0=right;
        right1=right;
        right2=right;
        //end of split arcade control


        int threshold = 0;
        if (abs(left) > threshold) {
            left0 = left;
            left1 = left;
            left2 = left;
        } else {
            left0 = 0;
            left1 = 0;
            left2 = 0;
        }
        if (abs(right) > threshold) {
            right0 = right;
            right1 = right;
            right2 = right;
        } else {
            right0 = 0;
            right1 = 0;
            right2 = 0;
        }


        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            bottom_rollers0 = 127;
            bottom_rollers1 = 127;
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            bottom_rollers0 = -127;
            bottom_rollers1 = -127;

        } else {
            bottom_rollers0 = 0;
            bottom_rollers1 = 0;
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
            top_roller = 127;
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            top_roller = -127;
        else {
            top_roller = 0;
        }


        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            left_intake = 127;
            right_intake = 127;
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            left_intake = -127;
            right_intake = -127;
        } else {
            left_intake = 0;
            right_intake = 0;
        }

        pros::delay(20);
    }
}
