#include "main.h"
#include "okapi/api.hpp"

using namespace okapi;
//update pros.c
//pros v5 run 8
pros::Controller master(pros::E_CONTROLLER_MASTER);
//Drivetrain
pros::Motor left0(2,pros::E_MOTOR_GEARSET_06,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor left1( 3,pros::E_MOTOR_GEARSET_06,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor left2(4,pros::E_MOTOR_GEARSET_06,true,pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor right0(12,pros::E_MOTOR_GEARSET_06,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor right1(13,pros::E_MOTOR_GEARSET_06,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor right2(14,pros::E_MOTOR_GEARSET_06,false,pros::E_MOTOR_ENCODER_COUNTS);
// Intake
pros::Motor left_intake(5,true);
pros::Motor right_intake(15,false);
//Rollers
pros::Motor bottom_rollers0(17,true);
pros::Motor bottom_rollers1(18,true);

pros::Motor top_roller(8,false);

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
//PID constants
#define ROBOT_TARGET_15
#ifdef ROBOT_TARGET_15
//turning
constexpr double TURN_P = 10;
constexpr double TURN_I = 1;
constexpr double TURN_D = 0.5;
constexpr double TURN_I_MAX = 50;
//driving
constexpr double DRIVE_P = 75;
constexpr double DRIVE_I = 0;
constexpr double DRIVE_D = 5;
constexpr double DRIVE_I_MAX = 50;
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
void turn(double degrees, Motorgroup& left_drive, Motorgroup& right_drive, double multiplier = 1, uint32_t timeout_ms = 0){
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
void drive_for_distance(int32_t distance, Motorgroup& left_drive, Motorgroup& right_drive, double multiplier = 1, uint32_t timeout_ms = 0){
    auto end = pros::millis() + timeout_ms;
    const auto start_angle = imu.get_rotation();
    const auto left_start = left_drive[0].get_raw_position(nullptr);
    const auto right_start = right_drive[0].get_raw_position(nullptr);

    SpencerPID::PID left_distance_pid{
            (double)left_start + distance,
            [&left_drive](){
                return left_drive[0].get_raw_position(nullptr);
            },
            DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_I_MAX
    };
    SpencerPID::PID right_distance_pid{
            (double)right_start + distance,
            [&right_drive](){
                return right_drive[0].get_raw_position(nullptr);
            },
            DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_I_MAX
    };
    SpencerPID::PID match_pid{
            start_angle,
            [](){
                return imu.get_rotation();
            },
            DRIVE_MATCH_P, DRIVE_MATCH_I, DRIVE_MATCH_D, DRIVE_MATCH_MAX_I
    };

    uint32_t count = 0;

    constexpr double forward_backward_tolerance = 50;

    while(count < 100 && (timeout_ms == 0 || pros::millis() < end)){
        auto left_result = left_distance_pid.run();
        auto right_result = right_distance_pid.run();
        auto match_result = match_pid.run();

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

//set motors brake and coast

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
    top_roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
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
void autonomous() {}

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


    while (true) {

        int left = master.get_analog(ANALOG_LEFT_Y);
        int right = master.get_analog(ANALOG_RIGHT_Y);
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

*/
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            bottom_rollers0 = 127;
            bottom_rollers1 = 127;
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
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
