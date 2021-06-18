#include "main.h"
#include "okapi/api.hpp"

using namespace okapi;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
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
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello Cuck!");

    pros::lcd::register_btn1_cb(on_center_button);
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
[[noreturn]] void opcontrol() {
//    okapi::MotorGroup left_drive_side = {-2, -3, -4};
//    okapi::MotorGroup right_drive_side = {12, 13, 14};
//    std::shared_ptr<okapi::ChassisController> driveController = okapi::ChassisControllerBuilder()
//            .withMotors(left_drive_side, right_drive_side)
//            .withDimensions(
//                    okapi::AbstractMotor::GearsetRatioPair{okapi::AbstractMotor::gearset::blue,3.0 / 5.0},
//                    {{3.25_in, 12.5_in}, okapi::imev5BlueTPR}
          //  );
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Motor left0(2,true);
    pros::Motor left1( 3,true);
    pros::Motor left2(4,true);
    pros::Motor right0(12);
    pros::Motor right1(13);
    pros::Motor right2(14);
    pros::Motor left_intake(5,true);
    pros::Motor top_roller(8);
    pros::Motor right_intake(15);
    pros::Motor bottom_rollers0(17,true);
    pros::Motor bottom_rollers1(18,true);
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
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
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
