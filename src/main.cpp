#include "main.h"
#include "lemlib/api.hpp"

// ===== PATH ASSETS =====
ASSET(intake_path_txt);

// ===== DRIVETRAIN MOTOR CONFIGURATION =====
// Left side motors (ports 1, 2, 3) - all reversed to fix direction
pros::MotorGroup left_motors({-1, -2, -3}, pros::v5::MotorGears::green);
// Right side motors (ports 4, 8, 10) - all forward to fix direction
pros::MotorGroup right_motors({4, 8, 10}, pros::v5::MotorGears::green);

// ===== INTAKE/OUTTAKE MOTORS =====
pros::Motor front_bottom(7, pros::v5::MotorGears::green);  // Front bottom motor
pros::Motor middle(5, pros::v5::MotorGears::green);        // Middle motor
pros::Motor back_top(6, pros::v5::MotorGears::green);      // Back top motor

// ===== PNEUMATICS =====
pros::adi::Pneumatics pneumatic_c('C', false);  // Pneumatic on port C (starts retracted)
pros::adi::Pneumatics pneumatic_d('D', false);  // Pneumatic on port D (starts retracted)

// ===== SENSORS =====
pros::Optical color_sensor(20);  // Color sensor on port 20 for ball detection
pros::Imu imu(13);  // IMU on port 13
pros::adi::Encoder horizontal_encoder('A', 'B', false);  // Horizontal tracking wheel (X-axis)
pros::adi::Encoder vertical_encoder('G', 'H', false);    // Vertical tracking wheel (Y-axis)

// ===== LEMLIB CONFIGURATION =====
// Tracking wheels (3.25" diameter omni wheels)
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_325, 0);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_325, 0);

// Drivetrain configuration
lemlib::Drivetrain drivetrain(
    &left_motors,           // Left motor group
    &right_motors,          // Right motor group
    10,                     // Track width (inches) - distance between left and right wheels
    lemlib::Omniwheel::NEW_325,  // Wheel type
    200,                    // Gear ratio (RPM) - 200 RPM with 18:1 gearset
    3                       // Drive motor count per side
);

// Odometry sensors
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,   // Vertical tracking wheel
    nullptr,                    // Second vertical tracking wheel (not used)
    &horizontal_tracking_wheel, // Horizontal tracking wheel
    nullptr,                    // Second horizontal tracking wheel (not used)
    &imu                        // IMU
);

// Lateral PID controller for moving forward/backward
lemlib::ControllerSettings lateral_controller(
    10,     // Proportional gain (kP)
    0,      // Integral gain (kI)
    3,      // Derivative gain (kD)
    3,      // Anti-windup
    1,      // Small error range (inches)
    100,    // Small error timeout (ms)
    3,      // Large error range (inches)
    500,    // Large error timeout (ms)
    80      // Maximum acceleration (slew)
);

// Angular PID controller for turning
lemlib::ControllerSettings angular_controller(
    2,      // Proportional gain (kP)
    0,      // Integral gain (kI)
    10,     // Derivative gain (kD)
    3,      // Anti-windup
    1,      // Small error range (degrees)
    100,    // Small error timeout (ms)
    3,      // Large error range (degrees)
    500,    // Large error timeout (ms)
    0       // Maximum acceleration (slew)
);

// Create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// ===== INTAKE/OUTTAKE CONTROL FUNCTIONS =====
void intake_store() {
    // Always run bottom motor to intake
    front_bottom.move(-127);  // Counter-clockwise to intake
    
    // Check if color sensor detects a ball (proximity threshold)
    if (color_sensor.get_proximity() > 100) {  // Adjust threshold as needed (0-255)
        // Ball detected - run middle motor to move it up
        middle.move(65);  // Clockwise
    } else {
        // No ball detected - stop middle motor
        middle.move(0);
    }
    
    // Never run top motor during storage
    back_top.move(0);  // Off
}

void outtake_top() {
    front_bottom.move(-127);  // Counter-clockwise
    middle.move(127);         // Clockwise
    back_top.move(127);       // Clockwise
}

void outtake_middle() {
    front_bottom.move(-127);  // Counter-clockwise
    middle.move(127);         // Clockwise
    back_top.move(-127);      // Counter-clockwise
}

void outtake_bottom() {
    front_bottom.move(127);   // Clockwise
    middle.move(-127);        // Counter-clockwise
    back_top.move(0);         // Off
}

void stop_intake() {                  // stops the intake
    front_bottom.move(0);
    middle.move(0);
    back_top.move(0);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Butterfingers Robot");
    pros::lcd::set_text(2, "Calibrating...");
    
    // Calibrate the IMU and chassis
    chassis.calibrate();
    
    // Wait for IMU calibration to complete
    while (imu.is_calibrating()) {
        pros::delay(10);
    }
    
    pros::lcd::set_text(2, "Ready!");
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
    // Debug: Show we entered autonomous
    pros::lcd::set_text(3, "Auton Started!");
    
    // Set starting pose at origin (0, 0, 0)
    chassis.setPose(0, 0, 0);
    pros::lcd::set_text(4, "Pose Set");
    
    // Start intaking
    intake_store();
    pros::lcd::set_text(5, "Intake Running");
    
    // Simple test movement instead of path following
    chassis.moveToPoint(24, 0, 2000);  // Move 24 inches forward
    
    pros::lcd::set_text(6, "Movement Done");
    
    // Stop intake after reaching position
    stop_intake();
    pros::lcd::set_text(7, "Auton Complete");
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
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        // Press UP arrow to run autonomous routine
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            autonomous();
        }
        
        // Pneumatic controls
        // Right arrow: toggle pneumatic C
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            pneumatic_c.toggle();
        }
        
        // Left arrow: toggle pneumatic D
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            pneumatic_d.toggle();
        }
        
        // Press Y button to align to nearest cardinal direction (0, 90, 180, 270)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            float current_heading = chassis.getPose().theta;
            // Normalize heading to 0-360 range
            while (current_heading < 0) current_heading += 360;
            while (current_heading >= 360) current_heading -= 360;
            
            // Find nearest cardinal direction
            float nearest = 0;
            float min_diff = 360;
            for (int cardinal : {0, 90, 180, 270, 360}) {
                float diff = std::abs(current_heading - cardinal);
                if (diff < min_diff) {
                    min_diff = diff;
                    nearest = cardinal;
                }
            }
            if (nearest == 360) nearest = 0;  // Treat 360 as 0
            
            // Turn to nearest cardinal direction using LemLib with custom params
            chassis.turnToHeading(nearest, 1000, {.maxSpeed = 127}, false);
        }
        
        // Arcade drive control - left stick Y for forward/back, right stick X for turning
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // Press A button to reverse forward/backward direction (flip front of robot)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            forward = -forward;  // Reverse the forward direction
        }
        
        // Move the chassis using arcade drive
        chassis.arcade(forward, turn);
        
        // Intake/Outtake controls using controller buttons
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_store();  // R1: Intake/Store
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            outtake_top();  // R2: Outtake from top
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            outtake_middle();  // L1: Outtake from middle
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            outtake_bottom();  // L2: Outtake from bottom
        } else {
            stop_intake();  // No button pressed: stop intake motors
        }
        
        // Display robot position on LCD
        pros::lcd::print(0, "X: %.2f, Y: %.2f", chassis.getPose().x, chassis.getPose().y);
        pros::lcd::print(1, "Heading: %.2f", chassis.getPose().theta);
        
        pros::delay(20);  // Run for 20 ms then update
    }
}