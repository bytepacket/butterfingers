## Project Overview
This is a VEX V5 robotics project using PROS (Purdue Robotics Operating System) with LemLib@0.5.6 for advanced motion control. Built for VEX EDR V5 platform with ARM Cortex-A9 processor.

## Robot Hardware Configuration

### Drivebase Specifications
**Drive Type:** 6-wheel tank drive

**Wheel Configuration:**
- 6 wheels total (3 per side)
- Left side: 3 wheels in a line
- Right side: 3 wheels in a line
- Each side has the middle wheel as a 3.25" traction wheel (260mm Anti-Static Wheel)
- Front and back wheels on each side are 3.25" omni wheels (260mm Anti-Static Omni-Directional Wheels)

**Motors:**
- 6 total V5 Smart Motors (one per wheel)
- All motors use 18:1 gear cartridges (200 RPM green cartridges / E_MOTOR_GEARSET_18)
- Direct drive (motor directly connected to wheel)

**Drive Control:**
- Tank drive control scheme (left motors controlled together, right motors controlled together)
- Center traction wheels provide pushing power and prevent side-to-side sliding
- Front/back omni wheels allow smooth turning without wheel scrubbing

### Intake/Outtake System
**Motors:**
- Port 7: Front bottom motor (11W V5 Smart Motor, 5.5V operation)
- Port 5: Middle motor (5.5W V5 Smart Motor, 5.5V operation)
- Port 6: Back top motor (5.5W V5 Smart Motor, 5.5V operation)
- All motors use standard voltage (5.5V) for consistent performance

**Motor Configurations:**
```cpp
pros::Motor front_bottom(7, pros::E_MOTOR_GEARSET_18);  // Port 7
pros::Motor middle(5, pros::E_MOTOR_GEARSET_18);        // Port 5
pros::Motor back_top(6, pros::E_MOTOR_GEARSET_18);      // Port 6
```

**Operating Modes:**

*Intake (Store):*
- Front bottom: Counter-clockwise (negative velocity)
- Middle: Clockwise (positive velocity)
- Back top: Off

*Outtake Top:*
- Front bottom: Counter-clockwise (negative velocity)
- Middle: Clockwise (positive velocity)
- Back top: Clockwise (positive velocity)

*Outtake Middle:*
- Front bottom: Counter-clockwise (negative velocity)
- Middle: Clockwise (positive velocity)
- Back top: Counter-clockwise (negative velocity)

*Outtake Bottom:*
- Front bottom: Clockwise (positive velocity)
- Middle: Counter-clockwise (negative velocity)
- Back top: Off

### Sensors
- Port 13: IMU (Inertial Measurement Unit) - used for heading/rotation tracking
- Port 4: Antenna (communication/wireless module)

### Odometry System (CRITICAL for LemLib)
**Tracking Wheels:**
- 2x 3.25" omni wheels for position tracking
- Port AB: Horizontal tracking wheel (measures X-axis movement, parallel to robot forward/back)
- Port GH: Vertical tracking wheel (measures Y-axis movement, perpendicular to robot left/right)

**LemLib Configuration:**
```cpp
// Tracking wheel setup
pros::ADIEncoder horizontal_encoder('A', 'B', false);  // AB ports - X axis
pros::ADIEncoder vertical_encoder('G', 'H', false);    // GH ports - Y axis

// Create rotation sensors for odometry
lemlib::TrackingWheel horizontal(&horizontal_encoder, 3.25, 0);  // 3.25" wheel, offset from center
lemlib::TrackingWheel vertical(&vertical_encoder, 3.25, 0);      // 3.25" wheel, offset from center

// IMU for heading
pros::Imu imu(13);

// Odometry sensors object
lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);
```

**Important Notes:**
- These tracking wheels are separate from drive wheels and spin freely
- They measure ground distance traveled for accurate position tracking
- Must be calibrated with correct wheel diameter (3.25") and offsets from robot center
- Essential for autonomous pathfinding and precise movement with LemLib
## Critical Build & Deploy Workflow
- **Never use `pros` CLI directly** - it's not in PATH. Use the PROS Terminal in VS Code
- Build: `make` or `make quick` (default target)
- Upload to robot: Use PROS Terminal → `pros upload` or PROS extension UI
- Templates managed via: `pros c apply <template>@<version>` (e.g., `pros c apply LemLib@0.5.6`)
- Hot/cold linking enabled (`USE_PACKAGE:=1` in Makefile) - kernel and user code compiled separately for faster uploads

## Architecture & Code Organization

### Core Entry Points (`src/main.cpp`)
Four required competition lifecycle functions (NEVER rename/remove):
- `initialize()` - Runs once at startup, setup sensors/motors here
- `competition_initialize()` - Pre-match setup (e.g., autonomous selector)
- `autonomous()` - 15-second autonomous period
- `opcontrol()` - Driver control loop (runs continuously until disabled)

### Motor Configuration Pattern
```cpp
// Port numbers: 1-21, prefix with - to reverse
pros::MotorGroup left_mg({1, -2, 3});    // Ports 1,3 forward, port 2 reversed
pros::MotorGroup right_mg({-4, 5, -6});  // Port 5 forward, ports 4,6 reversed

// Always specify gearset for custom configurations
pros::Motor motor(1, pros::E_MOTOR_GEARSET_06); // Blue cartridge (600 RPM)
// Gearsets: 36=red=100RPM, 18=green=200RPM, 06=blue=600RPM
```

### PROS Namespace Convention
- `PROS_USE_SIMPLE_NAMES` is enabled (see `include/main.h`)
- Use `pros::` namespace explicitly - **don't use `using namespace pros;`**
- Example: `pros::Controller`, `pros::MotorGroup`, `pros::lcd::print()`

### LemLib Integration (Not Yet Implemented)
Current code is template/starter. To use LemLib:
1. Create `lemlib::Chassis` object with drivetrain, sensors, and PID settings
2. Call `chassis.calibrate()` in `initialize()`
3. Use `chassis.moveToPose()` or `chassis.turnToHeading()` for autonomous
4. Driver control: `chassis.arcade()` or `chassis.curvature()` instead of direct motor control

Example setup pattern from LemLib docs:
```cpp
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10, 4, 360); // wheelDiameter, drivebaseWidth, gearRatio
lemlib::OdomSensors sensors(&vertical1, nullptr, &horizontal1, nullptr, &imu);
lemlib::Chassis chassis(drivetrain, linearSettings, angularSettings, sensors);
```

## Key Files & Their Roles
- `project.pros` - Template metadata, kernel version (4.2.1), installed libraries
- `Makefile` - User config (library exclusions, hot/cold linking settings)
- `common.mk` - Build system internals (compiler flags, ARM toolchain: `arm-none-eabi-*`)
- `firmware/` - Precompiled static libraries (LemLib.a, libpros.a)
- `include/` - Headers for PROS API, LemLib, and liblvgl (GUI library)
- `bin/` - Build outputs (`.bin`, `.elf` files) - gitignored

## Common Patterns

### Control Loop Structure
```cpp
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        int forward = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        // ... control logic ...
        pros::delay(20); // CRITICAL: Always delay in loops (20ms typical)
    }
}
```

### LCD Usage
```cpp
pros::lcd::initialize();                          // Call once in initialize()
pros::lcd::set_text(1, "Status");                // Set line 1
pros::lcd::print(0, "Value: %d", variable);      // Printf-style formatting
pros::lcd::register_btn1_cb(callback_function);  // Button callbacks
```

## Important Constraints
- C++23 standard (`--std=gnu++23`)
- No newlib/libc stubs - limited standard library support
- Firmware uses `-nostdlib` linking
- Motor ports must be 1-21 (validated at runtime)
- Control loops MUST include `pros::delay()` or task scheduler breaks

## Debugging Tips
- Check `temp.log` and `temp.errors` for build issues
- Use `pros::lcd::print()` for runtime debugging (no serial console access during competition)
- Motor errors: Check port configuration and ensure `ENODEV` errno handling
- Compile commands in `compile_commands.json` (for IntelliSense)

## What NOT to Do
- Don't modify files in `include/pros/`, `include/lemlib/`, or `firmware/` - these are template-managed
- Don't use `std::cout` in opcontrol loops (no stdout on V5 brain)
- Don't create tight loops without `pros::delay()` - will hang robot
- Don't assume POSIX or full C++ standard library availability
