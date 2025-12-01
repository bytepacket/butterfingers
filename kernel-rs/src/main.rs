#![no_main]

use vexide::prelude::*;
use std::{cell::RefCell, rc::Rc};
use evian::prelude::*;
use evian::tracking::wheeled::TrackingWheel;

struct Robot<> {}


struct Drivetrain<> {
    left_motors: Rc<RefCell<[Motor; 3]>>,
    right_motors: Rc<RefCell<[Motor; 3]>>,
}

struct IO<> {
    front_bottom: Motor,
    middle: Motor,
    back_top: Motor,
}

// Rc is necessary because we share motors between drivetrain and odometry, doesn't have Copy trait
pub struct Odometry<> {
    left_motors: Rc<RefCell<[Motor; 3]>>,
    right_motors: Rc<RefCell<[Motor; 3]>>,
    imu: InertialSensor,
    color_sensor: OpticalSensor,
    horizontal: TrackingWheel<AdiEncoder<360>>,
    vertical: TrackingWheel<AdiEncoder<360>>,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        println!("Driver!");
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {

    let controller = peripherals.primary_controller;

    let screen = peripherals.display;

    let robot = Robot {};

    let left_motors = Rc::new(RefCell::new([
        Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
    ]));

    let right_motors = Rc::new(RefCell::new([
        Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
    ]));

    let horizontal = AdiEncoder::new(peripherals.adi_a, peripherals.adi_b);
    let vertical = AdiEncoder::new(peripherals.adi_g, peripherals.adi_h);

    let horizontal_encoder = TrackingWheel::new(horizontal, 3.25, 0.0, None);

    let vertical_encoder = TrackingWheel::new(vertical, 3.25, 0.0, None);

    let mut imu = InertialSensor::new(peripherals.port_13);
    _ = imu.calibrate().await;

    let drivetrain = Drivetrain {
        left_motors: left_motors.clone(),
        right_motors: right_motors.clone(),
    };

    let odom = Odometry {
        left_motors: left_motors,
        right_motors: right_motors,
        imu: imu,
        color_sensor: OpticalSensor::new(peripherals.port_20),
        horizontal: horizontal_encoder,
        vertical: vertical_encoder,
    };

    let io = IO {
        front_bottom: Motor::new(peripherals.port_7, Gearset::Green, Direction::Forward),
        middle: Motor::new(peripherals.port_5, Gearset::Green, Direction::Forward),
        back_top: Motor::new(peripherals.port_6, Gearset::Green, Direction::Forward),
    };

    let mut pneumatic_c = AdiDigitalOut::new(peripherals.adi_c);
    let mut pneumatic_d = AdiDigitalOut::new(peripherals.adi_d);

    robot.compete().await;
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_adds_two() {
        assert_eq!(2 + 2, 4);
    }
}