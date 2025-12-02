mod traits;
use crate::traits::{IOFunc, TankExt};

use vexide::prelude::*;
use std::{cell::RefCell, rc::Rc,};
use evian::prelude::*;
use evian::drivetrain::model::Differential;
use evian::tracking::shared_motors;
use vexide::smart::motor::{BrakeMode, MotorType};
use vexide::smart::PortError;

struct Robot<> {}

struct IO<> {
    front_bottom: Motor,
    middle: Motor,
    back_top: Motor,
}
impl TankExt for Differential {
    fn brake_all(&mut self, mode: BrakeMode) -> Result<(), Self::Error> {
        let mut rtn = Ok(());

        for motor in self.left.borrow_mut().as_mut() {
            let result = motor.brake(mode);
            if result.is_err() {
                rtn = result;
            }
        }

        for motor in self.right.borrow_mut().as_mut() {
            let result = motor.brake(mode);
            if result.is_err() {
                rtn = result;
            }
        }

        rtn
    }

    fn move_all(&mut self, speed: i32) -> Result<(), Self::Error> {
        let mut rtn = Ok(());

        for motor in self.left.borrow_mut().as_mut() {
            let result = motor.set_velocity(speed);
            if result.is_err() {
                rtn = result;
            }
        }

        for motor in self.right.borrow_mut().as_mut() {
            let result = motor.set_velocity(speed);
            if result.is_err() {
                rtn = result;
            }
        }

        rtn
    }
}

impl IOFunc for IO {
    fn intake_store(&mut self) -> Result<(), PortError> {
        let mut rtn= Ok(());
        let result= self.front_bottom.set_velocity(-600);
        if result.is_err() {
            rtn = result;
        }
        let result = self.middle.set_velocity(300);
        if result.is_err() {
            rtn = result;
        }
        let result = self.back_top.set_velocity(0);
        if result.is_err() {
            rtn = result;
        }
        rtn
    }

    fn outtake_top(&mut self) -> Result<(), PortError> {
        let mut rtn = Ok(());
        let result = self.front_bottom.set_velocity(-600);

        if result.is_err() {
            rtn = result
        }
        let result = self.middle.set_velocity(600);

        if result.is_err() {
            rtn = result
        }
        let result =self.back_top.set_velocity(600);
        if result.is_err() {
            rtn = result;
        }
        rtn
    }

    fn outtake_middle(&mut self) -> Result<(), PortError> {
        let mut rtn = Ok(());
        let result = self.front_bottom.set_velocity(-600);

        if result.is_err() {
            rtn = result
        }
        let result = self.middle.set_velocity(600);

        if result.is_err() {
            rtn = result
        }
        let result =self.back_top.set_velocity(-600);
        if result.is_err() {
            rtn = result;
        }
        rtn
    }

    fn outtake_bottom(&mut self) -> Result<(), PortError> {
        let mut rtn = Ok(());
        let result = self.front_bottom.set_velocity(600);

        if result.is_err() {
            rtn = result
        }
        let result = self.middle.set_velocity(-600);

        if result.is_err() {
            rtn = result
        }
        let result =self.back_top.set_velocity(0);
        if result.is_err() {
            rtn = result;
        }
        rtn
    }

    fn stop_intake(&mut self) -> Result<(), PortError> {
        let mut rtn = Ok(());
        let result = self.front_bottom.set_velocity(0);

        if result.is_err() {
            rtn = result
        }
        let result = self.middle.set_velocity(0);

        if result.is_err() {
            rtn = result
        }
        let result =self.back_top.set_velocity(0);
        if result.is_err() {
            rtn = result;
        }
        rtn
    }
}

// Rc is necessary because we share motors between drivetrain and odometry, doesn't have Copy trait
pub struct Odometry<> {
    left_motors: Rc<RefCell<[Motor; 3]>>,
    right_motors: Rc<RefCell<[Motor; 3]>>,
    imu: InertialSensor,
    color_sensor: OpticalSensor,
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

    let left_motors = shared_motors![
        Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
    ];

    let right_motors = shared_motors![
        Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
    ];


    let mut dt = Differential::from_shared(left_motors.clone(), right_motors.clone());

    let _ = dt.brake_all(BrakeMode::Brake);

    let mut imu = InertialSensor::new(peripherals.port_13);
    _ = imu.calibrate().await;


    let odom = Odometry {
        left_motors: left_motors,
        right_motors: right_motors,
        imu: imu,
        color_sensor: OpticalSensor::new(peripherals.port_20),
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