mod traits;
use crate::traits::{Asterisk, IOFunc};

use evian::prelude::*;
use evian::tracking::shared_motors;
use evian::{
    control::loops::{AngularPid, Pid},
    drivetrain::model::{Arcade, Differential},
    motion::{Basic, Seeking},
    tracking::wheeled::{TrackingWheel, WheeledTracking},
};
use std::{cell::RefCell, rc::Rc};
use vexide::prelude::*;
use vexide::smart::PortError;
use vexide::smart::motor::{BrakeMode, MotorType};

struct DiffHolonomic {
    front_left: Rc<RefCell<Motor>>,
    front_right: Rc<RefCell<Motor>>,
    back_left: Rc<RefCell<Motor>>,
    back_right: Rc<RefCell<Motor>>,
    tank_wheels: Rc<RefCell<[Motor; 2]>>,
}

impl DiffHolonomic {
    fn new(
        front_left: Rc<RefCell<Motor>>,
        front_right: Rc<RefCell<Motor>>,
        back_left: Rc<RefCell<Motor>>,
        back_right: Rc<RefCell<Motor>>,
        tank_wheels: Rc<RefCell<[Motor; 2]>>,
    ) -> Self {
        DiffHolonomic {
            front_left,
            front_right,
            back_left,
            back_right,
            tank_wheels,
        }
    }
}
impl Asterisk for DiffHolonomic {
    async fn brake_all(&mut self, mode: BrakeMode) -> Result<(), PortError> {
        self.front_left.borrow_mut().brake(mode).ok();
        self.front_right.borrow_mut().brake(mode).ok();
        self.back_left.borrow_mut().brake(mode).ok();
        self.back_right.borrow_mut().brake(mode).ok();

        for motor in self.tank_wheels.borrow_mut().iter_mut() {
            motor.brake(mode).ok();
        }
        Ok(())
    }

    async fn move_all(
        &mut self,
        fl: f64,
        fr: f64,
        bl: f64,
        br: f64,
        y: f64,
    ) -> Result<(), PortError> {
        self.front_left
            .borrow_mut()
            .set_voltage(fl * Motor::V5_MAX_VOLTAGE)
            .ok();
        self.front_right
            .borrow_mut()
            .set_voltage(fr * Motor::V5_MAX_VOLTAGE)
            .ok();
        self.back_left
            .borrow_mut()
            .set_voltage(bl * Motor::V5_MAX_VOLTAGE)
            .ok();
        self.back_right
            .borrow_mut()
            .set_voltage(br * Motor::V5_MAX_VOLTAGE)
            .ok();

        for motor in self.tank_wheels.borrow_mut().iter_mut() {
            motor
                .set_voltage(y.clamp(-1.0, 1.0) * Motor::V5_MAX_VOLTAGE)
                .ok();
        }
        Ok(())
    }
}

struct Robot {
    controller: Controller,
    dt: DiffHolonomic,
}

struct IO {
    front_bottom: Motor,
    middle: Motor,
    back_top: Motor,
}

impl IOFunc for IO {
    fn intake_store(&mut self) -> Result<(), PortError> {
        self.front_bottom.set_velocity(-600).ok();

        self.middle.set_velocity(300).ok();

        self.back_top.set_velocity(0).ok();

        Ok(())
    }

    fn outtake_top(&mut self) -> Result<(), PortError> {
        self.front_bottom.set_velocity(-600).ok();

        self.middle.set_velocity(600).ok();

        self.back_top.set_velocity(600).ok();

        Ok(())
    }

    fn outtake_middle(&mut self) -> Result<(), PortError> {
        self.front_bottom.set_velocity(-600).ok();

        self.middle.set_velocity(600).ok();

        self.back_top.set_velocity(-600).ok();

        Ok(())
    }

    fn outtake_bottom(&mut self) -> Result<(), PortError> {
        self.front_bottom.set_velocity(600).ok();

        self.middle.set_velocity(-600).ok();

        self.back_top.set_velocity(0).ok();

        Ok(())
    }

    fn stop_intake(&mut self) -> Result<(), PortError> {
        self.front_bottom.set_velocity(0).ok();

        self.middle.set_velocity(0).ok();

        self.back_top.set_velocity(0).ok();

        Ok(())
    }
}

// Rc is necessary because we share motors between drivetrain and odometry, doesn't have Copy trait
pub struct Odometry {
    motors: [Rc<RefCell<Motor>>; 4],
    tank: Rc<RefCell<[Motor; 2]>>,
    imu: InertialSensor,
    color_sensor: OpticalSensor,
}

fn deadzone(value: f64, threshold: f64) -> f64 {
    if value.abs() < threshold { 0.0 } else { value }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            let forward = deadzone(state.left_stick.y(), 0.05);
            let sideways = deadzone(state.left_stick.x(), 0.05);
            let turn = deadzone(state.right_stick.x(), 0.05);

            let mut fl = forward + sideways + turn;
            let mut bl = forward - sideways + turn;
            let mut fr = forward - sideways - turn;
            let mut br = forward + sideways - turn;

            let max = fl.abs().max(fr.abs()).max(bl.abs()).max(br.abs());

            if max > 1.0 {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            if state.button_l1.is_pressed() {
                self.dt.brake_all(BrakeMode::Brake).await.ok();
                continue;
            }
            if state.button_r1.is_pressed() {
                self.dt.brake_all(BrakeMode::Hold).await.ok();
                continue;
            }

            self.dt.move_all(fl, fr, bl, br, forward).await.ok();

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let controller = peripherals.primary_controller;

    let screen = peripherals.display;

    let fl = Rc::new(RefCell::new(Motor::new(
        peripherals.port_1,
        Gearset::Blue,
        Direction::Reverse,
    )));
    let fr = Rc::new(RefCell::new(Motor::new(
        peripherals.port_2,
        Gearset::Blue,
        Direction::Reverse,
    )));
    let bl = Rc::new(RefCell::new(Motor::new(
        peripherals.port_3,
        Gearset::Blue,
        Direction::Reverse,
    )));
    let br = Rc::new(RefCell::new(Motor::new(
        peripherals.port_4,
        Gearset::Blue,
        Direction::Reverse,
    )));

    let tank_wheels = shared_motors![
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
    ];

    let mut imu = InertialSensor::new(peripherals.port_13);
    _ = imu.calibrate().await;

    let odom = Odometry {
        motors: [fl.clone(), fr.clone(), bl.clone(), br.clone()],
        tank: tank_wheels.clone(),
        imu: imu,
        color_sensor: OpticalSensor::new(peripherals.port_20),
    };

    let io = IO {
        front_bottom: Motor::new(peripherals.port_7, Gearset::Green, Direction::Forward),
        middle: Motor::new(peripherals.port_5, Gearset::Green, Direction::Forward),
        back_top: Motor::new(peripherals.port_6, Gearset::Green, Direction::Forward),
    };

    let robot = Robot {
        controller: controller,
        dt: DiffHolonomic::new(
            fl.clone(),
            fr.clone(),
            bl.clone(),
            br.clone(),
            tank_wheels.clone(),
        ),
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
