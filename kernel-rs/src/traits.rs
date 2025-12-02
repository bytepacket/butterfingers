use std::result;
use evian::prelude::Tank;
use vexide::smart::motor::BrakeMode;
use vexide::smart::PortError;

pub trait IOFunc {
    fn intake_store(&mut self) -> Result<(), PortError>;

    fn outtake_top(&mut self) -> Result<(), PortError>;

    fn outtake_middle(&mut self) -> Result<(), PortError>;

    fn outtake_bottom(&mut self) -> Result<(), PortError>;

    fn stop_intake(&mut self) -> Result<(), PortError>;
}

pub trait TankExt: Tank {
    fn brake_all(&mut self, mode: BrakeMode) -> Result<(), Self::Error>;

    fn move_all(&mut self, rpm: i32) -> Result<(), Self::Error>;
}