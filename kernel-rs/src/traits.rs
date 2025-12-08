use std::result;
use vexide::smart::motor::BrakeMode;
use vexide::smart::PortError;

pub trait Asterisk {
    async fn brake_all(&mut self, mode: BrakeMode) -> Result<(), PortError>;

    async fn move_all(&mut self, fl: f64, fr: f64, bl: f64, br: f64, y: f64) -> Result<(), PortError>;
}

pub trait IOFunc {
    fn intake_store(&mut self) -> Result<(), PortError>;

    fn outtake_top(&mut self) -> Result<(), PortError>;

    fn outtake_middle(&mut self) -> Result<(), PortError>;

    fn outtake_bottom(&mut self) -> Result<(), PortError>;

    fn stop_intake(&mut self) -> Result<(), PortError>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HolonomicHeading {
    North,
    Northeast,
    East,
    Southeast,
    South,
    Southwest,
    West,
    Northwest,
}
