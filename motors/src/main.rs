
mod motors;
mod vector2;
use motors::Bogie;
use std::error::Error;

use crate::vector2::Vector2;
use radians::{Angle, Rad32};
use std::fs::OpenOptions;
use tracing::{info, subscriber};
use tracing_panic::panic_hook;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{self, fmt, Registry};

#[inline]
fn set_up_logging() -> Result<(), Box<dyn Error>> {
    std::fs::create_dir_all("log")?;
    let log_file = OpenOptions::new()
        .append(true)
        .create(true)
        .open("./log/motors.log")?;
    let subscriber_param = Registry::default()
        .with(fmt::layer().with_ansi(true))
        .with(fmt::layer().with_ansi(false).with_writer(log_file));
    subscriber::set_global_default(subscriber_param)?;

    std::panic::set_hook(Box::new(panic_hook));
    info!("NEW START");
    Ok(())
}


fn main() -> Result<(), Box<dyn Error>> {
    set_up_logging()?;
    let mut bogie = Bogie::default().unwrap();
    info!("init ok");
    bogie.go_to(Vector2::new(1.0, 1.0), 1.0, Rad32::new(1.0));
    loop {
 
    }
    Ok(())
}
