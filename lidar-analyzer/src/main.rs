use std::error::Error;
use std::fs::OpenOptions;

use tracing::{info, subscriber};
use tracing_panic::panic_hook;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{self, fmt, Registry};

mod units;
mod parse;
mod analyze;
mod analyze_tests_data;

#[cfg(test)]
mod basic_viewport;
#[cfg(test)]
mod complex_viewport;

#[inline]
fn set_up_logging() -> Result<(), Box<dyn Error>> {
    std::fs::create_dir_all("log")?;
    let log_file = OpenOptions::new()
        .append(true)
        .create(true)
        .open("./log/rust-on-rasp.log")?;
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
    let mut lidar = parse::Lidar::new()?;
    // loop {
    //     let data = lidar.read()?;
    //     let points = parse::LidarPoint::from_data(data)?;
    //     let mut points_pos = Vec::with_capacity(points.len());
    //     for point in points.iter() {
    //         points_pos.push((point.distance, point.angle));
    //     }
    //     info!("got {:?}", points_pos);
    // }
    Ok(())
}
