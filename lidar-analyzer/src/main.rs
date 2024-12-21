use std::error::Error;
use std::fs::OpenOptions;

use tracing::{info, subscriber};
use tracing_panic::panic_hook;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{self, fmt, Registry};

mod parse;

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
    loop {
        let data = lidar.read()?;
        if let Some(values) = data {
            info!("{:?}", values);
        }
    }
}
