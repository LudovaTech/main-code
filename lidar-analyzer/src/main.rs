use std::error::Error;
use std::time::Duration;

mod analyze;
mod analyze_tests_data;
mod parse;
mod prelude;
mod units;
mod log_manager;

#[cfg(test)]
mod complex_viewport;

use crate::log_manager::set_up_logging;
use prelude::*;

fn main() -> Result<(), Box<dyn Error>> {
    let rec = rerun::RecordingStreamBuilder::new("lidar_analyzer").spawn()?;
    set_up_logging(rec.clone())?;
    //let lidar = parse::Lidar::new()?;
    // rec.log("test1", &rerun::Points2D::new([(0., 0.), (1., 1.)])).unwrap();
    // debug!("test1");
    // rec.log("test2", &rerun::TextLog::new("test2")).unwrap();
    // loop {
    //     let data = lidar.read()?;
    //     let points = parse::LidarPoint::from_data(data)?;
    //     let mut points_pos = Vec::with_capacity(points.len());
    //     for point in points.iter() {
    //         points_pos.push((point.distance, point.angle));
    //     }
    //     info!("got {:?}", points_pos);
    // }
    info!("it is working");
    warn!("ok !");
    std::thread::sleep(Duration::from_secs(2));
    analyze::test_call();
    Ok(())
}
