use std::error::Error;
use std::fs::OpenOptions;
use std::path::Display;
use std::time::Duration;

use rerun::external::image::error;
use tracing::field::Visit;
use tracing::{Level, Subscriber, debug, info, subscriber, warn};
use tracing_panic::panic_hook;
use tracing_subscriber::field::RecordFields;
use tracing_subscriber::fmt::FormatEvent;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{self, Registry, fmt};
use tracing_subscriber::{EnvFilter, Layer};

mod analyze;
mod analyze_tests_data;
mod parse;
mod units;

#[cfg(test)]
mod complex_viewport;

#[derive(Debug)]
struct RerunLayer {
    rec: rerun::RecordingStream,
}

impl RerunLayer {
    fn new() -> Self {
        Self {
            rec: rerun::RecordingStreamBuilder::new("aaaa").spawn().unwrap(),
        }
    }
}

impl<S> Layer<S> for RerunLayer
where
    S: tracing::Subscriber,
{
    fn on_event(
        &self,
        event: &tracing::Event<'_>,
        _ctx: tracing_subscriber::layer::Context<'_, S>,
    ) {
        let path_to: Vec<&str> = event.metadata().target().split("::").take(2).collect();
        let mut visitor = RerunLayerVisitor::default();
        event.record(&mut visitor);
        self.rec
            .log(
                "code/".to_owned() + &path_to.join("/"),
                &rerun::TextLog::new(format!(
                    "{} | from {} | env {}.",
                    visitor.message,
                    event.metadata().name(),
                    visitor.log_text
                ))
                .with_level(event.metadata().level().as_str()),
            )
            .unwrap()
    }
}

#[derive(Debug, Default)]
struct RerunLayerVisitor {
    pub message: String,
    pub log_text: String,
}

impl Visit for RerunLayerVisitor {
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            self.message = format!("{:?}", value);
            return;
        }
        if !self.log_text.is_empty() {
            self.log_text += ", ";
        }
        self.log_text += &format!("{}: {:?}", field.name(), value);
    }
}

#[inline]
fn set_up_logging() -> Result<(), Box<dyn Error>> {
    std::fs::create_dir_all("log")?;
    let log_file = OpenOptions::new()
        .append(true)
        .create(true)
        .open("./log/rust-on-rasp.log")?;
    let subscriber_param = Registry::default()
        .with(fmt::layer().with_ansi(true))
        .with(fmt::layer().with_ansi(false).with_writer(log_file))
        .with(RerunLayer::new())
        .with(EnvFilter::new("warn,lidar_analyzer=info")); // remember "-" must be replaced by "_" in the filter
    subscriber::set_global_default(subscriber_param)?;

    std::panic::set_hook(Box::new(panic_hook));
    info!("NEW START");
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    //let rec = rerun::RecordingStreamBuilder::new("aaaa").spawn()?;
    set_up_logging()?;
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
