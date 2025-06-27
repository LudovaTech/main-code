use std::{error::Error, fs::OpenOptions};

use tracing::field::Visit;
use tracing::subscriber;
use tracing_panic::panic_hook;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::{self, Registry, fmt};
use tracing_subscriber::{EnvFilter, Layer};

use crate::prelude::*;

#[inline]
pub fn set_up_logging(rec: rerun::RecordingStream) -> Result<(), Box<dyn Error>> {
    std::fs::create_dir_all("log")?;
    let log_file = OpenOptions::new()
        .append(true)
        .create(true)
        .open("./log/rust-on-rasp.log")?;
    let subscriber_param = Registry::default()
        .with(fmt::layer().with_ansi(true))
        .with(fmt::layer().with_ansi(false).with_writer(log_file))
        .with(RerunLayer::new(rec))
        .with(EnvFilter::new("warn,lidar_analyzer=info")); // remember "-" must be replaced by "_" in the filter
    subscriber::set_global_default(subscriber_param)?;

    std::panic::set_hook(Box::new(panic_hook));
    info!("NEW START");
    Ok(())
}

// Couche de compatibilité tracing/rerun

#[derive(Debug)]
struct RerunLayer {
    rec: rerun::RecordingStream,
}

impl RerunLayer {
    fn new(rec: rerun::RecordingStream) -> Self {
        Self { rec }
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
        let mut log_level = event.metadata().level().as_str();
        let mut visitor = RerunLayerVisitor::default();
        event.record(&mut visitor);
        if *event.metadata().level() == tracing::Level::ERROR
            && (visitor.message.starts_with("!!!")
                || visitor.message.starts_with("A panic occurred"))
        {
            // Then the error is critical (there is no critical type in tracing)
            log_level = "CRITICAL"
        }

        let mut text_log = rerun::TextLog::new(format!(
            "{} | from {} | env {}.",
            visitor.message,
            event.metadata().name(),
            visitor.log_text
        ))
        .with_level(log_level);

        if log_level == "CRITICAL" {
            text_log = text_log.with_color(colors::DARK_RED);
        }

        self.rec
            .log("code/".to_owned() + &path_to.join("/"), &text_log)
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
