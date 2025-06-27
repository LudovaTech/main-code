use std::error::Error;

use crate::analyze::PolarLine;
use crate::parse::LidarPoint;
use crate::prelude::*;

#[inline]
fn line_coords(vline: &ViewportLine) -> [(f32, f32); 2] {
    let perpendicular_angle = vline.line.angle + Rad::QUARTER_TURN;
    let length: f32 = 3.0;
    let x_end: f32 = (vline.line.distance.0 * vline.line.angle.cos()) as f32;
    let y_end: f32 = (vline.line.distance.0 * vline.line.angle.sin()) as f32;
    let x1: f32 = x_end + length * perpendicular_angle.cos() as f32;
    let y1: f32 = y_end + length * perpendicular_angle.sin() as f32;
    let x2: f32 = x_end - length * perpendicular_angle.cos() as f32;
    let y2: f32 = y_end - length * perpendicular_angle.sin() as f32;
    [(x1, y1), (x2, y2)]
}

pub fn log_lidar_points(
    rec: rerun::RecordingStream,
    points: Vec<LidarPoint>,
) -> Result<(), Box<dyn Error>> {
    rec.log(
        "lidar/points",
        &rerun::Points2D::new(points.iter().map(|e| e.point.to_carthesian_point_f32()))
            .with_labels(points.iter().map(|e| format!("{:#?}", e))),
    )?;
    Ok(())
}

pub fn log_lidar_lines(
    rec: rerun::RecordingStream,
    lines: Vec<ViewportLine>,
) -> Result<(), Box<dyn Error>> {
    let line_points: Vec<(f32, f32)> = lines.iter().map(|e| line_coords(e)).flatten().collect();
    rec.log(
        "lidar/lines",
        &rerun::LineStrips2D::new(line_points.chunks(2)).with_colors(
            lines
                .iter()
                .map(|e| rerun::Rgba32::from_rgb(e.color.0, e.color.1, e.color.2)),
        ),
    )?;
    Ok(())
}

#[derive(Debug)]
pub struct ViewportLine {
    pub line: PolarLine,
    pub color: (u8, u8, u8),
}
