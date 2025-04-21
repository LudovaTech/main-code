use eframe::egui::Pos2;
use eframe::{egui, EventLoopBuilderHook};
use winit::platform::wayland::EventLoopBuilderExtWayland;

use crate::analyze::{HoughLine, PolarLine};
use crate::parse::{LidarAngle, LidarDistance, LidarPoint};
use crate::units::*;

const DEFAULT_ZOOM: f32 = 180.0;

struct PolarPointsApp {
    zoom: f32,
    points: Vec<LidarPoint>,
    lines: Vec<PolarLine>,
    walls: Vec<PolarLine>,
    offset: egui::Vec2,                 // Décalage
    dragging: bool,                     // Indique si la souris est en train d'être maintenue
    last_mouse_pos: Option<egui::Pos2>, // Dernière position de la souris
}

impl Default for PolarPointsApp {
    fn default() -> Self {
        let points = (0u16..360u16)
            .map(|i| {
                let angle = LidarAngle(i * 100);
                let distance = LidarDistance(300);
                LidarPoint {
                    angle,
                    distance,
                    intensity: Intensity::NULL,
                }
            })
            .collect();
        Self {
            zoom: DEFAULT_ZOOM,
            points,
            lines: Vec::new(),
            walls: Vec::new(),
            offset: egui::vec2(0.0, 0.0),
            dragging: false,
            last_mouse_pos: None,
        }
    }
}

impl PolarPointsApp {
    pub fn new(points: Vec<LidarPoint>, lines: Vec<PolarLine>, walls: Vec<PolarLine>) -> Self {
        Self {
            zoom: DEFAULT_ZOOM,
            points,
            lines,
            walls,
            offset: egui::vec2(0.0, 0.0),
            dragging: false,
            last_mouse_pos: None,
        }
    }
}

impl eframe::App for PolarPointsApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Vérifier si la touche "H" est pressée
        if ctx.input(|i| i.key_pressed(egui::Key::H)) {
            self.zoom = DEFAULT_ZOOM; // Réinitialiser le zoom
            self.offset = egui::vec2(0.0, 0.0); // Réinitialiser le décalage
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.label("Basic Viewport");
            ui.label(format!("zoom : {}", self.zoom));
            ui.label(format!("décallage : {}", self.offset));

            // Zoom avec la molette de la souris
            let scroll_delta = ui.input(|i| i.smooth_scroll_delta);
            self.zoom *= 1.0 + scroll_delta.y * 0.01; // Ajustez le facteur de zoom ici

            // Gérer le décalage avec la souris
            let pointer = ui.input(|i| i.pointer.clone());
            if pointer.any_released() {
                self.dragging = false;
                self.last_mouse_pos = None;
            }

            if pointer.any_pressed() {
                self.dragging = true;
                self.last_mouse_pos = pointer.interact_pos();
            }

            if self.dragging {
                if let Some(current_mouse_pos) = pointer.interact_pos() {
                    if let Some(last_pos) = self.last_mouse_pos {
                        let delta = current_mouse_pos - last_pos;
                        self.offset += delta;
                    }
                    self.last_mouse_pos = Some(current_mouse_pos);
                }
            }

            // Dessiner les points
            let painter = ui.painter();
            let center = ui.available_rect_before_wrap().center() + self.offset;

            for point in &self.points {
                let x = center.x + (point.angle.to_deg().cos() * point.distance.to_meters().0) as f32 * self.zoom;
                let y = center.y + (point.angle.to_deg().sin() * point.distance.to_meters().0) as f32 * self.zoom;
                painter.circle(
                    egui::pos2(x, y),
                    5.0,
                    egui::Color32::GREEN.gamma_multiply(0.5),
                    egui::Stroke::NONE,
                );
            }

            self.draw_lines(ui, ctx, center, &self.lines, egui::Stroke::new(5.0, egui::Color32::DARK_RED));
            self.draw_lines(ui, ctx, center, &self.walls, egui::Stroke::new(5.0, egui::Color32::BLUE));
            
        });
    }

}

impl PolarPointsApp {
    fn draw_lines(&self, ui:&mut egui::Ui, ctx: &egui::Context, center: Pos2, lines: &Vec<PolarLine>, stroke: egui::Stroke) {
        for line in lines {
            let perpendicular_angle = line.angle + Rad::QUARTER_TURN;
            let (width, height) = ctx.screen_rect().size().into();
            let length: f32 = 2.0 * (width.powi(2) + height.powi(2)).sqrt(); // Assez bonne approximation
            let x_end: f32 = center.x + ((line.distance.0 * line.angle.cos()) as f32) * self.zoom;
            let y_end: f32 = center.y + ((line.distance.0 * line.angle.sin()) as f32) * self.zoom;
            let x1: f32 = x_end + length * perpendicular_angle.cos() as f32;
            let y1: f32 = y_end + length * perpendicular_angle.sin() as f32;
            let x2: f32 = x_end - length * perpendicular_angle.cos() as f32;
            let y2: f32 = y_end - length * perpendicular_angle.sin() as f32;
            ui.painter().line_segment(
                [
                    egui::pos2(x1, y1),
                    egui::pos2(x2, y2),
                ],
                stroke,
            );
        }
    }
}

pub fn show_viewport(points: Vec<LidarPoint>, lines: Vec<PolarLine>, walls: Vec<PolarLine>) -> Result<(), eframe::Error> {
    let event_loop_builder: Option<EventLoopBuilderHook> = Some(Box::new(|event_loop_builder| {
        event_loop_builder.with_any_thread(true);
    }));
    let options = eframe::NativeOptions {
        event_loop_builder,
        viewport: egui::ViewportBuilder::default().with_inner_size([800.0, 800.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Points en coordonnées polaires",
        options,
        Box::new(|_cc| Ok(Box::new(PolarPointsApp::new(points, lines, walls)))),
    )
}
