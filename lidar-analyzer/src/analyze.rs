use core::f32;

use crate::parse::LidarPoint;

// Constantes

// EN mm !
const LIDAR_DISTANCE_MIN: u16 = 90;
const LIDAR_DISTANCE_MAX: u16 = 3000;

// Hough transform

/// degrés entre chaque droite calculée par Hough transform (3° -> 30 ms, 1° -> 90ms)
const DEGRE_STEP: usize = 3;
/// on exclut les lignes qui contiennent moins de 10 points
const HOUGH_TRANSFORM_ACCUMULATORS_THRESHOLD: u16 = 10;
/// taille de la matrice de Hough
const HOUGH_TRANSFORM_MEMORY_SIZE: u32 = 24000;
/// si une ligne est proche d'une autre de moins de 50cm, on l'exclut
const RHO_TOLERANCE: f32 = 500.0;
/// si une ligne a un angle theta inférieur à 0,5 rad d'une autre, on l'exclut
const THETA_MARGIN: f32 = 0.5;
/// pour trouver le mur parallèle au premier, il faut une différence d'angle inférieur à 0,2 rad
const THETA_TOLERANCE_PARALLEL: f32 = 0.2;
/// pour trouver les murs perpendiculaires, il faut une différence d'angle inférieur à 0,2 rad (après - PI/2)
const THETA_TOLERANCE_PERPENDICULAIRE: f32 = 0.2;

/// On filtre les points du lidar reçus pour qu'ils soient compris entre la taille minimum et maximum
#[inline]
fn filter_found_points_to_respect_distances(points: Box<Vec<LidarPoint>>) -> Box<Vec<LidarPoint>> {
    Box::new(
        points
            .into_iter()
            .filter(|e| (LIDAR_DISTANCE_MIN..LIDAR_DISTANCE_MAX).contains(&e.distance))
            .collect(),
    )
}

/// Hough Transform
/// algo : https://www.keymolen.com/2013/05/hough-transformation-c-implementation.html
struct HoughLine {}

impl HoughLine {
    pub fn hough_transform(points: Box<Vec<LidarPoint>>, distance_max: u16) -> Box<Vec<Self>> {
        let lines: Box<Vec<Self>> = Box::new(Vec::with_capacity(capacity));

        let num_theta: u32 = 180;
        let theta_step: f32 = DEGRE_STEP as f32 * f32::consts::PI / 180.0;
        let rho_step: f32 =
            distance_max as f32 * 2.0 * num_theta as f32 / HOUGH_TRANSFORM_MEMORY_SIZE as f32;
        for point in points.iter() {
            for theta_index in (0..num_theta).step_by(DEGRE_STEP) {
                let theta = theta_index as f32 * theta_step / DEGRE_STEP as f32;
                let rho_index = round((distance_max + (point.distance * (point.angle as f32).cos())) / rho_step);
            }
        }
        lines
    }
}

fn main(points: Box<Vec<LidarPoint>>) {
    let filtered = filter_found_points_to_respect_distances(points);
}
