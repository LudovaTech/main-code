use crate::parse::LidarPoint;

// Constantes

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
/// distance minimale *en mm* à partir de laquelle les points détectés du lidar sont conservés
const LIDAR_DISTANCE_MIN: u16 = 9 * 10;
/// distance maximale *en mm* en dessous de laquelle les points détectés du lidar sont conservés
const LIDAR_DISTANCE_MAX: u16 = 300 * 10;

/// mm entre chaque droite calculée par Hough transform
pub const DISTANCE_RESOLUTION: u16 = 1 * 10;
/// 0.01 degrés entre chaque droite calculée par Hough transform (3° -> 30 ms, 1° -> 90ms)
pub const ANGLE_RESOLUTION: u16 = 3 * 100;

/// nombre d'angle différents possibles avec la résolution donnée
const ANGLE_DISTRIBUTION: usize = (360 * 100 / ANGLE_RESOLUTION) as usize + 1;
/// nombre de distances différentes possibles avec la résolution donnée
const DISTANCE_DISTRIBUTION: usize = (LIDAR_DISTANCE_MAX / DISTANCE_RESOLUTION) as usize + 1;


pub fn build_hough_accumulator(
    points: &Box<Vec<LidarPoint>>,
) -> [[u16; ANGLE_DISTRIBUTION]; DISTANCE_DISTRIBUTION] {
    let mut accumulator = [[0u16; ANGLE_DISTRIBUTION]; DISTANCE_DISTRIBUTION];
    for point in points.iter() {
        if point.distance > LIDAR_DISTANCE_MAX {
            continue;
        }
        let mut distance_rounded = (point.distance / DISTANCE_RESOLUTION) as usize;
        let mut angle_rounded = (point.angle / ANGLE_RESOLUTION) as usize;
        if ((point.distance % DISTANCE_RESOLUTION) as f32) >= (DISTANCE_RESOLUTION as f32 / 2.0) {
            // Si la distance est plus proche de la valeur arrondie supérieure que inférieure
            distance_rounded += 1;
        };
        if ((point.angle % ANGLE_RESOLUTION) as f32) >= (ANGLE_RESOLUTION as f32 / 2.0) {
            angle_rounded += 1;
        };

        accumulator[distance_rounded][angle_rounded] += 1;
    }
    accumulator
}
