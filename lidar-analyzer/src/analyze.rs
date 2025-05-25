use std::cmp::Reverse;
use std::error::Error;
use std::fmt::Display;

use crate::parse::{LidarAngle, LidarDistance, LidarPoint};
use crate::units::*;

// Constantes

// /// on exclut les lignes qui contiennent moins de x points
// const HOUGH_TRANSFORM_MIN_POINT_PER_LINE: u16 = 10;
// /// taille de la matrice de Hough
// const HOUGH_TRANSFORM_MEMORY_SIZE: u32 = 24000;
// /// si une ligne est proche d'une autre de moins de 50cm, on l'exclut
// const RHO_TOLERANCE: f32 = 500.0;
// /// si une ligne a un angle theta inférieur à 0,5 rad d'une autre, on l'exclut
// const THETA_MARGIN: f32 = 0.5;
// /// pour trouver le mur parallèle au premier, il faut une différence d'angle inférieur à 0,2 rad
// const THETA_TOLERANCE_PARALLEL: f32 = 0.2;
// /// pour trouver les murs perpendiculaires, il faut une différence d'angle inférieur à 0,2 rad (après - PI/2)
// const THETA_TOLERANCE_PERPENDICULAIRE: f32 = 0.2;
// /// distance minimale à partir de laquelle les points détectés du lidar sont conservés
// const LIDAR_DISTANCE_MIN: Meters = Meters::cm(9.0);
// /// distance maximale en dessous de laquelle les points détectés du lidar sont conservés
// const LIDAR_DISTANCE_MAX: Meters = Meters(3.0);

// /// distance entre chaque droite calculée par Hough transform
// pub const DISTANCE_RESOLUTION: Meters = Meters::cm(1.0);
// /// Degrés entre chaque droite calculée par Hough transform (3° -> 30 ms, 1° -> 90ms)
// /// Le passage en Rad est impossible ici car nous sommes en context constant
// pub const ANGLE_RESOLUTION_OLD_TYPE: u16 = 3 * 100;

// /// nombre d'angle différents possibles avec la résolution donnée
// pub const ANGLE_DISTRIBUTION: usize = (360_00 / ANGLE_RESOLUTION_OLD_TYPE) as usize + 1;
// /// nombre de distances différentes possibles avec la résolution donnée, attention à bien compter les distances négatives (d'où le *2)
// pub const DISTANCE_DISTRIBUTION: usize =
//     (LIDAR_DISTANCE_MAX.const_div(DISTANCE_RESOLUTION).0 * 2.0) as usize + 1;

/// en radians
const IS_PERPENDICULAR_TOLERANCE: f64 = 0.2;
/// en radians
const IS_PARALLEL_TOLERANCE: f64 = 0.2;
/// en radians
const IS_PARALLEL_EXACT_TOLERANCE: f64 = 10e-5;

#[derive(Debug, Clone, Copy)]
pub struct PolarLine {
    pub distance: Meters,
    pub angle: Rad,
}

impl PolarLine {
    /// Renvoit l'angle aigu entre deux lignes polaires
    /// (faire un schéma pour se convaincre rapidement de la formule)
    fn smallest_angle_between(&self, other: &Self) -> Rad {
        let alpha = (self.angle - other.angle).mag().val() % Rad::HALF_TURN.val();
        if alpha >= Rad::QUARTER_TURN.val() {
            Rad::new(Rad::HALF_TURN.val() - alpha)
        } else {
            Rad::new(alpha)
        }
    }

    /// Avec une tolérance
    fn is_parallel_with(&self, other: &Self) -> bool {
        self.smallest_angle_between(other) <= Rad::new(IS_PARALLEL_EXACT_TOLERANCE)
    }

    /// Avec une tolérance
    fn is_approx_parallel_with(&self, other: &Self) -> bool {
        self.smallest_angle_between(other) <= Rad::new(IS_PARALLEL_TOLERANCE)
    }

    /// Avec une tolérance
    fn is_approx_perpendicular_with(&self, other: &Self) -> bool {
        (self.smallest_angle_between(other) - Rad::QUARTER_TURN).mag()
            <= Rad::new(IS_PERPENDICULAR_TOLERANCE)
    }

    /// N'a du sens que si les lignes sont à peu près parallèles
    fn distance_center_with(&self, other: &Self) -> Meters {
        // Impossible de faire self.distance.abs() + other.distance.abs() car cela donnerait des résultats éloignés si la tolérance est trop élevée
        let (x1, y1) = self._to_carthesian_point();
        let (x2, y2) = other._to_carthesian_point();
        Meters(((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt())
    }

    /// A n'utiliser que si vous êtes sûr de ce que vous faites,
    /// *Vous transformez une droite en point !*
    fn _to_carthesian_point(&self) -> (f64, f64) {
        (
            self.distance.0 * self.angle.cos(),
            self.distance.0 * self.angle.sin(),
        )
    }

    /// Calcule le point d'intersection de deux droites
    /// Les droites ne doivent pas être parallèles pour que le résultat aie du sens
    /// Redémontrer avec r = r0 / cos(theta - theta0)
    fn intersect(&self, other: &Self) -> Option<PolarPoint> {
        let self_angle_wrap = Rad::new(self.angle.val() % Rad::FULL_TURN.val());
        let other_angle_wrap = Rad::new(other.angle.val() % Rad::FULL_TURN.val());
        if self.is_parallel_with(other) {
            // Les droites sont alignés
            return None;
        }
        let theta = Rad::atan2(
            self.distance.0 * other_angle_wrap.cos() - other.distance.0 * self_angle_wrap.cos(),
            self.distance.0 * other_angle_wrap.sin() - other.distance.0 * self_angle_wrap.sin(),
        )
        .mag();

        let r = Meters(self.distance.0 / (theta - self_angle_wrap).cos());
        Some(PolarPoint {
            distance: r,
            angle: theta,
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HoughLine {
    pub line: PolarLine,
    pub weight: u16,
}

/// ax + by + c = 0
#[derive(Debug, Clone)]
pub struct PolarPoint {
    pub distance: Meters,
    pub angle: Rad,
}

impl PolarPoint {
    fn from_lidar_point(lidar_point: &LidarPoint) -> Self {
        Self {
            distance: lidar_point.distance.to_meters(),
            angle: lidar_point.angle.to_deg().rad(),
        }
    }
}

/// ax + by + c = 0
#[derive(Debug, Clone)]
pub struct CarthesianLine {
    pub a: f64,
    pub b: f64,
    pub c: f64,
}

#[inline]
fn check_around(
    accumulator: &mut [[u16; ANGLE_TAILLE]; DISTANCE_TAILLE],
    distance_case: usize,
    angle_case: usize,
    dist: usize,
    reward: u16,
) {
    let distance_case_added_option = distance_case.checked_add(dist);
    let angle_case_added_option = angle_case.checked_add(dist);
    let distance_case_rem_option = distance_case.checked_sub(dist);
    let angle_case_rem_option = angle_case.checked_sub(dist);

    if let Some(distance_case_added) = distance_case_added_option {
        if let Some(angles) = accumulator.get_mut(distance_case_added) {
            if let Some(angle_case_added) = angle_case_added_option {
                if let Some(weight) = angles.get_mut(angle_case_added) {
                    *weight = weight.saturating_add(reward);
                }
            }
            if let Some(angle_case_rem) = angle_case_rem_option {
                if let Some(weight) = angles.get_mut(angle_case_rem) {
                    *weight = weight.saturating_add(reward);
                }
            }
        }
    }
    if let Some(distance_case_rem) = distance_case_rem_option {
        if let Some(angles) = accumulator.get_mut(distance_case_rem) {
            if let Some(angle_case_added) = angle_case_added_option {
                if let Some(weight) = angles.get_mut(angle_case_added) {
                    *weight = weight.saturating_add(reward);
                }
            }
            if let Some(angle_case_rem) = angle_case_rem_option {
                if let Some(weight) = angles.get_mut(angle_case_rem) {
                    *weight = weight.saturating_add(reward);
                }
            }
        }
    }
}

/// on exclut les lignes qui contiennent moins de x points
const HOUGH_TRANSFORM_MIN_POINT_PER_LINE: u16 = 50;

/// distance maximale en dessous de laquelle les points détectés du lidar sont conservés
const LIDAR_DISTANCE_MAX: LidarDistance = LidarDistance::meters(3);

/// angle auquel on crée une nouvelle ligne dans la transformation de Hough
const ANGLE_RESOLUTION: LidarAngle = LidarAngle::deg(1);

/// nombre d'angles différents dans la matrice de la transformation de Hough
/// On ne va que de 0 à 180° car les distances peuvent être négatives
const ANGLE_TAILLE: usize = (LidarAngle::deg(180).0 / ANGLE_RESOLUTION.0) as usize + 1;

/// distance à laquelle on crée une nouvelle ligne dans la transformation de Hough
const DISTANCE_RESOLUTION: LidarDistance = LidarDistance::cm(1);

/// nombre de distances différentes dans la matrice de la transformation de Hough
/// On compte 2x car il y a les distances positives et négatives
const DISTANCE_TAILLE: usize = (LIDAR_DISTANCE_MAX.0 * 2 / DISTANCE_RESOLUTION.0) as usize + 1;

/// guide : <https://www.keymolen.com/2013/05/hough-transformation-c-implementation.html>
fn build_hough_accumulator(points: &Vec<LidarPoint>) -> Vec<HoughLine> {
    // Création de la matrice de la transformation de Hough
    let mut accumulator = [[0u16; ANGLE_TAILLE]; DISTANCE_TAILLE];
    for point in points.iter() {
        if point.distance > LIDAR_DISTANCE_MAX {
            continue;
        }

        // Formule magique. `lpd * cos(lpa - a)`
        // Se prouve avec : lpd = lidarPoint.distance, lpa = lidarPoint.angle
        // `x = lpd * cos(lpa)`, `y = lpd * sin(lpa)`, `r_a = x * cos(a) + y * sin(a)`, et les formules d'addition du cosinus
        for calculated_angle in (0..=(LidarAngle::deg(180).0)).step_by(ANGLE_RESOLUTION.0.into()) {
            let calculated_angle = LidarAngle(calculated_angle);
            // println!("{:?}", calculated_angle);

            // cos(x) = cos(abs(x))
            let distance_factor = point
                .angle
                .distance_between_angle(calculated_angle)
                .to_deg()
                .cos();
            let offset = if distance_factor.is_sign_positive() {
                DISTANCE_TAILLE / 2
            } else {
                0
            };
            let calculated_distance = point.distance * distance_factor.abs();
            let distance_case: usize =
                offset + (calculated_distance / DISTANCE_RESOLUTION) as usize;
            let angle_case: usize = (calculated_angle / ANGLE_RESOLUTION).into();
            // println!("{} {} {}", distance_case, offset, distance_factor);
            accumulator[distance_case][angle_case] =
                accumulator[distance_case][angle_case].saturating_add(1);
            // TODO est-ce que check around améliore les résultats ?
            // check_around(&mut accumulator, distance_case, angle_case, 1, 80);
            // check_around(&mut accumulator, distance_case, angle_case, 2, 65);
            // check_around(&mut accumulator, distance_case, angle_case, 3, 40);
            // check_around(&mut accumulator, distance_case, angle_case, 4, 25);
            // check_around(&mut accumulator, distance_case, angle_case, 5, 10);
        }
    }

    // Tri des lignes selon le nombre de points
    let mut trie = Vec::with_capacity(ANGLE_TAILLE * DISTANCE_TAILLE);
    for (mut distance_case, angles) in accumulator.into_iter().enumerate() {
        for (angle_case, weight) in angles.into_iter().enumerate() {
            if weight < HOUGH_TRANSFORM_MIN_POINT_PER_LINE {
                continue;
            }
            let mut angle_offset = Rad::ZERO;
            if distance_case >= DISTANCE_TAILLE / 2 {
                distance_case -= DISTANCE_TAILLE / 2;
            } else {
                angle_offset = Rad::HALF_TURN;
            }

            trie.push(HoughLine {
                line: PolarLine {
                    distance: (DISTANCE_RESOLUTION * distance_case).to_meters(),
                    angle: angle_offset + (ANGLE_RESOLUTION * angle_case).to_deg().rad(),
                },
                weight,
            })
        }
    }
    trie.sort_by_key(|e| Reverse(e.weight));
    trie
}

// TODO should not be here
const FIELD_LENGTH: Meters = Meters::cm(243.0);
// TODO should not be here
const FIELD_WIDTH: Meters = Meters::cm(182.0);

#[derive(Debug, Clone, Copy)]
pub enum WallLine {
    HoughLine(HoughLine),
    GuessedLine(GuessedLine),
}

impl WallLine {
    #[inline]
    pub fn line(&self) -> PolarLine {
        match self {
            Self::HoughLine(hought) => hought.line,
            Self::GuessedLine(guessed) => guessed.line,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct GuessedLine {
    line: PolarLine,
}

#[derive(Debug, Clone, Copy)]
struct WallsConstruction {
    first_wall: WallLine,
    parallele_wall: Option<WallLine>,
    perpendicular_wall_1: Option<WallLine>,
    perpendicular_wall_2: Option<WallLine>,
}

impl WallsConstruction {
    fn new(first_wall: HoughLine) -> WallsConstruction {
        WallsConstruction {
            first_wall: WallLine::HoughLine(first_wall),
            parallele_wall: None,
            perpendicular_wall_1: None,
            perpendicular_wall_2: None,
        }
    }

    fn is_complete(&self) -> bool {
        self.parallele_wall.is_some()
            && self.perpendicular_wall_1.is_some()
            && self.perpendicular_wall_2.is_some()
    }

    fn build_walls(&self) -> Option<Walls> {
        if !self.is_complete() {
            return None;
        }
        Some(Walls {
            first_wall: self.first_wall,
            parallele_wall: self.parallele_wall.unwrap(),
            perpendicular_wall_1: self.perpendicular_wall_1.unwrap(),
            perpendicular_wall_2: self.perpendicular_wall_2.unwrap(),
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct Walls {
    first_wall: WallLine,
    parallele_wall: WallLine,
    perpendicular_wall_1: WallLine,
    perpendicular_wall_2: WallLine,
}

struct WallsIntoIterator {
    walls: Walls,
    step: u8,
}

// impl IntoIterator for Walls {
//     type IntoIter = WallsIntoIterator;
//     type Item = HoughLine;

//     fn into_iter(self) -> Self::IntoIter {
//         WallsIntoIterator {
//             walls: self,
//             step: 0,
//         }
//     }
// }

// impl Iterator for WallsIntoIterator {
//     type Item = HoughLine;

//     fn next(&mut self) -> Option<Self::Item> {
//         let result = match self.step {
//             0 => self.walls.first_wall,
//             1 => self.walls.perpendicular_wall_1,
//             2 => self.walls.parallele_wall,
//             3 => self.walls.perpendicular_wall_2,
//             _ => return None,
//         };
//         self.step += 1;
//         Some(result)
//     }
// }

#[derive(Debug)]
enum WallFinderError {
    DetectedLineEmpty,
    UnableToFindAllWalls,
}

impl Display for WallFinderError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl Error for WallFinderError {}

/// detected_lines doit déjà être trié
fn find_walls(detected_lines: Vec<HoughLine>) -> Result<WallsConstruction, WallFinderError> {
    if detected_lines.len() == 0 {
        return Err(WallFinderError::DetectedLineEmpty);
    }
    let mut walls = WallsConstruction::new(detected_lines[0]);
    for hough_line in detected_lines.iter().skip(1) {
        let line = hough_line.line;
        // On essaie de déterminer si cette ligne peut-être la perpendiculaire 1
        if walls.perpendicular_wall_1.is_none()
            && line.is_approx_perpendicular_with(&walls.first_wall.line())
        {
            walls.perpendicular_wall_1 = Some(WallLine::HoughLine(*hough_line));
            continue;
        }
        // On essaie de déterminer si cette ligne peut-être la parallèle
        // TODO on ne checke pas si on a deux distances FIELD_LENGTH ou deux distances FIELD_WIDTH
        println!(
            "distance center {:?}, vise {:?}, {:?} pourtant {:?} donc {:?}",
            line.distance_center_with(&walls.first_wall.line()),
            FIELD_LENGTH,
            FIELD_WIDTH,
            line.distance_center_with(&walls.first_wall.line())
                .in_the_aera_of(FIELD_WIDTH),
            line.smallest_angle_between(&walls.first_wall.line())
        );
        if walls.parallele_wall.is_none()
            && line.is_approx_parallel_with(&walls.first_wall.line())
            && (line
                .distance_center_with(&walls.first_wall.line())
                .in_the_aera_of(FIELD_LENGTH)
                || line
                    .distance_center_with(&walls.first_wall.line())
                    .in_the_aera_of(FIELD_WIDTH))
        {
            println!("inside");
            let mut ok = true;
            if let Some(perpendicular1) = walls.perpendicular_wall_1 {
                dbg!(line.is_approx_perpendicular_with(&perpendicular1.line()));
                ok = ok && line.is_approx_perpendicular_with(&perpendicular1.line())
            }
            if let Some(perpendicular2) = walls.perpendicular_wall_2 {
                dbg!(line.is_approx_perpendicular_with(&perpendicular2.line()));
                ok = ok && line.is_approx_perpendicular_with(&perpendicular2.line())
            }
            ok = true;
            if ok {
                walls.parallele_wall = Some(WallLine::HoughLine(*hough_line))
            }
        }
        // On essaie de déterminer si cette ligne peut-être la perpendiculaire 2
        // TODO on ne checke pas si on a deux distances FIELD_LENGTH ou deux distances FIELD_WIDTH
        if walls.perpendicular_wall_2.is_none()
            && line.is_approx_perpendicular_with(&walls.first_wall.line())
        {
            let mut ok = true;
            if let Some(perpendicular1) = walls.perpendicular_wall_1 {
                ok = ok
                    && line.is_approx_parallel_with(&perpendicular1.line())
                    && (line
                        .distance_center_with(&perpendicular1.line())
                        .in_the_aera_of(FIELD_LENGTH)
                        || line
                            .distance_center_with(&perpendicular1.line())
                            .in_the_aera_of(FIELD_WIDTH))
            }
            if let Some(parallele) = walls.parallele_wall {
                ok = ok && line.is_approx_perpendicular_with(&parallele.line())
            }
            if ok {
                walls.perpendicular_wall_2 = Some(WallLine::HoughLine(*hough_line))
            }
        }
        if walls.is_complete() {
            break;
        }
    }

    println!("{:#?}", walls);
    return Ok(walls);
    // walls
    //     .build_walls()
    //     .ok_or(WallFinderError::UnableToFindAllWalls)
}

fn try_to_complete_walls_from_uncomplete_data(
    detected_walls: WallsConstruction,
) -> Result<WallsConstruction, WallFinderError> {
    // TODO 2 murs doivent suffire à l'avenir
    match detected_walls {
        WallsConstruction {
            first_wall,
            parallele_wall: Some(parallele_wall),
            perpendicular_wall_1: Some(perpendicular_wall_1),
            perpendicular_wall_2: None,
        } => {
            // On va deviner le mur perpendicular 2
            // Note : on se rappelle que le robot est toujours sur le terrain et qu'on a donc un seul mur possible.
            let space = if first_wall
                .line()
                .distance_center_with(&parallele_wall.line())
                .in_the_aera_of(FIELD_LENGTH)
            {
                FIELD_WIDTH
            } else {
                FIELD_LENGTH
            };
            Ok(WallsConstruction {
                perpendicular_wall_2: Some(WallLine::GuessedLine(GuessedLine {
                    line: PolarLine {
                        distance: -space + perpendicular_wall_1.line().distance,
                        ..perpendicular_wall_1.line()
                    },
                })),
                ..detected_walls
            })
        }

        WallsConstruction {
            first_wall,
            parallele_wall: None,
            perpendicular_wall_1: Some(perpendicular_wall_1),
            perpendicular_wall_2: Some(perpendicular_wall_2),
        } => {
            // On va deviner le mur parallele
            // Note : on se rappelle que le robot est toujours sur le terrain et qu'on a donc un seul mur possible.
            let space = if perpendicular_wall_1
                .line()
                .distance_center_with(&perpendicular_wall_2.line())
                .in_the_aera_of(FIELD_LENGTH)
            {
                FIELD_WIDTH
            } else {
                FIELD_LENGTH
            };
            Ok(WallsConstruction {
                parallele_wall: Some(WallLine::GuessedLine(GuessedLine {
                    line: PolarLine {
                        distance: -space + first_wall.line().distance,
                        ..first_wall.line()
                    },
                })),
                ..detected_walls
            })
        }

        _ => Ok(detected_walls),
    }
}

#[cfg(test)]
mod tests {
    use eframe::egui;

    use super::*;
    use crate::analyze_tests_data::lidar_test_data::*;
    use crate::{
        basic_viewport::show_viewport,
        parse::{LidarAngle, LidarDistance},
    };
    use std::time::{Duration, Instant};

    fn load_log(from: &str) -> Box<Vec<LidarPoint>> {
        let mut vector = Box::new(Vec::with_capacity(from.len() / 10));
        for point in from.split(';') {
            let mut iter_num = point.trim_matches(|c| c == '(' || c == ')').split(',');
            let angle = iter_num.next().unwrap().parse::<u16>().unwrap();
            let distance = iter_num.next().unwrap().parse::<u16>().unwrap();
            assert!(iter_num.next().is_none());
            vector.push(LidarPoint {
                distance: LidarDistance(distance),
                intensity: Intensity::NULL,
                angle: LidarAngle(angle),
            });
        }
        vector
    }

    fn approx_equal_rad(a: Rad, b: Rad, epsilon: Rad) -> Result<(), String> {
        let mut diff = a - b;
        if diff < Rad::ZERO {
            diff = -diff;
        }
        if diff >= epsilon {
            return Err(format!("{} ~!= {}  {:?} ~!= {:?}", a, b, a, b));
        }
        Ok(())
    }

    fn approx_equal_meters(a: Meters, b: Meters, epsilon: Meters) -> Result<(), String> {
        if (a.0 - b.0).abs() >= epsilon.0 {
            return Err(format!("{:?} ~!= {:?}", a, b));
        }
        Ok(())
    }

    const GEOGEBRA_LIMIT: f64 = 1e-2;
    const APPROX_LIMIT: f64 = 1e-10;

    #[test]
    fn test_same_angle() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        };
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        };
        approx_equal_rad(
            line1.smallest_angle_between(&line2),
            Rad::new(0.0),
            Rad::new(APPROX_LIMIT),
        )
        .unwrap();
    }

    #[test]
    fn test_opposite_angles() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        };
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(std::f64::consts::PI),
        };
        approx_equal_rad(
            line1.smallest_angle_between(&line2),
            Rad::new(std::f64::consts::PI),
            Rad::new(APPROX_LIMIT),
        )
        .unwrap();
    }

    #[test]
    fn test_acute_angles() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        };
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(std::f64::consts::FRAC_PI_4),
        }; // 45 degrees
        approx_equal_rad(
            line1.smallest_angle_between(&line2),
            Rad::new(std::f64::consts::FRAC_PI_4),
            Rad::new(APPROX_LIMIT),
        )
        .unwrap();
    }

    #[test]
    fn test_obtuse_angles() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(std::f64::consts::FRAC_PI_3),
        }; // 60 degrees
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(std::f64::consts::FRAC_PI_2),
        }; // 90 degrees
        approx_equal_rad(
            line1.smallest_angle_between(&line2),
            Rad::new(std::f64::consts::FRAC_PI_6),
            Rad::new(APPROX_LIMIT),
        )
        .unwrap();
    }

    #[test]
    fn test_full_circle() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        };
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(2.0 * std::f64::consts::PI),
        }; // 360 degrees
        approx_equal_rad(
            line1.smallest_angle_between(&line2),
            Rad::new(0.0),
            Rad::new(APPROX_LIMIT),
        )
        .unwrap();
    }

    #[test]
    fn test_negative_angles() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(-std::f64::consts::FRAC_PI_4),
        }; // -45 degrees
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(std::f64::consts::FRAC_PI_4),
        }; // 45 degrees
        approx_equal_rad(
            line1.smallest_angle_between(&line2),
            Rad::new(std::f64::consts::FRAC_PI_2),
            Rad::new(APPROX_LIMIT),
        )
        .unwrap();
    }

    #[test]
    fn test_exact_parallel() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        }; // 0 degrees
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        }; // 0 degrees
        assert!(line1.is_approx_parallel_with(&line2));
    }

    #[test]
    fn test_near_parallel() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        }; // 0 degrees
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(IS_PARALLEL_TOLERANCE / 2.0),
        }; // Slightly off
        assert!(line1.is_approx_parallel_with(&line2));
    }

    #[test]
    fn test_not_parallel() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        }; // 0 degrees
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(std::f64::consts::FRAC_PI_2),
        }; // 90 degrees
        assert!(!line1.is_approx_parallel_with(&line2));
    }

    #[test]
    fn test_exact_perpendicular() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        }; // 0 degrees
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(std::f64::consts::FRAC_PI_2),
        }; // 90 degrees
        assert!(line1.is_approx_perpendicular_with(&line2));
    }

    #[test]
    fn test_near_perpendicular() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        }; // 0 degrees
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(std::f64::consts::FRAC_PI_2 + IS_PERPENDICULAR_TOLERANCE / 2.0),
        }; // Slightly off
        assert!(line1.is_approx_perpendicular_with(&line2));
    }

    #[test]
    fn test_not_perpendicular() {
        let line1 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(0.0),
        }; // 0 degrees
        let line2 = PolarLine {
            distance: Meters(1.0),
            angle: Rad::new(std::f64::consts::FRAC_PI_3),
        }; // 60 degrees
        assert!(!line1.is_approx_perpendicular_with(&line2));
    }

    #[test]
    fn test_intersection_normal() {
        let line1 = PolarLine {
            distance: Meters(5.0),
            angle: Rad::new(0.0),
        }; // Ligne horizontale
        let line2 = PolarLine {
            distance: Meters(5.0),
            angle: Rad::new(std::f64::consts::FRAC_PI_4),
        }; // Ligne à 45 degrés

        let intersection = line1.intersect(&line2);
        assert!(intersection.is_some());

        let point = intersection.unwrap();
        // Vérifié sur geogebra
        approx_equal_rad(point.angle, Rad::new(0.39), Rad::new(GEOGEBRA_LIMIT)).unwrap();
        approx_equal_meters(point.distance, Meters(5.41), Meters(GEOGEBRA_LIMIT)).unwrap();
    }

    #[test]
    fn test_intersection_more_than_full_circle() {
        let line1 = PolarLine {
            distance: Meters(5.0),
            angle: Rad::new(5.0),
        }; // Ligne horizontale
        let line2 = PolarLine {
            distance: Meters(10.0),
            angle: Rad::new(5.0),
        }; // Ligne horizontale parallèle

        let intersection = line1.intersect(&line2);
        dbg!(&intersection);
        assert!(intersection.is_none()); // Pas d'intersection
    }

    #[test]
    fn test_intersection_coincident_lines() {
        let line1 = PolarLine {
            distance: Meters(5.0),
            angle: Rad::new(0.0),
        }; // Ligne horizontale
        let line2 = PolarLine {
            distance: Meters(5.0),
            angle: Rad::new(0.0),
        }; // Même ligne

        let intersection = line1.intersect(&line2);
        assert!(intersection.is_none()); // Pas d'intersection unique
    }

    #[test]
    fn test_intersection_with_different_angles() {
        let line1 = PolarLine {
            distance: Meters(10.0),
            angle: Rad::new(0.5),
        }; // Ligne horizontale
        let line2 = PolarLine {
            distance: Meters(8.0),
            angle: Rad::new(std::f64::consts::FRAC_PI_2),
        }; // Ligne verticale

        let intersection = line1.intersect(&line2);
        assert!(intersection.is_some());

        let point = intersection.unwrap();
        approx_equal_rad(point.angle, Rad::new(0.85), Rad::new(GEOGEBRA_LIMIT)).unwrap();
        approx_equal_meters(point.distance, Meters(10.64), Meters(GEOGEBRA_LIMIT)).unwrap();
    }

    #[test]
    fn test_intersection_different_quarters() {
        let line1 = PolarLine {
            distance: Meters(10.0),
            angle: Rad::new(0.5),
        }; // Ligne horizontale
        let line2 = PolarLine {
            distance: Meters(8.0),
            angle: Rad::new(std::f64::consts::PI + 0.1),
        }; // Ligne verticale

        let intersection = line1.intersect(&line2);
        assert!(intersection.is_some());

        let point = intersection.unwrap();
        approx_equal_rad(point.angle, Rad::new(1.85), Rad::new(GEOGEBRA_LIMIT)).unwrap();
        approx_equal_meters(point.distance, Meters(45.31), Meters(GEOGEBRA_LIMIT)).unwrap();
    }

    #[test]
    fn test_intersection_quasi_inf() {
        let line1 = PolarLine {
            distance: Meters(10.0),
            angle: Rad::FULL_TURN + Rad::new(0.001),
        }; // Ligne horizontale
        let line2 = PolarLine {
            distance: Meters(2.4),
            angle: Rad::HALF_TURN,
        }; // Ligne verticale

        let intersection = line1.intersect(&line2);
        assert!(intersection.is_some());

        let point = intersection.unwrap();
        approx_equal_rad(point.angle, Rad::new(1.57), Rad::new(GEOGEBRA_LIMIT)).unwrap();
        approx_equal_meters(point.distance, Meters(12_400.0), Meters(GEOGEBRA_LIMIT)).unwrap();
    }

    #[test]
    fn test_intersection_up() {
        let line1 = PolarLine {
            distance: Meters(10.0),
            angle: Rad::new(2.0),
        }; // Ligne horizontale
        let line2 = PolarLine {
            distance: Meters(2.4),
            angle: Rad::HALF_TURN,
        }; // Ligne verticale

        let intersection = line1.intersect(&line2);
        assert!(intersection.is_some());

        let point = intersection.unwrap();
        approx_equal_rad(point.angle, Rad::new(1.81), Rad::new(GEOGEBRA_LIMIT)).unwrap();
        approx_equal_meters(point.distance, Meters(10.19), Meters(GEOGEBRA_LIMIT)).unwrap();
    }

    #[test]
    fn bench_hough_accumulator() {
        let nb_essais: u32 = 50;
        let mut moyenne = Duration::ZERO;
        for test in TESTS_DETECTION {
            let data = load_log(test);
            for _ in 0..nb_essais {
                let before = Instant::now();
                let _ = build_hough_accumulator(&data);
                moyenne += before.elapsed();
            }
        }
        let nb_tests = nb_essais * (u32::try_from(TESTS_DETECTION.len()).unwrap());
        eprintln!(
            "Bench Hough Accumulator (on {} values): {:#?}",
            nb_tests,
            moyenne / nb_tests
        )
    }

    #[test]
    fn test_1() {
        use crate::basic_viewport::ViewportLine;
        // TODO distance + cas : TEST_BAS_GAUCHE_ORIENTE_GAUCHE
        // TODO améliorer l'algo en prenant en compte la proximité des points entre eux. TEST_HAUT_DROITE_ORIENTE_DROITE
        let data = load_log(TEST_HAUT_DROITE_ORIENTE_DROITE);
        // println!("{:#?}", data);
        let ha = build_hough_accumulator(&data);
        println!("{:?}", ha.len());
        let walls = find_walls(ha.iter().copied().collect()).unwrap();

        let walls = try_to_complete_walls_from_uncomplete_data(walls).unwrap();

        let mut walls_vec = vec![walls.first_wall];

        if let Some(parallel_wall) = walls.parallele_wall {
            walls_vec.push(parallel_wall);
        }
        if let Some(perpendicular_wall_1) = walls.perpendicular_wall_1 {
            walls_vec.push(perpendicular_wall_1);
        }
        if let Some(perpendicular_wall_2) = walls.perpendicular_wall_2 {
            walls_vec.push(perpendicular_wall_2);
        }

        show_viewport(
            *data,
            ha.iter()
                .map(|e| ViewportLine {
                    line: e.line,
                    stroke: egui::Stroke::new(5.0, egui::Color32::DARK_RED),
                }) // egui::Stroke::new(5.0, egui::Color32::BLUE),
                .chain(walls_vec.into_iter().map(|e| match e {
                    WallLine::GuessedLine(GuessedLine { line }) => ViewportLine {
                        line: line,
                        stroke: egui::Stroke::new(5.0, egui::Color32::DARK_BLUE),
                    },
                    WallLine::HoughLine(HoughLine { line, weight: _ }) => ViewportLine {
                        line: line,
                        stroke: egui::Stroke::new(5.0, egui::Color32::BLUE),
                    },
                }))
                .collect(),
        )
        .unwrap();

        println!("{:#?}", walls);
        panic!("show");
        // show_viewport(
        //     *data,
        //     ha.into_iter().map(|e| e.line).collect(),
        //     walls.into_iter().map(|e| e.line).collect(),
        // )
        // .unwrap();
    }
}
