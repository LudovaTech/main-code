use core::{f64, time};

use rerun::external::arrow::datatypes::Field;

use crate::parse::{LidarPoint, PolarPoint};
use crate::prelude::*;

// TODO : virer cette histoire de conventions !

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

    /// A n'utiliser que si vous êtes sûr de ce que vous faites,
    /// *Vous transformez une droite en point !*
    fn _to_polar_point(&self) -> PolarPoint {
        PolarPoint {
            distance: self.distance,
            angle: self.angle,
        }
    }

    /// Calcule le point d'intersection de deux droites
    /// Les droites ne doivent pas être parallèles pour que le résultat aie du sens
    /// Redémontrer avec r = r0 / cos(theta - theta0)
    fn intersect(&self, other: &Self) -> Option<PolarPoint> {
        // angles between 0-2pi
        let self_angle_wrap = Rad::new((self.angle + Rad::FULL_TURN).val() % Rad::FULL_TURN.val());
        let other_angle_wrap =
            Rad::new((other.angle + Rad::FULL_TURN).val() % Rad::FULL_TURN.val());
        if self.is_parallel_with(other) {
            // Les droites sont alignés
            return None;
        }
        let theta = Rad::atan2(
            other.distance.0 * self_angle_wrap.cos() - self.distance.0 * other_angle_wrap.cos(),
            self.distance.0 * other_angle_wrap.sin() - other.distance.0 * self_angle_wrap.sin(),
        );

        let theta = Rad::new((theta + Rad::FULL_TURN).val() % Rad::FULL_TURN.val());

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
const HOUGH_TRANSFORM_MIN_POINT_PER_LINE: u16 = 30;

/// distance maximale en dessous de laquelle les points détectés du lidar sont conservés
const LIDAR_DISTANCE_MAX: Meters = Meters(3.0);

/// angle auquel on crée une nouvelle ligne dans la transformation de Hough
const ANGLE_RESOLUTION: f64 = 1.0 / 180.0 * f64::consts::PI;

/// nombre d'angles différents dans la matrice de la transformation de Hough
/// On ne va que de 0 à 180° car les distances peuvent être négatives
const ANGLE_TAILLE: usize = (f64::consts::PI / ANGLE_RESOLUTION) as usize + 1;

/// distance à laquelle on crée une nouvelle ligne dans la transformation de Hough
const DISTANCE_RESOLUTION: Meters = Meters::cm(1.0);

/// nombre de distances différentes dans la matrice de la transformation de Hough
/// On compte 2x car il y a les distances positives et négatives
const DISTANCE_TAILLE: usize = (LIDAR_DISTANCE_MAX.0 * 2.0 / DISTANCE_RESOLUTION.0) as usize + 1;

#[inline]
fn polar_point_to_case_angle_only(angle: Rad) -> usize {
    (angle / Rad::new(ANGLE_RESOLUTION)) as usize
}

/// Fonctionne avec les deux convention, distance négative et angle > 180
fn polar_point_to_case(point: PolarPoint) -> (usize, usize) {
    assert!(
        !(point.distance < Meters(0.0) && point.angle > Rad::HALF_TURN),
        "this should never happen, negative distance or angle over 180 are two different way to express the same thing and should not be used together : {:#?}",
        point
    );
    let offset = if point.distance.0.is_sign_positive() && point.angle < Rad::HALF_TURN {
        DISTANCE_TAILLE / 2
    } else {
        0
    };
    let distance_case: usize = offset + (point.distance.abs() / DISTANCE_RESOLUTION) as usize;
    let angle_case: usize = polar_point_to_case_angle_only(point.angle);
    (distance_case, angle_case)
}

/// renvoit une ligne qui respecte la convention (distance positive, angle > 180)
fn case_to_polar_line(distance_case: usize, angle_case: usize) -> PolarLine {
    let mut angle_offset = Rad::ZERO;
    let distance_corrected = if distance_case >= DISTANCE_TAILLE / 2 {
        distance_case - DISTANCE_TAILLE / 2
    } else {
        angle_offset = Rad::HALF_TURN;
        distance_case
    };

    PolarLine {
        distance: DISTANCE_RESOLUTION * distance_corrected as f64,
        angle: angle_offset + (Rad::new(ANGLE_RESOLUTION) * angle_case as f64),
    }
}

/// Attention compte pour 2, un en positif, un en négatif
const MARGE_DISTANCE: i32 = 10;
const MARGE_ANGLE: i32 = 10;

#[inline]
fn look_around_for_lines_angle_only(
    accumulator: &Box<[[u16; ANGLE_TAILLE]; DISTANCE_TAILLE]>,
    distance_case_applied: usize,
    wanted_angle_case: usize,
) -> Vec<HoughLine> {
    let mut found_lines = Vec::new();
    for alea_angle in -MARGE_ANGLE..=MARGE_ANGLE {
        if (wanted_angle_case as i32) + alea_angle < 0
            || (wanted_angle_case as i32) + alea_angle >= ANGLE_TAILLE as i32
        {
            // out of bounds
            continue;
        }

        let angle_case_applied = (wanted_angle_case as i32 + alea_angle) as usize;
        let found_weight = accumulator[distance_case_applied][angle_case_applied];
        if found_weight > HOUGH_TRANSFORM_MIN_POINT_PER_LINE {
            let found_line = HoughLine {
                line: case_to_polar_line(distance_case_applied, angle_case_applied),
                weight: found_weight,
            };
            println!("found : {:?}", found_line);
            found_lines.push(found_line);
        } else {
            //println!("too small {:?} {} {}", distance_case_applied, angle_case_applied, weight);
        }
    }
    found_lines
}

#[inline]
fn look_around_for_lines(
    accumulator: &Box<[[u16; ANGLE_TAILLE]; DISTANCE_TAILLE]>,
    wanted_distance_case: usize,
    wanted_angle_case: usize,
) -> Vec<HoughLine> {
    let mut found_lines = Vec::new();
    for alea_distance in -MARGE_DISTANCE..=MARGE_DISTANCE {
        if (wanted_distance_case as i32) + alea_distance < 0
            || (wanted_distance_case as i32) + alea_distance >= DISTANCE_TAILLE as i32
        {
            // out of bounds
            continue;
        }
        let distance_case_applied = (wanted_distance_case as i32 + alea_distance) as usize;
        found_lines.append(&mut look_around_for_lines_angle_only(
            accumulator,
            distance_case_applied,
            wanted_angle_case,
        ));
    }
    found_lines
}

/// guide : <https://www.keymolen.com/2013/05/hough-transformation-c-implementation.html>
fn build_hough_accumulator(
    points: &Vec<LidarPoint>,
) -> Box<[[u16; ANGLE_TAILLE]; DISTANCE_TAILLE]> {
    // Création de la matrice de la transformation de Hough
    let mut accumulator = [[0u16; ANGLE_TAILLE]; DISTANCE_TAILLE];
    for point in points.iter() {
        if point.point.distance > LIDAR_DISTANCE_MAX {
            continue;
        }

        // Formule magique. `lpd * cos(lpa - a)`
        // Se prouve avec : lpd = lidarPoint.distance, lpa = lidarPoint.angle
        // `x = lpd * cos(lpa)`, `y = lpd * sin(lpa)`, `r_a = x * cos(a) + y * sin(a)`, et les formules d'addition du cosinus
        let mut calculated_angle = Rad::ZERO;
        while calculated_angle <= Rad::HALF_TURN {
            // println!("{:?}", calculated_angle);

            // cos(x) = cos(abs(x))
            let (distance_case, angle_case) = polar_point_to_case(PolarPoint {
                angle: calculated_angle,
                distance: point.point.distance * (point.point.angle - calculated_angle).mag().cos(),
            });
            // println!("{} {} {}", distance_case, offset, distance_factor);
            accumulator[distance_case][angle_case] =
                accumulator[distance_case][angle_case].saturating_add(1);
            // TODO est-ce que check around améliore les résultats ?
            // check_around(&mut accumulator, distance_case, angle_case, 1, 80);
            // check_around(&mut accumulator, distance_case, angle_case, 2, 65);
            // check_around(&mut accumulator, distance_case, angle_case, 3, 40);
            // check_around(&mut accumulator, distance_case, angle_case, 4, 25);
            // check_around(&mut accumulator, distance_case, angle_case, 5, 10);

            calculated_angle += Rad::new(ANGLE_RESOLUTION);
        }
    }

    Box::new(accumulator)
}

fn conv_convention_big_angle_to_negative_distance(line: PolarLine) -> PolarLine {
    PolarLine {
        distance: if line.angle > Rad::HALF_TURN {
            -line.distance
        } else {
            line.distance
        },
        angle: if line.angle > Rad::HALF_TURN {
            line.angle - Rad::HALF_TURN
        } else {
            line.angle
        },
    }
}

fn search_all_parallel_lines(
    accumulator: &Box<[[u16; ANGLE_TAILLE]; DISTANCE_TAILLE]>,
    distance_between_lines: Meters,
) -> Vec<(HoughLine, HoughLine)> {
    let mut found_lines = Vec::new();

    // TODO diviser le nombre par deux ? (for p_distance in (DISTANCE_TAILLE/2)..DISTANCE_TAILLE)
    for p_angle in 0..ANGLE_TAILLE {
        for p_distance in 0..DISTANCE_TAILLE {
            let weight = accumulator[p_distance][p_angle];
            if weight < HOUGH_TRANSFORM_MIN_POINT_PER_LINE {
                continue;
            }
            // trace!("line at {} {}", p_angle, p_distance);
            let line = case_to_polar_line(p_distance, p_angle);

            // 0 <= angle <= 180 donc pas besoin de bound check
            // penser à prendre la distance en négatif quand cal_angle > 180
            // let wanted_angle_for_perpendicular = angle.to_deg().rad() + Rad::QUARTER_TURN;

            // convention distance négative angle < 180
            let sign_line = conv_convention_big_angle_to_negative_distance(line);

            // rappel angle est entre 0 et 180
            let wanted_angle_for_parallel = sign_line.angle;

            // on teste uniquement "en dessous" car le robot (l'origine) doit toujours être à l'intérieur du terrain
            let wanted_distance_for_parallel = sign_line.distance - distance_between_lines;
            // TODO assert!(distance.to_meters().0 * wanted_distance_for_parallel.0 > 0.0)

            let (wanted_distance_case, wanted_angle_case) = polar_point_to_case(PolarPoint {
                distance: wanted_distance_for_parallel,
                angle: wanted_angle_for_parallel,
            });

            found_lines.extend(
                look_around_for_lines(&accumulator, wanted_distance_case, wanted_angle_case)
                    .iter()
                    .map(|e| (HoughLine { line, weight }, *e)),
            );
        }
    }
    found_lines
}

fn search_perpendicular_lines_of(
    accumulator: &Box<[[u16; ANGLE_TAILLE]; DISTANCE_TAILLE]>,
    line: PolarLine,
) -> Vec<HoughLine> {
    // let (distance_case, angle_case) = polar_point_to_case(line._to_polar_point());
    let mut found_lines = Vec::new();
    let line = conv_convention_big_angle_to_negative_distance(line);
    let wanted_angle = line.angle + Rad::QUARTER_TURN; // ICI si wangle > 90
    let PolarLine {
        angle: wanted_angle_corrected,
        distance: _,
    } = conv_convention_big_angle_to_negative_distance(PolarLine {
        distance: line.distance,
        angle: wanted_angle,
    });
    let wanted_angle_case = polar_point_to_case_angle_only(wanted_angle_corrected);

    for p_distance in 0..DISTANCE_TAILLE {
        found_lines.append(&mut look_around_for_lines_angle_only(
            accumulator,
            p_distance,
            wanted_angle_case,
        ));
    }

    found_lines
}

// TODO should not be here
const FIELD_LENGTH: Meters = Meters::cm(243.0);
// TODO should not be here
const FIELD_WIDTH: Meters = Meters::cm(182.0);

#[derive(Debug, Clone, Copy)]
pub enum WallLine {
    FoundAsParallelLine(HoughLine),
    FoundAsPerpendicular(HoughLine),
    GuessedLine(PolarLine),
}

impl WallLine {
    #[inline]
    pub fn line(&self) -> PolarLine {
        match self {
            Self::FoundAsParallelLine(hought) => hought.line,
            Self::FoundAsPerpendicular(hought) => hought.line,
            Self::GuessedLine(guessed) => *guessed,
        }
    }
}

fn _anc_locate_field_with_4_walls(
    candidate_line_width: &Vec<(HoughLine, HoughLine)>,
    candidate_line_length: &Vec<(HoughLine, HoughLine)>,
) -> Option<FieldWalls> {
    let mut score = 0.;
    let mut field: Option<FieldWalls> = None;
    for parallel_width in candidate_line_width.iter() {
        for parallel_length in candidate_line_length.iter() {
            if parallel_width
                .0
                .line
                .is_approx_perpendicular_with(&parallel_length.0.line)
            {
                // la paire fonctionne
                let this_score = f64::from(parallel_width.0.weight)
                    * parallel_width.0.line.distance.0
                    + f64::from(parallel_width.1.weight) * parallel_width.1.line.distance.0
                    + f64::from(parallel_length.0.weight) * parallel_length.0.line.distance.0
                    + f64::from(parallel_length.1.weight) * parallel_length.1.line.distance.0;
                if this_score > score {
                    score = this_score;
                    field = Some(FieldWalls {
                        width1: WallLine::FoundAsParallelLine(parallel_width.0),
                        width2: WallLine::FoundAsParallelLine(parallel_width.1),
                        length1: WallLine::FoundAsParallelLine(parallel_length.0),
                        length2: WallLine::FoundAsParallelLine(parallel_length.1),
                    })
                }
            }
        }
    }
    field
}

fn locate_field_with_4_walls(
    candidate_line_width: &Vec<(HoughLine, HoughLine)>,
    candidate_line_length: &Vec<(HoughLine, HoughLine)>,
) -> Option<FieldWalls> {
    let mut score = 0.;
    let mut field: Option<FieldWalls> = None;
    for parallel_width in candidate_line_width.iter() {
        for parallel_length in candidate_line_length.iter() {
            if parallel_width
                .0
                .line
                .is_approx_perpendicular_with(&parallel_length.0.line)
            {
                // la paire fonctionne
                let this_score = f64::from(parallel_width.0.weight)
                    * parallel_width.0.line.distance.0
                    + f64::from(parallel_width.1.weight) * parallel_width.1.line.distance.0
                    + f64::from(parallel_length.0.weight) * parallel_length.0.line.distance.0
                    + f64::from(parallel_length.1.weight) * parallel_length.1.line.distance.0;
                if this_score > score {
                    score = this_score;
                    field = Some(FieldWalls {
                        width1: WallLine::FoundAsParallelLine(parallel_width.0),
                        width2: WallLine::FoundAsParallelLine(parallel_width.1),
                        length1: WallLine::FoundAsParallelLine(parallel_length.0),
                        length2: WallLine::FoundAsParallelLine(parallel_length.1),
                    })
                }
            }
        }
    }

    let Some(walls) = field else {
        return None;
    };
    
    let time1 = std::time::Instant::now();
    let moy_walls = perform_moy(walls, candidate_line_width, candidate_line_length);
    info!("perf perform_moy for 4 walls, took : {:?}", time1.elapsed());
    Some(moy_walls)
}

fn perform_moy(
    walls: FieldWalls,
    candidate_line_width: &Vec<(HoughLine, HoughLine)>,
    candidate_line_length: &Vec<(HoughLine, HoughLine)>,
) -> FieldWalls {
    let mut moy_parallel_width1 = (
        walls.width1.line().distance,
        walls.width1.line().angle,
        1.,
    );
    let mut moy_parallel_width2 = (
        walls.width2.line().distance,
        walls.width2.line().angle,
        1.,
    );
    let mut moy_parallel_length1 = (
        walls.length1.line().distance,
        walls.length1.line().angle,
        1.,
    );
    let mut moy_parallel_length2 = (
        walls.length2.line().distance,
        walls.length2.line().angle,
        1.,
    );

    for parallel_width in candidate_line_width.iter().map(|e| [e.0, e.1]).flatten() {
        if _is_near(&parallel_width.line, &walls.width1.line()) {
            moy_parallel_width1 = (
                moy_parallel_width1.0
                    + parallel_width.line.distance * f64::from(parallel_width.weight),
                moy_parallel_width1.1
                    + parallel_width.line.angle * f64::from(parallel_width.weight),
                moy_parallel_width1.2 + f64::from(parallel_width.weight),
            );
        }
        if _is_near(&parallel_width.line, &walls.width2.line()) {
            moy_parallel_width2 = (
                moy_parallel_width2.0
                    + parallel_width.line.distance * f64::from(parallel_width.weight),
                moy_parallel_width2.1
                    + parallel_width.line.angle * f64::from(parallel_width.weight),
                moy_parallel_width2.2 + f64::from(parallel_width.weight),
            );
        }
    }

    for parallel_length in candidate_line_length.iter().map(|e| [e.0, e.1]).flatten() {
        if _is_near(&parallel_length.line, &walls.length1.line()) {
            moy_parallel_length1 = (
                moy_parallel_length1.0
                    + parallel_length.line.distance * f64::from(parallel_length.weight),
                moy_parallel_length1.1
                    + parallel_length.line.angle * f64::from(parallel_length.weight),
                moy_parallel_length1.2 + f64::from(parallel_length.weight),
            );
        }
        if _is_near(&parallel_length.line, &walls.length2.line()) {
            moy_parallel_length2 = (
                moy_parallel_length2.0
                    + parallel_length.line.distance * f64::from(parallel_length.weight),
                moy_parallel_length2.1
                    + parallel_length.line.angle * f64::from(parallel_length.weight),
                moy_parallel_length2.2 + f64::from(parallel_length.weight),
            );
        }
    }

    FieldWalls {
        width1: WallLine::FoundAsParallelLine(HoughLine {
            line: PolarLine {
                distance: moy_parallel_width1.0 / moy_parallel_width1.2,
                angle: moy_parallel_width1.1 / moy_parallel_width1.2,
            },
            weight: 500, // TODO
        }),
        width2: WallLine::FoundAsParallelLine(HoughLine {
            line: PolarLine {
                distance: moy_parallel_width2.0 / moy_parallel_width2.2,
                angle: moy_parallel_width2.1 / moy_parallel_width2.2,
            },
            weight: 500,
        }),
        length1: WallLine::FoundAsParallelLine(HoughLine {
            line: PolarLine {
                distance: moy_parallel_length1.0 / moy_parallel_length1.2,
                angle: moy_parallel_length1.1 / moy_parallel_length1.2,
            },
            weight: 500,
        }),
        length2: WallLine::FoundAsParallelLine(HoughLine {
            line: PolarLine {
                distance: moy_parallel_length2.0 / moy_parallel_length2.2,
                angle: moy_parallel_length2.1 / moy_parallel_length2.2,
            },
            weight: 500,
        }),
    }
}

#[inline]
fn _is_near(line1: &PolarLine, line2: &PolarLine) -> bool {
    line1.smallest_angle_between(line2) < Deg::new(20.).rad() // TODO params
        && line1.distance_center_with(line2) < Meters::cm(20.)
}

#[derive(Debug)]
enum _LineSize {
    Width,
    Length,
}

fn fallback_on_3_walls(
    accumulator: &Box<[[u16; ANGLE_TAILLE]; DISTANCE_TAILLE]>,
    candidate_line_width: Vec<(HoughLine, HoughLine)>,
    candidate_line_length: Vec<(HoughLine, HoughLine)>,
) -> Option<FieldWalls> {
    // search if a perpendicular line exists for all the parallel lines detected
    let mut all_detected_lines: Vec<(_LineSize, (HoughLine, HoughLine))> = candidate_line_width
        .iter()
        .map(|e| (_LineSize::Width, *e))
        .chain(
            candidate_line_length
                .iter()
                .map(|e| (_LineSize::Length, *e)),
        )
        .collect();
    all_detected_lines.sort_by_key(|(_, (l1, l2))| l1.weight + l2.weight);

    if all_detected_lines.is_empty() {
        return None;
    }
    let perpendiculars =
        search_perpendicular_lines_of(&accumulator, all_detected_lines.first().unwrap().1.0.line);
    if !perpendiculars.is_empty() {
        let perpendicular = perpendiculars.first().unwrap();
        let prop = match all_detected_lines.first().unwrap() {
            (_LineSize::Width, pair) => FieldWalls {
                length1: WallLine::FoundAsParallelLine(pair.0),
                length2: WallLine::FoundAsParallelLine(pair.1),
                width1: WallLine::FoundAsPerpendicular(*perpendicular),
                width2: WallLine::GuessedLine(guess_last_wall(perpendicular.line, FIELD_WIDTH)),
            },
            (_LineSize::Length, pair) => FieldWalls {
                width1: WallLine::FoundAsParallelLine(pair.0),
                width2: WallLine::FoundAsParallelLine(pair.1),
                length1: WallLine::FoundAsPerpendicular(*perpendicular),
                length2: WallLine::GuessedLine(guess_last_wall(perpendicular.line, FIELD_LENGTH)),
            },
        };

        let time1 = std::time::Instant::now();
        let moy = perform_moy(prop, &candidate_line_width, &candidate_line_length);
        debug!("perf moy in 3 walls, took {:?}", time1.elapsed());

        Some(moy)
    } else {
        None
    }
}

fn guess_last_wall(parallel_known_wall: PolarLine, distance_between_lines: Meters) -> PolarLine {
    let patched_line = conv_convention_big_angle_to_negative_distance(parallel_known_wall);
    PolarLine {
        distance: if patched_line.distance >= Meters(0.0) {
            patched_line.distance - distance_between_lines
        } else {
            patched_line.distance + distance_between_lines
        },
        angle: patched_line.angle,
    }
}

#[derive(Debug)]
struct FieldWalls {
    width1: WallLine,
    width2: WallLine,
    length1: WallLine,
    length2: WallLine,
}

#[inline]
fn _moyenne_point_carthesian(point1: (f64, f64), point2: (f64, f64)) -> (f64, f64) {
    ((point1.0 + point2.0) / 2.0, (point1.1 + point2.1) / 2.0)
}

fn calculate_center_of_field_in_carthesian(
    rec: rerun::RecordingStream,
    walls: &FieldWalls,
) -> (f64, f64) {
    let inter1 = walls
        .width1
        .line()
        .intersect(&walls.length1.line())
        .unwrap();
    let inter2 = walls
        .width1
        .line()
        .intersect(&walls.length2.line())
        .unwrap()
        .to_carthesian_point();
    let inter3 = walls
        .width2
        .line()
        .intersect(&walls.length1.line())
        .unwrap()
        .to_carthesian_point();
    let inter4 = walls
        .width2
        .line()
        .intersect(&walls.length2.line())
        .unwrap()
        .to_carthesian_point();

    rec.log(
        "lidar/inter1",
        &rerun::Points2D::new([(
            inter1.to_carthesian_point().0 as f32,
            inter1.to_carthesian_point().1 as f32,
        )])
        .with_colors([colors::ORANGE])
        .with_radii([0.05])
        .with_labels(["inter1"])
        .with_show_labels(rerun::components::ShowLabels(rerun::datatypes::Bool(false))),
    )
    .unwrap();

    rec.log(
        "lidar/inter2",
        &rerun::Points2D::new([(inter2.0 as f32, inter2.1 as f32)])
            .with_colors([colors::ORANGE])
            .with_radii([0.05])
            .with_labels(["inter2"])
            .with_show_labels(rerun::components::ShowLabels(rerun::datatypes::Bool(false))),
    )
    .unwrap();

    rec.log(
        "lidar/inter3",
        &rerun::Points2D::new([(inter3.0 as f32, inter3.1 as f32)])
            .with_colors([colors::ORANGE])
            .with_radii([0.05])
            .with_labels(["inter3"])
            .with_show_labels(rerun::components::ShowLabels(rerun::datatypes::Bool(false))),
    )
    .unwrap();

    rec.log(
        "lidar/inter4",
        &rerun::Points2D::new([(inter4.0 as f32, inter4.1 as f32)])
            .with_colors([colors::ORANGE])
            .with_radii([0.05])
            .with_labels(["inter4"])
            .with_show_labels(rerun::components::ShowLabels(rerun::datatypes::Bool(false))),
    )
    .unwrap();

    let m1 = _moyenne_point_carthesian(inter1.to_carthesian_point(), inter2);
    let m2 = _moyenne_point_carthesian(inter1.to_carthesian_point(), inter3);
    let m3 = _moyenne_point_carthesian(inter3, inter4);
    let m4 = _moyenne_point_carthesian(inter2, inter4);

    ((m1.0 + m3.0) / 2.0, (m2.1 + m4.1) / 2.0)
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::complex_viewport::{log_lidar_lines, log_lidar_points};
    use crate::{analyze_tests_data::lidar_test_data::*, parse::PolarPoint};
    use std::time::{Duration, Instant};

    fn load_log(from: &str) -> Box<Vec<LidarPoint>> {
        let mut vector = Box::new(Vec::with_capacity(from.len() / 10));
        for point in from.split(';') {
            let mut iter_num = point.trim_matches(|c| c == '(' || c == ')').split(',');
            let angle = iter_num.next().unwrap().parse::<u16>().unwrap();
            let distance = iter_num.next().unwrap().parse::<u16>().unwrap();
            assert!(iter_num.next().is_none());
            vector.push(LidarPoint {
                point: PolarPoint {
                    distance: Meters::mm(f64::from(distance)),
                    angle: Deg::new(f64::from(angle) / 100.0).rad(),
                },
                intensity: Intensity::NULL,
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
        use crate::complex_viewport::ViewportLine;
        let rec = rerun::RecordingStreamBuilder::new("test_1")
            .spawn()
            .unwrap();
        crate::log_manager::set_up_logging(rec.clone()).unwrap();
        // TODO distance + cas : TEST_BAS_GAUCHE_ORIENTE_GAUCHE
        // TODO améliorer l'algo en prenant en compte la proximité des points entre eux. TEST_HAUT_DROITE_ORIENTE_DROITE
        rec.log(
            "lidar/real_robot_pos",
            &rerun::Points2D::new([(0., 0.)])
                .with_colors([colors::WHITE])
                .with_radii([0.01])
                .with_labels(["Position réelle du robot"])
                .with_show_labels(rerun::components::ShowLabels(rerun::datatypes::Bool(false))),
        )
        .unwrap();

        for log_data in TESTS_DETECTION {
            // notes : fonctionne en 2*2 : TEST_BAS_GAUCHE_ORIENTE_GAUCHE
            let data = load_log(&log_data);
            let time1 = std::time::Instant::now();
            let accumulator = build_hough_accumulator(&data);
            let candidate_line_width = search_all_parallel_lines(&accumulator, FIELD_LENGTH);
            let candidate_line_length = search_all_parallel_lines(&accumulator, FIELD_WIDTH);

            let field = locate_field_with_4_walls(&candidate_line_width, &candidate_line_length)
                .or_else(|| {
                    info!("Détection 4 murs échouée, tente avec 3 murs");
                    fallback_on_3_walls(&accumulator, candidate_line_width, candidate_line_length)
                });

            let mut vl = Vec::with_capacity(50);

            log_lidar_points(&rec, &data).unwrap();
            if let Some(field_found) = field {
                vl.push(ViewportLine {
                    line: field_found.length1.line(),
                    color: colors::BLUE,
                });
                vl.push(ViewportLine {
                    line: field_found.length2.line(),
                    color: colors::BLUE,
                });
                vl.push(ViewportLine {
                    line: field_found.width1.line(),
                    color: colors::BLUE,
                });
                vl.push(ViewportLine {
                    line: field_found.width2.line(),
                    color: colors::BLUE,
                });

                log_lidar_lines(&rec, vl).unwrap();

                let center = calculate_center_of_field_in_carthesian(rec.clone(), &field_found);
                rec.log(
                    "lidar/cal_center_field",
                    &rerun::Points2D::new([(center.0 as f32, center.1 as f32)])
                        .with_colors([colors::MAGENTA])
                        .with_radii([0.01])
                        .with_labels(["centre du terrain calculé"])
                        .with_show_labels(rerun::components::ShowLabels(rerun::datatypes::Bool(
                            false,
                        ))),
                )
                .unwrap();
            } else {
                error!("Echoue à détecter les murs")
            }

            debug!("perf all lidar_analyzer : {:?}", time1.elapsed());
        }

        // for ((first, second), color) in pl.iter().zip(COLORS.iter().cycle()) {
        //     vl.push(ViewportLine {
        //         line: first.line,
        //         stroke: egui::Stroke::new(5.0, *color),
        //     });
        //     vl.push(ViewportLine {
        //         line: second.line,
        //         stroke: egui::Stroke::new(5.0, *color),
        //     });
        // }

        // let perpendicular_lines = search_perpendicular_lines_of(
        //     &accumulator,
        //     candidate_line_width.first().unwrap().0.line,
        // );

        // println!("{}", perpendicular_lines.len());

        // for (perpendicular_line, color) in perpendicular_lines.iter().zip(COLORS.iter().cycle()) {
        //     vl.push(ViewportLine {
        //         line: perpendicular_line.line,
        //         stroke: egui::Stroke::new(5.0, *color),
        //     });
        // }
    }
}

pub fn test_call() {
    tracing::info!("is here")
}
