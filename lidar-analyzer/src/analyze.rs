use std::cmp::Reverse;
use std::error::Error;
use std::f64::consts::PI;
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

#[derive(Debug, Clone, Copy)]
pub struct PolarLine {
    pub distance: Meters,
    pub angle: Rad,
}

impl PolarLine {
    /// Renvoit l'angle aigu entre deux lignes polaires
    /// (faire un schéma pour se convaincre rapidement de la formule)
    fn smallest_angle_between(&self, other: &Self) -> Rad {
        let alpha = (self.angle - other.angle).mag();
        alpha.min(Rad::HALF_TURN - alpha)
    }

    /// Avec une tolérance
    fn is_parallel_with(&self, other: &Self) -> bool {
        self.smallest_angle_between(other) <= Rad::new(IS_PARALLEL_TOLERANCE)
    }

    /// Avec une tolérance
    fn is_perpendicular_with(&self, other: &Self) -> bool {
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
}

#[derive(Debug, Clone, Copy)]
pub struct HoughLine {
    pub line: PolarLine,
    pub weight: u16,
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

/// guide : https://www.keymolen.com/2013/05/hough-transformation-c-implementation.html
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
struct WallsConstruction {
    first_wall: HoughLine,
    parallele_wall: Option<HoughLine>,
    perpendicular_wall_1: Option<HoughLine>,
    perpendicular_wall_2: Option<HoughLine>,
}

impl WallsConstruction {
    fn new(first_wall: HoughLine) -> WallsConstruction {
        WallsConstruction {
            first_wall: first_wall,
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
    first_wall: HoughLine,
    parallele_wall: HoughLine,
    perpendicular_wall_1: HoughLine,
    perpendicular_wall_2: HoughLine,
}

struct WallsIntoIterator {
    walls: Walls,
    step: u8,
}

impl IntoIterator for Walls {
    type IntoIter = WallsIntoIterator;
    type Item = HoughLine;

    fn into_iter(self) -> Self::IntoIter {
        WallsIntoIterator {
            walls: self,
            step: 0,
        }
    }
}

impl Iterator for WallsIntoIterator {
    type Item = HoughLine;

    fn next(&mut self) -> Option<Self::Item> {
        let result = match self.step {
            0 => self.walls.first_wall,
            1 => self.walls.perpendicular_wall_1,
            2 => self.walls.parallele_wall,
            3 => self.walls.perpendicular_wall_2,
            _ => return None,
        };
        self.step += 1;
        Some(result)
    }
}

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
    let mut detected_line_iter = detected_lines.iter();
    detected_line_iter.next(); // skip first
    for hough_line in detected_line_iter {
        let line = hough_line.line;
        if walls.perpendicular_wall_1.is_none() {
            // On essaie de déterminer si cette ligne peut-être la perpendiculaire 1
            if line.is_perpendicular_with(&walls.first_wall.line) {
                walls.perpendicular_wall_1 = Some(*hough_line);
                continue;
            }
            // On essaie de déterminer si cette ligne peut-être la parallèle
            // TODO on ne checke pas si on a deux distances FIELD_LENGTH ou deux distances FIELD_WIDTH
            if line.is_parallel_with(&walls.first_wall.line)
                && (line
                    .distance_center_with(&walls.first_wall.line)
                    .in_the_aera_of(FIELD_LENGTH)
                    || line
                        .distance_center_with(&walls.first_wall.line)
                        .in_the_aera_of(FIELD_WIDTH))
            {
                let mut ok = true;
                if let Some(HoughLine {
                    line: perpendicular1,
                    weight: _,
                }) = walls.perpendicular_wall_1
                {
                    ok = ok && line.is_perpendicular_with(&perpendicular1)
                }
                if let Some(HoughLine {
                    line: perpendicular2,
                    weight: _,
                }) = walls.perpendicular_wall_2
                {
                    ok = ok && line.is_perpendicular_with(&perpendicular2)
                }
                ok = true;
                if ok {
                    walls.perpendicular_wall_1 = Some(*hough_line)
                }
            }
            // On essaie de déterminer si cette ligne peut-être la perpendiculaire 2
            // TODO on ne checke pas si on a deux distances FIELD_LENGTH ou deux distances FIELD_WIDTH
            if line.is_perpendicular_with(&walls.first_wall.line) {
                let mut ok = true;
                if let Some(HoughLine {
                    line: perpendicular1,
                    weight: _,
                }) = walls.perpendicular_wall_1
                {
                    ok = ok
                        && line.is_parallel_with(&perpendicular1)
                        && (line
                            .distance_center_with(&perpendicular1)
                            .in_the_aera_of(FIELD_LENGTH)
                            || line
                                .distance_center_with(&perpendicular1)
                                .in_the_aera_of(FIELD_WIDTH))
                }
                if let Some(HoughLine {
                    line: parallel,
                    weight: _,
                }) = walls.parallele_wall
                {
                    ok = ok && line.is_perpendicular_with(&parallel)
                }
                ok = true;
                if ok {
                    walls.perpendicular_wall_2 = Some(*hough_line)
                }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        basic_viewport::show_viewport,
        parse::{LidarAngle, LidarDistance},
    };

    const TEST1: &str = "(5564,1480);(5635,1489);(5706,1497);(5777,1506);(5848,1517);(5919,1526);(5990,1533);(6061,1540);(6132,1551);(6203,1565);(6274,1580);(6345,1596);(31547,464);(31620,466);(31693,467);(31766,469);(31839,471);(31912,473);(31985,475);(32058,478);(32131,480);(32204,482);(32277,485);(32350,487);(32424,490);(32494,492);(32564,496);(32634,499);(32704,503);(32774,506);(32844,510);(32914,514);(32984,522);(33054,527);(33124,533);(33194,539);(33275,545);(33346,551);(33417,556);(33488,561);(33559,566);(33630,571);(33701,573);(33772,576);(33843,580);(33914,584);(33985,591);(34056,596);(34131,601);(34207,608);(34283,617);(34359,625);(34435,633);(34511,640);(34587,647);(34663,655);(34739,663);(34815,671);(34891,678);(34967,686);(35047,695);(35118,704);(35189,712);(35260,719);(35331,725);(776,1090);(847,1110);(918,1140);(989,1169);(1060,1221);(1131,1255);(1202,1289);(1273,1325);(1344,1326);(1415,1328);(1486,1326);(1565,1322);(1641,1319);(1717,1316);(1793,1313);(1869,1311);(1945,1309);(2021,1307);(2097,1305);(2173,1304);(2249,1302);(2325,1301);(2401,1300);(2477,1300);(2544,1299);(2611,1299);(2678,1299);(2745,1299);(2812,1300);(2879,1300);(2946,1301);(3013,1302);(3080,1303);(3147,1304);(3214,1306);(3279,1308);(3350,1310);(3421,1310);(3492,1308);(3563,1306);(3634,1308);(3705,1312);(3776,1316);(3847,1320);(3918,1324);(3989,1328);(4060,1332);(4138,1339);(4209,1344);(4280,1350);(4351,1355);(4422,1361);(4493,1367);(4564,1373);(4635,1380);(4706,1387);(4777,1394);(4848,1402);(4919,1409);(4990,1416);(5061,1424);(5132,1431);(5203,1439);(5274,1446);(5345,1454);(5416,1461);(5487,1468);(5558,1476);(5629,1484);(5700,1492);(5771,1503);(5850,1514);(5921,1525);(5992,1533);(6063,1539);(6134,1546);(6205,1558);(6276,1571);(6347,1590);(6418,1606);(6489,1620);(6560,1638);(6631,1655);(6705,1672);(6776,1689);(6847,1707);(6918,1728);(6989,1747);(7060,1764);(7131,1779);(7202,1797);(7273,1822);(7344,1848);(7415,1874);(7486,1902);(7568,1930);(7639,1957);(7710,1989);(7781,2021);(7852,2050);(7923,2085);(7994,2119);(8065,2161);(8136,2201);(8207,2242);(8278,2278);(8349,2317);(8424,2364);(8499,2415);(8574,2409);(8649,239);(8724,246);(8799,2334);(8874,2327);(8949,2318);(9024,2305);(9099,2291);(9174,2278);(9249,2267);(9333,2253);(9404,2240);(9475,2228);(9546,2217);(9617,2207);(9688,2195);(9759,2184);(9830,2172);(9901,2161);(9972,2152);(10043,2142);(10114,2134);(10192,2127);(10263,2120);(10334,2113);(10405,2106);(10476,2099);(10547,2093);(10618,2088);(10689,2082);(10760,2076);(10831,2070);(10902,2064);(10973,732);(11044,731);(11116,729);(11188,729);(11260,729);(11332,731);(11404,733);(11476,735);(11548,736);(11620,737);(11692,737);(11764,739);(11836,754);(11907,774);(11979,792);(12051,2030);(12123,2036);(12195,2038);(12267,2040);(12339,2042);(12411,2045);(12483,2048);(12555,2051);(12627,2054);(12699,2057);(12777,2059);(12848,2063);(12919,2067);(12990,2074);(13061,2075);(13132,2076);(13203,1984);(13274,1858);(13345,1791);(13416,1694);(13487,1643);(13558,1590);(13633,1530);(13709,1485);(13785,1437);(13861,1406);(13937,1363);(14013,1328);(14089,1302);(14165,1285);(14241,1239);(14317,1192);(14393,1158);(14469,1121);(14542,1089);(14613,1067);(14684,1043);(14755,1031);(14826,1021);(14897,998);(14968,966);(15039,949);(15110,930);(15181,910);(15252,891);(15323,874);(15404,857);(15475,842);(15546,834);(15617,827);(15688,819);(15759,807);(15830,793);(15901,780);(15972,768);(16043,756);(16114,745);(16185,735);(16260,726);(16332,717);(16404,709);(16476,702);(16548,694);(16620,686);(16692,677);(16764,670);(16836,662);(16908,655);(16980,647);(17052,640);(17133,633);(17204,623);(17275,610);(18830,521);(18902,519);(18974,518);(19046,520);(19118,521);(19190,521);(19262,519);(19334,518);(19406,516);(19478,515);(19550,515);(19622,514);(19694,513);(19764,512);(19834,511);(19904,510);(19974,509);(20044,509);(20114,508);(20184,507);(20254,507);(20324,506);(20394,506);(20464,506);(20549,506);(20619,506);(20689,506);(20759,506);(20829,506);(20899,506);(20969,506);(21039,506);(21109,507);(21179,507);(21249,508);(21319,508);(21401,509);(21472,510);(21543,511);(21614,512);(21685,514);(21756,515);(21827,517);(21898,518);(21969,519);(22040,521);(22111,523);(22182,525);(22260,527);(22326,529);(22392,531);(22458,533);(22524,535);(22590,537);(22656,539);(22722,541);(22788,543);(22854,545);(22920,547);(22986,550);(23065,553);(23136,556);(23207,559);(23278,563);(23349,567);(23420,571);(23491,575);(23562,579);(23633,583);(23704,587);(23775,591);(23846,598);(23925,603);(23994,609);(24063,614);(24132,620);(24201,626);(24270,633);(24339,639);(24408,646);(24477,652);(24546,659);(24615,666);(24684,673);(24762,680);(24834,688);(24906,695);(24978,700);(25050,701);(25122,701);(25194,696);(25266,688);(25338,678);(25410,668);(25482,658);(25554,649);(25626,641);(25696,633);(25766,625);(25836,618);(25906,610);(25976,603);(26046,596);(26116,589);(26186,583);(26256,577);(26326,572);(26396,567);(26477,562);(26548,556);(26619,551);(26690,549);(26761,545);(26832,541);(26903,537);(26974,533);(27045,529);(27116,525);(27187,521);(27258,517);(27333,514);(27404,510);(27475,506);(27546,503);(27617,500);(27688,497);(27759,494);(27830,492);(27901,489);(27972,487);(28043,485);(28114,483);(28192,480);(28264,478);(28336,476);(28408,474);(28480,472);(28552,470);(28624,468);(28696,466);(28768,464);(28840,462);(28912,461);(28984,460);(29062,459);(29133,457);(29204,456);(29275,454);(29346,453);(29417,452);(29488,451);(29559,450);(29630,449);(29701,448);(29772,448);(29843,448);(29925,447);(29994,447);(30063,446);(30132,446);(30201,446);(30270,446);(30339,446);(30408,446);(30477,446);(30546,446);(30615,447);(30684,447);(30766,448);(30837,449);(30908,450);(30979,451);(31050,452);(31121,453);(31192,454);(31263,456);(31334,458);(31405,460);(31476,462);(31547,464);(31622,465);(31699,467);(31776,469);(31853,471);(31930,473);(32007,475);(32084,477);(32161,480);(32238,483);(32315,485);(32392,488);(32469,490);(32549,493);(32620,496);(32691,499);(32762,503);(32833,507);(32904,511);(32975,515);(33046,522);(33117,527);(33188,534);(33259,541);(33330,547);(33409,553);(33480,559);(33551,564);(33622,566);(33693,569);(33764,573);(33835,577);(33906,580);(33977,584);(34048,591);(34119,597);(34190,603);(34268,610);(34338,619);(34408,627);(34478,635);(34548,642);(34618,649);(34688,656);(34758,663);(34828,671);(34898,679);(34968,687);(35038,694);(35119,703);(35190,713);(35261,720);(35332,721);(772,1090);(843,1112);(914,1139);(985,1167);(1056,1220);(1127,1253);(1198,1281);(1269,1323);(1340,1325);(1411,1327);(1482,1326);(1553,1322);(1638,1319);(1714,1315);(1790,1313);(1866,1310);(1942,1307);(2018,1305);(2094,1304);(2170,1303);(2246,1302);(2322,1301);(2398,1300);(2474,1300);(2548,1300);(2614,1299);(2680,1299);(2746,1299);(2812,1300);(2878,1300);(2944,1301);(3010,1302);(3076,1303);(3142,1304);(3208,1305);(3274,1307);(3347,1308);(3418,1308);(3489,1306);(3560,1305);(3631,1307);(3702,1314);(3773,1315);(3844,1319);(3915,1323);(3986,1327);(4057,1334);(4128,1339);(4205,1344);(4274,1349);(4343,1354);(4412,1360);(4481,1366);(4550,1372);(4619,1379);(4688,1386);(4757,1393);(4826,1400);(4895,1408);(4964,1416)";
    const TEST2: &str =  "(26991,638);(27062,645);(27133,652);(27204,659);(27275,666);(27346,674);(27417,682);(27488,690);(27559,698);(27630,707);(27701,718);(27772,727);(27847,736);(27918,748);(27989,760);(28060,773);(28131,785);(28202,799);(28273,815);(28344,831);(28415,847);(28486,864);(28557,881);(28628,900);(28768,921);(28839,943);(28910,966);(28981,989);(29052,1010);(29123,1031);(29194,1054);(29265,1075);(29336,1099);(29407,1123);(29478,1147);(29549,1189);(29630,1221);(29701,1253);(29772,1287);(29843,1310);(29914,1311);(29985,1311);(30056,1308);(30127,1305);(30198,1302);(30269,1300);(30340,1297);(30411,1293);(30490,1290);(30561,1288);(30632,1285);(30703,1283);(30774,1283);(30845,1283);(30916,1284);(30987,1284);(31058,1284);(31129,1284);(31200,1283);(31271,1282);(31352,1282);(31416,1282);(31480,1282);(31544,1284);(31608,1286);(31672,1288);(31736,1291);(31800,1294);(31864,1295);(31928,1294);(31992,1290);(32056,1289);(32133,1297);(32204,1298);(32275,1301);(32346,1304);(32417,1308);(32488,1312);(32559,1315);(32630,1322);(32701,1327);(32772,1333);(32843,1335);(32914,1343);(32995,1349);(33066,1355);(33137,1361);(33208,1367);(33279,1375);(33350,1383);(33421,1392);(33492,1402);(33563,1410);(33634,1417);(33705,1426);(33776,1434);(33853,1443);(33924,1451);(33995,1459);(34066,1468);(34137,1477);(34208,1487);(34279,1497);(34350,1507);(34421,1516);(34492,1525);(34563,1535);(34634,1547);(34779,1560);(34849,1573);(34919,1591);(34989,1606);(35059,1620);(35129,1636);(35199,1652);(35269,1668);(774,2244);(845,2295);(916,2336);(987,2379);(1058,2426);(1129,2410);(1200,2393);(1271,2376);(1348,2358);(1419,2342);(1490,2326);(1561,2311);(1632,2296);(1703,2282);(1774,2266);(1845,259);(1916,265);(1987,2221);(2058,2213);(2129,2206);(2204,2196);(2275,2186);(2346,2177);(2417,2168);(2488,2158);(2559,2150);(2630,2142);(2701,2134);(2772,2126);(2843,2118);(2914,2111);(2985,2105);(3063,2099);(3133,2093);(3203,1452);(3273,1451);(3343,1450);(3413,1449);(3483,1448);(3553,1447);(3623,1446);(3693,1448);(3763,778);(3833,763);(3905,755);(3976,753);(4047,751);(4118,751);(4189,752);(4260,754);(4331,757);(4402,761);(4473,768);(4544,773);(4615,782);(4686,796);(4761,831);(4832,2059);(4903,2058);(4974,2058);(5045,2061);(5116,2065);(5187,2069);(5258,2073);(5329,2077);(5400,2081);(5471,2084);(5542,2086);(5619,2069);(5691,1933);(5763,1848);(5835,1779);(5907,1713);(5979,1646);(6051,1582);(6123,1535);(6195,1483);(6267,1446);(6339,1406);(6411,1367);(6493,1332);(6564,1296);(6635,1283);(6706,1259);(6777,1207);(6848,1167);(6919,1134);(6990,1102);(7061,1077);(7132,1058);(7203,1050);(7274,1043);(7352,1011);(7421,985);(7490,960);(7559,939);(7628,919);(7697,899);(7766,882);(7835,867);(7904,855);(7973,848);(8042,841);(8111,829);(8190,814);(8261,800);(8332,788);(8403,776);(8474,765);(8545,754);(8616,745);(8687,736);(8758,727);(8829,720);(8900,712);(8971,704);(9049,697);(9121,689);(9193,682);(9265,675);(9337,668);(9409,661);(9481,654);(9553,647);(9625,640);(9697,633);(9769,626);(9841,620);(9923,615);(9994,610);(10065,605);(10136,600);(10207,595);(10278,593);(10349,586);(10420,584);(10491,580);(10562,576);(10633,572);(10704,568);(10779,565);(10850,561);(10921,557);(10992,554);(11063,551);(11134,548);(11205,546);(11276,544);(11347,541);(11418,539);(11489,537);(11560,535);(11637,533);(11708,531);(11779,530);(11850,529);(11921,527);(11992,526);(12063,526);(12134,525);(12205,524);(12276,523);(12347,522);(12418,521);(12490,520);(12561,520);(12632,519);(12703,519);(12774,518);(12845,518);(12916,518);(12987,518);(13058,518);(13129,518);(13200,518);(13271,518);(13348,518);(13420,519);(13492,519);(13564,519);(13636,520);(13708,520);(13780,521);(13852,522);(13924,523);(13996,524);(14068,525);(14140,526);(14208,528);(14279,529);(14350,530);(14421,531);(14492,532);(14563,534);(14634,536);(14705,538);(14776,540);(14847,542);(14918,544);(14989,546);(15070,548);(15146,550);(15222,553);(15298,556);(15374,559);(15450,562);(15526,565);(15602,568);(15678,571);(15754,574);(15830,578);(15906,581);(15980,585);(16052,592);(16124,597);(16196,602);(16268,607);(16340,609);(16412,616);(16484,621);(16556,626);(16628,632);(16700,637);(16772,643);(16850,649);(16921,655);(16992,660);(17063,665);(17134,670);(17205,675);(18849,546);(18920,545);(18991,544);(19062,542);(19133,537);(19204,535);(19275,531);(19346,527);(19420,524);(19490,521);(19560,518);(19630,514);(19700,510);(19770,507);(19840,503);(19910,500);(19980,497);(20050,493);(20120,490);(20190,488);(20262,485);(20333,482);(20404,480);(20475,477);(20546,475);(20617,473);(20688,470);(20759,468);(20830,465);(20901,463);(20972,461);(21043,459);(21120,457);(21193,456);(21266,455);(21339,453);(21412,452);(21485,451);(21558,449);(21631,448);(21704,446);(21777,445);(21850,444);(21923,444);(21998,443);(22069,442);(22140,442);(22211,441);(22282,441);(22353,440);(22424,440);(22495,440);(22566,440);(22637,440);(22708,440);(22779,439);(22850,439);(22921,439);(22992,439);(23063,439);(23134,439);(23205,439);(23276,440);(23347,440);(23418,441);(23489,442);(23560,443);(23631,444);(23709,445);(23778,446);(23847,448);(23916,450);(23985,452);(24054,454);(24123,456);(24192,458);(24261,460);(24330,462);(24399,464);(24468,467);(24543,469);(24614,471);(24685,474);(24756,476);(24827,479);(24898,482);(24969,485);(25040,488);(25111,492);(25182,495);(25253,499);(25324,503);(25402,507);(25468,515);(25534,522);(25600,529);(25666,536);(25732,543);(25798,549);(25864,554);(25930,556);(25996,560);(26062,567);(26128,569);(26208,573);(26279,580);(26350,582);(26421,589);(26492,595);(26563,601);(26634,607);(26705,613);(26776,620);(26847,627);(26918,634);(26989,641);(27066,647);(27142,654);(27218,661);(27294,669);(27370,676);(27446,684);(27522,692);(27598,700);(27674,708);(27750,717);(27826,728);(27902,739);(27976,750);(28047,763);(28118,776);(28189,789);(28260,803);(28331,817);(28402,834);(28473,851);(28544,868);(28615,886);(28686,904);(28757,924);(28829,944);(28900,966);(28971,988);(29042,1009);(29113,1030);(29184,1051);(29255,1072);(29326,1096);(29397,1120);(29468,1145);(29539,1190);(29610,1218);(29684,1254);(29757,1285);(29830,1316);(29903,1314);(29976,1310);(30049,1307);(30122,1305);(30195,1303);(30268,1300);(30341,1297);(30414,1293);(30487,1290);(30565,1287);(30636,1285);(30707,1284);(30778,1283);(30849,1283);(30920,1283);(30991,1283);(31062,1283);(31133,1283);(31204,1282);(31275,1282);(31346,1282);(31420,1282);(31489,1282);(31558,1284);(31627,1286);(31696,1289);(31765,1293);(31834,1296);(31903,1298);(31972,1297);(32041,1289);(32110,1289);(32179,1295);(32258,1297);(32330,1300);(32402,1303);(32474,1306);(32546,1310);(32618,1313);(32690,1320);(32762,1326);(32834,1332);(32906,1337);(32978,1342);(33050,1348);(33124,1355);(33195,1361);(33266,1368);(33337,1377);(33408,1383);(33479,1390);(33550,1398);(33621,1406);(33692,1415);(33763,1424);(33834,1432);(33905,1440);(33982,1449);(34054,1457);(34126,1465);(34198,1475);(34270,1485);(34342,1495);(34414,1505);(34486,1514);(34558,1524);(34630,1533);(34702,1544);(34774,1556);(34847,1569);(34917,1584);(34987,1600);(35057,1616);(35127,1630);(35197,1646);(35267,1661);(35337,1674)";

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

    #[test]
    fn test1() {
        let data = load_log(TEST1);
        // println!("{:#?}", data);
        let ha = build_hough_accumulator(&data);
        println!("{:?}", ha.len());
        let walls = find_walls(ha.iter().copied().collect()).unwrap();

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
            ha.iter().map(|e| e.line).collect(),
            walls_vec.into_iter().map(|e| e.line).collect(),
        );

        println!("{:?}", walls);
        // show_viewport(
        //     *data,
        //     ha.into_iter().map(|e| e.line).collect(),
        //     walls.into_iter().map(|e| e.line).collect(),
        // )
        // .unwrap();
    }
}
