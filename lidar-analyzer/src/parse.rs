//! Lit les données du lidar depuis la connexion série et les transforme en une forme utilisable

use rppal::uart::{self, Uart};
use std::error::Error;
use std::fmt::Display;
use std::ops::{AddAssign, Div, Mul, Sub};
use std::time::{Duration, Instant};
use tracing::{error, info, instrument, warn};
use crate::units::*;

/// Représente l'ensemble des erreurs pouvant survenir lors de l'utilisation du lidar
#[derive(Debug)]
pub enum LidarError {
    /// Erreurs concernant la connexion uart
    UartError(uart::Error),
    /// Il n'y a pas assez de données pour constituer un tour complet de lidar
    NotEnoughData(usize),
    /// Erreur causée probablement par un bug
    /// devrait paniquer dans une situation normale mais on veut éviter que le comportement du robot soit affecté
    ShouldNotHappen(String),
    /// Les données ne sont pas arrivées / pas arrivées en nombre suffisant après le temps donné
    Timeout,
    /// Le début des données n'a pas été trouvé
    StartNotFound,
}

impl Display for LidarError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl Error for LidarError {}

impl From<uart::Error> for LidarError {
    fn from(error: uart::Error) -> Self {
        LidarError::UartError(error)
    }
}

/// Représente le lidar du robot
#[derive(Debug)]
pub struct Lidar {
    conn: Uart,
}

impl Lidar {
    /// Etablissement de la connexion série et création de l'objet Lidar
    pub fn new() -> Result<Self, uart::Error> {
        let mut conn = Uart::with_path("/dev/ttyAMA3", 230_400, rppal::uart::Parity::None, 8, 1)?;
        conn.set_hardware_flow_control(false)?;
        conn.set_software_flow_control(false)?;
        Ok(Self { conn })
    }
}

impl Lidar {
    /// Récupération des données depuis la connexion série
    #[instrument(skip(self))]
    pub fn read(&mut self) -> Result<Box<Vec<u8>>, LidarError> {
        // TODO : si le nombre d'entrée en attente est trop long, c'est que les données sont vielles
        let nb_data_waiting = self.conn.input_len()?;
        if nb_data_waiting < 47 {
            // 47 : Taille d'une donnée Lidar
            return Err(LidarError::NotEnoughData(nb_data_waiting));
        }
        let mut buffer: Box<Vec<u8>> = Box::new(vec![0; 47]);
        let nr = self.conn.read(&mut buffer)?;
        if nr != 47 {
            let warn_string = format!("mauvaise taille de lecture pour buffer (got {:?})", nr);
            warn!(warn_string);
            return Err(LidarError::ShouldNotHappen(warn_string));
        }
        if !(buffer[0] == 84 && buffer[1] == 44) {
            warn!("Le flux UART du lidar n'est pas aligné. Tentative de récupération...");
            if let Some(start_pos) = Self::look_for_start(&buffer) {
                // On compléter les données déjà récupérées avec celles qui manque
                // pour réaligner le flux sans perdre d'information
                buffer.copy_within(start_pos.., 0);
                let time_start = Instant::now();
                while self.conn.input_len()? < start_pos {
                    if time_start.elapsed() > Duration::from_secs(3) {
                        error!("Atteindre la fin de la séquence de données du lidar demande trop de temps.");
                        return Err(LidarError::Timeout);
                    }
                }
                let nd = self.conn.read(&mut buffer[(47 - start_pos)..])?;
                if nd != start_pos {
                    let warn_string = format!(
                        "mauvaise taille de lecture pour buffer ajout (got {:?})",
                        nd
                    );
                    warn!(warn_string);
                    return Err(LidarError::ShouldNotHappen(warn_string));
                }
                info!("Récupération réussite, le flux UART du lidar est de nouveau aligné.");
            } else {
                error!("Pas de début trouvé, 84 44 ne semble pas être présent dans la capture du flux UART du lidar");
                return Err(LidarError::StartNotFound);
            }
        }
        Ok(buffer)
    }

    fn look_for_start(buffer: &Box<Vec<u8>>) -> Option<usize> {
        let mut before_val = buffer[0];
        for i in 1..buffer.len() - 1 {
            let actual_val = buffer[i];
            if before_val == 84 && actual_val == 44 {
                return Some(i - 1);
            }
            before_val = actual_val;
        }
        None
    }
}


/// Représente une distance en mm sous forme de u16
/// La valeur n'est pas automatiquement convertie car elle est pratique pour la transformation de Hough
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct LidarDistance(pub u16);

impl LidarDistance {
    /// Transformation des unités :
    /// distance mm -> meters
    #[inline]
    pub fn to_meters(&self) -> Meters {
        Meters::mm(f64::from(self.0))
    }

    #[inline]
    pub const fn cm(distance: u16) -> Self {
        Self(distance * 10)
    }

    #[inline]
    pub const fn meters(distance: u16) -> Self {
        Self(distance * 1000)
    }
}

impl Div for LidarDistance {
    type Output = u16;

    fn div(self, rhs: Self) -> Self::Output {
        self.0 / rhs.0
    }
}

impl Mul<f64> for LidarDistance {
    type Output = LidarDistance;

    fn mul(self, rhs: f64) -> Self::Output {
        assert!(rhs.is_sign_positive());
        LidarDistance((f64::from(self.0) * rhs).round() as u16)
    }
}

impl Mul<usize> for LidarDistance {
    type Output = LidarDistance;

    fn mul(self, rhs: usize) -> Self::Output {
        LidarDistance(self.0 * rhs as u16)
    }
}

/// Représente un angle *en centièmes de degrés* sous forme de u16
/// La valeur n'est pas automatiquement convertie car elle est pratique pour la transformation de Hough
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct LidarAngle(pub u16);

impl LidarAngle {
    /// Transformation des unités :
    /// angle : 0.01 degrés -> degrés Deg
    #[inline]
    pub fn to_deg(&self) -> Deg {
        Deg::new(f64::from(self.0) / 100.0)
    }

    #[inline]
    pub const fn deg(angle: u16) -> Self {
        Self(angle * 100)
    }
}

impl AddAssign for LidarAngle {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0
    }
}

impl LidarAngle {
    pub fn distance_between_angle(self, other: Self) -> Self {
        if self >= other {
            Self(self.0 - other.0)
        } else {
            Self(other.0 - self.0)
        }
    }
}

// impl Sub for LidarAngle {
//     type Output = LidarAngle;

//     fn sub(self, rhs: Self) -> Self::Output {
//         LidarAngle(self.0 - rhs.0)
//     }
// }

impl Div for LidarAngle {
    type Output = u16;

    fn div(self, rhs: Self) -> Self::Output {
        self.0 / rhs.0
    }
}

impl Mul<usize> for LidarAngle {
    type Output = LidarAngle;

    fn mul(self, rhs: usize) -> Self::Output {
        LidarAngle(self.0 * rhs as u16)
    }
}

/// Représente un point donné par le lidar.
/// 
/// Unités :
///  - distance en mètres 
///  - intensité entre 0 et 1
///  - angle en radians
#[derive(Debug, Clone)]
pub struct LidarPoint {
    pub distance: LidarDistance,
    pub intensity: Intensity,
    pub angle: LidarAngle,
}

impl LidarPoint {
    /// Génère les points du lidar depuis une suite de bytes
    #[instrument(skip(data))]
    pub fn from_data(data: Box<Vec<u8>>) -> Result<Box<Vec<Self>>, Box<dyn Error>> {
        if data.len() != 47 {
            let str_err = format!("Wrong data len : {:?}", data.len());
            error!(str_err);
            return Err(str_err.into());
        }
        let mut points: Box<Vec<Self>> = Box::new(Vec::with_capacity(12));
        let _speed = Self::get_2_bytes_lsb_msb(&data, 2);
        let start_angle = Self::get_2_bytes_lsb_msb(&data, 4);
        for i in (6..=39).step_by(3) {
            // Transformation des unités :
            // intensité 0-255 u8 -> 0-1 f64
            points.push(LidarPoint {
                distance: LidarDistance(Self::get_2_bytes_lsb_msb(&data, i)),
                intensity: Intensity::from_u8(data[i + 2]),
                angle: LidarAngle(0),
            });
        }
        let end_angle = Self::get_2_bytes_lsb_msb(&data, 42);
        let _timestamp = Self::get_2_bytes_lsb_msb(&data, 44);
        let _crc_check = data[46];

        // TODO crc check

        // Nécessaire à cause du passage 360°-0°
        let angle_step = if start_angle <= end_angle {
            (end_angle - start_angle) / 11 // 11 est le nombre de points dans un packet (12) moins 1
        } else {
            (36_000 + end_angle - start_angle) / 11
        };

        for (n_point, point) in points.iter_mut().enumerate() {
            let angle = (start_angle + (angle_step * (n_point as u16))) % 36_000;
            point.angle = LidarAngle(angle);
        }
        Ok(points)
    }

    #[inline]
    fn get_2_bytes_lsb_msb(buffer: &[u8], index: usize) -> u16 {
        (buffer[index + 1] as u16) << 8 | (buffer[index] as u16)
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    // write unit tests here :-)
}