//! Lit les données du lidar depuis la connexion série et les transforme en une forme utilisable

use rppal::uart::{self, Uart};
use std::error::Error;
use std::fmt::Display;
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

/// Représente un point donné par le lidar.
/// 
/// Unités :
///  - distance en mètres 
///  - intensité entre 0 et 1
///  - angle en radians
#[derive(Debug)]
pub struct LidarPoint {
    pub distance: Meters,
    pub intensity: Intensity,
    pub angle: Rad,
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
            // distance mm -> meters
            // intensité 0-255 -> 0-1
            points.push(LidarPoint {
                distance: Meters::mm(f64::from(Self::get_2_bytes_lsb_msb(&data, i))),
                intensity: Intensity::from_u8(data[i + 2]),
                angle: Rad::ZERO,
            });
        }
        let end_angle = Self::get_2_bytes_lsb_msb(&data, 42);
        let _timestamp = Self::get_2_bytes_lsb_msb(&data, 44);
        let _crc_check = data[46];

        // TODO crc check

        // Nécessaire à cause du passage 360°-0°
        let angle_step = if start_angle <= end_angle { // TODO change to use directly Rad ?
            (end_angle - start_angle) / 11 // 11 est le nombre de points dans un packet (12) moins 1
        } else {
            (36_000 + end_angle - start_angle) / 11
        };

        for (n_point, point) in points.iter_mut().enumerate() {
            let angle = (start_angle + (angle_step * (n_point as u16))) % 36_000;
            // Transformation des unités :
            // angle : 0.01 degrés -> radians
            point.angle = Deg::new(f64::from(angle) / 100.0).rad();
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