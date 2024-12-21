use rppal::uart::Uart;
use std::time::{Duration, Instant};
use std::{error::Error};
use tracing::{error, info, instrument, warn};

#[derive(Debug)]
pub struct Lidar {
    conn: Uart,
}

impl Lidar {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        let mut conn = Uart::with_path("/dev/ttyAMA3", 230_400, rppal::uart::Parity::None, 8, 1)?;
        conn.set_hardware_flow_control(false)?;
        conn.set_software_flow_control(false)?;
        Ok(Self { conn })
    }
}

// Récupération des données
impl Lidar {
    #[instrument(skip(self))]
    pub fn read(&mut self) -> Result<Option<Box<Vec<u8>>>, Box<dyn Error>> {
        // TODO : si le nombre d'entrée en attente est trop long, c'est que les données sont vielles
        if self.conn.input_len()? < 47 {
            // Taille d'une donnée Lidar
            return Ok(None);
        }
        let mut buffer: Box<Vec<u8>> = Box::new(vec![0; 47]);
        let nr = self.conn.read(&mut buffer)?;
        if nr != 47 {
            warn!("mauvaise taille de lecture pour buffer (got {:?})", nr);
            return Ok(None);
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
                        return Ok(None);
                    }
                }
                let nd = self.conn.read(&mut buffer[(47 - start_pos)..])?;
                if nd != start_pos {
                    warn!(
                        "mauvaise taille de lecture pour buffer ajout (got {:?})",
                        nd
                    );
                    return Ok(None);
                }
                info!("Récupération réussite, le flux UART du lidar est de nouveau aligné.");
            } else {
                error!("Pas de début trouvé, 84 44 ne semble pas être présent dans la capture du flux UART du lidar");
                return Ok(None);
            }
        }
        Ok(Some(buffer))
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

#[derive(Debug)]
pub struct LidarPoint {
    pub distance: u16,
    pub intensity: u8,
    pub angle: f32,
}

impl LidarPoint {
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
            points.push(LidarPoint {
                distance: Self::get_2_bytes_lsb_msb(&data, i),
                intensity: data[i + 2],
                angle: 0.0,
            });
        }
        let end_angle = Self::get_2_bytes_lsb_msb(&data, 42);
        let _timestamp = Self::get_2_bytes_lsb_msb(&data, 44);
        let _crc_check = data[46];

        // TODO crc check

        // Nécessaire à cause du passage 360°-0°
        let angle_step = if start_angle <= end_angle {
            (end_angle - start_angle) / 11 // 11 est le nombre de point dans un packet (12) moins 1
        } else {
            (36_000 + end_angle - start_angle) / 11
        };

        for (n_point, point) in points.iter_mut().enumerate() {
            point.angle = ((start_angle + (angle_step * (n_point as u16))) % 36_000) as f32;
        }
        Ok(points)
    }

    fn get_2_bytes_lsb_msb(buffer: &[u8], index: usize) -> u16 {
        (buffer[index + 1] as u16) << 8 | (buffer[index] as u16)
    }
}
