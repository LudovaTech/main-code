use rppal::uart::Uart;
use std::error::Error;
use tracing::{debug, error, info, instrument, warn};

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

    #[instrument(skip(self))]
    pub fn read(&mut self) -> Result<Option<Box<Vec<u8>>>, Box<dyn Error>> {
        // Attendre jusqu'à ce que le nbr de bits soit suffisant pour 1 partie de donnée de lidar. Lire cette partie, si elle commence par T, tout va bien sinon on se permet de
        // - blocker le code ? jusqu'à avoir les caractères restants et être de nouveau synchronisé. Si la lecture avec blockage de code échoue... on est mal
        // - sinon on stocke pour le prochain appel à la fonction le décalage de trop et teste avec le nbr de bits restant au lieu du normal.
        //self.conn.flush(rppal::uart::Queue::Both)?;
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
                while self.conn.input_len()? < start_pos {}
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
