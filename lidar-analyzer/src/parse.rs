use rppal::uart::Uart;
use std::error::Error;
use tracing::{info, instrument, warn};

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
            Ok(None)
        } else {
            let mut buffer: Box<Vec<u8>> = Box::new(vec![0; 47]);
            let nr = self.conn.read(&mut buffer)?;
            if nr != 47 {
                warn!("mauvaise taille de lecture pour buffer (got {:?})", nr);
                return Ok(None);
            }
            if !(buffer[0] == 84 && buffer[1] == 44) {
                warn!(
                    "Les données du Lidar ne sont pas alignés (has {:?})",
                    buffer
                );
                let mut align = [0u8; 1];
                let mut after_84: bool = false;
                while !(after_84 && align[0] == 44) {
                    // On enlève une par une les données jusqu'à atteindre 84 44
                    if align[0] == 84 {
                        after_84 = true;
                    } else {
                        after_84 = false;
                    }
                    while self.conn.input_len()? < 1 {}
                    let na = self.conn.read(&mut align)?;
                    if na != 1 {
                        warn!("mauvaise taille de lecture pour align (got {:?})", na)
                    }
                    // debug!("Found: {:?}", align);
                }
                let mut discard = [0u8; 45];
                while self.conn.input_len()? < 45 {}
                let nd = self.conn.read(&mut discard)?;
                if nd != 45 {
                    warn!("mauvaise taille de lecture pour discard (got {:?})", nd)
                }
                info!("Les données du Lidar sont maintenant alignées.");
                return Ok(None);
            }
            Ok(Some(buffer))
        }
    }
}
