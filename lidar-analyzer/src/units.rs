//! Contient tous les types unités utilisés par l'ensemble du code.
//! 
//! Ce fichier est destiné à être `use units::*` donc peu de choses sont publiques

pub type Rad = radians::Rad64;
pub type Deg = radians::Deg64;

#[derive(Debug, Clone, Copy)]
pub struct Meters(pub f64);

/// Représente une intensité entre 0 et 1 inclus
#[derive(Debug, Clone, Copy)]
pub struct Intensity(f64);

impl Intensity {
    pub fn new(intensity: f64) -> Result<Intensity, ()> {
        if (0.0..=1.0).contains(&intensity) {
            Ok(Self(intensity))
        } else {
            Err(())
        }
    }

    pub fn from_u8(intensity: u8) -> Intensity {
        Self(f64::from(intensity) / 255.0)
    }
}