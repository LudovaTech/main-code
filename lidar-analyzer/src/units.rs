//! Contient tous les types unités utilisés par l'ensemble du code.
//! 
//! Ce fichier est destiné à être `use units::*` donc peu de choses sont publiques

use std::ops::{Add, Div, Mul, Neg, Sub};

pub type Rad = radians::Rad64;
pub type Deg = radians::Deg64;

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Meters(pub f64);

impl Meters {
    #[inline]
    pub const fn cm(distance: f64) -> Self {
        Self(distance / 100.0)
    }

    #[inline]
    pub const fn mm(distance: f64) -> Self {
        Self(distance / 1000.0)
    }

    pub const fn const_div(self, rhs: Self) -> Self {
        Self(self.0 / rhs.0)
    }

    /// Teste si cette distance est dans les environs de celle passée en paramètres
    /// entre 0.9 et 1.1 fois
    pub fn in_the_aera_of(&self, other: Meters) -> bool {
        ((other.0 * 0.9)..=(other.0 * 1.1)).contains(&self.0)
    }
}

impl Div for Meters {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        Self(self.0 / rhs.0)
    }
}

impl Neg for Meters {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self(-self.0)
    }    
}

impl Add for Meters {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Meters {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

/// Représente une intensité entre 0 et 1 inclus
#[derive(Debug, Clone, Copy)]
pub struct Intensity(f64);

impl Intensity {
    pub const NULL: Intensity = Intensity(0.0);
    pub const FULL: Intensity = Intensity(1.0);

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