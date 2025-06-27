//! Ce fichier est destiné à être `use prelude::*` donc peu de choses sont publiques

pub use crate::units::*;

// Système de log :

#[allow(unused)]
pub use tracing::{trace, debug, info, warn, error};

pub mod colors {
    pub const BLUE: (u8, u8, u8) = (0, 191, 255);      // Electric Blue
    pub const PURPLE: (u8, u8, u8) = (128, 0, 128);   // Purple
    pub const TEAL: (u8, u8, u8) = (0, 128, 128);     // Teal
    pub const PINK: (u8, u8, u8) = (255, 105, 180);    // Hot Pink
    pub const YELLOW: (u8, u8, u8) = (255, 215, 0);    // Gold
    pub const GREEN: (u8, u8, u8) = (0, 255, 0);       // Bright Green
    pub const ORANGE: (u8, u8, u8) = (255, 165, 0);    // Orange
    pub const MAGENTA: (u8, u8, u8) = (255, 0, 255);   // Magenta
    pub const SKY_BLUE: (u8, u8, u8) = (135, 206, 235); // Sky Blue
    pub const SPRING_GREEN: (u8, u8, u8) = (0, 255, 127); // Spring Green
    pub const CORAL: (u8, u8, u8) = (255, 127, 80);    // Coral
    pub const DARK_RED: (u8, u8, u8) = (178, 34, 34);
    
    pub const COLORS: [(u8, u8, u8); 12] = [
        BLUE,
        PURPLE,
        TEAL,
        PINK,
        YELLOW,
        GREEN,
        ORANGE,
        MAGENTA,
        SKY_BLUE,
        SPRING_GREEN,
        CORAL,
        DARK_RED,
    ];
}