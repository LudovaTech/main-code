//! Code permettant l'utilisation de Vector2 (un moyen de stocker des duos de donnés)

/// Définition du type Vector2 comme étant le Vector2 de nalgebra pour des f32
pub type Vector2 = nalgebra::Vector2<f32>;

// see https://doc.rust-lang.org/rust-by-example/generics/new_types.html
// Permet d'être sûr à la compilation que l'on passe bien une coordonnée Locale/Globale et que l'on ne s'est pas trompé

// Représente une coordonée dont l'origine est le centre du robot et l'orientation y suit l'axe du kicker
#[derive(Debug, Clone, Copy)]
pub struct LocalCoord(Vector2);

impl LocalCoord {
    pub fn to_global_coordinates(&self, robot_position: GlobalCoord) -> GlobalCoord {
        GlobalCoord(self.0 + robot_position.0)
    }
}

// Représente une coordonée dont l'origine est le centre du terrain et l'orientation y se dirige vers le goal adverse
#[derive(Debug, Clone, Copy)]
pub struct GlobalCoord(Vector2);

impl GlobalCoord {
    pub fn to_local_coordinates(&self, robot_position: GlobalCoord) -> LocalCoord {
        LocalCoord(self.0 - robot_position.0)
    }
}

// Exemple d'utilisation :
// fn main() {
//     println!("{:?}", Vector2::new(1.0, 2.0));
//     println!("{:?}", GlobalCoord(Vector2::new(2.0, 1.0)));
//     let robot_pos = GlobalCoord(Vector2::new(1.0, 1.0));
//     println!("{:?}", GlobalCoord(Vector2::new(2.0, 1.0)).to_local_coordinates(robot_pos));
//     println!("id: {:?}", GlobalCoord(Vector2::new(2.0, 1.0)).to_local_coordinates(robot_pos).to_global_coordinates(robot_pos));
// }