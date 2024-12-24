//! Ce dossier est dédié au developpement des stratégies.

mod consts;
mod vector2;
use vector2::Vector2;

/// Ce struct contient l'ensemble des données qui sont passés
/// à l'algorithme des stratégies.
///
/// Chacunes des valeurs de ce struct sont optionelles car elles peuvent être
/// indisponibles. La dépendance de chacun des champs est indiqué en documentation.
/// (= disponible uniquement si tous les élèments de la liste réussissent)
///
/// Pour les positions absolues que l'origine serait le centre du terrain et que si
/// on est placé au niveau de notre goal regardant ver le goal adverse, les y positifs
/// se dirigent vers le goal adverse et les x positifs sont à droite.
///
/// Pour les coordonées relatives au robot, l'origine est le centre du robot, les y positifs
/// vers le devant du robot et les x positifs à droite.
/// 
/// Toutes les distances sont en cm et tous les angles en radians.
#[derive(Debug, Default)]
struct Informations {
    /// position *absolue* du robot sur le terrain.
    /// # Dépendances :
    /// - lidar analyzer complet
    robot_position: Option<Vector2>,

    /// angle du robot *en radians* par rapport à la droite notre goal / leur goal,
    /// vu depuis notre goal, augmente dans le sens des aiguilles d'une montre
    /// # Dépendances :
    /// - lidar analyzer complet
    robot_angle: Option<f32>,

    /// distance au mur le plus proche.
    /// Cette valeur est quasiment toujours calculée par lidar analyzer. Elle permet d'éviter de
    /// sortir du terrain y compris lorsque la détermination des murs (lidar analyzer complet) échoue.
    /// Si lidar analyzer complet est disponible, ses données sont à privilégier.
    /// # Dépendances :
    /// - lidar analyzer dégradé
    distance_to_nearest_wall: Option<f32>,

    /// position *absolue* du robot ami (coéquipier) sur le terrain.
    /// # Dépendances :
    /// - bluetooth entre les deux robots
    /// - lidar analyzer du robot ami
    friend_position: Option<Vector2>,

    /// angle du robot ami *en radians* par rapport à la droite notre goal / leur goal,
    /// # Dépendances :
    /// - bluetooth entre les deux robots
    /// - lidar analyzer du robot ami
    friend_angle: Option<Vector2>,

    /// position *relative* de la balle par rapport au robot,
    /// # Dépendances :
    /// - la caméra arrive à détecter la balle
    ball_relative_position: Option<Vector2>,

    /// position *absolue* de la balle calculée par le robot ami,
    /// Cette valeur sert surtout si la balle est trop loin pour être bien
    /// détectée par ce robot mais que l'autre ami est plus proche.
    /// # Dépendances :
    /// - bluetooth entre les deux robots
    /// - la caméra de l'autre robot arrive à détecter la balle
    friend_ball_position: Option<Vector2>,

    /// tuple des 2 positions des robots adverses
    /// l'une des positions peut-être disponible et l'autre non.
    /// Si la position à l'emplacement 0 du tuple est indisponible, celle à la position
    /// l'est aussi.
    /// # Dépendances :
    /// - lidar analyzer NG
    enemy_positions: (Option<Vector2>, Option<Vector2>),
}

fn main() {
    println!("Hello, world!");
    println!("Vector2 {:?}", Vector2::new(10., 5.));
}
