//mod consts;
mod vector2;
use vector2::Vector2;
use radians::Rad32;

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
    robot_angle: Option<Rad32>,

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

    robot_has_ball: bool,
    friend_has_ball: bool,

    /// tuple des 2 positions des robots adverses
    /// l'une des positions peut-être disponible et l'autre non.
    /// Si la position à l'emplacement 0 du tuple est indisponible, celle à la position
    /// l'est aussi.
    /// # Dépendances :
    /// - lidar analyzer NG
    enemy_positions: (Option<Vector2>, Option<Vector2>),
}

/// Représente l'ensemble des actions à faire par le robot qui ont été décidés par les stratégies
#[derive(Debug)]
struct Action {
    /// la position vers laquelle on voudrait aller
    move_to: Vector2, //TODO : prise en charge Action sans pos du robot
    /// l'orientation final que l'on voudra avoir lorsque l'on aura atteint cette position
    final_orientation: Rad32,
    /// est-ce que on active le kicker pour lancer la balle
    kick: bool,
    /// vitesse que l'on donne au dribbler :
    /// - 0 ne bouge pas
    /// - +1 avance en ramenant la balle vers le robot
    /// - -1 tourne en expulsant la balle
    dribbler: f32,
}

fn main() {
    println!("Vector2 {:?}", Vector2::new(10., 5.));
    println!("décision prise : {:?}", decision(&Informations::default()));
}

fn decision(info: &Informations) -> Action {
    if let Some(refrain) = is_outside(info) {
        return refrain;
    }
}

fn is_outside(info: &Informations) -> Option<Action> {
    let robot_angle = info.robot_angle.unwrap_or(Rad32::new(0.0));
    if let Some(distance_to_nearest_wall) = info.distance_to_nearest_wall {
        if distance_to_nearest_wall < 5.0 {
            //-> bug pour 0; 90; 180 et 270  pt choisir vers cages adverses pour majorité des cas

            let mut new_position = Vector2::new(0.0, 0.0);
            let mut new_orientation = Rad32::ZERO;

            if robot_angle <= Rad32::QUARTER_TURN {
                // o entre 0 et 90
                // pos : x-5 ; y + 5
                //o : o - 90
                new_position = Vector2::new(robot_position.x - 5.0, robot_position.y + 5.0);
                new_orientation = robot_angle - Rad32::QUARTER_TURN;
            }

            if robot_angle > Rad32::QUARTER_TURN && robot_angle < Rad32::HALF_TURN {
                // o entre 90 et 180
                // pos : x-5 ; y - 5
                //o : o + 90
                new_position = Vector2::new(robot_position.x - 5.0, robot_position.y - 5.0);
                new_orientation = robot_angle + Rad32::QUARTER_TURN;
            }

            if robot_angle >= Rad32::HALF_TURN && robot_angle < (Rad32::HALF_TURN + Rad32::QUARTER_TURN) {
                // o entre 180 et 270
                // pos : x+5 ; y - 5
                //o : o - 90
                new_position = Vector2::new(robot_position.x + 5.0, robot_position.y - 5.0);
                new_orientation = robot_angle - Rad32::QUARTER_TURN;
            }

            if robot_angle >= (Rad32::HALF_TURN + Rad32::QUARTER_TURN) && robot_angle < Rad32::FULL_TURN {
                // o entre 270 et 0
                // pos : x+5 ; y + 5
                //o : o + 90
                new_position = Vector2::new(robot_position.x + 5.0, robot_position.y + 5.0);
                new_orientation = robot_angle + Rad32::QUARTER_TURN;
            }

            return Some(Action {
                move_to: new_position,
                final_orientation: new_orientation,
                kick: false,
                dribbler: if info.robot_has_ball { 1.0 } else { 0.0 },
            });
        } else {
            return None;
        }
    } else {
        return None;
    } //TO DO
}
