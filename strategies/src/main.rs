mod consts;
mod vector2;
use consts::*;
use radians::Rad32;
use vector2::{GlobalCoord, LocalCoord, Vector2};

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
    robot_position: Option<GlobalCoord>,

    /// angle du robot *en radians* par rapport à la droite notre goal / leur goal,
    /// vu depuis notre goal, augmente dans le sens des aiguilles d'une montre
    /// # Dépendances :
    /// - lidar analyzer complet
    robot_angle: Option<Rad32>,

    /// distance au mur le plus proche.
    /// Cette valeur est quasiment toujours calculée par lidar analyzer. Elle permet d'éviter de
    /// sortir du terrain y compris lorsque la détermination des murs (lidar analyzer complet) échoue.
    /// # Dépendances :
    /// - lidar analyzer dégradé
    nearest_wall_distance: Option<f32>,

    /// angle au mur le plus proche.
    /// Cette valeur est quasiment toujours calculée par lidar analyzer. Elle permet d'éviter de
    /// sortir du terrain y compris lorsque la détermination des murs (lidar analyzer complet) échoue.
    /// # Dépendances :
    /// - lidar analyzer dégradé
    nearest_wall_angle: Option<Rad32>,

    /// position *absolue* du robot ami (coéquipier) sur le terrain.
    /// # Dépendances :
    /// - bluetooth entre les deux robots
    /// - lidar analyzer du robot ami
    friend_position: Option<GlobalCoord>,

    /// angle du robot ami *en radians* par rapport à la droite notre goal / leur goal,
    /// # Dépendances :
    /// - bluetooth entre les deux robots
    /// - lidar analyzer du robot ami
    friend_angle: Option<Rad32>,

    /// position *relative* de la balle par rapport au robot,
    /// # Dépendances :
    /// - la caméra arrive à détecter la balle
    ball_relative_position: Option<LocalCoord>,

    /// position *absolue* de la balle calculée par le robot ami,
    /// Cette valeur sert surtout si la balle est trop loin pour être bien
    /// détectée par ce robot mais que l'autre ami est plus proche.
    /// # Dépendances :
    /// - bluetooth entre les deux robots
    /// - la caméra de l'autre robot arrive à détecter la balle
    friend_ball_position: Option<GlobalCoord>,

    robot_has_ball: bool,
    friend_has_ball: Option<bool>,

    /// tuple des 2 positions des robots adverses
    /// l'une des positions peut-être disponible et l'autre non.
    /// Si la position à l'emplacement 0 du tuple est indisponible, celle à la position
    /// l'est aussi.
    /// # Dépendances :
    /// - lidar analyzer NG
    enemy_positions: (Option<GlobalCoord>, Option<GlobalCoord>),
}

/// Représente l'ensemble des actions à faire par le robot qui ont été décidés par les stratégies
#[derive(Debug)]
struct Action {
    /// la position vers laquelle on voudrait aller
    move_to: LocalCoord, //TODO : prise en charge Action sans pos du robot
    /// l'orientation final que l'on voudra avoir lorsque l'on aura atteint cette position
    /// Si None, on garde la même qu'avant
    final_orientation: Option<Rad32>,
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
    } else {
        detecter_obstacles(choose_position(info));
    };
}

/// On teste si le robot est en dehors du terrain.
/// Si c'est le cas, on décide de la nouvelle direction où aller pour rejoindre le terrain
fn is_outside(info: &Informations) -> Option<Action> {
    if let Some(distance_to_nearest_wall) = info.nearest_wall_distance {
        if distance_to_nearest_wall > BORDER_LENGTH {
            return None;
        }

        //let mut future_local_position = LocalCoord(Vector2::new(0.0, 0.0));

        let future_local_position = if let Some(robot_angle) = info.robot_angle {
            // Comme on a l'angle du robot on calcule de combien revenir en arrière

            //-> bug pour 0; 90; 180 et 270  pt choisir vers cages adverses pour majorité des cas
            if robot_angle <= Rad32::QUARTER_TURN {
                // o entre 0 et 90
                // pos : x-5 ; y + 5
                //o : o - 90
                LocalCoord(Vector2::new(5.0, 5.0))
            } else if robot_angle > Rad32::QUARTER_TURN && robot_angle < Rad32::HALF_TURN {
                // o entre 90 et 180
                // pos : x-5 ; y - 5
                //o : o + 90
                LocalCoord(Vector2::new(-5.0, -5.0))
            } else if robot_angle >= Rad32::HALF_TURN
                && robot_angle < (Rad32::HALF_TURN + Rad32::QUARTER_TURN)
            {
                // o entre 180 et 270
                // pos : x+5 ; y - 5
                //o : o - 90
                LocalCoord(Vector2::new(5.0, -5.0))
            } else if robot_angle >= (Rad32::HALF_TURN + Rad32::QUARTER_TURN)
                && robot_angle < Rad32::FULL_TURN
            {
                // o entre 270 et 0
                // pos : x+5 ; y + 5
                //o : o + 90
                LocalCoord(Vector2::new(5.0, 5.0))
            } else {
                // TODO log error
                LocalCoord(Vector2::zeros())
            }
        } else {
            // TODO discussion
            LocalCoord(Vector2::zeros())
        };

        return Some(Action {
            move_to: future_local_position,
            final_orientation: None, // On garde la même orientation
            kick: false,
            dribbler: if info.robot_has_ball { 1.0 } else { 0.0 },
        });
    } else {
        // TODO log
        return None;
    }
}

fn choose_position(info: &Informations) -> Action {
    if let Some(robot_position) = info.robot_position {
        if info.robot_has_ball {
            let mut new_position = Vector2::new(0.0, 0.0);
            let mut new_orientation = Rad32::ZERO;
            let mut kick_decision = 0;

            if robot_position.y >= 70.0 {
                // coordonnées cages adverses  G : -53 ; 91.5    D : 53  ; 91.5 il faut que le robot soit en face des cages sinon changer orientation
                if robot_position.x > -53 && robot_position.x < 53 {
                    //pos = position actuelle
                    //o = orientation 0.0
                    //tirer

                    new_position = robot_position;
                    new_orientation = 0.0;
                    kick_decision = -1;
                } else if robot_position.x >= 53.0 {
                    let CdR = (robot_position - Vector2::new(53, 91.5)).length();
                    let CdD =
                        (Vector2::new(53, 91.5) - Vector2::new(53, robot_position.y)).length();
                    let R = ((Vector2::new(53, robot_position.y) - robot_position).length() / CdR)
                        .acos();

                    if R <= 45 {
                        //pos = pos actuelle
                        //o = 360 - 90 + R
                        //tirer
                        new_position = robot_position;
                        new_orientation = 0.0; //à changer
                        kick_decision = -1;
                    } else {
                        //pos = Vector2::new(50, robot_position.y),
                        //o = 0.0
                        //ne pas tirer
                        new_position = Vector2::new(50, robot_position.y);
                        new_orientation = 0.0;
                        kick_decision = 0;
                    }
                } else if robot_position.x <= -53 {
                    let CgR = (robot_position - Vector2::new(-53, 91.5)).length();
                    let CgD =
                        (Vector2::new(-53, 91.5) - Vector2::new(-53, robot_position.y)).length();
                    let R = ((Vector2::new(-53, robot_position.y) - robot_position).length() / CgR)
                        .acos();

                    if R <= 45 {
                        //pos = robot.position
                        //o = 90 - R
                        //tirer
                        new_position = robot_position;
                        new_orientation = 0.0; //à changer en pi - R
                        kick_decision = -1;
                    } else {
                        //pos = Vector2::new(-50, robot_position.y),
                        //o = 0.0
                        //ne pas tirer
                        new_position = Vector2::new(-50, robot_position.y);
                        new_orientation = 0.0;
                        kick_decision = 0;
                    }
                }
            }
            return Action {
                move_to: new_position,
                final_orientation: new_orientation,
                kick: kick_decision,
                dribbler: 0,
            };
        } else {
            
        }
    } else {
        if let Some(ball_pos) = info.ball_relative_position.or(info.friend_ball_position) {
            return Action {
                move_to: ball_pos,
                final_orientation: 0.0,
                kick: false,
                dribbler: 255,
            };
        } else {
            return Action {
                move_to: Vector2::new(0.0, 0.0), //à changer
                final_orientation: Rad32::new(0.0),
                kick: false,
                dribbler: 0,
            };
        }
    }
}

// fn detecter_obstacles(info: &Informations) -> Action {
//     //recup position dans Action de choose_position
//     if let Some(robot_position) = info.robot_position {
//         let action = choose_position(info);
//         let destination = action.move_to;
//         let mut position = Vector2::new(robot_position.x, robot_position.y);

//         if let Some(friend_position) = info.friend_position {
//             let x_r = robot_position.x;
//             let y_r = robot_position.y;
//             let x_o = friend_position.x;
//             let y_o = friend_position.y;
//             let x_d = destination.x;
//             let y_d = destination.y;

//             position = tester_obstacles(x_r, y_r, x_o, y_o, x_d, y_d);
//         }

//         if let Some(enemy_positions) = info.enemy_positions {
//             let x_r = robot_position.x;
//             let y_r = robot_position.y;
//             let x_o = enemy_positions.0.x;
//             let y_o = enemy_positions.0.y;
//             let x_d = destination.x;
//             let y_d = destination.y;

//             position = tester_obstacles(x_r, y_r, x_o, y_o, x_d, y_d);

//             let x_r = robot_position.x;
//             let y_r = robot_position.y;
//             let x_o = enemy_positions.1.x;
//             let y_o = enemy_positions.1.y;
//             let x_d = destination.x;
//             let y_d = destination.y;

//             position = tester_obstacles(x_r, y_r, x_o, y_o, x_d, y_d);
//         }

//         return Action {
//             move_to: position,
//             final_orientation: action.final_orientation,
//             kick: action.kick,
//             dribbler: action.dribbler,
//         };
//     } else {
//         return None;
//     }
// }

// fn tester_obstacles(x_r: f32, y_r: f32, x_o: f32, y_o: f32, x_d: f32, y_d: f32) -> Option<Action> {
//     let mut new_position = Vector2::new(0.0, 0.0);
//     if x_r = x_d {
//         //si |x_r - x_o| <= 9 && |y_o - y_d|
//         if (x_r - x_o).abs() <= 9 && (y_o - y_d).abs() <= 9 { //verif (y_o - y_d).abs() <= 9
//             new_position = Vector2::new(x_r, y_r + 5.0); //à changer pour que se soit obstacle + 9 vers le centre
//         } else {
//             new_position = Vector2::new(x_d, y_d);

//     } else {
//         let y_lo = calculer_eq_droite(x_r, y_r, x_d, y_d, x_o);
//         if (y_o - y_lo).abs() <= 9 {
//             new_position = Vector2::new(x_r, y_r + 5.0); //à changer pour que se soit obstacle + 9 vers le centre
//         } else {
//             new_position = Vector2::new(x_d, y_d);
//         }
//     }
// }

// fn calculer_eq_droite(xr: f32, yr: f32, xd: f32, yd: f32, xo: f32) -> (f32) {
//     let y = ((yd - yr) * xo + yr * yd - yd * yr) / (xd - xr);
//     (y)
// }
