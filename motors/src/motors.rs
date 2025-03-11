use crate::vector2::Vector2;
use radians::{Angle, Rad32};
use rppal::gpio::{Gpio, OutputPin};
use std::error::Error;
use std::thread::sleep;
use std::time::{Duration, Instant};
use tracing::{error, info, instrument, warn};

/// Valeurs par défaut des pins moteur
const FR_PWM: u8 = 23;
const FR_CWCCW: u8 = 24;
const FR_ANGLE: f32 = -0.698132;
const FL_PWM: u8 = 27;
const FL_CWCCW: u8 = 22;
const FL_ANGLE: f32 = 0.698132;
const BR_PWM: u8 = 17;
const BR_CWCCW: u8 = 5;
const BR_ANGLE: f32 = -2.44346;
const BL_PWM: u8 = 16;
const BL_CWCCW: u8 = 4;
const BL_ANGLE: f32 = 2.44346;

/// fréquence par défaut du PWM
const PWM_DEFAULT_PERIOD: f64 = 10000.0;

/// pins par défaut pour le kicker et le dribbler
const DRIBBLER_PWM: u8 = 0;
const DRIBBLER_CWCCW: u8 = 0;
const KICKER_PIN1: u8 = 0;
const KICKER_PIN2: u8 = 0;

/// temps min entre deux kicks
const TIME_BETWEEN_KICK: Duration = Duration::from_secs(2);
/// temps d'un kick (temps où on donne de l'énergie aux pins)
const KICK_TIME: Duration = Duration::from_millis(40);

/// Représente un moteur entrainant une roue
/// Ancien nom MotorMov
#[derive(Debug)]
pub struct Wheel {
    pwm: OutputPin,
    cwccw: OutputPin,
    pub angle_axis_kicker: Rad32,
}

impl Wheel {
    pub fn new(
        pin_pwm: u8,
        pin_cwccw: u8,
        angle_axis_kicker: Rad32,
    ) -> Result<Self, Box<dyn Error>> {
        let pwm = Gpio::new()?.get(pin_pwm)?.into_output_high();
        let cwccw = Gpio::new()?.get(pin_cwccw)?.into_output_high();
        Ok(Self {
            pwm,
            cwccw,
            angle_axis_kicker: angle_axis_kicker - Angle::QUARTER_TURN,
        })
    }

    #[inline]
    pub fn default_fr() -> Result<Self, Box<dyn Error>> {
        Self::new(FR_PWM, FR_CWCCW, Rad32::new(FR_ANGLE))
    }

    #[inline]
    pub fn default_fl() -> Result<Self, Box<dyn Error>> {
        Self::new(FL_PWM, FL_CWCCW, Rad32::new(FL_ANGLE))
    }

    #[inline]
    pub fn default_br() -> Result<Self, Box<dyn Error>> {
        Self::new(BR_PWM, BR_CWCCW, Rad32::new(BR_ANGLE))
    }

    #[inline]
    pub fn default_bl() -> Result<Self, Box<dyn Error>> {
        Self::new(BL_PWM, BL_CWCCW, Rad32::new(BL_ANGLE))
    }

    /// Change le PWM du moteur
    /// vitesse entre -1 et 1
    #[instrument]
    pub fn rotate(&mut self, speed: f32) -> () {
        if -1.0 <= speed && speed <= 1.0 {
            if 0.0 <= speed && speed <= 1.0 {
                self.cwccw.set_high();
            } else {
                self.cwccw.set_low();
            }
            if self
                .pwm
                .set_pwm_frequency(PWM_DEFAULT_PERIOD, 1.0 - speed.abs() as f64)
                .is_err()
            {
                error!("Attrapé. Impossible de changer la valeur du PWM moteur, la vitesse du moteur reste inchangée.");
            }
        } else {
            error!("Attrapé. Valeur de vitesse '{speed}' incorrecte.");
        }
    }

    pub fn stop(&mut self) -> () {
        self.rotate(0.0);
    }
}

#[derive(Debug)]
pub struct Bogie {
    pub front_right: Wheel,
    pub front_left: Wheel,
    pub back_right: Wheel,
    pub back_left: Wheel,
}

impl Bogie {
    pub fn default() -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            front_right: Wheel::default_fr()?,
            front_left: Wheel::default_fl()?,
            back_left: Wheel::default_br()?,
            back_right: Wheel::default_bl()?,
        })
    }

    pub fn full_stop(&mut self) -> () {
        self.front_right.stop();
        self.front_left.stop();
        self.back_right.stop();
        self.back_left.stop();
    }

    #[instrument]
    pub fn go_to(&mut self, to_local: Vector2, speed: f32, orientation: Rad32) -> () {
        const ABCSISSE: Vector2 = Vector2::new(1.0, 0.0);

        // The speed to be sent to the motors is calculated
        let to_local_angle = Rad32::new(to_local.angle(&ABCSISSE));
        let mut fr_speed = (to_local_angle - self.front_right.angle_axis_kicker).cos();
        let mut fl_speed = (to_local_angle - self.front_left.angle_axis_kicker).cos();
        let mut br_speed = (to_local_angle - self.back_right.angle_axis_kicker).cos();
        let mut bl_speed = (to_local_angle - self.back_left.angle_axis_kicker).cos();

        // The ratio to be used to calculate the speeds to be sent to the motors is calculated, taking into account the desired speed.
        let maximum = fr_speed
            .abs()
            .max(fl_speed.abs().max(br_speed.abs().max(bl_speed.abs())));
        let rapport = speed / maximum;

        // Speeds are recalculated taking into account the desired speed and
        // Sends speeds to motors
        fr_speed *= rapport;
        fl_speed *= rapport;
        br_speed *= rapport;
        bl_speed *= rapport;

        let minimum = fr_speed.min(fl_speed.min(br_speed.min(bl_speed)));
        let rotation = orientation * speed * 0.6; // TODO Why 0.6 ?

        if minimum - rotation.val() < -1.0 {
            let rapport = (rotation.val() - 1.0) / minimum;
            fr_speed *= rapport;
            fl_speed *= rapport;
            br_speed *= rapport;
            bl_speed *= rapport;
        }

        self.front_right.rotate(fr_speed - rotation.val());
        self.front_left.rotate(fl_speed - rotation.val());
        self.back_right.rotate(br_speed - rotation.val());
        self.back_left.rotate(bl_speed - rotation.val());
    }
}

/// S'occupe du dribler et du kicker
/// ancien DriblerKicker
#[derive(Debug)]
struct BallControl {
    dribbler: Wheel,
    kicker1: OutputPin,
    kicker2: OutputPin,
    last_kick_time: Option<Instant>,
}

impl BallControl {
    pub fn new(
        pin_dribbler_pwm: u8,
        pin_dribbler_cwccw: u8,
        pin_kicker1: u8,
        pin_kicker2: u8,
    ) -> Result<Self, Box<dyn Error>> {
        let kicker1 = Gpio::new()?.get(pin_kicker1)?.into_output_low();
        let kicker2 = Gpio::new()?.get(pin_kicker2)?.into_output_low();
        Ok(Self {
            dribbler: Wheel::new(pin_dribbler_pwm, pin_dribbler_cwccw, Rad32::ZERO)?,
            kicker1,
            kicker2,
            last_kick_time: None,
        })
    }

    pub fn default() -> Result<Self, Box<dyn Error>> {
        Self::new(DRIBBLER_PWM, DRIBBLER_CWCCW, KICKER_PIN1, KICKER_PIN2)
    }

    pub fn dribble(&mut self, speed: f32) {
        self.dribbler.rotate(speed);
    }

    pub fn kick(&mut self) {
        let now = Instant::now();
        if let Some(last_kick_time) = self.last_kick_time {
            if (last_kick_time - now) > TIME_BETWEEN_KICK {
                self.last_kick_time = Some(now);
                self.perform_kick();
            }
        } else {
            self.last_kick_time = Some(now);
            self.perform_kick();
        }
    }

    fn perform_kick(&mut self) {
        self.kicker1.set_high();
        self.kicker2.set_high();
        sleep(KICK_TIME); // TODO : C'est pas bien d'arrêter tout le code pour le kick
        self.kicker1.set_low();
        self.kicker2.set_low();
    }
}
