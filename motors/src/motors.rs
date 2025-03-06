use radians::{Angle, Rad32};
use rppal::gpio::{Gpio, OutputPin};
use std::error::Error;
use crate::vector2::Vector2;

/// Valeurs par défaut des pins moteur
const FR_PWM: u8 = 0;
const FR_CWCCW: u8 = 0;
const FR_angle: f32 = 0.0;
const FL_PWM: u8 = 0;
const FL_CWCCW: u8 = 0;
const FL_angle: f32 = 0.0;
const BR_PWM: u8 = 0;
const BR_CWCCW: u8 = 0;
const BR_angle: f32 = 0.0;
const BL_PWM: u8 = 0;
const BL_CWCCW: u8 = 0;
const BL_angle: f32 = 0.0;

const PWM_DEFAULT_PERIOD: f64 = 1000.0;

/// Représente un moteur entrainant une roue
/// Ancien nom MotorMov
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
        let pwm = Gpio::new()?.get(pin_pwm)?.into_output();
        let cwccw = Gpio::new()?.get(pin_cwccw)?.into_output();
        Ok(Self {
            pwm,
            cwccw,
            angle_axis_kicker: angle_axis_kicker - Angle::QUARTER_TURN,
        })
    }

    #[inline]
    pub fn default_fr() -> Result<Self, Box<dyn Error>> {
        Self::new(FR_PWM, FR_CWCCW, Rad32::new(FR_angle))
    }

    #[inline]
    pub fn default_fl() -> Result<Self, Box<dyn Error>> {
        Self::new(FL_PWM, FL_CWCCW, Rad32::new(FL_angle))
    }

    #[inline]
    pub fn default_br() -> Result<Self, Box<dyn Error>> {
        Self::new(BR_PWM, BR_CWCCW, Rad32::new(BR_angle))
    }

    #[inline]
    pub fn default_bl() -> Result<Self, Box<dyn Error>> {
        Self::new(BL_PWM, BL_CWCCW, Rad32::new(BL_angle))
    }

    /// Change le PWM du moteur
    /// vitesse entre -1 et 1
    pub fn rotate(&mut self, speed: f32) -> () {
        if -1.0 <= speed && speed <= 1.0 {
            if 0.0 <= speed && speed <= 1.0 {
                self.cwccw.set_high();
            } else {
                self.cwccw.set_low();
            }
            if self
                .pwm
                .set_pwm_frequency(PWM_DEFAULT_PERIOD, speed.abs() as f64)
                .is_err()
            {
                // TODO log
            }
        } else {
            // TODO do nothing but log error
        }
    }

    pub fn stop(&mut self) -> () {
        self.rotate(0.0);
    }
}

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


    pub fn go_to(&mut self, to_local: Vector2, speed: f32, orientation: Rad32) -> () {
        const ABCSISSE: Vector2 = Vector2::new(1.0, 0.0);

        // The speed to be sent to the motors is calculated
        let to_local_angle = Rad32::new(to_local.angle(&ABCSISSE));
        let mut fr_speed = (to_local_angle - self.front_right.angle_axis_kicker).cos();
        let mut fl_speed = (to_local_angle - self.front_left.angle_axis_kicker).cos();
        let mut br_speed = (to_local_angle - self.back_right.angle_axis_kicker).cos();
        let mut bl_speed = (to_local_angle - self.back_left.angle_axis_kicker).cos();

        // The ratio to be used to calculate the speeds to be sent to the motors is calculated, taking into account the desired speed.
        let maximum = fr_speed.abs().max(fl_speed.abs().max(br_speed.abs().max(bl_speed.abs())));
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
            let rapport = (rotation.val() -1.0) / minimum;
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
