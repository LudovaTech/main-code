
mod motors;
mod vector2;
use motors::Bogie;

fn main() {
    let mut bogie = Bogie::default().unwrap();
    bogie.front_right.rotate(0.5);
}
