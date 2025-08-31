use std::error::Error;

use libcamera::{camera_manager::CameraManager, stream::StreamRole};


// documentation of libcamera at : https://libcamera.org/guides/application-developer.html

fn main() -> Result<(), Box<dyn Error>>{
    let camera_manager = CameraManager::new().expect("cannot initiate camera manager");

    let unactive_camera = camera_manager.cameras().get(0).expect("No camera detected");
    let camera = unactive_camera.acquire().expect("unable to lock camera");
    println!("got camera {}", camera.id());

    // camera configuration

    let default_conf = camera.generate_configuration(StreamRole::ViewFinder);
    println!("conf : {:?}", default_conf);

    Ok(())
}