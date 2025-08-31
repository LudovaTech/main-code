// use std::ffi::CStr;

// use libcamera_sys::*;

// fn main() {
//     unsafe {
//         let mgr = libcamera_camera_manager_create();

//         let version = CStr::from_ptr(libcamera_camera_manager_version(mgr)).to_str().unwrap();
//         println!("libcamera: {}", version);

//         libcamera_camera_manager_destroy(mgr);
//     }
// }


use libcamera::{camera_manager::CameraManager, logging::LoggingLevel, stream::StreamRole};
use std::fs::File;
use std::io::{self, Write};

fn main() -> io::Result<()> {
    // Crée ou écrase le fichier "camera_info.txt"
    let mut file = File::create("camera_info.txt")?;

    let mgr = CameraManager::new().unwrap();

    mgr.log_set_level("Camera", LoggingLevel::Error);

    let cameras = mgr.cameras();

    for i in 0..cameras.len() {
        let cam = cameras.get(i).unwrap();

        // Écrire les informations de la caméra dans le fichier
        writeln!(file, "Camera {}", i)?;
        writeln!(file, "ID: {}", cam.id())?;
        writeln!(file, "Properties: {:#?}", cam.properties())?;

        let config = cam.generate_configuration(&[StreamRole::ViewFinder]).unwrap();
        let view_finder_cfg = config.get(0).unwrap();
        writeln!(file, "Available formats: {:#?}", view_finder_cfg.formats())?;
        writeln!(file, "----------------------------------------")?;
    }

    Ok(())
}


// BGR888 car compatilité opencv