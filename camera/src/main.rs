use std::error::Error;

use libcamera::{
    camera_manager::CameraManager,
    framebuffer::AsFrameBuffer,
    framebuffer_allocator::{FrameBuffer, FrameBufferAllocator},
    request::RequestStatus,
    stream::{Stream, StreamRole},
};

// documentation of libcamera at : https://libcamera.org/guides/application-developer.html

fn main() -> Result<(), Box<dyn Error>> {
    panic!("Hello crossing World!");
    let camera_manager = CameraManager::new().expect("cannot initiate camera manager");

    let cameras = camera_manager.cameras();
    let unactive_camera = cameras.get(0).expect("No camera detected");
    let camera = unactive_camera.acquire().expect("unable to lock camera");
    println!("got camera {}", camera.id());

    // camera configuration

    let mut default_conf = camera
        .generate_configuration(&[StreamRole::VideoRecording])
        .expect("no default conf");

    // match default_conf.validate() {
    //     CameraConfigurationStatus::Valid => println!("Configuration de la caméra valide !"),
    //     CameraConfigurationStatus::Adjusted => {
    //         println!("La configuration a été ajustée : {:#?}", default_conf)
    //     }
    //     CameraConfigurationStatus::Invalid => {
    //         panic!("Erreur lors de la validation de la configuration")
    //     }
    // }
    // let configuration = default_conf;
    // let stream = configuration
    //     .get(0)
    //     .expect("malformed config")
    //     .stream()
    //     .expect("unable to get stream from configuration");

    // // Capture images

    // // let's use default frame allocator for now

    // let allocator = FrameBufferAllocator::new(&camera);
    // // for cfg in configuration.get(0) {
    // //     allocator.alloc(cfg.stream())
    // // }
    // let buffers = allocator.alloc(&stream).expect("cannot allocate buffers");
    // let requests = buffers
    //     .iter()
    //     .for_each(|buffer| {
    //         let mut request = camera
    //             .create_request(None)
    //             .expect("unable to create request to camera");
    //         request
    //             .add_buffer(&stream, buffer)
    //             .expect("unable to add buffer");
    //         request
    //     })
    //     .collect::<Vec<_>>();

    // camera.on_request_completed(|request| {
    //     if request.status() != RequestStatus::Complete {
    //         return;
    //     }
    //     let frame_buffer: &FrameBuffer = request.buffer(&stream).expect("unable to get buffer");
    //     println!("metadata {:?}", frame_buffer.metadata());
    // });

    Ok(())
}
