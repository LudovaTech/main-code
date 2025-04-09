use std::time::Duration;
use std::{collections::HashMap, error::Error};

use imageproc::contours::Contour;
use libcamera::{
    camera::CameraConfigurationStatus,
    camera_manager::CameraManager,
    framebuffer::AsFrameBuffer,
    framebuffer_allocator::{FrameBuffer, FrameBufferAllocator},
    framebuffer_map::MemoryMappedFrameBuffer,
    pixel_format::PixelFormat,
    properties,
    stream::StreamRole,
};

use image::{ImageBuffer, Luma, Rgba, RgbaImage};
use imageproc::{
    contours::find_contours,
    drawing::{draw_filled_circle_mut, draw_hollow_circle_mut},
};
use imageproc::{drawing::draw_cross_mut, region_labelling::connected_components};

type RgbaBufferImage = ImageBuffer<Rgba<u8>, Vec<u8>>;
type LumaBufferImage = ImageBuffer<Luma<u8>, Vec<u8>>;

const PIXEL_FORMAT_NV12: PixelFormat =
    PixelFormat::new(u32::from_le_bytes([b'N', b'V', b'1', b'2']), 0);

// https://docs.rs/nokhwa-core/0.1.5/src/nokhwa_core/types.rs.html#1720-1724
#[inline]
fn yuyv444_to_rgb(y: i32, u: i32, v: i32) -> [u8; 3] {
    let c298 = (y - 16) * 298;
    let d = u - 128;
    let e = v - 128;
    let r = ((c298 + 409 * e + 128) >> 8).clamp(0, 255) as u8;
    let g = ((c298 - 100 * d - 208 * e + 128) >> 8).clamp(0, 255) as u8;
    let b = ((c298 + 516 * d + 128) >> 8).clamp(0, 255) as u8;

    [r, g, b]
}

#[inline]
fn yuyv444_to_rgba(y: i32, u: i32, v: i32) -> [u8; 4] {
    let [r, g, b] = yuyv444_to_rgb(y, u, v);
    [r, g, b, 255]
}

#[inline]
fn nv12_to_rgba_buffer(
    width: u32,
    height: u32,
    nv12_data: &Vec<u8>,
) -> RgbaBufferImage {
    ImageBuffer::from_fn(width, height, |x, y| {
        // Convertir NV12 en RGBA
        let frame_size = width * height;
        let y_index: usize = (y * width + x).try_into().unwrap();
        let uv_index: usize = (frame_size + (y / 2) * width + (x & !1))
            .try_into()
            .unwrap();

        let y_value = i32::from(nv12_data[y_index]);
        let u_value = i32::from(nv12_data[uv_index]);
        let v_value = i32::from(nv12_data[uv_index + 1]);

        // Créer le pixel RGBA
        Rgba(yuyv444_to_rgba(y_value, u_value, v_value)) // Image originale en RGBA
    })
}

#[inline]
fn create_color_mask(width: u32, height: u32, rgb_data: &RgbaBufferImage) -> LumaBufferImage {
    ImageBuffer::from_fn(width, height, |x, y| {
        let pixel = rgb_data.get_pixel(x, y);

        if (100..255).contains(&pixel.0[0])
            && (80..210).contains(&pixel.0[1])
            && (0..200).contains(&pixel.0[2])
        {
            Luma([255u8])
        } else {
            Luma([0u8])
        }
    })
}

#[inline]
fn draw_marker(rgb_data: &mut RgbaBufferImage, center: (i32, i32)) {
    draw_hollow_circle_mut(rgb_data, center, 20, Rgba([100, 100, 100, 255]));
    draw_hollow_circle_mut(rgb_data, center, 19, Rgba([100, 100, 100, 255]));
    draw_hollow_circle_mut(rgb_data, center, 18, Rgba([100, 100, 100, 255]));
    draw_hollow_circle_mut(rgb_data, center, 17, Rgba([100, 100, 100, 255]));
}

#[inline]
fn draw_contour(rgb_data: &mut RgbaBufferImage, contours: &Vec<Contour<u32>>) {
    for contour in contours.iter() {
        for point in contour.points.iter() {
            let pixel = rgb_data.get_pixel_mut(point.x, point.y);
            pixel.0 = [255, 255, 255, 255];
        }
    }
}

#[inline]
fn compute_centers(contours: &Vec<Contour<u32>>) -> Vec<(f32, f32)>{
    let mut centers: Vec<(f32, f32)> = Vec::new();

    for contour in contours.iter() {

        let mut sum_x = 0.0;
        let mut sum_y = 0.0;

        for point in contour.points.iter() {
            sum_x += point.x as f32;
            sum_y += point.y as f32;
        }

        let center_x = sum_x / contour.points.len() as f32;
        let center_y = sum_y / contour.points.len() as f32;

        centers.push((center_x, center_y));
    }
    centers
}


fn main() -> Result<(), Box<dyn Error>> {
    let mgr = CameraManager::new().unwrap();
    let cameras = mgr.cameras();
    let cam = cameras.get(0).expect("Aucune caméra trouvée");
    println!(
        "Utilisation de la caméra : {}",
        *cam.properties().get::<properties::Model>().unwrap()
    );
    let mut cam = cam.acquire().expect("Impossible d'acquérir la caméra");

    let mut cfgs = cam
        .generate_configuration(&[StreamRole::ViewFinder])
        .unwrap();
    cfgs.get_mut(0).unwrap().set_pixel_format(PIXEL_FORMAT_NV12);

    match cfgs.validate() {
        CameraConfigurationStatus::Valid => println!("Configuration de la caméra valide !"),
        CameraConfigurationStatus::Adjusted => {
            println!("La configuration a été ajustée : {:#?}", cfgs)
        }
        CameraConfigurationStatus::Invalid => {
            panic!("Erreur lors de la validation de la configuration")
        }
    }

    assert_eq!(
        cfgs.get(0).unwrap().get_pixel_format(),
        PIXEL_FORMAT_NV12,
        "NV12 n'est pas supporté par la caméra"
    );

    let width = cfgs.get(0).unwrap().get_size().width;
    let height = cfgs.get(0).unwrap().get_size().height;
    println!("Dimensions de l'image: {}x{}", width, height);

    cam.configure(&mut cfgs)
        .expect("Impossible de configurer la caméra");
    let mut alloc = FrameBufferAllocator::new(&cam);
    let stream = cfgs.get(0).unwrap().stream().unwrap();
    let buffers = alloc.alloc(&stream).unwrap();
    println!("{} buffers alloués", buffers.len());

    let buffers = buffers
        .into_iter()
        .map(|buf| MemoryMappedFrameBuffer::new(buf).unwrap())
        .collect::<Vec<_>>();

    let mut reqs = buffers
        .into_iter()
        .map(|buf| {
            let mut req = cam.create_request(None).unwrap();
            req.add_buffer(&stream, buf).unwrap();
            req
        })
        .collect::<Vec<_>>();

    let (tx, rx) = std::sync::mpsc::channel();
    cam.on_request_completed(move |req| {
        tx.send(req).unwrap();
    });

    cam.start(None).unwrap();
    cam.queue_request(reqs.pop().unwrap()).unwrap();

    println!("En attente d'une capture...");
    let req = rx
        .recv_timeout(Duration::from_secs(2))
        .expect("La capture a échoué");
    println!("Capture terminée!");
    println!("Métadonnées: {:#?}", req.metadata());

    let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> = req.buffer(&stream).unwrap();
    println!("Métadonnées du FrameBuffer: {:#?}", framebuffer.metadata());

    let nb_planes = framebuffer.data().len();
    println!("Nombre de plans dans le buffer : {}", nb_planes);

    let mut nv12_data = Vec::new();
    for (i, plane) in framebuffer.data().iter().enumerate() {
        let plane_info = framebuffer.metadata().unwrap().planes().get(i).unwrap();
        let bytes_used = plane_info.bytes_used as usize;
        println!("Plan {} : {} bytes utilisés.", i, bytes_used);
        nv12_data.extend_from_slice(&plane[..bytes_used]);
    }

    let mut rgb_data: RgbaBufferImage =
        nv12_to_rgba_buffer(width, height, &nv12_data);

    let mask = create_color_mask(width, height, &rgb_data);

    let mut contours = find_contours::<u32>(&mask);

    contours.retain(|e| e.points.len() > 30); // TODO: paramètre, on filtre pour éviter les zones trop petites

    let centers = compute_centers(&contours);

    for (x, y) in centers.iter() {
        draw_marker(&mut rgb_data, (x.round() as i32, y.round() as i32));
    }

    draw_contour(&mut rgb_data, &contours);

    // Sauvegarder l'image résultante
    rgb_data
        .save("output_image_with_mask.png")
        .expect("Erreur lors de la sauvegarde de l'image");

    Ok(())
}
