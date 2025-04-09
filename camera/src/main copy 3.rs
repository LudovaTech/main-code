use std::time::Duration;

use std::error::Error;

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

use opencv::core::Mat;

use image::{ImageBuffer, Luma, Rgba, RgbaImage};
use imageproc::contours::find_contours;
use imageproc::drawing::draw_cross_mut;

/// Définir le format NV12 via ses 4 octets.
/// Si libcamera fournit déjà une constante pour NV12, vous pouvez l'utiliser directement.
const PIXEL_FORMAT_NV12: PixelFormat =
    PixelFormat::new(u32::from_le_bytes([b'N', b'V', b'1', b'2']), 0);

fn main() -> Result<(), Result<(), Box<dyn Error>>> {
    // Création et récupération de la caméra
    let mgr = CameraManager::new().unwrap();
    let cameras = mgr.cameras();
    let cam = cameras.get(0).expect("Aucune caméra trouvée");
    println!(
        "Utilisation de la caméra : {}",
        *cam.properties().get::<properties::Model>().unwrap()
    );
    let mut cam = cam.acquire().expect("Impossible d'acquérir la caméra");

    // Génération de la configuration par défaut pour le rôle ViewFinder
    let mut cfgs = cam
        .generate_configuration(&[StreamRole::ViewFinder])
        .unwrap();
    cfgs.get_mut(0).unwrap().set_pixel_format(PIXEL_FORMAT_NV12);

    println!("Configuration générée : {:#?}", cfgs);

    // Validation de la configuration
    match cfgs.validate() {
        CameraConfigurationStatus::Valid => println!("Configuration de la caméra valide !"),
        CameraConfigurationStatus::Adjusted => {
            println!("La configuration a été ajustée : {:#?}", cfgs)
        }
        CameraConfigurationStatus::Invalid => {
            panic!("Erreur lors de la validation de la configuration")
        }
    }

    // Vérifier que le format est bien NV12
    assert_eq!(
        cfgs.get(0).unwrap().get_pixel_format(),
        PIXEL_FORMAT_NV12,
        "NV12 n'est pas supporté par la caméra"
    );

    // Récupération des dimensions de l'image à partir de la configuration
    let width = cfgs.get(0).unwrap().get_size().width;
    let height = cfgs.get(0).unwrap().get_size().height;
    println!("Dimensions de l'image: {}x{}", width, height);

    // Configuration de la caméra et allocation des buffers
    cam.configure(&mut cfgs)
        .expect("Impossible de configurer la caméra");
    let mut alloc = FrameBufferAllocator::new(&cam);
    let stream = cfgs.get(0).unwrap().stream().unwrap();
    let buffers = alloc.alloc(&stream).unwrap();
    println!("{} buffers alloués", buffers.len());

    // Conversion en MemoryMappedFrameBuffer pour l'accès aux octets
    let buffers = buffers
        .into_iter()
        .map(|buf| MemoryMappedFrameBuffer::new(buf).unwrap())
        .collect::<Vec<_>>();

    // Création des requêtes de capture et attachement des buffers
    let mut reqs = buffers
        .into_iter()
        .map(|buf| {
            let mut req = cam.create_request(None).unwrap();
            req.add_buffer(&stream, buf).unwrap();
            req
        })
        .collect::<Vec<_>>();

    // Le callback nous fournit la requête une fois la capture terminée
    let (tx, rx) = std::sync::mpsc::channel();
    cam.on_request_completed(move |req| {
        tx.send(req).unwrap();
    });

    cam.start(None).unwrap();

    // On ne met en file d'attente qu'un seul frame pour cet exemple
    cam.queue_request(reqs.pop().unwrap()).unwrap();

    println!("En attente d'une capture...");
    let req = rx
        .recv_timeout(Duration::from_secs(2))
        .expect("La capture a échoué");
    println!("Capture terminée!");
    println!("Métadonnées: {:#?}", req.metadata());

    // Récupération du framebuffer pour le flux
    let framebuffer: &MemoryMappedFrameBuffer<FrameBuffer> = req.buffer(&stream).unwrap();
    println!("Métadonnées du FrameBuffer: {:#?}", framebuffer.metadata());

    // Récupérer les données NV12.
    let nb_planes = framebuffer.data().len();
    println!("Nombre de plans dans le buffer : {}", nb_planes);

    // Concaténation des données effectives de chaque plan (bytes_used défini dans les métadonnées)
    let mut nv12_data = Vec::new();
    for (i, plane) in framebuffer.data().iter().enumerate() {
        let plane_info = framebuffer.metadata().unwrap().planes().get(i).unwrap();
        let bytes_used = plane_info.bytes_used as usize;
        println!("Plan {} : {} bytes utilisés.", i, bytes_used);
        nv12_data.extend_from_slice(&plane[..bytes_used]);
    }

    // Taille totale de l'image NV12 en mémoire
    let total_height = height + height / 2;

    // Créer une ImageBuffer à partir des données NV12
    let mut img_buffer = ImageBuffer::new(width, height); // Notez que nous utilisons uniquement 'height'

    // Remplir l'ImageBuffer avec les données NV12
for y in 0..height {
    for x in 0..width {
        let y_index = (y * width + x) as usize;
        let y_value = nv12_data[y_index];

        // Les valeurs U et V sont stockées dans le plan UV
        let uv_index = (height + (y / 2) * width + (x / 2) * 2) as usize; // Index pour U et V
        let u_value = nv12_data[uv_index];
        let v_value = nv12_data[uv_index + 1];

        // Convertir YUV en RGB
        let r = (y_value as f32 + 1.402 * (u_value as f32 - 128.0)).clamp(0.0, 255.0) as u8;
        let g = (y_value as f32 - 0.344136 * (u_value as f32 - 128.0) - 0.714136 * (v_value as f32 - 128.0)).clamp(0.0, 255.0) as u8;
        let b = (y_value as f32 + 1.772 * (u_value as f32 - 128.0)).clamp(0.0, 255.0) as u8;

        img_buffer.put_pixel(x, y, Rgba([r, g, b, 255]));
    }
}


    // Convertir l'image en niveaux de gris pour la détection de contours
    // let gray_image: ImageBuffer<Luma<u8>, Vec<u8>> = ImageBuffer::from_fn(width, height, |x, y| {
    //     let pixel = img_buffer.get_pixel(x, y);
    //     let gray_value =
    //         (0.299 * pixel[0] as f32 + 0.587 * pixel[1] as f32 + 0.114 * pixel[2] as f32) as u8;
    //     Luma([gray_value])
    // });

    // Appliquer un seuil pour détecter les pixels orange
    let mut mask = ImageBuffer::new(width.try_into().unwrap(), height.try_into().unwrap());
    for (y, x, pixel) in img_buffer.enumerate_pixels() {
        if y < height && x < width {
            // Ensure you are accessing the correct pixel values
            if pixel[0] >= 200 && pixel[1] < 150 && pixel[2] < 150 {
                mask.put_pixel(x, y, Luma([255])); // Pixel blanc pour la détection
            } else {
                mask.put_pixel(x, y, Luma([0])); // Pixel noir pour le reste
            }
        }
    }

    // Trouver les contours des objets détectés
    let contours = find_contours(&mask);

    // Dessiner une croix au centre de chaque balle détectée
    for contour in contours {
        // Calculer le rectangle englobant manuellement
        let (min_x, min_y) = contour
            .points
            .iter()
            .fold((u32::MAX, u32::MAX), |(min_x, min_y), point| {
                (min_x.min(point.x), min_y.min(point.y))
            });
        let (max_x, max_y) = contour.points.iter().fold((0, 0), |(max_x, max_y), point| {
            (max_x.max(point.x), max_y.max(point.y))
        });

        let center_x = (min_x + max_x) / 2;
        let center_y = (min_y + max_y) / 2;

        // Tracer une croix au centre
        draw_cross_mut(
            &mut img_buffer,
            Rgba([255, 0, 0, 255]),
            center_x as i32,
            center_y as i32,
        );
    }

    // Sauvegarder ou afficher l'image avec les croix
    img_buffer
        .save("output_image_with_crosses.png")
        .expect("Erreur lors de la sauvegarde de l'image");

    // Les ressources (caméra, buffers, etc.) sont libérées automatiquement.
    Ok(())
}
