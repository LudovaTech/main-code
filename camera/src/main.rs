use std::time::Duration;

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

use opencv::{
    core::{Mat, MatTrait, Size, Vector},
    highgui, imgproc,
    prelude::*,
    Result,
};

/// Définir le format NV12 via ses 4 octets.
/// Si libcamera fournit déjà une constante pour NV12, vous pouvez l'utiliser directement.
const PIXEL_FORMAT_NV12: PixelFormat =
    PixelFormat::new(u32::from_le_bytes([b'N', b'V', b'1', b'2']), 0);

fn main() -> Result<()> {
    // L'argument n'est plus le nom du fichier, mais simplement pour lancer l'application.
    let _arg = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "nv12_display".to_string());

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
    let width = cfgs.get(0).unwrap().get_size().width as i32;
    let height = cfgs.get(0).unwrap().get_size().height as i32;
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
    // Le format NV12 comporte 2 plans : le plan Y et le plan intercalé UV.
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

    // Pour OpenCV, nous devons construire une Mat à partir du buffer NV12.
    // La taille de l'image NV12 en mémoire est: hauteur totale = height + height/2
    // car le plan UV représente un quart de la taille pour U et V chacun, soit height/2 lignes.
    let total_height = height + height / 2;

    // Créer une Mat CV_8UC1 de taille total_height x width qui contient les données NV12.
    // Attention : OpenCV attend que la donnée soit contiguë en mémoire.
    let nv12_mat = Mat::new_rows_cols_with_data(
        total_height,
        width,
        &nv12_data, // Pass a reference to the Vec
    )?;

    // Convertir le NV12 en BGR pour affichage.
    // cvtColor supporte la conversion depuis COLOR_YUV2BGR_NV12.
    let mut bgr_mat = Mat::default();
    imgproc::cvt_color(&nv12_mat, &mut bgr_mat, imgproc::COLOR_YUV2BGR_NV12, 0)?;

    // Afficher l'image convertie dans une fenêtre.
    highgui::named_window("Image NV12", highgui::WINDOW_AUTOSIZE)?;
    highgui::imshow("Image NV12", &bgr_mat)?;
    println!("Appuyez sur une touche pour quitter...");
    highgui::wait_key(0)?;

    // Les ressources (caméra, buffers, etc.) sont libérées automatiquement.
    Ok(())
}
