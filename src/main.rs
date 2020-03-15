use std::env;
use std::fs;
use std::io::Write;
use std::sync::Arc;

use futures::executor;
use futures::io::{Error, ErrorKind};
use futures::task::SpawnExt;

use meshproc::{geom, threed};
use meshproc::csg::{CsgObj, ToCsg, BlenderCsgObj};
use meshproc::geom::{Cube, Polygon, Shape, FaceLike};
use meshproc::load_mesh_stl;
use meshproc::scalar::FloatRange;
use meshproc::threed::{Pt3, Ray3, Vec3};
use meshproc::mesh::Mesh;

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 1 {
        println!("An stl file is required as the first parameter.");
        std::process::exit(1);
    }

    match &args[0] as &str {
        "cube-normal-test" => return cube_normal_test(),
        "cube-mesh-test" => return cube_mesh_test(),
        _ => {}
    }

    let stl_path = &match get_path(&args[1]) {
        Ok(s) => s,
        Err(e) => {
            println!("Error canonicalizing path: {:#?}", e);
            std::process::exit(1);
        }
    };

    println!("Loading {}", stl_path);

    let mesh = load_mesh_stl(stl_path);
    if let Err(e) = mesh {
        println!("Unable to load mesh: {}", e);
        std::process::exit(1);
    }
    let mesh = mesh.unwrap();

    println!("Loaded: mesh has {} faces.", mesh.face_count());

    let bounds = &mesh.bounds;
    println!("Bounds: {} to {}", bounds.0, bounds.1);

    let mut csg = mesh.to_csg();
    let mesh = Arc::new(mesh);


    match &args[0] as &str {
        "internal-pillars" => {
            for pillar in generate_internal_pillars(Arc::clone(&mesh)) {
                csg = csg.difference(&pillar.to_csg());
            }
        },
        "internal-plateaus" => {
            for plateau in generate_plateau_supports(Arc::clone(&mesh)) {
                csg = csg.difference(&plateau.to_csg());
            }
        },
        "roundtrip" => {
            // no-op, we just leave it alone.
        },
        s => {
            eprintln!("Unknown command {}", s);
            return;
        }
    }


    match csg.render_stl("test3.stl") {
        Ok(()) => {
            println!("Done.");
        }
        Err(e) => {
            println!("Unable to render mesh: {}", e);
            std::process::exit(1);
        }
    }
}

fn generate_plateau_supports(mesh: Arc<Mesh>) -> Vec<Box<dyn ToCsg<BlenderCsgObj>>> {
    let clearance = 1.0;

    let up = Vec3::up();

    for face in mesh.faces() {
        if face.normal().dot(&up) < 0.99 {
            // Isn't horizontal.
            continue;
        }

        if face.centroid().z < mesh.bounds.0.z + clearance {
            // Is too close to the bottom of the mesh to require support.
            continue;
        }


    }
    unimplemented!();
}

fn cube_normal_test() {
    let cube = geom::Cube::new(Pt3::zero(), Vec3::new(10., 10., 10.), true);

    let mut csg = cube.to_csg();

    for face in cube.faces() {
        println!("Face: {}, {}", face.normal(), face.normal() * 10.);
        for point in face.points(1.) {
            csg = csg.union(
                &geom::Cube::new(point + (face.normal() * 3.), Vec3::new(0.5, 0.5, 0.5), true)
                    .to_csg(),
            );
        }
    }

    csg.render_stl("test-normals.stl").expect("Expected test normals to render.");
}

fn cube_mesh_test() {
    let cube = geom::Cube::new(Pt3::zero(), Vec3::new(10., 10., 10.), true);
    let mut csg = cube.mesh.to_csg();
    csg.render_stl("cube-mesh-test.stl").expect("Failed to render cube mesh stl.");
}

fn generate_internal_pillars(mesh: Arc<Mesh>) -> Vec<geom::Cube> {
    let mut pillars = vec![];

    let resolution = 12.;
    let clearance = 1.0;
    let cell_width = resolution - clearance;

    let (mind, maxd) = &mesh.bounds;

    let mut futures = vec![];
    let threadpool = executor::ThreadPoolBuilder::new()
        .create()
        .expect("Unable to create thread pool for pillar generation!");

    for x in FloatRange::from_step_size(
        mind.x + clearance,
        maxd.x - cell_width - clearance,
        resolution,
    ) {
        for y in FloatRange::from_step_size(
            mind.y + clearance,
            maxd.y - cell_width - clearance,
            resolution,
        ) {
            let mesh = Arc::clone(&mesh);
            let pillar_future = threadpool.spawn_with_handle(async move {
                eprint!("Generating pillar {} x {}              \r", x, y);
                std::io::stderr().flush().unwrap();
                generate_pillar(x, y, cell_width, &mesh, clearance)
            });
            if let Ok(pillar_future) = pillar_future {
                futures.push(pillar_future);
            }
        }
    }

    for pillar_future in futures {
        if let Some(pillar) = futures::executor::block_on(pillar_future) {
            pillars.push(pillar);
        }
    }

    println!("\nGenerated {} pillars.", pillars.len());
    pillars
}

fn generate_pillar(
    base_x: f64,
    base_y: f64,
    side: f64,
    mesh: &Mesh,
    clearance: f64,
) -> Option<geom::Cube> {
    let ray_spacing = 1.0;
    let (mind, _maxd) = &mesh.bounds;
    let base_z = mind.z + clearance;

    let bottom_face = Polygon::new(vec![
        Pt3::new(base_x, base_y, base_z),
        Pt3::new(base_x + side, base_y, base_z),
        Pt3::new(base_x + side, base_y + side, base_z),
        Pt3::new(base_x, base_y + side, base_z),
    ]);

    let mut height: Option<f64> = None;
    for pt in bottom_face.face().points(ray_spacing) {
        let hit = mesh.raycast(&Ray3::new(pt, Vec3::up()));
        if let Some(hit) = hit {
            if height.is_none() || height.unwrap() > hit.distance {
                height = Some(hit.distance);
            }
        }
    }

    if let Some(height) = height {
        let height = height - clearance;
        if height < clearance {
            // NB: We use the clearance as the minimum valid height, which seems reasonable
            // but may not be obvious.
            return None;
        }

        let cube = Cube::new(
            Pt3::new(base_x, base_y, base_z),
            Vec3::new(side, side, height),
            false,
        );

        let mut lowest_tangent_collision: Option<f64> = None;
        for face in cube.faces() {
            for point in face.points(ray_spacing) {
                let hit = mesh.raycast(&Ray3::new(point.clone(), face.normal()));
                let hit_height = if let Some(hit) = hit {
                    if hit.distance >= clearance {
                        continue; // This is fine.
                    }
                    hit.point.z - base_z
                } else {
                    // If we didn't hit anything, we're outside the mesh! This is just as bad as
                    // hitting something too close.
                    point.z - base_z
                };
                if lowest_tangent_collision.is_none()
                    || lowest_tangent_collision.unwrap() > hit_height
                {
                    lowest_tangent_collision = Some(hit_height);
                }
            }
        }
        if let Some(lowest_tangent_collision) = lowest_tangent_collision {
            let new_height = lowest_tangent_collision - clearance;
            if new_height < clearance {
                return None;
            }
            return Some(Cube::new(
                Pt3::new(base_x, base_y, base_z),
                Vec3::new(side, side, new_height),
                false,
            ));
        }

        return Some(cube);
    }
    None
}

fn csg_test() {
    println!("Running CSG test.");

    let cube = geom::Cube::new(threed::Pt3::zero(), threed::Vec3::new(10., 20., 30.), false);
    let sphere = geom::Sphere::new(threed::Pt3::new(5., 10., 15.), 5.);

    let csg = cube.to_csg().union(&sphere.to_csg());
    match csg.render_stl("test2.stl") {
        Ok(_output) => {}
        Err(e) => {
            eprintln!("Error: {:#?}", e);
            std::process::exit(1);
        }
    }
}

fn get_path<'a>(path: &'a str) -> std::io::Result<String> {
    let canonical_path = fs::canonicalize(path)?;
    let str_path = &canonical_path.to_str();
    match str_path {
        None => Err(Error::new(
            ErrorKind::InvalidData,
            "Unable to convert path to string.",
        )),
        Some(s) => Ok(s.to_string()),
    }
}
