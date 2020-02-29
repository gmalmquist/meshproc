use std::env;
use std::fs;

use futures::io::{Error, ErrorKind};
use meshproc::csg::{CsgObj, ToCsg};
use meshproc::geom::{Cube, Mesh, Polygon, Shape};
use meshproc::load_mesh_stl;
use meshproc::scad::{StlImport, ToScad};
use meshproc::scalar::FloatRange;
use meshproc::threed::{Pt3, Ray3, Vec3};
use meshproc::{csg, geom, scad, threed};
use std::io::Write;

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 1 {
        println!("An stl file is required as the first parameter.");
        std::process::exit(1);
    }

    let stl_path = &match get_path(&args[0]) {
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

    println!("Loaded: mesh has {} faces.", mesh.polygons.len());

    let bounds = &mesh.bounds;
    println!("Bounds: {} to {}", bounds.0, bounds.1);

    let mut csg = mesh.to_csg();
    for pillar in generate_internal_pillars(&mesh) {
        csg = csg.difference(&pillar.to_csg());
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
fn generate_internal_pillars(mesh: &Mesh) -> Vec<geom::Cube> {
    let mut pillars = vec![];

    let resolution = 5.;
    let clearance = 1.;
    let cell_width = resolution - clearance;

    let (mind, maxd) = &mesh.bounds;



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
            print!("Generating pillar {} x {}              \r", x, y);
            std::io::stdout().flush();
            let pillar = generate_pillar(x, y, cell_width, &mesh, clearance);
            if let Some(pillar) = pillar {
                pillars.push(pillar);
            }
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
    let (mind, maxd) = &mesh.bounds;

    let base_z = mind.z + clearance;

    let mut top_face = Polygon::new(vec![
        Pt3::new(base_x, base_y, base_z),
        Pt3::new(base_x + side, base_y, base_z),
        Pt3::new(base_x + side, base_y + side, base_z),
        Pt3::new(base_x, base_y + side, base_z),
    ]);

    let mut height: Option<f64> = None;
    for pt in top_face.points(1.) {
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

        return Some(Cube::new(
            Pt3::new(base_x, base_y, base_z),
            Vec3::new(side, side, height),
            false,
        ));
    }
    None
}

fn csg_test() {
    println!("Running CSG test.");

    let cube = geom::Cube::new(threed::Pt3::zero(), threed::Vec3::new(10., 20., 30.), false);
    let sphere = geom::Sphere::new(threed::Pt3::new(5., 10., 15.), 5.);

    let csg = cube.to_csg().union(&sphere.to_csg());
    match csg.render_stl("test2.stl") {
        Ok(output) => {}
        Err(e) => {
            println!("Error: {:#?}", e);
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
