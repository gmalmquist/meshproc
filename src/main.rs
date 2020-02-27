use std::env;
use std::fs;

use futures::io::{Error, ErrorKind};
use meshproc::csg::{CsgObj, ToCsg};
use meshproc::load_mesh_stl;
use meshproc::scad::{StlImport, ToScad};
use meshproc::{csg, geom, scad, threed};

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 1 {
        println!("An stl file is required as the first parameter.");
        std::process::exit(1);
    }

    if &args[0] == "csg-test" {
        csg_test();
        return;
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
    println!("Bounds: {:#?}", bounds);

    let sphere = geom::Sphere::new(
        (0.5 * bounds.0) + (0.5 * bounds.1),
        (bounds.1 - bounds.0).mag() / 2.0,
    );

    let scad = StlImport::new(stl_path)
        .to_scad()
        .difference(sphere.to_scad());

    println!("ScadCode:\n{}", scad.to_string());
    match scad.render("test1.stl") {
        Ok(output) => {
            println!("OpenScad returned {:#?}", output.status.code());
            println!(
                "STDOUT\n{}\n",
                String::from_utf8(output.stderr).unwrap_or(String::from("(n/a)"))
            );
            println!(
                "STDERR\n{}",
                String::from_utf8(output.stdout).unwrap_or(String::from("(n/a)"))
            );
        }
        Err(e) => {
            println!("Unable to render mesh: {}", e);
            std::process::exit(1);
        }
    }
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
