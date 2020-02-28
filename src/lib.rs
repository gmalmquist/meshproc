use std::fs;
use stl_io;

pub mod csg;
pub mod geom;
pub mod scad;
pub mod scalar;
pub mod threed;

use crate::geom::{Mesh, Polygon};
use crate::threed::{Pt3, Vec3};

pub fn load_mesh_stl(path: &str) -> Result<Mesh, std::io::Error> {
    let mut file = fs::OpenOptions::new().read(true).open(path);
    if let Err(e) = file {
        return Err(e);
    }
    let mut file = file.unwrap();

    let stl = stl_io::read_stl(&mut file);
    match stl {
        Ok(stl) => Ok(Mesh::new(
            path,
            stl.faces
                .iter()
                .map(|f| {
                    Polygon::new(
                        f.vertices
                            .iter()
                            .map(|v| {
                                let co = &stl.vertices[*v];
                                Pt3::from_tuple((co[0] as f64, co[1] as f64, co[2] as f64))
                            })
                            .collect(),
                    )
                })
                .collect(),
        )),
        Err(e) => Err(e),
    }
}
