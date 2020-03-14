use std::fs;
use stl_io;

pub mod csg;
pub mod geom;
pub mod scad;
pub mod scalar;
pub mod threed;
pub mod mesh;

use crate::geom::Polygon;
use crate::threed::Pt3;
use crate::mesh::Mesh;

pub fn load_mesh_stl(path: &str) -> Result<Mesh, std::io::Error> {
    let file = fs::OpenOptions::new().read(true).open(path);
    if let Err(e) = file {
        return Err(e);
    }
    let mut file = file.unwrap();

    let stl = stl_io::read_stl(&mut file);
    match stl {
        Ok(stl) => Ok(Mesh::new(
            stl.vertices.iter()
                .map(|v| Pt3::new(v[0] as f64, v[1] as f64, v[2] as f64))
                .collect(),
            stl.faces.iter()
                .map(|face| face.vertices.iter()
                    .map(|i| *i)
                    .collect::<Vec<_>>())
                .collect(),
            Some(String::from(path)),
        )),
        Err(e) => Err(e),
    }
}
