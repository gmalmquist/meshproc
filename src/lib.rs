use std::fs;
use stl_io;

pub mod csg;
pub mod geom;
pub mod scad;
pub mod scalar;
pub mod threed;
pub mod mesh;

use crate::geom::{Polygon, FaceLike};
use crate::threed::Pt3;
use crate::mesh::{Mesh, MeshFace};
use std::collections::{HashSet, VecDeque};

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

pub fn walk_faces<'a, F>(mesh: &Mesh, face: &'a MeshFace, mut visit: F) -> ()
    where F: FnMut(&MeshFace) -> VisitResult {
    let mut visited_faces: HashSet<usize> = HashSet::new();
    let mut visited_corners: HashSet<usize> = HashSet::new();
    let mut corner_frontier: VecDeque<usize> = VecDeque::new();
    for c in 0..face.vertex_count() {
        let c = face.corner(c);
        corner_frontier.push_front(c.corner_index());
    }

    while !corner_frontier.is_empty() {
        let c = corner_frontier.pop_back();
        if c.is_none() {
            break;
        }
        let c = c.unwrap();

        if visited_corners.contains(&c) {
            continue;
        }
        visited_corners.insert(c);

        let c = mesh.corner(c);
        let face = c.face_index();
        if visited_faces.contains(&face) {
            continue;
        }
        visited_faces.insert(face);
        let face = c.face();

        match (visit)(&face) {
            VisitResult::Continue => {
                // No-op.
            },
            VisitResult::Skip => {
                continue;
            },
            VisitResult::Stop => {
                break;
            },
        }

        corner_frontier.push_front(c.left().corner_index());
        corner_frontier.push_front(c.right().corner_index());
        corner_frontier.push_front(c.next().corner_index());
        corner_frontier.push_front(c.prev().corner_index());
    }
}

pub enum VisitResult {
    Continue,
    Skip,
    Stop,
}