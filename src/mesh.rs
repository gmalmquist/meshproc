use crate::threed::{Pt3, Vec3};
use crate::geom;
use crate::geom::HasVertices;

pub struct Mesh {
    pub vertices: Vec<Pt3>,
    pub face_loops: Vec<Vec<usize>>,
    face_normals: Vec<Vec3>,
}

impl Mesh {
    pub fn vertex(&self, index: usize) -> Option<&Pt3> {
        self.vertices.get(index)
    }

    pub fn face(&self, index: usize) -> Option<MeshFace> {
        if index < 0 || index >= self.face_loops.len() {
            return None;
        }
        Some(MeshFace {
            mesh: self,
            index,
        })
    }

    pub fn faces(&self) -> MeshFaceIter {
        MeshFaceIter { mesh: self, index: 0 }
    }

    pub fn recalculate_normals(&mut self) {
        self.face_normals.clear();
        for (index, lp) in self.face_loops.iter().enumerate() {
            if lp.len() < 2 {
                self.face_normals.push(Vec3::zero());
                continue;
            }
            let edge_one = &self.vertices[lp[1]] - &self.vertices[lp[0]];
            let edge_two = &self.vertices[lp[2]] - &self.vertices[lp[1]];
            self.face_normals.push(edge_two.cross(&edge_one).normalized());
        }
    }
}

pub struct MeshFace<'m> {
    mesh: &'m Mesh,
    index: usize,
}


impl<'a> MeshFace<'a> {}

impl<'a> geom::HasVertices for MeshFace<'a> {
    fn vertex(&self, index: usize) -> &Pt3 {
        &self.mesh.vertices[self.mesh.face_loops[self.index][index]]
    }

    fn vertex_count(&self) -> usize {
        self.mesh.face_loops[self.index].len()
    }

    fn edges(&self) -> geom::FaceEdgeIter {
        geom::FaceEdgeIter::new(self)
    }
}

pub struct MeshFaceIter<'a> {
    mesh: &'a Mesh,
    index: usize,
}

impl<'a> Iterator for MeshFaceIter<'a> {
    type Item = MeshFace<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let next = self.mesh.face(self.index);
        self.index += 1;
        next
    }
}