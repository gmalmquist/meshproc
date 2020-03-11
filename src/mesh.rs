use crate::threed::{Pt3, Vec3};
use crate::geom;
use crate::geom::HasVertices;

pub struct Mesh {
    pub vertices: Vec<Pt3>,
    pub face_loops: Vec<Vec<usize>>,
    face_normals: Vec<Vec3>,
    face_centroids: Vec<Pt3>,
    vertex_normals: Vec<Vec3>,
}

impl Mesh {
    pub fn new(vertices: Vec<Pt3>, face_loops: Vec<Vec<usize>>) -> Self {
        let mut new_loops = vec![];
        for (loop_index, lp) in face_loops.into_iter().enumerate() {
            if lp.len() < 3 {
                eprintln!("Dropping degenerate face loop {} (|v| = {})",
                         loop_index, lp.len());
                continue;
            }
            let mut valid_vertices = true;
            for v in &lp {
                if *v < 0 || *v >= vertices.len() {
                    valid_vertices = false;
                    eprintln!("Face loop {} has invalid vertex index {}!",
                             loop_index, v);
                }
            }
            if !valid_vertices {
                continue;
            }

            new_loops.push(lp);
        }
        let mut mesh = Self {
            vertices,
            face_loops: new_loops,
            face_normals: vec![],
            face_centroids: vec![],
            vertex_normals: vec![],
        };
        // NB: Maybe it makes more sense to do this lazily? But I think almost anything we might
        // want to do with a mesh will care about this data.
        mesh.recalculate_geometry();
        mesh
    }

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
            let edge_one = &self.vertices[lp[1]] - &self.vertices[lp[0]];
            let edge_two = &self.vertices[lp[2]] - &self.vertices[lp[1]];
            self.face_normals.push(edge_two.cross(&edge_one).normalized());
        }
    }

    pub fn recalculate_centroids(&mut self) {
        self.face_centroids.clear();
        for (index, lp) in self.face_loops.iter().enumerate() {
            let mut centroid = Pt3::zero();
            for vertex_index in lp {
                let v = self.vertices[*vertex_index];
                centroid.x += v.x;
                centroid.y += v.y;
                centroid.z += v.z;
            }
            centroid.x /= lp.len() as f64;
            centroid.y /= lp.len() as f64;
            centroid.z /= lp.len() as f64;
            self.face_centroids.push(centroid);
        }
    }

    pub fn recalculate_vertex_normals(&mut self) {
        self.vertex_normals.clear();

        for i in 0..self.vertices.len() {
            self.vertex_normals.push(Vec3::zero());
        }

        for (loop_index, lp) in self.face_loops.iter().enumerate() {
            for vertex_index in lp {
                // TODO: Might be good to overload +=, /=, etc on points and vectors to dry this up?
                self.vertex_normals[*vertex_index].x += self.face_normals[loop_index].x;
                self.vertex_normals[*vertex_index].y += self.face_normals[loop_index].y;
                self.vertex_normals[*vertex_index].z += self.face_normals[loop_index].z;
            }
        }

        for v in &mut self.vertex_normals {
            v.normalize();
        }
    }

    pub fn recalculate_geometry(&mut self) {
        self.recalculate_centroids();
        self.recalculate_normals();
        self.recalculate_vertex_normals();
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
