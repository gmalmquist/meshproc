use crate::threed::{Pt3, Vec3, Ray3};
use crate::geom;
use crate::geom::{HasVertices, Shape};
use std::f64::INFINITY;

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
                if *v >= vertices.len() {
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
        if index >= self.face_loops.len() {
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

    pub fn face_normal(&self, index: usize) -> Option<&Vec3> {
        self.face_normals.get(index)
    }

    pub fn vertex_normal(&self, index: usize) -> Option<&Vec3> {
        self.vertex_normals.get(index)
    }

    pub fn face_centroid(&self, index: usize) -> Option<&Pt3> {
        self.face_centroids.get(index)
    }

    pub fn recalculate_normals(&mut self) {
        self.face_normals.clear();
        for (_index, lp) in self.face_loops.iter().enumerate() {
            let edge_one = &self.vertices[lp[1]] - &self.vertices[lp[0]];
            let edge_two = &self.vertices[lp[2]] - &self.vertices[lp[1]];
            self.face_normals.push(edge_two.cross(&edge_one).normalized());
        }
    }

    pub fn recalculate_centroids(&mut self) {
        self.face_centroids.clear();
        for (_index, lp) in self.face_loops.iter().enumerate() {
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

        for _i in 0..self.vertices.len() {
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

impl geom::Shape for Mesh {
    fn raycast(&self, ray: &Ray3) -> Option<geom::RaycastHit> {
        let mut closest_hit: Option<geom::RaycastHit> = None;
        for poly in self.faces() {
            let poly_hit = poly.raycast(ray);
            if let Some(poly_hit) = poly_hit {
                closest_hit = match closest_hit {
                    None => Some(poly_hit),
                    Some(closest_hit) => {
                        if closest_hit.distance < poly_hit.distance {
                            Some(closest_hit)
                        } else {
                            Some(poly_hit)
                        }
                    }
                };
            }
        }
        closest_hit
    }

    fn signed_distance(&self, pt: Pt3) -> f64 {
        let mut distance = INFINITY;
        for poly in self.faces() {
            let dist = poly.signed_distance(pt);
            if dist < distance {
                distance = dist;
            }
        }
        distance
    }
}

pub struct MeshFace<'m> {
    mesh: &'m Mesh,
    index: usize,
}

impl<'a> MeshFace<'a> {
    pub fn plane(&self) -> geom::Plane {
        geom::Plane::new(self.centroid(), self.normal())
    }

    pub fn contains(&self, pt: Pt3) -> bool {
        let normal = self.normal();
        let centroid = self.centroid();
        for edge in self.edges() {
            let v = edge.vector();
            let edge_normal = v ^ normal;
            let pt_side = edge_normal * (pt - edge.src) >= 0.0;
            let centroid_side = edge_normal * (centroid - edge.src) >= 0.0;
            if pt_side != centroid_side {
                return false;
            }
        }
        true
    }
}

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

    fn normal(&self) -> Vec3 {
        self.mesh.face_normal(self.index).expect("Face normal wasn't calculated").clone()
    }

    fn centroid(&self) -> Pt3 {
        self.mesh.face_centroid(self.index).expect("Face centroid wasn't calculated").clone()
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

impl<'a> geom::Shape for MeshFace<'a> {
    fn raycast(&self, ray: &Ray3) -> Option<geom::RaycastHit> {
        let hit_on_plane = self.plane().raycast(ray);
        return match hit_on_plane {
            Some(hit) => {
                if self.contains(hit.point) {
                    Some(hit)
                } else {
                    None
                }
            }
            None => None,
        };
    }

    fn signed_distance(&self, pt: Pt3) -> f64 {
        let plane = self.plane();
        let sign = if plane.normal * (pt - plane.origin) >= 0.0 {
            1.0
        } else {
            -1.0
        };

        // If the projection of the point onto the plane of this polygon is contained within the
        // boundaries of this polygon, that projection will be the closest point.
        let projection = self.plane().project(pt);
        if self.contains(projection) {
            return sign * (projection - pt).mag();
        }

        // If we're outside the bounds of the polygon, our distance to it is the closest distance
        // to any of its edges.
        let mut distance = 0.0;
        for edge in self.edges() {
            let edge_distance = edge.distance(pt);
            if edge_distance < distance {
                distance = edge_distance;
            }
        }

        distance * sign
    }
}
