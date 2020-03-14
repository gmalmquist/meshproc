use crate::scalar::FloatRange;
use crate::threed::{Basis3, Frame3, Pt3, Ray3, Vec3};
use crate::mesh::Mesh;
use std::f64::INFINITY;

pub trait Shape {
    fn raycast(&self, ray: &Ray3) -> Option<RaycastHit>;
    fn signed_distance(&self, pt: Pt3) -> f64;
}

pub struct RaycastHit {
    pub point: Pt3,
    pub distance: f64,
    pub normal: Vec3,
}

pub struct Plane {
    pub origin: Pt3,
    pub normal: Vec3,
}

impl Plane {
    pub fn new(origin: Pt3, normal: Vec3) -> Self {
        Self { origin, normal }
    }

    pub fn project(&self, pt: Pt3) -> Pt3 {
        return (pt - self.origin).off_axis(&self.normal) + self.origin;
    }
}

impl Shape for Plane {
    fn raycast(&self, ray: &Ray3) -> Option<RaycastHit> {
        // (Q+tD-O)*N = 0
        // (Q-O)*N + t D*N = 0
        // t D*N = -(Q-O)*N
        // t = -(Q-O)*N / D*N
        let dir_on_normal = ray.direction * self.normal;
        if dir_on_normal == 0.0 {
            return None;
        }
        let t = (self.origin - ray.origin) * self.normal / dir_on_normal;
        if t < 0.0 {
            return None;
        }
        Some(RaycastHit {
            point: ray.at(t),
            distance: t,
            normal: self.normal,
        })
    }

    fn signed_distance(&self, pt: Pt3) -> f64 {
        return (pt - self.origin) * self.normal;
    }
}

pub trait HasVertices {
    fn vertex(&self, index: usize) -> &Pt3;
    fn vertex_count(&self) -> usize;
    fn edges(&self) -> FaceEdgeIter;

    fn normal(&self) -> Vec3 {
        let edge_one = self.vertex(1) - self.vertex(0);
        let edge_two = self.vertex(2) - self.vertex(1);
        edge_two.cross(&edge_one).normalized()
    }

    fn centroid(&self) -> Pt3 {
        Pt3::centroid(&(0..self.vertex_count())
            .collect::<Vec<_>>()
            .iter()
            .map(|&i| self.vertex(i))
            .collect())
    }
}

pub struct Face<'a> {
    pub vertices: Vec<&'a Pt3>,
}

impl<'a> Face<'a> {
    pub fn new(vertices: Vec<&'a Pt3>) -> Self {
        if vertices.len() < 3 {
            panic!("Cannot create a face with less than three vertices!");
        }
        Self {
            vertices,
        }
    }
}

impl<'a> HasVertices for Face<'a> {
    fn vertex(&self, index: usize) -> &Pt3 {
        self.vertices[index]
    }

    fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    fn edges(&self) -> FaceEdgeIter {
        FaceEdgeIter::new(self)
    }
}

pub struct FaceEdgeIter<'a> {
    face: &'a dyn HasVertices,
    edge_index: usize,
}

impl<'a> FaceEdgeIter<'a> {
    pub fn new(face: &'a dyn HasVertices) -> Self {
        Self {
            face,
            edge_index: 0,
        }
    }
}

impl<'a> Iterator for FaceEdgeIter<'a> {
    type Item = Edge<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.edge_index >= self.face.vertex_count() {
            return None;
        }
        let i = self.edge_index;
        self.edge_index += 1;
        Some(Edge {
            src: self.face.vertex(i),
            dst: self.face.vertex((i + 1) % self.face.vertex_count()),
        })
    }
}

pub struct Polygon {
    pub points: Vec<Pt3>,
    pub normal: Vec3,
    pub centroid: Pt3,
}

impl Polygon {
    pub fn new(points: Vec<Pt3>) -> Self {
        if points.len() < 3 {
            panic!(
                "Not enough points for polygon (need at least 3, got {}).",
                points.len()
            );
        }
        let face = Face::new(points.iter().collect());
        let normal = face.normal();
        let centroid = face.centroid();
        Self {
            points,
            normal,
            centroid,
        }
    }

    pub fn plane(&self) -> Plane {
        return Plane::new(self.centroid, self.normal);
    }

    pub fn contains(&self, pt: Pt3) -> bool {
        for edge in self.edges() {
            let v = edge.vector();
            let edge_normal = v ^ self.normal;
            let pt_side = edge_normal * (pt - edge.src) >= 0.0;
            let centroid_side = edge_normal * (self.centroid - edge.src) >= 0.0;
            if pt_side != centroid_side {
                return false;
            }
        }
        true
    }

    pub fn points<'a>(&'a self, resolution: f64) -> FacePointIter<'a> {
        let basis = Basis3::from_normal(self.normal);
        let frame = Frame3::new(self.centroid, basis);

        let mut min = (None, None);
        let mut max = (None, None);
        for pt in &self.points {
            let local = frame.project(pt);
            if min.0.is_none() || min.0.unwrap() < local.i {
                min.0 = Some(local.i);
            }
            if max.0.is_none() || max.0.unwrap() > local.i {
                max.0 = Some(local.i);
            }
            if min.1.is_none() || min.1.unwrap() < local.j {
                min.1 = Some(local.j);
            }
            if max.1.is_none() || max.1.unwrap() > local.j {
                max.1 = Some(local.j);
            }
        }

        let min = (min.0.unwrap(), min.1.unwrap());
        let max = (max.0.unwrap(), max.1.unwrap());

        let i_values = FloatRange::from_step_size(min.0, max.0, resolution).collect();
        let j_values = FloatRange::from_step_size(min.1, max.1, resolution).collect();

        FacePointIter {
            polygon: &self,
            frame,
            i_values,
            j_values,
            index: 0,
        }
    }
}

impl HasVertices for Polygon {
    fn vertex(&self, index: usize) -> &Pt3 {
        &self.points[index]
    }

    fn vertex_count(&self) -> usize {
        self.points.len()
    }

    fn edges(&self) -> FaceEdgeIter {
        FaceEdgeIter::new(self)
    }
}

pub struct FacePointIter<'a> {
    polygon: &'a Polygon,
    frame: Frame3,
    i_values: Vec<f64>,
    j_values: Vec<f64>,
    index: usize,
}

impl<'a> Iterator for FacePointIter<'a> {
    type Item = Pt3;

    fn next(&mut self) -> Option<Self::Item> {
        let i_index = self.index / self.i_values.len();
        let j_index = self.index % self.i_values.len();
        self.index += 1;
        if i_index >= self.i_values.len() || j_index >= self.j_values.len() {
            return None;
        }
        let local = self
            .frame
            .local(self.i_values[i_index], self.j_values[j_index], 0.);
        Some(self.frame.unproject(&local))
    }
}

impl Shape for Polygon {
    fn raycast(&self, ray: &Ray3) -> Option<RaycastHit> {
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
        let sign = if self.normal * (pt - self.centroid) >= 0.0 {
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

pub struct Edge<'a> {
    pub src: &'a Pt3,
    pub dst: &'a Pt3,
}

impl<'a> Edge<'a> {
    pub fn vector(&self) -> Vec3 {
        return self.dst - self.src;
    }

    pub fn distance(&self, pt: Pt3) -> f64 {
        let frame = Frame3::new(self.src.clone(), Basis3::new1(self.vector()));
        let mut local = frame.project(&pt);
        if local.i > 0.0 && local.i < 1.0 {
            // Closest point is on the interior of the edge, so return the distance from that.
            // Zero out orthogonal components to get the closest point.
            local.j = 0.0;
            local.k = 0.0;
            return (local.unproject() - pt).mag();
        }

        let src_dist2 = (pt - self.src).mag2();
        let dst_dist2 = (pt - self.dst).mag2();
        return if src_dist2 < dst_dist2 {
            src_dist2.sqrt()
        } else {
            dst_dist2.sqrt()
        };
    }
}

pub struct Cube {
    pub center: Pt3,
    pub dimensions: Vec3,
    pub mesh: Mesh,
}

impl Cube {
    pub fn new(origin: Pt3, dimensions: Vec3, centered: bool) -> Self {
        let center: Pt3 = if centered {
            origin
        } else {
            origin + dimensions / 2.0
        };

        let right = dimensions.on_axis(&Vec3::right()) / 2.;
        let forward = dimensions.on_axis(&Vec3::forward()) / 2.;
        let up = dimensions.on_axis(&Vec3::up()) / 2.;

        let push_vert = |v: Pt3, verts: &mut Vec<Pt3>| {
            verts.push(v);
            verts.len()
        };

        let mut vertices = vec![];
        let rfu = (push_vert)((right + forward + up) + center, &mut vertices);
        let rfd = (push_vert)((right + forward - up) + center, &mut vertices);
        let rbd = (push_vert)((right - forward - up) + center, &mut vertices);
        let rbu = (push_vert)((right - forward + up) + center, &mut vertices);
        let lfu = (push_vert)((-right + forward + up) + center, &mut vertices);
        let lfd = (push_vert)((-right + forward - up) + center, &mut vertices);
        let lbd = (push_vert)((-right - forward - up) + center, &mut vertices);
        let lbu = (push_vert)((-right - forward + up) + center, &mut vertices);

        let faces = vec![
            vec![rfu, rbu, lbu, lfu],
            vec![rfd, lfd, lbd, rbd],
            vec![lfu, lbu, lbd, lfd],
            vec![rfu, rfd, rbd, rbu],
            vec![rfu, lfu, lfd, rfd],
            vec![lbu, rbu, rbd, lbd],
        ];

        let mesh = Mesh::new(vertices, faces, None);
        Self {
            center,
            dimensions,
            mesh,
        }
    }
}

impl Shape for Cube {
    fn raycast(&self, ray: &Ray3) -> Option<RaycastHit> {
        self.mesh.raycast(ray)
    }

    fn signed_distance(&self, pt: Pt3) -> f64 {
        self.mesh.signed_distance(pt)
    }
}

pub struct Sphere {
    pub center: Pt3,
    pub radius: f64,
}

impl Sphere {
    pub fn new(center: Pt3, radius: f64) -> Sphere {
        return Sphere { center, radius };
    }
}

impl Shape for Sphere {
    fn raycast(&self, ray: &Ray3) -> Option<RaycastHit> {
        // Distance from (ray origin + direction * time) to center of sphere = radius.
        // ||Q+Dt - C|| = r
        // ||CQ + Dt|| = r
        // ||CQ + Dt||**2 = r**2
        // A vector's squared magnitude is equal to its dot-product with itself.
        // (CQ + Dt)*(CQ + Dt) = r**2
        // FOIL
        // CQ*CQ + CQ*Dt + Dt*CQ + Dt*Dt = r**2
        // CQ*CQ + 2 CQ*D t + D*D t**2 - r**2 = 0
        // Quadratic formula
        // a = D*D (t**2)
        // b = 2 CQ*D (t)
        // c = CQ*CQ - r**2
        // t = (-b +|- sqrt(bb - 4ac))/(2a)
        let a = ray.direction.mag2();
        if a == 0. {
            // Ray's direction has 0 magnitude, and will never reach the sphere.
            return None;
        }

        let cq = ray.origin - self.center;
        let b = 2. * (cq * ray.direction);
        let c = (cq * cq) - self.radius * self.radius;

        let radicand = b * b - 4. * a * c;
        if radicand < 0. {
            // Ray is not pointing at the sphere.
            return None;
        }

        let radical = radicand.sqrt();

        // We have 0-2 positive hits.
        // Both negative: ray is pointing the opposite direction of the sphere
        // Both positive: ray hits the sphere twice (goes through it and hits the back side).
        // One positive, one negative: ray's origin is inside the sphere.
        // Same number: ray is tangent to the sphere, grazing it a single point.
        let t0 = (-b + radical) / (2. * a);
        let t1 = (-b - radical) / (2. * a);

        if t0 < 0. && t1 < 0. {
            return None;
        }

        let distance = min_positive(t0, t1);
        if distance < 0. {
            return None;
        }
        let point = ray.at(distance);
        Some(RaycastHit {
            distance,
            point,
            normal: (point - self.center).normalized(),
        })
    }

    fn signed_distance(&self, pt: Pt3) -> f64 {
        return (pt - self.center).mag() - self.radius;
    }
}

fn min_positive(a: f64, b: f64) -> f64 {
    if a < 0.0 {
        return b;
    }
    if b < 0.0 {
        return a;
    }
    if a < b {
        a
    } else {
        b
    }
}
