use std::f64::INFINITY;
use std::ops::Div;
use std::panic::resume_unwind;

use crate::mesh::{Mesh, MeshBuilder, MeshFace, MeshFaceIter};
use crate::scalar::FloatRange;
use crate::threed::{Basis3, Frame3, Pt3, Ray3, Vec3};

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

pub trait FaceLike<T: FaceLike<T> = Self>: Shape + Edgecast {
    fn vertex(&self, index: usize) -> &Pt3;

    fn vertex_count(&self) -> usize;

    fn vertices(&self) -> FaceVertexIter<T> {
        FaceVertexIter {
            face: self.self_ref(),
            index: 0,
        }
    }

    fn clone_vertices(&self) -> Vec<Pt3> {
        let mut arr = vec![];
        for i in 0..self.vertex_count() {
            arr.push(self.vertex(i).clone());
        }
        arr
    }

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

    fn contains(&self, pt: &Pt3) -> bool {
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

    fn plane(&self) -> Plane {
        Plane::new(self.centroid(), self.normal())
    }

    fn edges(&self) -> FaceEdgeIter<T> {
        FaceEdgeIter::new(self.self_ref())
    }

    fn points(&self, resolution: f64) -> FacePointIter<T> {
        let basis = Basis3::from_normal(self.normal());
        let frame = Frame3::new(self.centroid(), basis);

        let mut min = (None, None);
        let mut max = (None, None);
        for i in 0..self.vertex_count() {
            let pt = self.vertex(i);
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
            face: self.self_ref(),
            frame,
            i_values,
            j_values,
            index: 0,
        }
    }

    /// Used internally to pass self as a dynamic dispatch trait reference.
    fn self_ref(&self) -> &dyn FaceLike<T>;
}

impl<T: FaceLike<T>> Shape for T {
    fn raycast(&self, ray: &Ray3) -> Option<RaycastHit> {
        let hit_on_plane = self.plane().raycast(ray);
        return match hit_on_plane {
            Some(hit) => {
                if self.contains(&hit.point) {
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
        if self.contains(&projection) {
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

pub trait Edgecast {
    fn edgecast(&self, edge: &Edge, direction: &Vec3) -> Option<RaycastHit>;
}

impl<T: FaceLike<T>> Edgecast for T {
    fn edgecast(&self, edge: &Edge, direction: &Vec3) -> Option<RaycastHit> {
        let direction = direction.clone();
        let src_hit = self.raycast(&Ray3::new(edge.src.clone(), direction));
        let dst_hit = self.raycast(&Ray3::new(edge.dst.clone(), direction));

        if let (Some(src), Some(dst)) = (&src_hit, &dst_hit) {
            return if src.distance < dst.distance {
                src_hit
            } else {
                dst_hit
            };
        }

        if src_hit.is_some() {
            return src_hit;
        }

        if dst_hit.is_some() {
            return dst_hit;
        }

        let mut closest_edge_hit: Option<RaycastHit> = None;
        for e in self.edges() {
            if let Some(hit) = e.edgecast(edge, &direction) {
                if closest_edge_hit.is_none() || closest_edge_hit.as_ref().unwrap().distance > hit.distance {
                    closest_edge_hit = Some(hit);
                }
            }
        }

        match closest_edge_hit {
            None => None,
            Some(hit) => Some(RaycastHit {
                point: hit.point,
                distance: hit.distance,
                normal: self.normal(),
            })
        }
    }
}

pub trait Facecast {
    fn facecast<T: FaceLike<T>>(&self, face: &dyn FaceLike<T>, direction: &Vec3) -> Option<RaycastHit>;
}

impl<T: FaceLike<T>> Facecast for T {
    fn facecast<U: FaceLike<U>>(&self, face: &dyn FaceLike<U>, direction: &Vec3) -> Option<RaycastHit> {
        // We just edgecast this face at the other and vice-versa, and take the closest hit between
        // them. I think this covers all cases, but it's a sort of tricky problem to think about.

        let mut closest: Option<RaycastHit> = None;

        // The other face's edges against self.
        for edge in face.edges() {
            if let Some(hit) = self.edgecast(&edge, direction) {
                if closest.is_none() || closest.as_ref().unwrap().distance > hit.distance {
                    closest = Some(hit);
                }
            }
        }

        // Backwards: our edges against the face.
        let reverse_dir = direction.scaled(-1.);
        for edge in self.edges() {
            if let Some(hit) = face.edgecast(&edge, &reverse_dir) {
                if closest.is_none() || closest.as_ref().unwrap().distance > hit.distance {
                    // We have to tweak the fields in the hit to account for the fact that we're
                    // raycasting backwards here.
                    closest = Some(RaycastHit {
                        point: hit.point + (direction.scaled(hit.distance)),
                        distance: hit.distance,
                        normal: self.normal(),
                    });
                }
            }
        }

        closest
    }
}

pub struct FaceVertexIter<'a, T: FaceLike<T>> {
    face: &'a dyn FaceLike<T>,
    index: usize,
}

impl<'a, T: FaceLike<T>> Iterator for FaceVertexIter<'a, T> {
    type Item = &'a Pt3;

    fn next(&mut self) -> Option<Self::Item> {
        let index = self.index;
        if index < self.face.vertex_count() {
            self.index += 1;
            Some(self.face.vertex(index))
        } else {
            None
        }
    }
}

pub struct FacePointIter<'a, T: FaceLike<T>> {
    face: &'a dyn FaceLike<T>,
    frame: Frame3,
    i_values: Vec<f64>,
    j_values: Vec<f64>,
    index: usize,
}

impl<'a, T: FaceLike<T>> Iterator for FacePointIter<'a, T> {
    type Item = Pt3;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let i_index = self.index / self.i_values.len();
            let j_index = self.index % self.i_values.len();
            self.index += 1;
            if i_index >= self.i_values.len() || j_index >= self.j_values.len() {
                return None;
            }
            let local = self
                .frame
                .local(self.i_values[i_index], self.j_values[j_index], 0.);
            let result = self.frame.unproject(&local);
            if self.face.contains(&result) {
                return Some(result);
            }
        }
    }
}

pub struct FaceEdgeIter<'a, T: FaceLike<T>> {
    face: &'a dyn FaceLike<T>,
    edge_index: usize,
}

impl<'a, T: FaceLike<T>> FaceEdgeIter<'a, T> {
    pub fn new(face: &'a dyn FaceLike<T>) -> Self {
        Self {
            face,
            edge_index: 0,
        }
    }
}

impl<'a, T: FaceLike<T>> Iterator for FaceEdgeIter<'a, T> {
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
    pub vertices: Vec<Pt3>,
}

impl Polygon {
    pub fn new(vertices: Vec<Pt3>) -> Self {
        Self {
            vertices
        }
    }
}

impl FaceLike<Polygon> for Polygon {
    fn vertex(&self, index: usize) -> &Pt3 {
        &self.vertices[index]
    }

    fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    fn self_ref(&self) -> &dyn FaceLike<Polygon> {
        self
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

impl<'a> Edgecast for Edge<'a> {
    fn edgecast(&self, edge: &Edge, direction: &Vec3) -> Option<RaycastHit> {
        let mut axis = self.vector().cross(&edge.vector()).normalized();

        // A     Q  A = edge.src
        //  \_  /   B = edge.dst
        //    \_    Q = self.src
        //   /  \   R = self.dst
        // R     B


        let aq = self.src - edge.src;
        let ortho_dir = direction.on_axis(&axis);

        if ortho_dir.mag2() == 0.0 {
            return None; // Lines are not moving at all relative to each other.
        }

        // Compute the time that the edges would collide, if they will collide.
        let t = aq.dot(&axis) / direction.dot(&axis);
        if t < 0. {
            return None; // Collision is back in time, edges are moving away from each other.
        }

        // Move the edge forward in time to see where they'll be at the potential point of collision.
        let edge1: (Pt3, Pt3) = (
            edge.src.clone() + direction.scaled(t),
            edge.dst.clone() + direction.scaled(t)
        );
        let edge2 = (self.src, self.dst);

        let edge1_normal = edge.vector().cross(&axis).normalized();

        // Now we just raycast edge1 at the plane of edge2.
        // (A + AB*d - R)*N = 0
        // (RA + AB*d)*N = 0
        // RA*N + AB*N d = 0
        // d = AR*N / AB*N
        let ab = edge.vector();
        let ab_norm = ab.dot(&edge1_normal);
        if ab_norm == 0.0 {
            return None; // Line segments are parallel.
        }
        let d = (edge2.0 - edge1.0).dot(&edge1_normal) / ab_norm;
        if d < 0. {
            return None; // Line segments don't overlap.
        }

        // Point of collision on edge2's infinite line.
        // Now we just need to see if it's within the actual bounds of edge2.
        let poc = ab.scaled(d) + edge1.0;

        if (poc - self.src).dot(&(self.dst - self.src)) < 0. {
            // Point is before the beginning of the line segment.
            return None;
        }
        if (poc - self.dst).dot(&(self.src - self.dst)) < 0. {
            // Point is after the end of the line segment.
            return None;
        }

        Some(RaycastHit {
            point: poc,
            distance: t,
            normal: axis,
        })
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

        let mut builder = MeshBuilder::new();

        let rfu = builder.add_vertex((right + forward + up) + center);
        let rfd = builder.add_vertex((right + forward - up) + center);
        let rbd = builder.add_vertex((right - forward - up) + center);
        let rbu = builder.add_vertex((right - forward + up) + center);
        let lfu = builder.add_vertex((-right + forward + up) + center);
        let lfd = builder.add_vertex((-right + forward - up) + center);
        let lbd = builder.add_vertex((-right - forward - up) + center);
        let lbu = builder.add_vertex((-right - forward + up) + center);

        builder.add_face(vec![rfu, rbu, lbu, lfu]);
        builder.add_face(vec![rfd, lfd, lbd, rbd]);
        builder.add_face(vec![lfu, lbu, lbd, lfd]);
        builder.add_face(vec![rfu, rfd, rbd, rbu]);
        builder.add_face(vec![rfu, lfu, lfd, rfd]);
        builder.add_face(vec![lbu, rbu, rbd, lbd]);

        let mesh = builder.build();
        Self {
            center,
            dimensions,
            mesh,
        }
    }

    pub fn faces(&self) -> MeshFaceIter {
        self.mesh.faces()
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
