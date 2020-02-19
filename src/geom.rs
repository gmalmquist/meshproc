use crate::threed::{Basis3, Frame3, Pt3, Ray3, Vec3};
use std::f64::{NAN, INFINITY};

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

pub struct Polygon {
    pub points: Vec<Pt3>,
    pub edges: Vec<Edge>,
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
        let mut edges = Vec::new();
        for (i, pt) in points.iter().enumerate() {
            let src = pt.clone();
            let dst = points[(i + 1) % points.len()];
            edges.push(Edge { src, dst });
        }
        let normal = edges[0].vector().cross(&edges[1].vector()).normalized();
        let (mut cx, mut cy, mut cz) = (0.0, 0.0, 0.0);
        for pt in &points {
            cx += pt.x;
            cy += pt.y;
            cz += pt.z;
        }
        let centroid = Pt3::new(cx, cy, cz);
        Self {
            points,
            edges,
            normal,
            centroid,
        }
    }

    pub fn plane(&self) -> Plane {
        return Plane::new(self.centroid, self.normal);
    }

    pub fn contains(&self, pt: Pt3) -> bool {
        for edge in &self.edges {
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
        let sign = if self.normal * (pt - self.centroid) >= 0.0 { 1.0 } else { -1.0 };

        // If the projection of the point onto the plane of this polygon is contained within the
        // boundaries of this polygon, that projection will be the closest point.
        let projection = self.plane().project(pt);
        if self.contains(projection) {
            return sign * (projection - pt).mag();
        }

        // If we're outside the bounds of the polygon, our distance to it is the closest distance
        // to any of its edges.
        let mut distance = 0.0;
        for edge in &self.edges {
            let edge_distance = edge.distance(pt);
            if edge_distance < distance {
                distance = edge_distance;
            }
        }

        distance * sign
    }
}

pub struct Edge {
    pub src: Pt3,
    pub dst: Pt3,
}

impl Edge {
    pub fn vector(&self) -> Vec3 {
        return self.dst - self.src;
    }

    pub fn distance(&self, pt: Pt3) -> f64 {
        let frame = Frame3::new(self.src, Basis3::new1(self.vector()));
        let mut local = frame.project(pt);
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
    vertices: Vec<Pt3>,
    faces: Vec<Polygon>,
}

impl Cube {
    pub fn new(origin: Pt3, dimensions: Vec3, centered: bool) -> Cube {
        let mut center: Pt3 = if centered {
            origin
        } else {
            origin + dimensions / 2.0
        };

        let right = dimensions.on_axis(&Vec3::right());
        let forward = dimensions.on_axis(&Vec3::forward());
        let up = dimensions.on_axis(&Vec3::forward());


        let rfu = (right + forward + up) + center;
        let rfd = (right + forward - up) + center;
        let rbd = (right - forward - up) + center;
        let rbu = (right - forward + up) + center;
        let lfu = (-right + forward + up) + center;
        let lfd = (-right + forward - up) + center;
        let lbd = (-right - forward - up) + center;
        let lbu = (-right - forward + up) + center;

        let vertices = vec![
            rfu, rfd, rbd, rbu,
            lfu, lfd, lbd, lbu
        ];

        let faces = vec![
            Polygon::new(vec![rfu, rbu, lbu, lfu]),
            Polygon::new(vec![rfd, lfd, lbd, rbd]),
            Polygon::new(vec![lfu, lbu, lbd, lfd]),
            Polygon::new(vec![rfu, rfd, rbd, rbu]),
            Polygon::new(vec![rfu, lfu, lfd, rfd]),
            Polygon::new(vec![lbu, rbu, rbd, lbd]),
        ];

        Cube {
            center,
            dimensions: dimensions,
            vertices: vertices,
            faces: faces,
        }
    }
}

impl Shape for Cube {
    fn raycast(&self, ray: &Ray3) -> Option<RaycastHit> {
        Mesh::raycast_polygons(&self.faces, ray)
    }

    fn signed_distance(&self, pt: Pt3) -> f64 {
        Mesh::signed_distance_polygons(&self.faces, pt)
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

        let radicand = b*b - 4.*a*c;
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
            return None
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

pub struct Mesh {
    pub polygons: Vec<Polygon>,
}

impl Mesh {
    pub fn new(polygons: Vec<Polygon>) -> Mesh {
        return Mesh { polygons };
    }

    fn raycast_polygons(polygons: &Vec<Polygon>, ray: &Ray3) -> Option<RaycastHit> {
        let mut hit: Option<RaycastHit> = None;
        for poly in polygons {
            let poly_hit = poly.raycast(ray);
            if let Some(poly_hit) = poly_hit {
                hit = match hit {
                    None => None,
                    Some(hit) => if hit.distance < poly_hit.distance {
                        Some(hit)
                    } else {
                        Some(poly_hit)
                    }
                };
            }
        }
        hit
    }

    fn signed_distance_polygons(polygons: &Vec<Polygon>, pt: Pt3) -> f64 {
        let mut distance = INFINITY;
        for poly in polygons {
            let dist = poly.signed_distance(pt);
            if dist < distance {
                distance = dist;
            }
        }
        distance
    }
}

impl Shape for Mesh {
    fn raycast(&self, ray: &Ray3) -> Option<RaycastHit> {
        Self::raycast_polygons(&self.polygons, ray)
    }

    fn signed_distance(&self, pt: Pt3) -> f64 {
        Self::signed_distance_polygons(&self.polygons, pt)
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