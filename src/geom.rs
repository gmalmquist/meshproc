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

pub struct Mesh {
    pub polygons: Vec<Polygon>,
}

impl Mesh {
    pub fn new(polygons: Vec<Polygon>) -> Mesh {
        return Mesh { polygons };
    }
}

impl Shape for Mesh {
    fn raycast(&self, ray: &Ray3) -> Option<RaycastHit> {
        let mut hit = None;
        for poly in &self.polygons {
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

    fn signed_distance(&self, pt: Pt3) -> f64 {
        unimplemented!()
    }
}
