use std::ops;

#[derive(Debug, PartialEq, Copy, Clone)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub struct Pt3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub struct ScaledPt3 {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Debug, PartialEq)]
pub struct Ray3 {
    pub origin: Pt3,
    pub direction: Vec3,
}

#[derive(Debug, PartialEq)]
pub struct Basis3 {
    pub axes: (Vec3, Vec3, Vec3),
}

#[derive(Debug, PartialEq)]
pub struct Frame3 {
    pub origin: Pt3,
    pub basis: Basis3,
}

#[derive(Debug, PartialEq)]
pub struct LocalPoint<'a> {
    frame: &'a Frame3,
    pub i: f64,
    pub j: f64,
    pub k: f64,
}

#[derive(Debug, PartialEq)]
pub struct LocalVec<'a> {
    basis: &'a Basis3,
    pub i: f64,
    pub j: f64,
    pub k: f64,
}

impl Vec3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        return Self { x, y, z };
    }

    pub fn from_tuple(tup: (f64, f64, f64)) -> Self {
        return Self::new(tup.0, tup.1, tup.2);
    }

    pub fn to_tuple(&self) -> (f64, f64, f64) {
        return (self.x, self.y, self.z);
    }

    pub fn zero() -> Self {
        return Self {
            x: 0.,
            y: 0.,
            z: 0.,
        };
    }

    pub fn up() -> Self {
        return Self {
            x: 0.,
            y: 0.,
            z: 1.,
        };
    }

    pub fn down() -> Self {
        return Self {
            x: 0.,
            y: 0.,
            z: -1.,
        };
    }

    pub fn left() -> Self {
        return Self {
            x: -1.,
            y: 0.,
            z: 0.,
        };
    }

    pub fn right() -> Self {
        return Self {
            x: 1.,
            y: 0.,
            z: 0.,
        };
    }

    pub fn forward() -> Self {
        return Self {
            x: 0.,
            y: 1.,
            z: 0.,
        };
    }

    pub fn backward() -> Self {
        return Self {
            x: 0.,
            y: -1.,
            z: 0.,
        };
    }

    pub fn sum(vecs: Vec<Vec3>) -> Option<Self> {
        let mut total: Option<Self> = None;
        for v in vecs {
            total = match total {
                None => Some(v),
                Some(total) => Some(total + v),
            }
        }
        total
    }

    pub fn on_axis(&self, axis: &Vec3) -> Vec3 {
        let mut copy = self.clone();
        copy.project(axis);
        copy
    }

    pub fn off_axis(&self, axis: &Vec3) -> Vec3 {
        let mut copy = self.clone();
        copy.flatten(axis);
        copy
    }

    pub fn normalized(&self) -> Vec3 {
        let mut copy = self.clone();
        copy.normalize();
        copy
    }

    pub fn scaled(&self, scale: f64) -> Vec3 {
        let mut copy = self.clone();
        copy.scale(scale);
        copy
    }

    pub fn scaled3(&mut self, scale: (f64, f64, f64)) -> Vec3 {
        let mut copy = self.clone();
        copy.scale3(scale);
        copy
    }

    pub fn project(&mut self, axis: &Vec3) {
        let s = self.dot(axis) / axis.dot(axis);
        self.x = s * axis.x;
        self.y = s * axis.y;
        self.z = s * axis.z;
    }

    pub fn flatten(&mut self, axis: &Vec3) {
        let s = self.dot(axis) / axis.dot(axis);
        self.x -= s * axis.x;
        self.y -= s * axis.y;
        self.z -= s * axis.z;
    }

    pub fn normalize(&mut self) {
        let mag2 = self.mag2();
        if mag2 == 0.0 || mag2 == 1.0 {
            return;
        }
        let mag = mag2.sqrt();
        self.x /= mag;
        self.y /= mag;
        self.z /= mag;
    }

    pub fn scale(&mut self, scale: f64) {
        self.x *= scale;
        self.y *= scale;
        self.z *= scale;
    }

    pub fn scale3(&mut self, scale: (f64, f64, f64)) {
        self.x *= scale.0;
        self.y *= scale.1;
        self.z *= scale.2;
    }

    pub fn cross(&self, other: &Vec3) -> Vec3 {
        let a = self;
        let b = other;
        return Vec3::new(
            a.y * b.z - a.z * b.y,
            -(a.x * b.z - a.z * b.x),
            a.x * b.y - a.y * b.x,
        );
    }

    pub fn dot(&self, other: &Vec3) -> f64 {
        return self.x * other.x + self.y * other.y + self.z * other.z;
    }

    pub fn mag2(&self) -> f64 {
        return self.dot(self);
    }

    pub fn mag(&self) -> f64 {
        return self.mag2().sqrt();
    }
}

impl Pt3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        return Self { x, y, z };
    }

    pub fn from_tuple(tup: (f64, f64, f64)) -> Self {
        return Self::new(tup.0, tup.1, tup.2);
    }

    pub fn to_tuple(&self) -> (f64, f64, f64) {
        return (self.x, self.y, self.z);
    }

    pub fn zero() -> Self {
        return Self {
            x: 0.,
            y: 0.,
            z: 0.,
        };
    }

    pub fn lerp(self, s: f64, other: Pt3) -> Pt3 {
        return (1.0 - s) * self + (s * other);
    }

    pub fn sum(pts: Vec<ScaledPt3>) -> Option<Self> {
        let mut total: Option<Self> = None;
        for pt in pts {
            total = match total {
                None => Some(pt + Self::zero()),
                Some(total) => Some(total + pt),
            }
        }
        total
    }
}

impl ScaledPt3 {
    fn new(x: f64, y: f64, z: f64) -> Self {
        return Self { x, y, z };
    }

    pub fn of(scale: f64, point: Pt3) -> Self {
        return Self {
            x: scale * point.x,
            y: scale * point.y,
            z: scale * point.z,
        };
    }
}

impl Ray3 {
    pub fn new(origin: Pt3, direction: Vec3) -> Self {
        return Self {
            origin,
            direction: direction.normalized(),
        };
    }

    pub fn at(&self, distance: f64) -> Pt3 {
        return self.origin + distance * self.direction;
    }
}

impl Basis3 {
    pub fn new(axes: (Vec3, Vec3, Vec3)) -> Self {
        Self { axes }
    }

    pub fn new2(axes: (Vec3, Vec3)) -> Self {
        Self::new((axes.0, axes.1, (axes.0 ^ axes.1).normalized()))
    }

    pub fn new1(axis: Vec3) -> Self {
        let mut ortho = axis.cross(&Vec3::up());
        if ortho.mag2() == 0.0 {
            ortho = axis.cross(&Vec3::right());
        }
        ortho.normalize();
        Self::new2((axis, ortho))
    }

    pub fn project(&self, v: &Vec3) -> LocalVec {
        LocalVec {
            basis: &self,
            i: v.dot(&self.axes.0) / self.axes.0.mag2(),
            j: v.dot(&self.axes.1) / self.axes.1.mag2(),
            k: v.dot(&self.axes.2) / self.axes.2.mag2(),
        }
    }

    pub fn unproject(&self, local: &LocalVec) -> Vec3 {
        (local.i * self.axes.0) + (local.j * self.axes.1) + (local.k * self.axes.2)
    }

    pub fn identity() -> Self {
        Self {
            axes: (Vec3::right(), Vec3::forward(), Vec3::up()),
        }
    }
}

impl Frame3 {
    pub fn new(origin: Pt3, basis: Basis3) -> Self {
        Self { origin, basis }
    }

    pub fn project(&self, pt: Pt3) -> LocalPoint {
        LocalPoint {
            frame: &self,
            i: (pt - self.origin).dot(&self.basis.axes.0) / self.basis.axes.0.mag2(),
            j: (pt - self.origin).dot(&self.basis.axes.1) / self.basis.axes.1.mag2(),
            k: (pt - self.origin).dot(&self.basis.axes.2) / self.basis.axes.2.mag2(),
        }
    }

    pub fn unproject(&self, local: &LocalPoint) -> Pt3 {
        self.origin
            + (local.i * self.basis.axes.0)
            + (local.j * self.basis.axes.1)
            + (local.k * self.basis.axes.2)
    }

    pub fn identity() -> Self {
        Self {
            origin: Pt3::zero(),
            basis: Basis3::identity(),
        }
    }
}

impl<'a> LocalPoint<'a> {
    pub fn unproject(&self) -> Pt3 {
        return self.frame.unproject(self);
    }

    pub fn clone(&self) -> LocalPoint<'a> {
        LocalPoint {
            frame: &self.frame,
            i: self.i,
            j: self.j,
            k: self.k,
        }
    }
}

impl<'a> LocalVec<'a> {
    pub fn unproject(&self) -> Vec3 {
        return self.basis.unproject(self);
    }

    pub fn clone(&self) -> LocalVec<'a> {
        LocalVec {
            basis: &self.basis,
            i: self.i,
            j: self.j,
            k: self.k,
        }
    }
}

impl <'a, 'b> ops::Add<&'b LocalVec<'b>> for &'a LocalVec<'a> {
    type Output = LocalVec<'a>;

    fn add(self, v: &'b LocalVec) -> LocalVec<'a> {
        LocalVec {
            basis: &self.basis,
            i: self.i + v.i,
            j: self.j + v.j,
            k: self.k + v.k,
        }
    }
}

impl <'a, 'b> ops::Add<&'b LocalVec<'b>> for &'a LocalPoint<'a> {
    type Output = LocalPoint<'a>;

    fn add(self, v: &'b LocalVec) -> LocalPoint<'a> {
        LocalPoint {
            frame: &self.frame,
            i: self.i + v.i,
            j: self.j + v.j,
            k: self.k + v.k,
        }
    }
}

impl <'a, 'b> ops::Sub<&'b LocalVec<'b>> for &'a LocalPoint<'a> {
    type Output = LocalPoint<'a>;

    fn sub(self, v: &'b LocalVec) -> LocalPoint<'a> {
        LocalPoint {
            frame: &self.frame,
            i: self.i - v.i,
            j: self.j - v.j,
            k: self.k - v.k,
        }
    }
}

impl <'a, 'b> ops::Add<&'b LocalPoint<'b>> for &'a LocalVec<'a> {
    type Output = LocalPoint<'b>;

    fn add(self, pt: &'b LocalPoint) -> LocalPoint<'b> {
        LocalPoint {
            frame: &pt.frame,
            i: self.i + pt.i,
            j: self.j + pt.j,
            k: self.k + pt.k,
        }
    }
}

impl <'a, 'b> ops::Sub<&'b LocalPoint<'b>> for &'a LocalPoint<'a> {
    type Output = LocalVec<'a>;

    fn sub(self, pt: &'b LocalPoint) -> LocalVec<'a> {
        LocalVec {
            basis: &self.frame.basis,
            i: self.i - pt.i,
            j: self.j - pt.j,
            k: self.k - pt.k,
        }
    }
}

impl <'a,> ops::Mul<f64> for &'a LocalVec<'a> {
    type Output = LocalVec<'a>;

    fn mul(self, s: f64) -> LocalVec<'a> {
        LocalVec {
            basis: &self.basis,
            i: self.i * s,
            j: self.j * s,
            k: self.k * s,
        }
    }
}

impl <'a,> ops::Mul<f64> for &'a LocalPoint<'a> {
    type Output = LocalPoint<'a>;

    fn mul(self, s: f64) -> LocalPoint<'a> {
        LocalPoint {
            frame: &self.frame,
            i: self.i * s,
            j: self.j * s,
            k: self.k * s,
        }
    }
}


// Vector & Point Addition

// Vec + Vec = Vec
impl ops::Add<Vec3> for Vec3 {
    type Output = Vec3;

    fn add(self, rhs: Vec3) -> Vec3 {
        return Vec3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        };
    }
}

// Pt + Vec = Pt
impl ops::Add<Pt3> for Vec3 {
    type Output = Pt3;

    fn add(self, rhs: Pt3) -> Pt3 {
        return Pt3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        };
    }
}

// Vec + Pt = Pt
impl ops::Add<Vec3> for Pt3 {
    type Output = Pt3;

    fn add(self, rhs: Vec3) -> Pt3 {
        return Pt3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        };
    }
}

// s Pt + s Pt = Pt
impl ops::Add<ScaledPt3> for ScaledPt3 {
    type Output = Pt3;

    fn add(self, rhs: ScaledPt3) -> Pt3 {
        return Pt3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        };
    }
}

// s Pt + Pt = Pt
impl ops::Add<Pt3> for ScaledPt3 {
    type Output = Pt3;

    fn add(self, rhs: Pt3) -> Pt3 {
        return Pt3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        };
    }
}

// Pt + s Pt = Pt
impl ops::Add<ScaledPt3> for Pt3 {
    type Output = Pt3;

    fn add(self, rhs: ScaledPt3) -> Pt3 {
        return Pt3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        };
    }
}

impl ops::AddAssign<Vec3> for Vec3 {
    fn add_assign(&mut self, rhs: Vec3) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl ops::AddAssign<Vec3> for Pt3 {
    fn add_assign(&mut self, rhs: Vec3) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

// Vector & Point Subtraction

// Vec - Vec = Vec
impl ops::Sub<Vec3> for Vec3 {
    type Output = Vec3;

    fn sub(self, rhs: Vec3) -> Vec3 {
        return Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        };
    }
}

impl ops::SubAssign<Vec3> for Vec3 {
    fn sub_assign(&mut self, rhs: Vec3) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

// Pt - Vec = Pt
impl ops::Sub<Vec3> for Pt3 {
    type Output = Pt3;

    fn sub(self, rhs: Vec3) -> Pt3 {
        return Pt3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        };
    }
}

// Pt - Vec = Pt
impl ops::SubAssign<Vec3> for Pt3 {
    fn sub_assign(&mut self, rhs: Vec3) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

// Pt - Pt = Vec
impl ops::Sub<Pt3> for Pt3 {
    type Output = Vec3;

    fn sub(self, rhs: Pt3) -> Vec3 {
        return Vec3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        };
    }
}

// Scaling
impl ops::Mul<f64> for Vec3 {
    type Output = Vec3;

    fn mul(self, scale: f64) -> Vec3 {
        return Vec3::new(self.x * scale, self.y * scale, self.z * scale);
    }
}

impl ops::Mul<f64> for Pt3 {
    type Output = ScaledPt3;

    fn mul(self, scale: f64) -> ScaledPt3 {
        return ScaledPt3::new(self.x * scale, self.y * scale, self.z * scale);
    }
}

impl ops::Mul<Vec3> for f64 {
    type Output = Vec3;

    fn mul(self, vec: Vec3) -> Vec3 {
        return Vec3::new(vec.x * self, vec.y * self, vec.z * self);
    }
}

impl ops::Mul<Pt3> for f64 {
    type Output = ScaledPt3;

    fn mul(self, pt: Pt3) -> ScaledPt3 {
        return ScaledPt3::new(pt.x * self, pt.y * self, pt.z * self);
    }
}

impl ops::Div<f64> for Vec3 {
    type Output = Vec3;

    fn div(self, scale: f64) -> Vec3 {
        return Vec3::new(self.x / scale, self.y / scale, self.z / scale);
    }
}

impl ops::Div<f64> for Pt3 {
    type Output = ScaledPt3;

    fn div(self, scale: f64) -> ScaledPt3 {
        return ScaledPt3::new(self.x / scale, self.y / scale, self.z / scale);
    }
}

impl ops::MulAssign<f64> for Vec3 {
    fn mul_assign(&mut self, scale: f64) {
        self.x *= scale;
        self.y *= scale;
        self.z *= scale;
    }
}

impl ops::DivAssign<f64> for Vec3 {
    fn div_assign(&mut self, scale: f64) {
        self.x /= scale;
        self.y /= scale;
        self.z /= scale;
    }
}

// Inner (dot) Product
impl ops::Mul<Vec3> for Vec3 {
    type Output = f64;

    fn mul(self, rhs: Vec3) -> f64 {
        return self.dot(&rhs);
    }
}

// Outer (cross) Product
impl ops::BitXor<Vec3> for Vec3 {
    type Output = Vec3;

    fn bitxor(self, rhs: Vec3) -> Vec3 {
        return self.cross(&rhs);
    }
}

impl ops::Index<u8> for Vec3 {
    type Output = f64;

    fn index(&self, index: u8) -> &f64 {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Index out of bounds for vector index: {}", index),
        }
    }
}

impl ops::Index<u8> for Pt3 {
    type Output = f64;

    fn index(&self, index: u8) -> &f64 {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Index out of bounds for vector index: {}", index),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adds() {
        let mut lhs = Vec3::new(0., 1., 2.);
        assert_eq!(Vec3::new(1., 2., 3.), lhs + Vec3::new(1., 1., 1.));

        assert_eq!(Vec3::new(0., 1., 2.), lhs);

        lhs += Vec3::new(3., 3., 3.);
        assert_eq!(lhs, Vec3::new(3., 4., 5.));
    }

    #[test]
    fn subtracts() {
        let a = v456();
        let b = v234();
        assert_eq!(a - b, Vec3::new(2.0, 2.0, 2.0));
        assert_eq!(b - a, Vec3::new(-2.0, -2.0, -2.0));
        assert_eq!(p456() - p345(), v111())
    }

    #[test]
    fn scale() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(a * 2.0, Vec3::new(2.0, 4.0, 6.0));
        assert_eq!(a / 2.0, Vec3::new(0.5, 1.0, 1.5));
    }

    #[test]
    fn adds_local() {
        let basis = Basis3::identity();
        let a = basis.project(&Vec3::right());
        let b = basis.project(&Vec3::up());
        assert_eq!(&a + &b, LocalVec {
            basis: &basis,
            i: 1.0,
            j: 0.0,
            k: 1.0,
        });
    }

    fn v111() -> Vec3 {
        return Vec3::new(1.0, 1.0, 1.0);
    }

    fn v123() -> Vec3 {
        return Vec3::new(1.0, 2.0, 3.0);
    }

    fn v234() -> Vec3 {
        return Vec3::new(2.0, 3.0, 4.0);
    }

    fn v345() -> Vec3 {
        return Vec3::new(3.0, 4.0, 5.0);
    }

    fn v456() -> Vec3 {
        return Vec3::new(4.0, 5.0, 6.0);
    }

    fn p111() -> Pt3 {
        return Pt3::new(1.0, 1.0, 1.0);
    }

    fn p123() -> Pt3 {
        return Pt3::new(1.0, 2.0, 3.0);
    }

    fn p234() -> Pt3 {
        return Pt3::new(2.0, 3.0, 4.0);
    }

    fn p345() -> Pt3 {
        return Pt3::new(3.0, 4.0, 5.0);
    }

    fn p456() -> Pt3 {
        return Pt3::new(4.0, 5.0, 6.0);
    }
}
