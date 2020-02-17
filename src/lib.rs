use std::ops;

pub trait VecN {
    type Type: VecN;

    /// Returns how many dimensions (components) this vector has.
    fn dimensions(&self) -> isize;

    /// Gets then value of the nth component.
    fn get(&self, index: isize) -> f64;

    /// Sets the value of the nth component.
    ///
    /// # Panics
    /// Panics if `index` is less than `0` or greater than or
    /// equal to the number of dimensions.
    fn set(&mut self, index: isize, value: f64) -> ();
}

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

pub struct ScaledPt3 {
    x: f64,
    y: f64,
    z: f64,
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
        return Self { x: 0., y: 0., z: 0. };
    }

    pub fn up() -> Self {
        return Self { x: 0., y: 0., z: 1. };
    }

    pub fn down() -> Self {
        return Self { x: 0., y: 0., z: -1. };
    }

    pub fn left() -> Self {
        return Self { x: -1., y: 0., z: 0. };
    }

    pub fn right() -> Self {
        return Self { x: 1., y: 0., z: 0. };
    }

    pub fn forward() -> Self {
        return Self { x: 0., y: 1., z: 0. };
    }

    pub fn backward() -> Self {
        return Self { x: 0., y: -1., z: 0. };
    }

    pub fn scaled_add(&self, scale: f64, other: &Vec3) -> Vec3 {
        return Vec3::new(
            self.x + scale * other.x,
            self.y + scale * other.y,
            self.z + scale * other.z,
        );
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

    pub fn norm2(&self) -> f64 {
        return self.dot(self);
    }

    pub fn norm(&self) -> f64 {
        return self.norm2().sqrt();
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
        return Self { x: 0., y: 0., z: 0. };
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
}
