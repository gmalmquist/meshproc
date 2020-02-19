use std::process::Command;

use crate::threed::{Pt3, Vec3};
use crate::geom;

pub trait ToScad {
    fn to_scad(&self) -> ScadCode;
}

pub struct ScadCode {
    script: String,
}

impl ToScad for geom::Sphere {
    fn to_scad(&self) -> ScadCode {
        scad(format!("{} sphere(r={});", translate(&self.center), self.radius))
    }
}

impl ToScad for geom::Cube {
    fn to_scad(&self) -> ScadCode {
        scad(format!("{} cube([{:#?} {:#?} {:#?}]);",
            translate(&(self.center - self.dimensions/2.0)),
            self.dimensions.x, self.dimensions.y, self.dimensions.z))
    }
}

fn scad(text: String) -> ScadCode {
    ScadCode { script: text }
}

fn translate(point: &Pt3) -> String {
    format!("[{:#?}, {:#?}, {:#?}]", point.x, point.y, point.z)
}
