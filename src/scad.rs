use std::process::Command;

use crate::threed::{Pt3, Vec3};
use crate::geom;
use std::ops::Add;
use std::fmt::Display;
use std::intrinsics::transmute;

pub trait ToScad {
    fn to_scad(&self) -> ScadCode;
}

impl ToScad for geom::Sphere {
    fn to_scad(&self) -> ScadCode {
        translate(&self.center)
            .wrap(scad(&format!("sphere(r={:#?});", &self.radius)))
    }
}

impl ToScad for geom::Cube {
    fn to_scad(&self) -> ScadCode {
        translate(&self.center)
            .wrap(scad(&format!("cube([{:#?}, {:#?}, {:#?}]);",
                                 self.dimensions.x,
                                 self.dimensions.y,
                                 self.dimensions.z)))
    }
}

pub struct StlImport {
    path: String,
}

impl StlImport {
    pub fn new(path: &str) -> StlImport {
        StlImport {
            path: path.to_string(),
        }
    }
}

impl ToScad for StlImport {
    fn to_scad(&self) -> ScadCode {
        scad(&format!("import(\"{}\");", self.path))
    }
}

pub struct ScadCode {
    script: Vec<String>,
}

impl ScadCode {
    pub fn wrap(self, code: ScadCode) -> ScadCode {
        self + scad(" { ") + code + scad(" } ")
    }
}

impl ToString for ScadCode {
    fn to_string(&self) -> String {
        (&self.script).join("")
    }
}

impl Add<ScadCode> for ScadCode {
    type Output = ScadCode;

    fn add(self, rhs: ScadCode) -> Self::Output {
        let mut script = self.script;
        script.extend(rhs.script);
        ScadCode {
            script,
        }
    }
}

impl Add<&ScadCode> for ScadCode {
    type Output = ScadCode;

    fn add(self, rhs: &ScadCode) -> Self::Output {
        let mut script = self.script;
        rhs.script.iter().for_each(|s| {
            script.push(s.clone());
        });
        ScadCode {
            script,
        }
    }
}

fn scad(text: &str) -> ScadCode {
    ScadCode { script: vec![text.to_string()] }
}

fn translate(point: &Pt3) -> ScadCode {
    scad(&format!("[{:#?}, {:#?}, {:#?}]", point.x, point.y, point.z))
}

